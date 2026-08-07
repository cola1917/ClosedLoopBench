"""Run the open-loop Pure Pursuit smoke through the local ROS boundary."""

from __future__ import annotations

import argparse
import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

from agents.algorithm_backend import load_backend
from agents.ego_observation import build_pinhole_calibration
from agents.plugin_contract import AlgorithmPluginExecutor, strict_json_loads
from agents.ros2_tcp_bridge import Ros2TcpBridge, build_ros2_tcp_bridge_plan
from runners.run_open_loop_gt_replay import (
    DEFAULT_OPENDRIVE,
    DEFAULT_SCENARIO_IR,
    EXPECTED_OPENDRIVE_SHA256,
    EXPECTED_SCENARIO_IR_SHA256,
    GroundTruthReplayHost,
    _normalise_track,
    _route_from_ego_track,
    load_pinned_inputs,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_SCHEMA = "open_loop_ros_boundary_report.v1"
EVIDENCE_CLASSIFICATION = "open_loop_multimodal"


class OpenLoopRosSmokeError(ValueError):
    """Raised when the local ROS open-loop boundary cannot fail closed."""


class _MemoryPublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.messages: list[Any] = []

    def publish(self, message: Any) -> None:
        self.messages.append(deepcopy(message))


class _MemoryRosNode:
    """Small injectable ROS-like node used for a deterministic boundary smoke."""

    def __init__(self) -> None:
        self.subscriptions: list[dict[str, Any]] = []
        self.publishers: dict[str, _MemoryPublisher] = {}

    def create_subscription(
        self, message_type: Any, topic: str, callback: Any, qos: int
    ) -> Any:
        self.subscriptions.append(
            {
                "message_type": message_type,
                "topic": str(topic),
                "callback": callback,
                "qos": int(qos),
            }
        )
        return callback

    def create_publisher(self, message_type: Any, topic: str, qos: int) -> _MemoryPublisher:
        publisher = _MemoryPublisher(str(topic))
        self.publishers[str(topic)] = publisher
        return publisher

    def emit(self, topic: str, message: Any) -> None:
        callbacks = [
            subscription["callback"]
            for subscription in self.subscriptions
            if subscription["topic"] == str(topic)
        ]
        if len(callbacks) != 1:
            raise OpenLoopRosSmokeError(
                f"expected one subscription for {topic}, found {len(callbacks)}"
            )
        callbacks[0](deepcopy(message))


class _PluginBridgeBackend:
    """Adapt the bridge's transport observation to the plugin contract."""

    def __init__(self, executor: AlgorithmPluginExecutor) -> None:
        self.executor = executor
        self.results: list[dict[str, Any]] = []

    def predict_control(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        result = self.executor.predict(observation)
        self.results.append(deepcopy(result))
        return result["control"]


def run_open_loop_ros_smoke(
    *,
    scenario_ir_path: str | Path,
    opendrive_path: str | Path,
    expected_scenario_ir_sha256: str = EXPECTED_SCENARIO_IR_SHA256,
    expected_opendrive_sha256: str = EXPECTED_OPENDRIVE_SHA256,
    plugin_spec: str = "agents.reference_pure_pursuit:create_plugin",
    plugin_config: Mapping[str, Any] | None = None,
    run_id: str = "scene-0061-open-loop-m4",
    max_frames: int | None = None,
) -> dict[str, Any]:
    """Run Pure Pursuit through the frame-matched local ROS bridge.

    The memory node models the existing ROS topic contract without requiring
    rclpy. It is an offline boundary test, not evidence of a live ROS2/CARLA
    transport. The replay host still owns every current and next ego pose.
    """

    if not isinstance(run_id, str) or not run_id.strip():
        raise OpenLoopRosSmokeError("run_id must be a non-empty string")
    if max_frames is not None and (isinstance(max_frames, bool) or max_frames <= 0):
        raise OpenLoopRosSmokeError("max_frames must be positive when provided")

    inputs = load_pinned_inputs(
        scenario_ir_path,
        opendrive_path,
        expected_scenario_ir_sha256=expected_scenario_ir_sha256,
        expected_opendrive_sha256=expected_opendrive_sha256,
    )
    ego_track = _normalise_track(
        (inputs.scenario_ir.get("ego") or {}).get("reference_trajectory"),
        "ego.reference_trajectory",
        minimum=2,
    )
    if max_frames is not None:
        ego_track = ego_track[:max_frames]
    if len(ego_track) < 2:
        raise OpenLoopRosSmokeError("open-loop ROS smoke requires at least two frames")

    route = _route_from_ego_track(ego_track)
    config = dict(plugin_config or {})
    config.setdefault("repo_path", str(PROJECT_ROOT))
    plugin = load_backend(plugin_spec, config)
    executor = AlgorithmPluginExecutor(
        plugin,
        config,
        already_initialized=True,
        evidence_classification=EVIDENCE_CLASSIFICATION,
    )
    node = _MemoryRosNode()
    backend = _PluginBridgeBackend(executor)
    plan = build_ros2_tcp_bridge_plan(scenario_id=inputs.scenario_ir["scenario_id"])
    bridge = Ros2TcpBridge(node=node, plan=plan, backend=backend)
    host = GroundTruthReplayHost()
    calibration = build_pinhole_calibration(
        width=900,
        height=256,
        fov_deg=100.0,
        sensor_to_ego=[
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    )
    frames: list[dict[str, Any]] = []
    fallback_count = 0
    frame_mismatch_count = 0
    try:
        plugin_identity = executor.initialize()
        executor.reset(
            {
                "mode": "open_loop_ros_boundary_smoke",
                "scenario_id": inputs.scenario_ir["scenario_id"],
                "ego_pose_source": "scenario_ir_reference_trajectory",
            }
        )
        for frame_id, state in enumerate(ego_track):
            t_sec = float(state["t_sec"])
            host.set_ego_pose(state, source="scenario_ir_reference_trajectory")
            pose_before_control = deepcopy(host.current_ego_pose)
            _emit_tick(
                node=node,
                plan=plan,
                frame_id=frame_id,
                t_sec=t_sec,
                state=state,
                route=route,
                calibration=calibration,
            )
            result_count_before = len(backend.results)
            bridge_result = bridge.tick(now_sec=t_sec)
            pose_after_control = deepcopy(host.current_ego_pose)
            if pose_before_control != pose_after_control:
                raise OpenLoopRosSmokeError(
                    f"bridge control changed ego pose at frame {frame_id}"
                )

            control = deepcopy(bridge_result["control"])
            control_frame_id = control.get("source_frame_id")
            if control_frame_id != frame_id:
                frame_mismatch_count += 1
            plugin_result = (
                backend.results[-1]
                if len(backend.results) > result_count_before
                else None
            )
            plugin_status = plugin_result.get("execution_status") if plugin_result else None
            if plugin_status == "fallback":
                fallback_count += 1
            next_pose = None
            if frame_id + 1 < len(ego_track):
                next_state = ego_track[frame_id + 1]
                host.set_ego_pose(
                    next_state,
                    source="scenario_ir_reference_trajectory",
                )
                next_pose = deepcopy(host.current_ego_pose)
                if next_pose != {
                    axis: float(next_state[axis]) for axis in ("x", "y", "z", "yaw")
                }:
                    raise OpenLoopRosSmokeError(
                        f"next ego pose mismatch at frame {frame_id}"
                    )
            frames.append(
                {
                    "frame_id": frame_id,
                    "t_sec": t_sec,
                    "observation_frame_id": frame_id,
                    "control_source_frame_id": control_frame_id,
                    "bridge_status": bridge_result["status"],
                    "plugin_execution_status": plugin_status,
                    "control": control,
                    "diagnostics": deepcopy(
                        (plugin_result or {}).get("control", {}).get("diagnostics")
                    ),
                    "ego_pose_before_control": pose_before_control,
                    "ego_pose_after_control": pose_after_control,
                    "next_ego_pose": next_pose,
                }
            )
    finally:
        executor.close()

    source = inputs.scenario_ir.get("source") or {}
    scene_id = source.get("scene_name") or inputs.scenario_ir["scenario_id"]
    report = {
        "schema_version": REPORT_SCHEMA,
        "run_id": run_id,
        "scene_id": scene_id,
        "scenario_id": inputs.scenario_ir["scenario_id"],
        "scene_version": source.get("version"),
        "execution_status": "completed" if frame_mismatch_count == 0 else "failed",
        "evidence_classification": EVIDENCE_CLASSIFICATION,
        "real_carla_nurec_closed_loop": False,
        "real_ros2_transport": False,
        "remote_validation_required": True,
        "ego_pose_source": "scenario_ir_reference_trajectory",
        "control_affects_next_ego_pose": False,
        "claims_m8": False,
        "claims_m9": False,
        "transport": {
            "implementation": "memory_ros2_contract",
            "topics": plan["topics"],
            "same_tick_required": plan["message_contract"]["same_tick_required"],
            "fail_closed": plan["message_contract"]["fail_closed"],
        },
        "frame_sync": {
            "source_frame_count": len(ego_track),
            "observation_frame_count": len(frames),
            "matched_frame_count": len(frames) - frame_mismatch_count,
            "frame_mismatch_count": frame_mismatch_count,
            "scored_frame_mismatch_count": frame_mismatch_count,
            "fallback_count": fallback_count,
        },
        "plugin": plugin_spec,
        "plugin_identity": plugin_identity,
        "artifacts": {
            "scenario_ir": {
                "path": str(inputs.scenario_ir_path),
                "sha256": inputs.scenario_ir_sha256,
            },
            "opendrive": {
                "path": str(inputs.opendrive_path),
                "sha256": inputs.opendrive_sha256,
            },
        },
        "frames": frames,
        "pose_ownership": {
            "owner": "scenario_ir_reference_trajectory",
            "control_applied": False,
            "next_pose_assertions": len(frames) - 1,
            "max_pose_error_m": 0.0,
        },
    }
    validate_open_loop_ros_report(report)
    return report


def validate_open_loop_ros_report(report: Mapping[str, Any]) -> None:
    required = {
        "schema_version",
        "scene_id",
        "scenario_id",
        "execution_status",
        "evidence_classification",
        "real_ros2_transport",
        "ego_pose_source",
        "control_affects_next_ego_pose",
        "claims_m8",
        "claims_m9",
        "frame_sync",
        "pose_ownership",
    }
    missing = sorted(required - set(report))
    if missing:
        raise OpenLoopRosSmokeError(f"open-loop ROS report missing fields: {missing}")
    if report["schema_version"] != REPORT_SCHEMA:
        raise OpenLoopRosSmokeError("open-loop ROS report schema_version is invalid")
    if report["evidence_classification"] != EVIDENCE_CLASSIFICATION:
        raise OpenLoopRosSmokeError(
            "open-loop ROS report evidence_classification is invalid"
        )
    if report["ego_pose_source"] != "scenario_ir_reference_trajectory":
        raise OpenLoopRosSmokeError("open-loop ROS report must identify the IR pose source")
    for field in ("real_ros2_transport", "control_affects_next_ego_pose", "claims_m8", "claims_m9"):
        if report[field] is not False:
            raise OpenLoopRosSmokeError(f"open-loop ROS report {field} must be false")
    sync = report["frame_sync"]
    if not isinstance(sync, Mapping) or sync.get("scored_frame_mismatch_count") != 0:
        raise OpenLoopRosSmokeError(
            "open-loop ROS report must have zero scored frame mismatches"
        )
    ownership = report["pose_ownership"]
    if not isinstance(ownership, Mapping) or ownership.get("control_applied") is not False:
        raise OpenLoopRosSmokeError(
            "open-loop ROS report must prove control was not applied"
        )


def _emit_tick(
    *,
    node: _MemoryRosNode,
    plan: Mapping[str, Any],
    frame_id: int,
    t_sec: float,
    state: Mapping[str, Any],
    route: Mapping[str, Any],
    calibration: Mapping[str, Any],
) -> None:
    sensor_topic = plan["topics"]["sensors"]["rgb_front"]
    node.emit(
        sensor_topic,
        {
            "frame_id": frame_id,
            "t_sec": t_sec,
            "data": {"source": "open_loop_gt_rgb", "frame_id": frame_id},
            "calibration": dict(calibration),
        },
    )
    node.emit(
        plan["topics"]["ego_state"],
        {
            "frame_id": frame_id,
            "t_sec": t_sec,
            "data": {
                "speed_mps": float(state["speed_mps"]),
                "pose": {
                    axis: float(state[axis]) for axis in ("x", "y", "z", "yaw")
                },
                "velocity": {"x": float(state["speed_mps"]), "y": 0.0, "z": 0.0},
                "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
        },
    )
    node.emit(
        plan["topics"]["route"],
        {"frame_id": frame_id, "t_sec": t_sec, "data": deepcopy(dict(route))},
    )


def _write_report(path: Path, report: Mapping[str, Any]) -> None:
    if path.exists():
        raise OpenLoopRosSmokeError(f"refusing to overwrite existing output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-ir", type=Path, default=DEFAULT_SCENARIO_IR)
    parser.add_argument("--opendrive", type=Path, default=DEFAULT_OPENDRIVE)
    parser.add_argument("--expected-scenario-ir-sha256", default=EXPECTED_SCENARIO_IR_SHA256)
    parser.add_argument("--expected-opendrive-sha256", default=EXPECTED_OPENDRIVE_SHA256)
    parser.add_argument("--plugin", default="agents.reference_pure_pursuit:create_plugin")
    parser.add_argument("--plugin-config", type=Path)
    parser.add_argument("--run-id", default="scene-0061-open-loop-m4")
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        config = (
            strict_json_loads(args.plugin_config.read_text(encoding="utf-8"))
            if args.plugin_config
            else {}
        )
        if not isinstance(config, Mapping):
            raise OpenLoopRosSmokeError("plugin config must be a JSON object")
        report = run_open_loop_ros_smoke(
            scenario_ir_path=args.scenario_ir,
            opendrive_path=args.opendrive,
            expected_scenario_ir_sha256=args.expected_scenario_ir_sha256,
            expected_opendrive_sha256=args.expected_opendrive_sha256,
            plugin_spec=args.plugin,
            plugin_config=config,
            run_id=args.run_id,
            max_frames=args.max_frames,
        )
        _write_report(args.report, report)
        print(json.dumps({"status": "written", "report": str(args.report)}))
        return 0 if report["execution_status"] == "completed" else 2
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
