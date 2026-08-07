"""Run the deterministic open-loop GT replay skeleton.

This runner owns the ego pose from the Scenario IR reference trajectory. An
algorithm may produce a control command for plumbing and logging, but the
command is never applied to the replay host and cannot determine the next
ego pose. CARLA sensor materialization and ROS transport are added in later
open-loop milestones.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from agents.algorithm_backend import load_backend
from agents.plugin_contract import (
    AlgorithmPluginExecutor,
    PluginContractError,
    strict_json_loads,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENARIO_IR = Path("outputs/scene0061_exchange_v2/scene_ir.json")
DEFAULT_OPENDRIVE = Path("outputs/scene-0061/road.xodr")
EXPECTED_SCENARIO_IR_SHA256 = "754a48f2a8eff3878229d3c6f80d0912bdd00016c86c77d5d295fc8f51e418d0"
EXPECTED_OPENDRIVE_SHA256 = "46e759ff00aff53b489b175822c33c7e03dc1f78d93c287b538d9b70801273a4"
EVIDENCE_CLASSIFICATION = "open_loop_multimodal"


class OpenLoopReplayError(ValueError):
    """Raised when an open-loop input or pose-ownership invariant fails."""


@dataclass(frozen=True)
class PinnedInputs:
    scenario_ir_path: Path
    scenario_ir_sha256: str
    opendrive_path: Path
    opendrive_sha256: str
    scenario_ir: dict[str, Any]


class GroundTruthReplayHost:
    """Minimal host abstraction used until the CARLA adapter lands in M2."""

    def __init__(self) -> None:
        self.current_ego_pose: dict[str, float] | None = None
        self.pose_history: list[dict[str, Any]] = []
        self.control_application_count = 0

    def set_ego_pose(self, pose: Mapping[str, Any], *, source: str) -> None:
        self.current_ego_pose = _pose(pose)
        self.pose_history.append(
            {"pose": deepcopy(self.current_ego_pose), "source": str(source)}
        )

    def apply_control(self, control: Mapping[str, Any]) -> None:
        """Record an illegal operation so tests can prove it is never called."""

        self.control_application_count += 1
        raise OpenLoopReplayError(
            "open-loop replay must not apply control to determine ego pose"
        )


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_pinned_inputs(
    scenario_ir_path: str | Path,
    opendrive_path: str | Path,
    *,
    expected_scenario_ir_sha256: str = EXPECTED_SCENARIO_IR_SHA256,
    expected_opendrive_sha256: str = EXPECTED_OPENDRIVE_SHA256,
) -> PinnedInputs:
    """Load and fail closed on the exact IR/XODR identity used by the plan."""

    ir_path = Path(scenario_ir_path)
    xodr_path = Path(opendrive_path)
    for path, label in ((ir_path, "Scenario IR"), (xodr_path, "OpenDRIVE")):
        if not path.is_file():
            raise OpenLoopReplayError(f"{label} pin is missing: {path}")

    actual_ir_sha256 = sha256_file(ir_path)
    if actual_ir_sha256 != expected_scenario_ir_sha256:
        raise OpenLoopReplayError(
            "Scenario IR SHA-256 mismatch: "
            f"expected={expected_scenario_ir_sha256} actual={actual_ir_sha256}"
        )
    actual_xodr_sha256 = sha256_file(xodr_path)
    if actual_xodr_sha256 != expected_opendrive_sha256:
        raise OpenLoopReplayError(
            "OpenDRIVE SHA-256 mismatch: "
            f"expected={expected_opendrive_sha256} actual={actual_xodr_sha256}"
        )

    try:
        scenario_ir = strict_json_loads(ir_path.read_text(encoding="utf-8"))
        xodr_root = ET.parse(xodr_path).getroot()
    except (OSError, ValueError, json.JSONDecodeError, ET.ParseError) as exc:
        raise OpenLoopReplayError(f"pinned artifact is not readable: {exc}") from exc
    if not isinstance(scenario_ir, dict):
        raise OpenLoopReplayError("Scenario IR must be a JSON object")
    if xodr_root.tag != "OpenDRIVE":
        raise OpenLoopReplayError(
            f"OpenDRIVE root must be OpenDRIVE, got {xodr_root.tag!r}"
        )
    _validate_scenario_ir(scenario_ir)
    return PinnedInputs(
        scenario_ir_path=ir_path.resolve(),
        scenario_ir_sha256=actual_ir_sha256,
        opendrive_path=xodr_path.resolve(),
        opendrive_sha256=actual_xodr_sha256,
        scenario_ir=scenario_ir,
    )


def run_open_loop_gt_replay(
    *,
    scenario_ir_path: str | Path,
    opendrive_path: str | Path,
    expected_scenario_ir_sha256: str = EXPECTED_SCENARIO_IR_SHA256,
    expected_opendrive_sha256: str = EXPECTED_OPENDRIVE_SHA256,
    plugin_spec: str = "agents.reference_pure_pursuit:create_plugin",
    plugin_config: Mapping[str, Any] | None = None,
    run_id: str = "scene-0061-open-loop-m1",
    max_frames: int | None = None,
) -> dict[str, Any]:
    """Replay IR poses through a plugin without giving the plugin pose authority."""

    if not isinstance(run_id, str) or not run_id.strip():
        raise OpenLoopReplayError("run_id must be a non-empty string")
    if max_frames is not None and (isinstance(max_frames, bool) or max_frames <= 0):
        raise OpenLoopReplayError("max_frames must be positive when provided")

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
        raise OpenLoopReplayError("open-loop replay requires at least two ego frames")

    actors = [
        actor
        for actor in inputs.scenario_ir.get("actors", [])
        if isinstance(actor, Mapping)
    ]
    actor_tracks = {
        str(actor.get("actor_id", "actor")): _normalise_track(
            actor.get("reference_trajectory"),
            f"actor[{actor.get('actor_id', 'actor')}].reference_trajectory",
            minimum=0,
        )
        for actor in actors
    }
    route = _route_from_ego_track(ego_track)
    config = dict(plugin_config or {})
    config.setdefault("repo_path", str(PROJECT_ROOT))
    backend = load_backend(plugin_spec, config)
    executor = AlgorithmPluginExecutor(
        backend,
        config,
        already_initialized=True,
        evidence_classification=EVIDENCE_CLASSIFICATION,
    )
    host = GroundTruthReplayHost()
    frames: list[dict[str, Any]] = []
    started = ego_track[0]["t_sec"]
    fallback_count = 0
    try:
        plugin_identity = executor.initialize()
        executor.reset(
            {
                "mode": "open_loop_gt_replay",
                "scenario_id": inputs.scenario_ir["scenario_id"],
                "ego_pose_source": "scenario_ir_reference_trajectory",
            }
        )
        for frame_id, state in enumerate(ego_track):
            current_pose = _pose(state)
            host.set_ego_pose(
                current_pose,
                source="scenario_ir_reference_trajectory",
            )
            pose_before_control = deepcopy(host.current_ego_pose)
            actor_states = _actor_states_at_time(actor_tracks, state["t_sec"])
            observation = _build_observation(
                frame_id=frame_id,
                state=state,
                route=route,
                actor_states=actor_states,
            )
            result = executor.predict(observation)
            pose_after_control = deepcopy(host.current_ego_pose)
            if not _poses_equal(pose_before_control, pose_after_control):
                raise OpenLoopReplayError(
                    f"control changed ego pose at frame {frame_id}; GT replay invariant failed"
                )
            if result["execution_status"] == "fallback":
                fallback_count += 1

            next_expected = (
                _pose(ego_track[frame_id + 1])
                if frame_id + 1 < len(ego_track)
                else None
            )
            next_actual = None
            if next_expected is not None:
                host.set_ego_pose(
                    next_expected,
                    source="scenario_ir_reference_trajectory",
                )
                next_actual = deepcopy(host.current_ego_pose)
                if not _poses_equal(next_expected, next_actual):
                    raise OpenLoopReplayError(
                        f"next ego pose mismatch at frame {frame_id}: "
                        "Scenario IR does not own the replay pose"
                    )
            frames.append(
                {
                    "frame_id": frame_id,
                    "t_sec": state["t_sec"],
                    "ego_pose_before_control": pose_before_control,
                    "ego_pose_after_control": pose_after_control,
                    "next_ego_pose_expected": next_expected,
                    "next_ego_pose_actual": next_actual,
                    "next_ego_pose_source": (
                        "scenario_ir_reference_trajectory"
                        if next_actual is not None
                        else None
                    ),
                    "actor_count": len(actor_states),
                    "actor_states": actor_states,
                    "execution_status": result["execution_status"],
                    "control": result["control"],
                    "detail": result.get("detail", {}),
                }
            )
    finally:
        executor.close()

    source = inputs.scenario_ir.get("source") or {}
    scene_id = source.get("scene_name") or inputs.scenario_ir["scenario_id"]
    return {
        "schema_version": "open_loop_gt_replay_report.v1",
        "run_id": run_id,
        "scene_id": scene_id,
        "scenario_id": inputs.scenario_ir["scenario_id"],
        "scene_name": source.get("scene_name"),
        "scene_version": source.get("version"),
        "execution_status": "completed" if fallback_count == 0 else "completed_with_fallback",
        "evidence_classification": EVIDENCE_CLASSIFICATION,
        "real_carla_nurec_closed_loop": False,
        "remote_validation_required": True,
        "ego_pose_source": "scenario_ir_reference_trajectory",
        "control_affects_next_ego_pose": False,
        "claims_m8": False,
        "claims_m9": False,
        "matrix_actor_ready_ir_bound": False,
        "control_application": "not_applied",
        "frame_count": len(frames),
        "control_count": len(frames) - fallback_count,
        "fallback_count": fallback_count,
        "actor_count": len(actors),
        "actor_pose_source": "scenario_ir_reference_trajectory",
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
        "scenario_ir_path": str(inputs.scenario_ir_path),
        "scenario_ir_sha256": inputs.scenario_ir_sha256,
        "opendrive_path": str(inputs.opendrive_path),
        "opendrive_sha256": inputs.opendrive_sha256,
        "frames": frames,
        "pose_ownership": {
            "owner": "scenario_ir_reference_trajectory",
            "control_applied": False,
            "next_pose_assertions": len(frames) - 1,
            "max_pose_error_m": 0.0,
        },
        "started_t_sec": started,
        "finished_t_sec": ego_track[-1]["t_sec"],
    }


def _build_observation(
    *,
    frame_id: int,
    state: Mapping[str, Any],
    route: dict[str, Any],
    actor_states: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "frame_id": frame_id,
        "timestamp": float(state["t_sec"]),
        "source": "scenario_ir_reference_trajectory",
        "rgb": {},
        "lidar": None,
        "sensor_validity": {},
        "ego_state": {
            "speed_mps": float(state["speed_mps"]),
            "pose": _pose(state),
        },
        "actors": actor_states,
        "route": deepcopy(route),
        "synchronization": {
            "frame_id": frame_id,
            "t_sec": float(state["t_sec"]),
            "source": "scenario_ir_reference_trajectory",
        },
    }


def _validate_scenario_ir(scenario_ir: Mapping[str, Any]) -> None:
    if scenario_ir.get("schema_version") != "scenario_ir.v1":
        raise OpenLoopReplayError("Scenario IR must use schema_version scenario_ir.v1")
    scenario_id = scenario_ir.get("scenario_id")
    source = scenario_ir.get("source")
    if not isinstance(scenario_id, str) or not scenario_id:
        raise OpenLoopReplayError("Scenario IR scenario_id must be non-empty")
    if not isinstance(source, Mapping) or source.get("scene_token") != scenario_id:
        raise OpenLoopReplayError("Scenario IR source.scene_token must match scenario_id")
    _normalise_track(
        (scenario_ir.get("ego") or {}).get("reference_trajectory"),
        "ego.reference_trajectory",
        minimum=2,
    )
    actors = scenario_ir.get("actors")
    if not isinstance(actors, list):
        raise OpenLoopReplayError("Scenario IR actors must be a list")
    actor_ids: set[str] = set()
    for index, actor in enumerate(actors):
        if not isinstance(actor, Mapping):
            raise OpenLoopReplayError(f"actors[{index}] must be an object")
        actor_id = str(actor.get("actor_id", ""))
        if not actor_id or actor_id in actor_ids:
            raise OpenLoopReplayError(f"actors[{index}] has a duplicate or empty actor_id")
        actor_ids.add(actor_id)
        _normalise_track(
            actor.get("reference_trajectory"),
            f"actor[{actor_id}].reference_trajectory",
            minimum=0,
        )


def _normalise_track(value: Any, label: str, *, minimum: int) -> list[dict[str, float]]:
    if not isinstance(value, list) or len(value) < minimum:
        raise OpenLoopReplayError(f"{label} must contain at least {minimum} states")
    result = []
    previous_t = -math.inf
    for index, raw in enumerate(value):
        if not isinstance(raw, Mapping):
            raise OpenLoopReplayError(f"{label}[{index}] must be an object")
        state = {
            "t_sec": _finite(raw.get("t_sec"), f"{label}[{index}].t_sec"),
            "x": _finite(raw.get("x"), f"{label}[{index}].x"),
            "y": _finite(raw.get("y"), f"{label}[{index}].y"),
            "z": _finite(raw.get("z", 0.0), f"{label}[{index}].z"),
            "yaw": _finite(raw.get("yaw", 0.0), f"{label}[{index}].yaw"),
            "speed_mps": _finite(
                raw.get("speed_mps", 0.0), f"{label}[{index}].speed_mps"
            ),
        }
        if state["t_sec"] < previous_t:
            raise OpenLoopReplayError(f"{label} timestamps must be monotonic")
        previous_t = state["t_sec"]
        result.append(state)
    return result


def _actor_states_at_time(
    actor_tracks: Mapping[str, list[dict[str, float]]], t_sec: float
) -> list[dict[str, Any]]:
    states = []
    for actor_id, track in actor_tracks.items():
        if not track:
            continue
        states.append({"actor_id": actor_id, **_state_at_time(track, t_sec)})
    return states


def _state_at_time(track: list[dict[str, float]], t_sec: float) -> dict[str, float]:
    if t_sec <= track[0]["t_sec"]:
        return deepcopy(track[0])
    if t_sec >= track[-1]["t_sec"]:
        return deepcopy(track[-1])
    for left, right in zip(track, track[1:]):
        if left["t_sec"] <= t_sec <= right["t_sec"]:
            duration = right["t_sec"] - left["t_sec"]
            if duration <= 0.0:
                return deepcopy(right)
            ratio = (t_sec - left["t_sec"]) / duration
            result = {
                key: left[key] + ratio * (right[key] - left[key])
                for key in ("t_sec", "x", "y", "z", "speed_mps")
            }
            result["t_sec"] = float(t_sec)
            result["yaw"] = _interpolate_angle(left["yaw"], right["yaw"], ratio)
            return result
    return deepcopy(track[-1])


def _route_from_ego_track(track: list[dict[str, float]]) -> dict[str, Any]:
    waypoints = [[state["x"], state["y"]] for state in track]
    return {
        "route_waypoints": waypoints,
        "route_command": "LANE_FOLLOW",
        "target_point": deepcopy(waypoints[-1]),
        "source": "scenario_ir_reference_trajectory",
    }


def _pose(state: Mapping[str, Any]) -> dict[str, float]:
    return {
        axis: float(state[axis])
        for axis in ("x", "y", "z", "yaw")
    }


def _poses_equal(left: Mapping[str, Any] | None, right: Mapping[str, Any] | None) -> bool:
    if left is None or right is None:
        return left is right
    return all(math.isclose(float(left[axis]), float(right[axis]), abs_tol=1e-9) for axis in ("x", "y", "z", "yaw"))


def _interpolate_angle(left: float, right: float, ratio: float) -> float:
    delta = (right - left + 180.0) % 360.0 - 180.0
    return left + ratio * delta


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise OpenLoopReplayError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise OpenLoopReplayError(f"{label} must be a finite number")
    return result


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    if path.exists():
        raise OpenLoopReplayError(f"refusing to overwrite existing output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _write_trace(path: Path, report: Mapping[str, Any]) -> None:
    if path.exists():
        raise OpenLoopReplayError(f"refusing to overwrite existing output: {path}")
    identity = {
        "schema_version": "open_loop_gt_replay_frame.v1",
        "run_id": report["run_id"],
        "scene_id": report["scene_id"],
        "scenario_id": report["scenario_id"],
        "evidence_classification": report["evidence_classification"],
        "ego_pose_source": report["ego_pose_source"],
        "control_affects_next_ego_pose": report["control_affects_next_ego_pose"],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(
            json.dumps({**identity, **frame}, ensure_ascii=False, sort_keys=True) + "\n"
            for frame in report["frames"]
        ),
        encoding="utf-8",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run open-loop GT replay with Scenario IR pose ownership."
    )
    parser.add_argument("--scenario-ir", type=Path, default=DEFAULT_SCENARIO_IR)
    parser.add_argument("--opendrive", type=Path, default=DEFAULT_OPENDRIVE)
    parser.add_argument("--expected-scenario-ir-sha256", default=EXPECTED_SCENARIO_IR_SHA256)
    parser.add_argument("--expected-opendrive-sha256", default=EXPECTED_OPENDRIVE_SHA256)
    parser.add_argument(
        "--plugin",
        default="agents.reference_pure_pursuit:create_plugin",
        help="Algorithm plugin module:factory; M1 defaults to the deterministic local baseline.",
    )
    parser.add_argument("--plugin-config", type=Path)
    parser.add_argument("--run-id", default="scene-0061-open-loop-m1")
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--trace", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.trace is not None and args.trace.exists():
            raise OpenLoopReplayError(
                f"refusing to overwrite existing output: {args.trace}"
            )
        config = (
            strict_json_loads(args.plugin_config.read_text(encoding="utf-8"))
            if args.plugin_config
            else {}
        )
        if not isinstance(config, Mapping):
            raise OpenLoopReplayError("plugin config must be a JSON object")
        report = run_open_loop_gt_replay(
            scenario_ir_path=args.scenario_ir,
            opendrive_path=args.opendrive,
            expected_scenario_ir_sha256=args.expected_scenario_ir_sha256,
            expected_opendrive_sha256=args.expected_opendrive_sha256,
            plugin_spec=args.plugin,
            plugin_config=config,
            run_id=args.run_id,
            max_frames=args.max_frames,
        )
        _write_json(args.report, report)
        if args.trace:
            _write_trace(args.trace, report)
        result = {"status": "written", "report": str(args.report)}
        if args.trace:
            result["trace"] = str(args.trace)
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except (OSError, ValueError, json.JSONDecodeError, PluginContractError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
