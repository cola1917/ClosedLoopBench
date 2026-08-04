"""Run or preflight the open-loop TransFuser++ Stage A evaluation.

Stage A consumes native CARLA RGB/LiDAR observations captured at Scenario IR
poses. This runner never advances ego pose from a control output. When the
immutable TF++ runtime or a native observation trace is absent, it writes a
blocked report instead of substituting a fake model or sensor source.
"""

from __future__ import annotations

import argparse
import json
import math
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

from agents.algorithm_backend import load_backend
from agents.plugin_contract import AlgorithmPluginExecutor, strict_json_loads
from agents.transfuserpp_contract import (
    build_runtime_manifest,
    validate_intermediate_record,
)
from metrics.open_loop import score_open_loop_predictions, validate_open_loop_report
from runners.run_open_loop_gt_replay import (
    DEFAULT_OPENDRIVE,
    DEFAULT_SCENARIO_IR,
    EXPECTED_OPENDRIVE_SHA256,
    EXPECTED_SCENARIO_IR_SHA256,
    _normalise_track,
    load_pinned_inputs,
    sha256_file,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_SCHEMA = "open_loop_multimodal_report.v1"
EVIDENCE_CLASSIFICATION = "open_loop_multimodal"
STAGE_NAME = "M5_tfpp_stage_a"


class OpenLoopTransFuserPPError(ValueError):
    """Raised when a Stage A report cannot be proven from native evidence."""


def run_open_loop_transfuserpp_stage_a(
    *,
    scenario_ir_path: str | Path,
    opendrive_path: str | Path,
    expected_scenario_ir_sha256: str = EXPECTED_SCENARIO_IR_SHA256,
    expected_opendrive_sha256: str = EXPECTED_OPENDRIVE_SHA256,
    runtime_config_path: str | Path | None = None,
    observation_trace_path: str | Path | None = None,
    run_id: str = "scene-0061-open-loop-m5",
    max_frames: int | None = None,
) -> dict[str, Any]:
    """Run real TF++ on a native Stage A trace, or return a blocked report."""

    if not isinstance(run_id, str) or not run_id.strip():
        raise OpenLoopTransFuserPPError("run_id must be a non-empty string")
    if max_frames is not None and (isinstance(max_frames, bool) or max_frames <= 0):
        raise OpenLoopTransFuserPPError("max_frames must be positive when provided")

    inputs = load_pinned_inputs(
        scenario_ir_path,
        opendrive_path,
        expected_scenario_ir_sha256=expected_scenario_ir_sha256,
        expected_opendrive_sha256=expected_opendrive_sha256,
    )
    blockers: list[str] = []
    config: dict[str, Any] | None = None
    runtime_manifest: dict[str, Any] | None = None
    config_path = Path(runtime_config_path) if runtime_config_path is not None else None
    trace_path = Path(observation_trace_path) if observation_trace_path is not None else None

    if config_path is None:
        blockers.append("runtime_config_missing")
    elif not config_path.is_file():
        blockers.append("runtime_config_unavailable")
    else:
        try:
            loaded_config = strict_json_loads(config_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            blockers.append(f"runtime_config_unreadable:{exc}")
        else:
            if not isinstance(loaded_config, dict):
                blockers.append("runtime_config_must_be_object")
            else:
                config = loaded_config
                blockers.extend(_open_loop_config_problems(config, inputs.scenario_ir_sha256, inputs.opendrive_sha256))
                runtime_manifest = build_runtime_manifest(config)
                if runtime_manifest["execution_status"] != "prepared":
                    blockers.extend(
                        f"runtime_manifest:{problem}"
                        for problem in runtime_manifest.get("problems", [])
                    )

    if trace_path is None:
        blockers.append("native_stage_a_observation_trace_missing")
    elif not trace_path.is_file():
        blockers.append("native_stage_a_observation_trace_unavailable")

    if blockers:
        return _blocked_report(
            inputs=inputs,
            run_id=run_id,
            blockers=sorted(set(blockers)),
            runtime_config_path=config_path,
            observation_trace_path=trace_path,
            runtime_manifest=runtime_manifest,
        )

    assert config is not None
    assert trace_path is not None
    observations = _load_observation_trace(trace_path)
    if max_frames is not None:
        observations = observations[:max_frames]
    if len(observations) < 2:
        raise OpenLoopTransFuserPPError("Stage A requires at least two observations")
    ego_track = _normalise_track(
        (inputs.scenario_ir.get("ego") or {}).get("reference_trajectory"),
        "ego.reference_trajectory",
        minimum=2,
    )
    _validate_observation_binding(observations, ego_track)

    config = deepcopy(config)
    config.setdefault("repo_path", str(PROJECT_ROOT))
    backend = load_backend("agents.transfuserpp_plugin:create_plugin", config)
    executor = AlgorithmPluginExecutor(
        backend,
        config,
        already_initialized=True,
        evidence_classification=EVIDENCE_CLASSIFICATION,
    )
    predictions: list[dict[str, Any]] = []
    frame_rows: list[dict[str, Any]] = []
    fallback_count = 0
    intermediate_count = 0
    try:
        plugin_identity = executor.initialize()
        executor.reset(
            {
                "mode": "open_loop_transfuserpp_stage_a",
                "scenario_id": inputs.scenario_ir["scenario_id"],
                "ego_pose_source": "scenario_ir_reference_trajectory",
                "sensor_source": "carla_stage_a_native_rgb_lidar",
            }
        )
        for observation in observations:
            frame_id = int(observation["frame_id"])
            result = executor.predict(observation)
            control = deepcopy(result["control"])
            record_ref = control.get("intermediate_record_ref")
            record = None
            if result["execution_status"] == "fallback":
                fallback_count += 1
            else:
                record = _load_intermediate_record(record_ref, frame_id)
                intermediate_count += 1
                state = ego_track[frame_id]
                predictions.append(
                    {
                        "frame_id": frame_id,
                        "observation_frame_id": frame_id,
                        "inference_ms": float(control.get("inference_ms", 0.0)),
                        "predicted_waypoints": _waypoints_to_world(
                            record["outputs"]["waypoints_ego_m"],
                            state,
                            horizon_spacing_sec=float(
                                config["open_loop"].get("waypoint_spacing_sec", 0.5)
                            ),
                        ),
                    }
                )
            frame_rows.append(
                {
                    "frame_id": frame_id,
                    "timestamp": float(observation["timestamp"]),
                    "observation_frame_id": frame_id,
                    "control_source_frame_id": control.get("source_frame_id"),
                    "execution_status": result["execution_status"],
                    "control": control,
                    "intermediate_record_ref": deepcopy(record_ref),
                    "intermediate_identity": deepcopy(record.get("identity")) if record else None,
                }
            )
    finally:
        executor.close()

    report = score_open_loop_predictions(
        inputs.scenario_ir,
        {"frames": predictions},
        scenario_ir_path=str(inputs.scenario_ir_path),
        scenario_ir_sha256=inputs.scenario_ir_sha256,
        opendrive_path=str(inputs.opendrive_path),
        opendrive_sha256=inputs.opendrive_sha256,
    )
    report.update(
        {
            "schema_version": REPORT_SCHEMA,
            "run_id": run_id,
            "stage": STAGE_NAME,
            "real_tfpp_checkpoint_loaded": True,
            "real_carla_stage_a_open_loop": True,
            "sensor_source": "carla_stage_a_native_rgb_lidar",
            "runtime_manifest": runtime_manifest,
            "plugin": "agents.transfuserpp_plugin:create_plugin",
            "plugin_identity": plugin_identity,
            "tfpp": {
                "intermediate_count": intermediate_count,
                "fallback_count": fallback_count,
                "required_intermediates": [
                    "perspective_semantic",
                    "bev_semantic",
                    "depth",
                    "bounding_boxes",
                    "waypoints",
                    "route_checkpoints",
                    "target_speed_distribution",
                    "target_speed_mps",
                    "vehicle_control",
                ],
            },
            "frames": frame_rows,
            "matrix_actor_ready_ir_bound": False,
        }
    )
    if fallback_count or intermediate_count != len(observations):
        report["execution_status"] = "failed"
    validate_open_loop_report(report)
    return report


def _open_loop_config_problems(
    config: Mapping[str, Any], scenario_ir_sha256: str, opendrive_sha256: str
) -> list[str]:
    problems: list[str] = []
    open_loop = config.get("open_loop")
    if not isinstance(open_loop, Mapping):
        return ["runtime_config.open_loop_missing"]
    expected = {
        "evidence_classification": EVIDENCE_CLASSIFICATION,
        "scenario_ir_sha256": scenario_ir_sha256,
        "opendrive_sha256": opendrive_sha256,
        "control_affects_next_ego_pose": False,
        "claims_m8": False,
        "claims_m9": False,
        "sensor_source": "carla_stage_a_native_rgb_lidar",
    }
    for name, value in expected.items():
        if open_loop.get(name) != value:
            problems.append(f"runtime_config.open_loop.{name}_mismatch")
    experiment = config.get("experiment")
    if not isinstance(experiment, Mapping) or experiment.get("scenario_ir_sha256") != scenario_ir_sha256:
        problems.append("runtime_config.experiment.scenario_ir_sha256_mismatch")
    spacing = open_loop.get("waypoint_spacing_sec", 0.5)
    if isinstance(spacing, bool) or not isinstance(spacing, (int, float)) or not math.isfinite(float(spacing)) or float(spacing) <= 0:
        problems.append("runtime_config.open_loop.waypoint_spacing_sec_invalid")
    return problems


def _load_observation_trace(path: Path) -> list[dict[str, Any]]:
    try:
        text = path.read_text(encoding="utf-8")
        if path.suffix.lower() == ".jsonl":
            values = [strict_json_loads(line) for line in text.splitlines() if line.strip()]
        else:
            loaded = strict_json_loads(text)
            values = loaded.get("frames", []) if isinstance(loaded, Mapping) else loaded
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise OpenLoopTransFuserPPError(f"cannot read Stage A observation trace: {exc}") from exc
    if not isinstance(values, list) or any(not isinstance(value, dict) for value in values):
        raise OpenLoopTransFuserPPError("Stage A observation trace must contain object frames")
    return [deepcopy(value) for value in values]


def _validate_observation_binding(
    observations: list[dict[str, Any]], ego_track: list[dict[str, float]]
) -> None:
    if len(observations) > len(ego_track):
        raise OpenLoopTransFuserPPError("Stage A observation trace exceeds IR frame count")
    for expected_frame_id, observation in enumerate(observations):
        if observation.get("frame_id") != expected_frame_id:
            raise OpenLoopTransFuserPPError(
                f"Stage A observation frame_id mismatch at index {expected_frame_id}"
            )
        if observation.get("source") != "carla_stage_a_native_rgb_lidar":
            raise OpenLoopTransFuserPPError(
                f"Stage A observation {expected_frame_id} has an unverified sensor source"
            )
        provenance = observation.get("provenance")
        if not isinstance(provenance, Mapping) or provenance.get("gt_pose_replay") is not True:
            raise OpenLoopTransFuserPPError(
                f"Stage A observation {expected_frame_id} does not prove GT pose replay"
            )
        expected = ego_track[expected_frame_id]
        timestamp = observation.get("timestamp", observation.get("t_sec"))
        if not _close(timestamp, expected["t_sec"]):
            raise OpenLoopTransFuserPPError(
                f"Stage A observation {expected_frame_id} timestamp is not bound to IR"
            )
        pose = ((observation.get("ego_state") or {}).get("pose") or {})
        for name in ("x", "y", "yaw"):
            if not _close(pose.get(name), expected[name]):
                raise OpenLoopTransFuserPPError(
                    f"Stage A observation {expected_frame_id} pose is not bound to IR"
                )


def _load_intermediate_record(reference: Any, frame_id: int) -> dict[str, Any]:
    if not isinstance(reference, Mapping):
        raise OpenLoopTransFuserPPError(
            f"TF++ frame {frame_id} did not return an intermediate record reference"
        )
    path = Path(str(reference.get("path") or ""))
    if not path.is_file():
        raise OpenLoopTransFuserPPError(f"TF++ intermediate is missing: {path}")
    declared_sha256 = str(reference.get("sha256") or "")
    actual_sha256 = sha256_file(path)
    if declared_sha256 != actual_sha256:
        raise OpenLoopTransFuserPPError(
            f"TF++ intermediate SHA-256 mismatch at frame {frame_id}"
        )
    try:
        record = strict_json_loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise OpenLoopTransFuserPPError(
            f"cannot read TF++ intermediate frame {frame_id}: {exc}"
        ) from exc
    if not isinstance(record, dict):
        raise OpenLoopTransFuserPPError(f"TF++ intermediate frame {frame_id} is not an object")
    validate_intermediate_record(record)
    if record.get("frame_id") != frame_id:
        raise OpenLoopTransFuserPPError(
            f"TF++ intermediate frame identity mismatch: expected {frame_id}"
        )
    return record


def _waypoints_to_world(
    points: Any,
    state: Mapping[str, Any],
    *,
    horizon_spacing_sec: float,
) -> list[dict[str, float]]:
    if not isinstance(points, list) or not points:
        raise OpenLoopTransFuserPPError("TF++ emitted no waypoints")
    yaw = math.radians(float(state["yaw"]))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    result = []
    for index, point in enumerate(points):
        if not isinstance(point, list) or len(point) != 2:
            raise OpenLoopTransFuserPPError("TF++ waypoint is not an x/y pair")
        forward = float(point[0])
        right = float(point[1])
        result.append(
            {
                "horizon_sec": (index + 1) * horizon_spacing_sec,
                "x": float(state["x"]) + cos_yaw * forward + sin_yaw * right,
                "y": float(state["y"]) + sin_yaw * forward - cos_yaw * right,
            }
        )
    return result


def _blocked_report(
    *,
    inputs: Any,
    run_id: str,
    blockers: list[str],
    runtime_config_path: Path | None,
    observation_trace_path: Path | None,
    runtime_manifest: Mapping[str, Any] | None,
) -> dict[str, Any]:
    source = inputs.scenario_ir.get("source") or {}
    report = {
        "schema_version": REPORT_SCHEMA,
        "run_id": run_id,
        "stage": STAGE_NAME,
        "scene_id": source.get("scene_name") or inputs.scenario_ir["scenario_id"],
        "scenario_id": inputs.scenario_ir["scenario_id"],
        "scene_version": source.get("version"),
        "execution_status": "blocked",
        "evidence_classification": EVIDENCE_CLASSIFICATION,
        "real_carla_nurec_closed_loop": False,
        "real_carla_stage_a_open_loop": False,
        "real_tfpp_checkpoint_loaded": False,
        "remote_validation_required": True,
        "ego_pose_source": "scenario_ir_reference_trajectory",
        "control_affects_next_ego_pose": False,
        "claims_m8": False,
        "claims_m9": False,
        "matrix_actor_ready_ir_bound": False,
        "sensor_source": "carla_stage_a_native_rgb_lidar",
        "blockers": blockers,
        "runtime_manifest": deepcopy(dict(runtime_manifest or {})),
        "runtime_config_path": str(runtime_config_path) if runtime_config_path else None,
        "observation_trace_path": str(observation_trace_path) if observation_trace_path else None,
        "frame_sync": {
            "source_frame_count": len(
                (inputs.scenario_ir.get("ego") or {}).get("reference_trajectory") or []
            ),
            "observation_frame_count": 0,
            "matched_frame_count": 0,
            "frame_mismatch_count": 0,
            "scored_frame_mismatch_count": 0,
            "fallback_count": 0,
        },
        "metrics": {},
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
        "frames": [],
        "tfpp": {"intermediate_count": 0, "fallback_count": 0},
    }
    validate_open_loop_report(report)
    return report


def _close(left: Any, right: Any, tolerance: float = 1e-5) -> bool:
    if isinstance(left, bool) or not isinstance(left, (int, float)):
        return False
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=tolerance)


def _write_report(path: Path, report: Mapping[str, Any]) -> None:
    if path.exists():
        raise OpenLoopTransFuserPPError(f"refusing to overwrite existing output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-ir", type=Path, default=DEFAULT_SCENARIO_IR)
    parser.add_argument("--opendrive", type=Path, default=DEFAULT_OPENDRIVE)
    parser.add_argument("--runtime-config", type=Path)
    parser.add_argument("--observations", type=Path)
    parser.add_argument("--expected-scenario-ir-sha256", default=EXPECTED_SCENARIO_IR_SHA256)
    parser.add_argument("--expected-opendrive-sha256", default=EXPECTED_OPENDRIVE_SHA256)
    parser.add_argument("--run-id", default="scene-0061-open-loop-m5")
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        report = run_open_loop_transfuserpp_stage_a(
            scenario_ir_path=args.scenario_ir,
            opendrive_path=args.opendrive,
            expected_scenario_ir_sha256=args.expected_scenario_ir_sha256,
            expected_opendrive_sha256=args.expected_opendrive_sha256,
            runtime_config_path=args.runtime_config,
            observation_trace_path=args.observations,
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
