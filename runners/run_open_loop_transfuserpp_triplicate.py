"""Run TF++ on one of the frozen three-way open-loop sensor traces."""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agents.algorithm_backend import load_backend
from agents.plugin_contract import AlgorithmPluginExecutor, strict_json_loads
from agents.transfuserpp_contract import (
    build_runtime_manifest,
    validate_intermediate_record,
    validate_observation,
)
from adapters.open_loop_bbox_binding import frame_binding, load_actor_manifest
from metrics.open_loop import (
    OPEN_LOOP_REPORT_SCHEMA,
    score_open_loop_predictions,
    validate_open_loop_report,
)
from runners.run_open_loop_gt_replay import (
    DEFAULT_OPENDRIVE,
    DEFAULT_SCENARIO_IR,
    EXPECTED_OPENDRIVE_SHA256,
    EXPECTED_SCENARIO_IR_SHA256,
    _normalise_track,
    load_pinned_inputs,
    sha256_file,
)


REPORT_SCHEMA = OPEN_LOOP_REPORT_SCHEMA
EVIDENCE_CLASSIFICATION = "open_loop_multimodal"
STAGE_NAME = "M8_tfpp_triplicate_route"
ROUTE_METADATA = {
    "carla_stage_a_native_rgb_lidar": {
        "route_id": "raw_original",
        "rgb_source": "carla_stage_a_native",
        "lidar_source": "carla_stage_a_native",
        "harmonizer_rgb_only": False,
    },
    "reconstructed_rgb_lidar": {
        "route_id": "reconstructed",
        "rgb_source": "neural_scene_bridge_reconstructed",
        "lidar_source": "neural_scene_bridge_reconstructed",
        "harmonizer_rgb_only": False,
    },
    "harmonized_rgb_reconstructed_lidar": {
        "route_id": "harmonized",
        "rgb_source": "nvidia_harmonizer",
        "lidar_source": "neural_scene_bridge_reconstructed",
        "harmonizer_rgb_only": True,
    },
}
REQUIRED_INTERMEDIATES = [
    "perspective_semantic",
    "bev_semantic",
    "depth",
    "bounding_boxes",
    "waypoints",
    "route_checkpoints",
    "target_speed_distribution",
    "target_speed_mps",
    "vehicle_control",
]


class TriplicateRunnerError(ValueError):
    """Raised when a route trace cannot be run fail-closed."""


def _load_trace(path: Path) -> dict[str, Any]:
    value = strict_json_loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or not isinstance(value.get("frames"), list):
        raise TriplicateRunnerError("triplicate trace must be an object with frames")
    return value


def _load_record(reference: Any, frame_id: int) -> dict[str, Any]:
    if not isinstance(reference, Mapping):
        raise TriplicateRunnerError(f"TF++ frame {frame_id} has no intermediate record reference")
    path = Path(str(reference.get("path") or ""))
    if not path.is_file():
        raise TriplicateRunnerError(f"TF++ intermediate is missing: {path}")
    if reference.get("sha256") != sha256_file(path):
        raise TriplicateRunnerError(f"TF++ intermediate SHA-256 mismatch at frame {frame_id}")
    value = strict_json_loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise TriplicateRunnerError(f"TF++ intermediate frame {frame_id} is not an object")
    validate_intermediate_record(value)
    if value.get("frame_id") != frame_id:
        raise TriplicateRunnerError(f"TF++ intermediate frame identity mismatch at {frame_id}")
    return value


def _waypoints_to_world(
    points: Any,
    state: Mapping[str, Any],
    *,
    horizon_spacing_sec: float,
) -> list[dict[str, float]]:
    if not isinstance(points, list) or not points:
        raise TriplicateRunnerError("TF++ emitted no waypoints")
    yaw = math.radians(float(state["yaw"]))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    result = []
    for index, point in enumerate(points):
        if not isinstance(point, list) or len(point) != 2:
            raise TriplicateRunnerError("TF++ waypoint is not an x/y pair")
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


def _validate_trace_binding(
    trace: Mapping[str, Any],
    observations: list[Mapping[str, Any]],
    ego_track: list[dict[str, float]],
    *,
    expected_source: str,
    actor_manifest: Mapping[str, Any],
) -> None:
    if len(observations) != len(ego_track):
        raise TriplicateRunnerError(
            f"trace frame count mismatch: expected {len(ego_track)}, got {len(observations)}"
        )
    trace_source = str(trace.get("source") or "")
    if trace_source != expected_source:
        raise TriplicateRunnerError(
            f"trace source mismatch: expected {expected_source}, got {trace_source}"
        )
    route = ROUTE_METADATA.get(expected_source)
    if route is None:
        raise TriplicateRunnerError(f"unsupported triplicate input source: {expected_source}")
    trace_binding = trace.get("input_binding")
    trace_actor_manifest = trace.get("actor_manifest")
    if not isinstance(trace_actor_manifest, Mapping):
        raise TriplicateRunnerError("trace actor_manifest reference is missing")
    if trace_actor_manifest.get("sha256") != actor_manifest.get("manifest_sha256"):
        raise TriplicateRunnerError("trace actor manifest SHA-256 does not match the shared manifest")
    if expected_source != "carla_stage_a_native_rgb_lidar":
        if not isinstance(trace_binding, Mapping):
            raise TriplicateRunnerError("reconstructed trace input_binding is missing")
        if trace_binding.get("source") != expected_source:
            raise TriplicateRunnerError("trace input_binding source is not frame-bound")
        if trace_binding.get("variant") not in {"reconstructed", "harmonized"}:
            raise TriplicateRunnerError("trace input_binding variant is invalid")
        if trace_binding.get("harmonizer_rgb_only") is not route["harmonizer_rgb_only"]:
            raise TriplicateRunnerError("trace Harmonizer modality declaration is invalid")
        expected_lidar_source = (
            "reconstructed_original"
            if route["harmonizer_rgb_only"]
            else "reconstructed_rgb_lidar"
        )
        if trace_binding.get("lidar_source") != expected_lidar_source:
            raise TriplicateRunnerError(
                "trace LiDAR source is not the required reconstructed LiDAR source"
            )
        source_bindings = trace_binding.get("source_frame_bindings")
        if not isinstance(source_bindings, list) or len(source_bindings) < len(observations):
            raise TriplicateRunnerError("trace source frame bindings are incomplete")
    for frame_id, observation in enumerate(observations):
        if observation.get("frame_id") != frame_id:
            raise TriplicateRunnerError(f"trace frame_id mismatch at {frame_id}")
        if observation.get("source") != expected_source:
            raise TriplicateRunnerError(f"trace frame source mismatch at {frame_id}")
        validate_observation(observation)
        actor_frame = frame_binding(actor_manifest, frame_id)
        state = ego_track[frame_id]
        if abs(float(observation["timestamp"]) - state["t_sec"]) > 0.02:
            raise TriplicateRunnerError(f"trace timestamp is not IR-bound at {frame_id}")
        pose = (observation.get("ego_state") or {}).get("pose") or {}
        if math.hypot(float(pose.get("x", 0.0)) - state["x"], float(pose.get("y", 0.0)) - state["y"]) > 0.05:
            raise TriplicateRunnerError(f"trace ego pose is not IR-bound at {frame_id}")
        provenance = observation.get("provenance") or {}
        if provenance.get("gt_pose_replay") is not True:
            raise TriplicateRunnerError(f"trace lacks GT pose provenance at {frame_id}")
        if provenance.get("input_source") != expected_source:
            raise TriplicateRunnerError(f"trace input provenance mismatch at {frame_id}")
        actor_provenance = provenance.get("actor_manifest")
        if not isinstance(actor_provenance, Mapping):
            raise TriplicateRunnerError(f"actor manifest provenance is missing at {frame_id}")
        for field in ("actor_manifest_sha256", "actor_manifest_file_sha256"):
            expected = (
                actor_manifest.get("manifest_sha256")
                if field == "actor_manifest_sha256"
                else actor_manifest.get("manifest_file_sha256")
            )
            if actor_provenance.get(field) != expected:
                raise TriplicateRunnerError(f"actor manifest {field} mismatch at {frame_id}")
        if actor_provenance.get("frame_id") != frame_id:
            raise TriplicateRunnerError(f"actor manifest frame binding mismatch at {frame_id}")
        for field in (
            "active_actor_ids",
            "active_actor_set_sha256",
            "pose_digest",
            "manifest_dynamic_object_sha256",
        ):
            if actor_provenance.get(field) != (
                actor_frame["active_actor_ids"]
                if field == "active_actor_ids"
                else actor_frame[
                    "dynamic_object_sha256"
                    if field == "manifest_dynamic_object_sha256"
                    else field
                ]
            ):
                raise TriplicateRunnerError(
                    f"actor manifest {field} mismatch at frame {frame_id}"
                )
        if (observation.get("synchronization") or {}).get("dynamic_object_sha256") != actor_frame[
            "dynamic_object_sha256"
        ]:
            raise TriplicateRunnerError(
                f"shared dynamic-object digest mismatch at frame {frame_id}"
            )
        rgb_ref = (observation.get("rgb") or {}).get("camera_front")
        lidar_ref = observation.get("lidar")
        if not isinstance(rgb_ref, Mapping) or not isinstance(lidar_ref, Mapping):
            raise TriplicateRunnerError(f"trace sensor payloads are incomplete at {frame_id}")
        rgb_path = Path(str(rgb_ref.get("path") or ""))
        lidar_path = Path(str(lidar_ref.get("path") or ""))
        expected_parent = f"frame_{frame_id:08d}"
        if (
            rgb_path.parent != lidar_path.parent
            or rgb_path.parent.name != expected_parent
            or lidar_path.parent.name != expected_parent
        ):
            raise TriplicateRunnerError(
                f"RGB/LiDAR are not same-frame payloads at frame {frame_id}"
            )
        if expected_source == "carla_stage_a_native_rgb_lidar":
            if provenance.get("input_variant") != "raw_original_rgb_lidar":
                raise TriplicateRunnerError(f"raw input variant is invalid at {frame_id}")
            continue

        binding = provenance.get("source_frame_binding")
        if not isinstance(binding, Mapping) or binding.get("ir_frame_id") != frame_id:
            raise TriplicateRunnerError(f"source frame binding is invalid at {frame_id}")
        rgb_materialization = rgb_ref.get("materialization")
        lidar_materialization = lidar_ref.get("materialization")
        if not isinstance(rgb_materialization, Mapping) or not isinstance(
            lidar_materialization, Mapping
        ):
            raise TriplicateRunnerError(
                f"reconstructed RGB/LiDAR materialization is missing at {frame_id}"
            )
        if binding.get("rgb_materialized_sha256") != rgb_ref.get("sha256"):
            raise TriplicateRunnerError(f"RGB materialization is not frame-bound at {frame_id}")
        if binding.get("lidar_materialized_sha256") != lidar_ref.get("sha256"):
            raise TriplicateRunnerError(f"LiDAR materialization is not frame-bound at {frame_id}")
        if lidar_materialization.get("source_sha256") != binding.get("lidar_source_sha256"):
            raise TriplicateRunnerError(f"LiDAR source hash binding is invalid at {frame_id}")
        if expected_source == "harmonized_rgb_reconstructed_lidar":
            if binding.get("rgb_mode") != "harmonized":
                raise TriplicateRunnerError(f"Harmonizer RGB binding is invalid at {frame_id}")
            if binding.get("lidar_mode") != "reconstructed_original":
                raise TriplicateRunnerError(
                    f"Harmonizer route does not use reconstructed LiDAR at {frame_id}"
                )
            if not binding.get("lidar_source_path"):
                raise TriplicateRunnerError(f"Harmonizer LiDAR source path is missing at {frame_id}")


def run_route(
    *,
    scenario_ir_path: Path,
    opendrive_path: Path,
    runtime_config_path: Path,
    trace_path: Path,
    actor_manifest_path: Path,
    expected_source: str,
    run_id: str,
    expected_scenario_ir_sha256: str = EXPECTED_SCENARIO_IR_SHA256,
    expected_opendrive_sha256: str = EXPECTED_OPENDRIVE_SHA256,
    max_frames: int | None = None,
) -> dict[str, Any]:
    inputs = load_pinned_inputs(
        scenario_ir_path,
        opendrive_path,
        expected_scenario_ir_sha256=expected_scenario_ir_sha256,
        expected_opendrive_sha256=expected_opendrive_sha256,
    )
    config = strict_json_loads(runtime_config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise TriplicateRunnerError("runtime config must be an object")
    manifest = build_runtime_manifest(config)
    if manifest.get("execution_status") != "prepared":
        raise TriplicateRunnerError(
            "TransFuser++ runtime is not prepared: "
            + ", ".join(str(item) for item in manifest.get("problems") or [])
        )
    experiment = config.get("experiment") or {}
    if experiment.get("scenario_ir_sha256") != inputs.scenario_ir_sha256:
        raise TriplicateRunnerError("runtime config and Scenario IR hashes differ")
    trace = _load_trace(trace_path)
    actor_manifest = load_actor_manifest(
        actor_manifest_path,
        expected_scenario_ir_sha256=inputs.scenario_ir_sha256,
        expected_scene_id=str(inputs.scenario_ir["scenario_id"]),
    )
    observations = [item for item in trace["frames"] if isinstance(item, dict)]
    if len(observations) != len(trace["frames"]):
        raise TriplicateRunnerError("triplicate trace contains a non-object frame")
    if max_frames is not None:
        if isinstance(max_frames, bool) or max_frames <= 0:
            raise TriplicateRunnerError("max_frames must be positive")
        observations = observations[:max_frames]
    ego_track = _normalise_track(
        (inputs.scenario_ir.get("ego") or {}).get("reference_trajectory"),
        "ego.reference_trajectory",
        minimum=2,
    )
    _validate_trace_binding(
        trace,
        observations,
        ego_track,
        expected_source=expected_source,
        actor_manifest=actor_manifest,
    )

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
    plugin_identity: dict[str, Any]
    warmup_result: dict[str, Any]
    try:
        plugin_identity = executor.initialize()
        scene_context = {
            "mode": "open_loop_transfuserpp_triplicate",
            "scenario_id": inputs.scenario_ir["scenario_id"],
            "ego_pose_source": "scenario_ir_reference_trajectory",
            "sensor_source": expected_source,
        }
        bound_observations = []
        for source_observation in observations:
            observation = copy.deepcopy(source_observation)
            run_context = copy.deepcopy(observation.get("run_context") or {})
            run_context["run_id"] = run_id
            observation["run_context"] = run_context
            bound_observations.append(observation)
        warmup = getattr(backend, "warmup", None)
        if not callable(warmup):
            raise TriplicateRunnerError("TF++ backend does not expose formal in-process warmup")
        executor.reset(scene_context)
        warmup_result = warmup(
            bound_observations[0],
            iterations=_runtime_warmup_iterations(config),
        )
        # Warm-up is outside the scored lifecycle. Reset both executor and
        # model state so frame 0 remains the first formal prediction.
        executor.reset(scene_context)
        for observation in bound_observations:
            frame_id = int(observation["frame_id"])
            result = executor.predict(observation)
            control = copy.deepcopy(result["control"])
            record_ref = control.get("intermediate_record_ref")
            record = None
            if result["execution_status"] == "fallback":
                fallback_count += 1
            else:
                record = _load_record(record_ref, frame_id)
                intermediate_count += 1
                predictions.append(
                    {
                        "frame_id": frame_id,
                        "observation_frame_id": frame_id,
                        "inference_ms": float(control.get("inference_ms", 0.0)),
                        "predicted_waypoints": _waypoints_to_world(
                            record["outputs"]["waypoints_ego_m"],
                            ego_track[frame_id],
                            horizon_spacing_sec=float(
                                (config.get("open_loop") or {}).get(
                                    "waypoint_spacing_sec", 0.5
                                )
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
                    "execution_detail": copy.deepcopy(result.get("detail")),
                    "control": control,
                    "input_provenance": copy.deepcopy(observation.get("provenance")),
                    "input_payloads": {
                        "camera_front": copy.deepcopy(
                            (observation.get("rgb") or {}).get("camera_front")
                        ),
                        "lidar_top": copy.deepcopy(observation.get("lidar")),
                    },
                    "intermediate_record_ref": copy.deepcopy(record_ref),
                    "intermediate_identity": copy.deepcopy(record.get("identity")) if record else None,
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
    input_binding = copy.deepcopy(trace.get("input_binding") or {})
    trace_lidar_source = input_binding.get("lidar_source")
    route = copy.deepcopy(ROUTE_METADATA[expected_source])
    input_binding.update(
        {
            "route": route["route_id"],
            "rgb_source": route["rgb_source"],
            "lidar_source": route["lidar_source"],
            "trace_lidar_source": trace_lidar_source,
            "harmonizer_rgb_only": route["harmonizer_rgb_only"],
            "same_frame_rgb_lidar_required": True,
            "same_frame_rgb_lidar_verified": True,
        }
    )
    report.update(
        {
            "schema_version": REPORT_SCHEMA,
            "run_id": run_id,
            "stage": STAGE_NAME,
            "real_tfpp_checkpoint_loaded": plugin_identity.get("real_checkpoint_loaded") is True,
            "real_carla_nurec_closed_loop": False,
            "real_carla_stage_a_open_loop": expected_source == "carla_stage_a_native_rgb_lidar",
            "real_reconstruction_open_loop": expected_source != "carla_stage_a_native_rgb_lidar",
            "sensor_source": expected_source,
            "input_route": route,
            "input_binding": input_binding,
            "runtime_config_path": str(runtime_config_path),
            "runtime_config_sha256": sha256_file(runtime_config_path),
            "observation_trace_path": str(trace_path),
            "observation_trace_sha256": sha256_file(trace_path),
            "actor_manifest": {
                "path": str(actor_manifest_path.resolve()),
                "sha256": actor_manifest["manifest_sha256"],
                "file_sha256": actor_manifest["manifest_file_sha256"],
                "summary": copy.deepcopy(actor_manifest["summary"]),
            },
            "runtime_manifest": manifest,
            "plugin": "agents.transfuserpp_plugin:create_plugin",
            "plugin_identity": plugin_identity,
            "tfpp": {
                "intermediate_count": intermediate_count,
                "fallback_count": fallback_count,
                "warmup": warmup_result,
                "formal_frames_excluded_from_warmup": True,
                "required_intermediates": REQUIRED_INTERMEDIATES,
            },
            "frames": frame_rows,
            "evidence_classification": EVIDENCE_CLASSIFICATION,
            "control_affects_next_ego_pose": False,
            "claims_m8": False,
            "claims_m9": False,
            "matrix_actor_ready_ir_bound": True,
        }
    )
    if fallback_count or intermediate_count != len(observations):
        report["execution_status"] = "failed"
    validate_open_loop_report(report)
    return report


def _runtime_warmup_iterations(config: Mapping[str, Any]) -> int:
    gate = config.get("cuda_gate")
    value = gate.get("warmup_iterations") if isinstance(gate, Mapping) else 1
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise TriplicateRunnerError(
            "runtime CUDA warmup_iterations must be a positive integer"
        )
    return value


def write_report(path: Path, report: Mapping[str, Any]) -> None:
    if path.exists():
        raise TriplicateRunnerError(f"refusing to overwrite report: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-ir", type=Path, default=DEFAULT_SCENARIO_IR)
    parser.add_argument("--opendrive", type=Path, default=DEFAULT_OPENDRIVE)
    parser.add_argument("--runtime-config", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--actor-manifest", type=Path, required=True)
    parser.add_argument("--expected-input-source", required=True)
    parser.add_argument("--expected-scenario-ir-sha256", default=EXPECTED_SCENARIO_IR_SHA256)
    parser.add_argument("--expected-opendrive-sha256", default=EXPECTED_OPENDRIVE_SHA256)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = run_route(
            scenario_ir_path=args.scenario_ir,
            opendrive_path=args.opendrive,
            runtime_config_path=args.runtime_config,
            trace_path=args.trace,
            actor_manifest_path=args.actor_manifest,
            expected_source=args.expected_input_source,
            run_id=args.run_id,
            expected_scenario_ir_sha256=args.expected_scenario_ir_sha256,
            expected_opendrive_sha256=args.expected_opendrive_sha256,
            max_frames=args.max_frames,
        )
        write_report(args.report, report)
    except (OSError, ValueError, TypeError, RuntimeError, json.JSONDecodeError, TriplicateRunnerError) as exc:
        print(json.dumps({"status": "failed", "error": f"{type(exc).__name__}: {exc}"}))
        return 2
    print(json.dumps({"status": "written", "report": str(args.report)}))
    return 0 if report["execution_status"] == "completed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
