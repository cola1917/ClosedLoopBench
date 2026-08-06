"""Run the real TransFuser++ backend on the M6 NuRec open-loop trace.

The trace is captured outside CARLA: Scenario IR supplies every ego pose and
NuRec supplies six RGB responses plus ``lidar_top``. This runner only consumes
those immutable observations, never applies control, and uses the existing
open-loop scorer for IR trajectory and offline actor collision-proxy metrics.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

from agents.algorithm_backend import load_backend
from agents.plugin_contract import AlgorithmPluginExecutor, strict_json_loads
from agents.transfuserpp_contract import build_runtime_manifest, validate_observation
from metrics.open_loop import score_open_loop_predictions, validate_open_loop_report
from runners.run_open_loop_gt_replay import (
    DEFAULT_OPENDRIVE,
    DEFAULT_SCENARIO_IR,
    EXPECTED_OPENDRIVE_SHA256,
    EXPECTED_SCENARIO_IR_SHA256,
    load_pinned_inputs,
    sha256_file,
)
from runners.run_open_loop_transfuserpp_stage_a import (
    _load_intermediate_record,
    _waypoints_to_world,
)
from runtime.scene0061_lidar_axis_normalization import (
    LiDARAxisNormalizationError,
    verify_normalized_lidar_payload,
)


REPORT_SCHEMA = "open_loop_multimodal_report.v1"
EVIDENCE_CLASSIFICATION = "open_loop_multimodal"
STAGE_NAME = "M6_tfpp_stage_b"
SOURCE = "nurec_stage_b_6cam_rgb_lidar"
EXPECTED_CAMERA_IDS = (
    "camera_front",
    "camera_front_left",
    "camera_front_right",
    "camera_back",
    "camera_back_left",
    "camera_back_right",
)
EXPECTED_SENSOR_IDS = (*EXPECTED_CAMERA_IDS, "lidar_top")
EMPTY_DYNAMIC_DIGEST = "4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945"


class OpenLoopTransFuserPPStageBError(ValueError):
    """Raised when M6 evidence cannot be proven from the captured trace."""


def run_open_loop_transfuserpp_stage_b(
    *,
    scenario_ir_path: str | Path,
    opendrive_path: str | Path,
    runtime_config_path: str | Path | None = None,
    observation_trace_path: str | Path | None = None,
    expected_scenario_ir_sha256: str = EXPECTED_SCENARIO_IR_SHA256,
    expected_opendrive_sha256: str = EXPECTED_OPENDRIVE_SHA256,
    run_id: str = "scene-0061-open-loop-m6",
    max_frames: int | None = None,
) -> dict[str, Any]:
    if not isinstance(run_id, str) or not run_id.strip():
        raise OpenLoopTransFuserPPStageBError("run_id must be a non-empty string")
    if max_frames is not None and (isinstance(max_frames, bool) or max_frames <= 0):
        raise OpenLoopTransFuserPPStageBError("max_frames must be positive when provided")
    inputs = load_pinned_inputs(
        scenario_ir_path,
        opendrive_path,
        expected_scenario_ir_sha256=expected_scenario_ir_sha256,
        expected_opendrive_sha256=expected_opendrive_sha256,
    )
    config_path = Path(runtime_config_path) if runtime_config_path is not None else None
    trace_path = Path(observation_trace_path) if observation_trace_path is not None else None
    blockers: list[str] = []
    config: dict[str, Any] | None = None
    runtime_manifest: dict[str, Any] | None = None

    if config_path is None:
        blockers.append("runtime_config_missing")
    elif not config_path.is_file():
        blockers.append("runtime_config_unavailable")
    else:
        try:
            loaded = strict_json_loads(config_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            blockers.append(f"runtime_config_unreadable:{exc}")
        else:
            if not isinstance(loaded, dict):
                blockers.append("runtime_config_must_be_object")
            else:
                config = loaded
                blockers.extend(
                    _open_loop_config_problems(
                        config,
                        inputs.scenario_ir_sha256,
                        inputs.opendrive_sha256,
                    )
                )
                runtime_manifest = build_runtime_manifest(config)
                if runtime_manifest["execution_status"] != "prepared":
                    blockers.extend(
                        f"runtime_manifest:{problem}"
                        for problem in runtime_manifest.get("problems", [])
                    )

    if trace_path is None:
        blockers.append("nurec_stage_b_observation_trace_missing")
    elif not trace_path.is_file():
        blockers.append("nurec_stage_b_observation_trace_unavailable")

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
    assert config_path is not None
    assert trace_path is not None
    assert runtime_manifest is not None
    trace = _load_trace(trace_path)
    if trace.get("source") != SOURCE:
        raise OpenLoopTransFuserPPStageBError("trace source is not the M6 NuRec source")
    trace_config = trace.get("runtime_config")
    if not isinstance(trace_config, Mapping) or trace_config.get("sha256") != sha256_file(config_path):
        raise OpenLoopTransFuserPPStageBError("M6 trace is not bound to the runtime config")
    observations = trace.get("frames")
    if not isinstance(observations, list) or any(not isinstance(item, dict) for item in observations):
        raise OpenLoopTransFuserPPStageBError("M6 trace frames must be objects")
    observations = [copy.deepcopy(item) for item in observations]
    if max_frames is not None:
        observations = observations[:max_frames]
    if len(observations) < 2:
        raise OpenLoopTransFuserPPStageBError("M6 requires at least two observations")

    ego_track = _normalise_track(
        (inputs.scenario_ir.get("ego") or {}).get("reference_trajectory")
    )
    if len(observations) > len(ego_track):
        raise OpenLoopTransFuserPPStageBError("M6 trace exceeds Scenario IR frame count")
    for expected_frame_id, observation in enumerate(observations):
        _validate_observation_binding(
            observation,
            expected_frame_id=expected_frame_id,
            expected_state=ego_track[expected_frame_id],
            experiment=config["experiment"],
        )
        _validate_nurec_evidence(observation, expected_frame_id)
        validate_observation(
            observation,
            max_synchronization_error_ms=float(config.get("max_synchronization_error_ms", 1.0)),
        )

    backend = load_backend("agents.transfuserpp_plugin:create_plugin", copy.deepcopy(config))
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
    warmup_result: dict[str, Any]
    try:
        plugin_identity = executor.initialize()
        scene_context = {
            "mode": "open_loop_transfuserpp_stage_b_nurec",
            "scenario_id": inputs.scenario_ir["scenario_id"],
            "ego_pose_source": "scenario_ir_reference_trajectory",
            "sensor_source": SOURCE,
            "carla_dynamic_actor_creation": False,
        }
        executor.reset(scene_context)
        warmup = getattr(backend, "warmup", None)
        if not callable(warmup):
            raise OpenLoopTransFuserPPStageBError(
                "M6 TF++ backend does not expose formal in-process warmup"
            )
        warmup_result = warmup(
            observations[0],
            iterations=_runtime_warmup_iterations(config),
        )
        # Warm-up is outside the scored lifecycle. Reset both executor and
        # model state so frame 0 remains the first formal prediction.
        executor.reset(scene_context)
        for observation in observations:
            frame_id = int(observation["frame_id"])
            result = executor.predict(observation)
            control = copy.deepcopy(result["control"])
            record_ref = control.get("intermediate_record_ref")
            record = None
            if result["execution_status"] == "fallback":
                fallback_count += 1
            else:
                record = _load_intermediate_record(record_ref, frame_id)
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
                                config["open_loop"].get("waypoint_spacing_sec", 0.5)
                            ),
                        ),
                    }
                )
            evidence = observation["nurec_evidence"]
            frame_rows.append(
                {
                    "frame_id": frame_id,
                    "timestamp": float(observation["timestamp"]),
                    "observation_frame_id": frame_id,
                    "control_source_frame_id": control.get("source_frame_id"),
                    "execution_status": result["execution_status"],
                    "control": control,
                    "intermediate_record_ref": copy.deepcopy(record_ref),
                    "intermediate_identity": copy.deepcopy(record.get("identity")) if record else None,
                    "nurec_evidence": {
                        "frame_sha256": observation["provenance"]["nurec_frame_sha256"],
                        "evidence_sha256": observation["provenance"]["nurec_evidence_sha256"],
                        "dynamic_object_count": evidence["dynamic_object_count"],
                        "rgb_passed_count": evidence["modalities"]["rgb"]["passed_count"],
                        "lidar_passed_count": evidence["modalities"]["lidar"]["passed_count"],
                        "raw_lidar": copy.deepcopy(
                            observation["nurec_sensor_payloads"]["lidar_top_raw_response"]
                        ),
                        "normalized_lidar": copy.deepcopy(
                            observation["nurec_sensor_payloads"]["lidar_top_normalized"]
                        ),
                    },
                }
            )
    finally:
        executor.close()

    report_runtime_manifest = copy.deepcopy(runtime_manifest)
    report_runtime_manifest["real_checkpoint_loaded"] = (
        plugin_identity["real_checkpoint_loaded"] is True
    )
    report_runtime_manifest["runtime_evidence"] = {
        "real_checkpoint_loaded": report_runtime_manifest["real_checkpoint_loaded"],
        "formal_warmup": copy.deepcopy(warmup_result),
        "formal_frames_excluded_from_warmup": True,
    }

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
            "real_tfpp_checkpoint_loaded": plugin_identity["real_checkpoint_loaded"] is True,
            "real_carla_nurec_closed_loop": False,
            "real_carla_stage_b_open_loop": True,
            "sensor_source": SOURCE,
            "runtime_config_path": str(config_path),
            "runtime_config_sha256": sha256_file(config_path),
            "observation_trace_path": str(trace_path),
            "observation_trace_sha256": sha256_file(trace_path),
            "runtime_manifest": report_runtime_manifest,
            "plugin": "agents.transfuserpp_plugin:create_plugin",
            "plugin_identity": plugin_identity,
            "nurec": _nurec_report_summary(config, trace, observations),
            "tfpp": {
                "intermediate_count": intermediate_count,
                "fallback_count": fallback_count,
                "warmup": warmup_result,
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
            "evidence_classification": EVIDENCE_CLASSIFICATION,
            "control_affects_next_ego_pose": False,
            "claims_m8": False,
            "claims_m9": False,
            "matrix_actor_ready_ir_bound": False,
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
        raise OpenLoopTransFuserPPStageBError(
            "runtime CUDA warmup_iterations must be a positive integer"
        )
    return value


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
        "sensor_source": SOURCE,
        "ego_pose_source": "scenario_ir_reference_trajectory",
        "dynamic_actor_creation": False,
        "dynamic_object_count": 0,
        "matrix_actor_ready_ir_bound": False,
        "real_carla_nurec_closed_loop": False,
        "real_carla_stage_b_open_loop": True,
    }
    for name, value in expected.items():
        if open_loop.get(name) != value:
            problems.append(f"runtime_config.open_loop.{name}_mismatch")
    experiment = config.get("experiment")
    if not isinstance(experiment, Mapping):
        problems.append("runtime_config.experiment_missing")
    elif experiment.get("scenario_ir_sha256") != scenario_ir_sha256:
        problems.append("runtime_config.experiment.scenario_ir_sha256_mismatch")
    if not isinstance(experiment, Mapping) or experiment.get("immutable_matrix_role") != (
        "open_loop_input_set_not_actor_ready_matrix"
    ):
        problems.append("runtime_config.experiment.immutable_matrix_role_mismatch")
    nurec = config.get("nurec_runtime")
    if not isinstance(nurec, Mapping):
        problems.append("runtime_config.nurec_runtime_missing")
    else:
        if nurec.get("runtime_scene_id") != "scene-0061":
            problems.append("runtime_config.nurec_runtime_scene_id_mismatch")
        if nurec.get("dynamic_actor_creation") is not False or nurec.get("dynamic_object_count") != 0:
            problems.append("runtime_config.nurec_runtime_dynamic_objects_nonzero")
        if nurec.get("api") != "SensorsimService/26.04":
            problems.append("runtime_config.nurec_runtime_api_mismatch")
        if nurec.get("target") != "127.0.0.1:46443":
            problems.append("runtime_config.nurec_runtime_target_mismatch")
        if not isinstance(nurec.get("camera_specs"), list) or len(nurec["camera_specs"]) != 6:
            problems.append("runtime_config.nurec_runtime_camera_specs_mismatch")
        if not isinstance(nurec.get("lidar_specs"), list) or len(nurec["lidar_specs"]) != 1:
            problems.append("runtime_config.nurec_runtime_lidar_specs_mismatch")
        try:
            from runtime.scene0061_lidar_axis_normalization import validate_lidar_axis_normalization

            validate_lidar_axis_normalization(nurec.get("lidar_axis_normalization"))
        except (ValueError, TypeError):
            problems.append("runtime_config.nurec_runtime_lidar_normalization_invalid")
    spacing = open_loop.get("waypoint_spacing_sec", 0.5)
    if isinstance(spacing, bool) or not isinstance(spacing, (int, float)) or not math.isfinite(float(spacing)) or float(spacing) <= 0:
        problems.append("runtime_config.open_loop.waypoint_spacing_sec_invalid")
    return problems


def _load_trace(path: Path) -> dict[str, Any]:
    try:
        value = strict_json_loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise OpenLoopTransFuserPPStageBError(f"cannot read M6 trace: {exc}") from exc
    if not isinstance(value, dict):
        raise OpenLoopTransFuserPPStageBError("M6 trace must be a JSON object")
    return value


def _normalise_track(value: Any) -> list[dict[str, float]]:
    if not isinstance(value, list) or len(value) < 2:
        raise OpenLoopTransFuserPPStageBError("Scenario IR ego trajectory has fewer than two frames")
    result = []
    previous_t = -math.inf
    for index, row in enumerate(value):
        if not isinstance(row, Mapping):
            raise OpenLoopTransFuserPPStageBError(f"IR ego frame {index} is not an object")
        state = {
            "t_sec": float(row["t_sec"]),
            "x": float(row["x"]),
            "y": float(row["y"]),
            "z": float(row.get("z", 0.0)),
            "yaw": float(row.get("yaw", 0.0)),
            "speed_mps": float(row.get("speed_mps", 0.0)),
        }
        if any(not math.isfinite(value) for value in state.values()) or state["t_sec"] < previous_t:
            raise OpenLoopTransFuserPPStageBError(f"IR ego frame {index} is invalid or non-monotonic")
        previous_t = state["t_sec"]
        result.append(state)
    return result


def _validate_observation_binding(
    observation: Mapping[str, Any],
    *,
    expected_frame_id: int,
    expected_state: Mapping[str, float],
    experiment: Mapping[str, Any],
) -> None:
    if observation.get("frame_id") != expected_frame_id:
        raise OpenLoopTransFuserPPStageBError(f"M6 observation frame_id mismatch at {expected_frame_id}")
    if observation.get("source") != SOURCE:
        raise OpenLoopTransFuserPPStageBError(f"M6 observation {expected_frame_id} has an invalid source")
    if not _close(observation.get("timestamp"), expected_state["t_sec"]):
        raise OpenLoopTransFuserPPStageBError(f"M6 observation {expected_frame_id} timestamp is not IR-bound")
    pose = ((observation.get("ego_state") or {}).get("pose") or {})
    for name in ("x", "y", "yaw"):
        if not _close(pose.get(name), expected_state[name]):
            raise OpenLoopTransFuserPPStageBError(f"M6 observation {expected_frame_id} pose is not IR-bound")
    provenance = observation.get("provenance")
    if not isinstance(provenance, Mapping) or provenance.get("gt_pose_replay") is not True:
        raise OpenLoopTransFuserPPStageBError(f"M6 observation {expected_frame_id} lacks GT pose provenance")
    for name, expected in (
        ("control_applied", False),
        ("control_affects_next_ego_pose", False),
        ("carla_dynamic_actor_creation", False),
        ("carla_dynamic_actor_count", 0),
    ):
        if provenance.get(name) != expected:
            raise OpenLoopTransFuserPPStageBError(
                f"M6 observation {expected_frame_id} provenance {name} is invalid"
            )
    context = observation.get("run_context")
    if not isinstance(context, Mapping):
        raise OpenLoopTransFuserPPStageBError("M6 observation run_context is missing")
    for name in ("scene_id", "case_id", "seed"):
        if context.get(name) != experiment.get(name):
            raise OpenLoopTransFuserPPStageBError(f"M6 observation run_context {name} mismatch")
    identity = context.get("identity")
    if not isinstance(identity, Mapping):
        raise OpenLoopTransFuserPPStageBError("M6 observation formal identity is missing")
    for name in (
        "artifact_sha256",
        "scene_package_sha256",
        "scenario_ir_sha256",
        "immutable_matrix_sha256",
        "source_run_config_sha256",
        "variant_config_sha256",
        "run_config_sha256",
    ):
        if identity.get(name) != experiment.get(name):
            raise OpenLoopTransFuserPPStageBError(f"M6 observation identity {name} mismatch")


def _validate_nurec_evidence(observation: Mapping[str, Any], frame_id: int) -> None:
    frame = observation.get("nurec_frame")
    evidence = observation.get("nurec_evidence")
    if not isinstance(frame, Mapping) or not isinstance(evidence, Mapping):
        raise OpenLoopTransFuserPPStageBError(f"M6 observation {frame_id} lacks NuRec frame/evidence")
    if frame.get("frame_id") != frame_id or evidence.get("frame_id") != frame_id:
        raise OpenLoopTransFuserPPStageBError(f"M6 NuRec frame {frame_id} identity mismatch")
    if frame.get("shared_dynamic_objects") != [] or frame.get("shared_dynamic_object_sha256") != EMPTY_DYNAMIC_DIGEST:
        raise OpenLoopTransFuserPPStageBError(f"M6 NuRec frame {frame_id} contains dynamic actors")
    if evidence.get("status") != "passed" or evidence.get("dynamic_object_count") != 0:
        raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} is not passed/empty")
    _validate_evidence_contract(evidence, frame_id)
    records = evidence.get("records")
    if not isinstance(records, list) or {record.get("sensor_id") for record in records} != set(EXPECTED_SENSOR_IDS):
        raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} sensor set is incomplete")
    modalities = evidence.get("modalities") or {}
    if modalities.get("rgb", {}).get("passed_count") != 6 or modalities.get("lidar", {}).get("passed_count") != 1:
        raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} modality counts are incomplete")
    for record in records:
        if record.get("status") != "passed":
            raise OpenLoopTransFuserPPStageBError(f"M6 NuRec sensor {record.get('sensor_id')} failed")
        metadata = record.get("response_metadata")
        if not isinstance(metadata, Mapping) or not isinstance(metadata.get("materialized_payload"), Mapping):
            raise OpenLoopTransFuserPPStageBError(f"M6 NuRec sensor {record.get('sensor_id')} has no materialized payload")
        materialized = metadata["materialized_payload"]
        if record.get("modality") == "rgb":
            if metadata.get("width") != 1600 or metadata.get("height") != 900:
                raise OpenLoopTransFuserPPStageBError(f"M6 RGB {record.get('sensor_id')} is not 1600x900")
            _verify_payload(materialized, f"M6 RGB {record.get('sensor_id')}", encoding="jpeg")
        else:
            _verify_payload(materialized, "M6 normalized lidar_top", encoding="float32_xyzi_little_endian")
            raw = observation.get("nurec_sensor_payloads", {}).get("lidar_top_raw_response")
            if not isinstance(raw, Mapping):
                raise OpenLoopTransFuserPPStageBError(f"M6 LiDAR {frame_id} raw response is missing")
            try:
                verified = verify_normalized_lidar_payload(
                    raw_response_payload=raw,
                    normalized_payload=materialized,
                    normalization=metadata.get("axis_normalization"),
                )
            except (LiDARAxisNormalizationError, ValueError, OSError) as exc:
                raise OpenLoopTransFuserPPStageBError(
                    f"M6 LiDAR {frame_id} raw/normalized hash verification failed: {exc}"
                ) from exc
            if verified["point_count"] < 1:
                raise OpenLoopTransFuserPPStageBError(f"M6 LiDAR {frame_id} has no points")
    payloads = observation.get("nurec_sensor_payloads")
    if not isinstance(payloads, Mapping):
        raise OpenLoopTransFuserPPStageBError(f"M6 observation {frame_id} sensor payload map is missing")
    for sensor_id in EXPECTED_CAMERA_IDS:
        ref = payloads.get(sensor_id)
        if not isinstance(ref, Mapping):
            raise OpenLoopTransFuserPPStageBError(f"M6 camera payload missing: {sensor_id}")
        _verify_payload(ref, f"M6 camera {sensor_id}", encoding="jpeg")
    _verify_payload(payloads.get("lidar_top_normalized"), "M6 lidar_top normalized", encoding="float32_xyzi_little_endian")
    _verify_payload(payloads.get("lidar_top_raw_response"), "M6 lidar_top raw", encoding="float32_xyzi_little_endian")


def _validate_evidence_contract(evidence: Mapping[str, Any], frame_id: int) -> None:
    if evidence.get("schema_version") != "nurec_multimodal_evidence.v1":
        raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} schema is invalid")
    records = evidence.get("records")
    issues = evidence.get("issues")
    if not isinstance(records, list) or not isinstance(issues, list) or issues:
        raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} has issues")
    if evidence.get("status") != "passed" or any(record.get("status") != "passed" for record in records):
        raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} status is inconsistent")
    modalities = evidence.get("modalities")
    if not isinstance(modalities, Mapping) or set(modalities) != {"rgb", "lidar"}:
        raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} modality summary is invalid")
    for modality, expected_count in (("rgb", 6), ("lidar", 1)):
        selected = [record for record in records if record.get("modality") == modality]
        summary = modalities.get(modality)
        if not isinstance(summary, Mapping) or summary.get("requested_count") != expected_count or summary.get("passed_count") != expected_count or len(selected) != expected_count:
            raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} {modality} count is invalid")
    if evidence.get("dynamic_object_count") != 0 or evidence.get("dynamic_object_sha256") != EMPTY_DYNAMIC_DIGEST:
        raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} dynamic-object digest is invalid")
    for record in records:
        metadata = record.get("response_metadata")
        if not isinstance(metadata, Mapping):
            raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} response metadata is missing")
        materialized = metadata.get("materialized_payload")
        if not isinstance(materialized, Mapping) or not materialized.get("path") or not materialized.get("sha256"):
            raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} materialized payload is incomplete")
        if record.get("modality") == "rgb":
            if metadata.get("encoding") != "jpeg" or int(metadata.get("width") or 0) != 1600 or int(metadata.get("height") or 0) != 900:
                raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} RGB metadata is invalid")
        elif record.get("modality") == "lidar":
            if metadata.get("encoding") != "float_xyz_intensity" or int(metadata.get("point_count") or 0) < 1:
                raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} LiDAR metadata is invalid")
            if not isinstance(metadata.get("raw_response_payload"), Mapping) or not isinstance(metadata.get("axis_normalization"), Mapping):
                raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} LiDAR raw/normalization metadata is missing")
        else:
            raise OpenLoopTransFuserPPStageBError(f"M6 NuRec evidence {frame_id} has an unknown modality")


def _verify_payload(value: Any, label: str, *, encoding: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise OpenLoopTransFuserPPStageBError(f"{label} reference is missing")
    path = Path(str(value.get("path") or ""))
    if not path.is_file():
        raise OpenLoopTransFuserPPStageBError(f"{label} is unavailable: {path}")
    if value.get("encoding") != encoding:
        raise OpenLoopTransFuserPPStageBError(f"{label} encoding is invalid")
    expected_size = value.get("byte_count")
    body = path.read_bytes()
    if not isinstance(expected_size, int) or expected_size != len(body):
        raise OpenLoopTransFuserPPStageBError(f"{label} byte count mismatch")
    if value.get("sha256") != hashlib.sha256(body).hexdigest():
        raise OpenLoopTransFuserPPStageBError(f"{label} SHA-256 mismatch")
    return dict(value)


def _nurec_report_summary(
    config: Mapping[str, Any], trace: Mapping[str, Any], observations: list[Mapping[str, Any]]
) -> dict[str, Any]:
    nurec = config.get("nurec_runtime") or {}
    experiment = config.get("experiment") or {}
    return {
        "api": nurec.get("api"),
        "target": nurec.get("target"),
        "runtime_scene_id": nurec.get("runtime_scene_id"),
        "artifact_sha256": experiment.get("artifact_sha256"),
        "scene_package_sha256": experiment.get("scene_package_sha256"),
        "input_set_sha256": experiment.get("immutable_matrix_sha256"),
        "sensor_source": SOURCE,
        "camera_ids": list(EXPECTED_CAMERA_IDS),
        "lidar_ids": ["lidar_top"],
        "camera_count": 6,
        "lidar_count": 1,
        "frame_count": len(observations),
        "all_frames_rgb6_passed": all(
            item.get("nurec_evidence", {}).get("modalities", {}).get("rgb", {}).get("passed_count") == 6
            for item in observations
        ),
        "all_frames_lidar_passed": all(
            item.get("nurec_evidence", {}).get("modalities", {}).get("lidar", {}).get("passed_count") == 1
            for item in observations
        ),
        "all_frames_raw_normalized_lidar_verified": True,
        "dynamic_actor_creation": False,
        "dynamic_object_count": 0,
        "trace_schema_version": trace.get("schema_version"),
    }


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
        "real_carla_stage_b_open_loop": False,
        "real_tfpp_checkpoint_loaded": False,
        "remote_validation_required": True,
        "ego_pose_source": "scenario_ir_reference_trajectory",
        "control_affects_next_ego_pose": False,
        "claims_m8": False,
        "claims_m9": False,
        "matrix_actor_ready_ir_bound": False,
        "sensor_source": SOURCE,
        "blockers": blockers,
        "runtime_manifest": copy.deepcopy(dict(runtime_manifest or {})),
        "runtime_config_path": str(runtime_config_path) if runtime_config_path else None,
        "observation_trace_path": str(observation_trace_path) if observation_trace_path else None,
        "nurec": {
            "sensor_source": SOURCE,
            "camera_count": 6,
            "lidar_count": 1,
            "dynamic_actor_creation": False,
            "dynamic_object_count": 0,
        },
        "frame_sync": {
            "source_frame_count": len(
                (inputs.scenario_ir.get("ego") or {}).get("reference_trajectory") or []
            ),
            "prediction_frame_count": 0,
            "matched_frame_count": 0,
            "dropped_frame_count": 0,
            "frame_mismatch_count": 0,
            "scored_frame_mismatch_count": 0,
        },
        "metrics": {},
        "artifacts": {
            "scenario_ir_path": str(inputs.scenario_ir_path),
            "scenario_ir_sha256": inputs.scenario_ir_sha256,
            "opendrive_path": str(inputs.opendrive_path),
            "opendrive_sha256": inputs.opendrive_sha256,
        },
        "per_frame": [],
    }
    validate_open_loop_report(report)
    return report


def _close(left: Any, right: Any, tolerance: float = 1e-5) -> bool:
    if isinstance(left, bool) or not isinstance(left, (int, float)):
        return False
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=tolerance)


def write_report(path: Path, report: Mapping[str, Any]) -> None:
    if path.exists():
        raise OpenLoopTransFuserPPStageBError(f"refusing to overwrite existing output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-ir", type=Path, default=DEFAULT_SCENARIO_IR)
    parser.add_argument("--opendrive", type=Path, default=DEFAULT_OPENDRIVE)
    parser.add_argument("--runtime-config", type=Path, required=True)
    parser.add_argument("--observations", type=Path, required=True)
    parser.add_argument("--expected-scenario-ir-sha256", default=EXPECTED_SCENARIO_IR_SHA256)
    parser.add_argument("--expected-opendrive-sha256", default=EXPECTED_OPENDRIVE_SHA256)
    parser.add_argument("--run-id", default="scene-0061-open-loop-m6")
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        report = run_open_loop_transfuserpp_stage_b(
            scenario_ir_path=args.scenario_ir,
            opendrive_path=args.opendrive,
            runtime_config_path=args.runtime_config,
            observation_trace_path=args.observations,
            expected_scenario_ir_sha256=args.expected_scenario_ir_sha256,
            expected_opendrive_sha256=args.expected_opendrive_sha256,
            run_id=args.run_id,
            max_frames=args.max_frames,
        )
        write_report(args.report, report)
        print(json.dumps({"status": "written", "report": str(args.report)}))
        return 0 if report["execution_status"] == "completed" else 2
    except (OSError, ValueError, TypeError, RuntimeError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "failed", "error": f"{type(exc).__name__}: {exc}"}))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
