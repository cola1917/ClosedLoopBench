from __future__ import annotations

"""Materialize a strict TF++ warmup observation from one live scene0061 frame."""

import argparse
import json
import math
import re
import shutil
import sys
from copy import deepcopy
from pathlib import Path, PurePosixPath
from typing import Any, Mapping

from agents.plugin_contract import canonical_sha256, file_sha256, strict_json_loads
from agents.ros2_observation_control_driver import (
    _monotonic_nearest_route_index,
    _normalize_route_command,
    _route_lookahead_index,
    _world_to_ego,
)
from agents.transfuserpp_contract import camera_adaptation_contract, validate_observation


REQUIRED_CAMERAS = (
    "camera_front",
    "camera_front_left",
    "camera_front_right",
    "camera_back",
    "camera_back_left",
    "camera_back_right",
)
REQUIRED_SENSORS = frozenset((*REQUIRED_CAMERAS, "lidar_top"))
REQUIRED_MANIFEST_ARTIFACTS = frozenset(
    (
        "basic_agent_plan.json",
        "runtime_result.json",
        "frame_trace.jsonl",
        "nurec_multimodal_trace.jsonl",
        "metrics_trace.jsonl",
        "cleanup_audit.json",
        "closed_loop_report.json",
        "live_tick_validation.json",
        "runtime_environment.json",
        "lidar_axis_evidence.json",
    )
)
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
GIT_COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
FRAME_DIR_RE = re.compile(r"^frame_[0-9]{8}$")


class Scene0061TransFuserPPWarmupError(ValueError):
    """The physical frame or the target S0 bundle is not safely bound."""


def _fail(message: str) -> None:
    raise Scene0061TransFuserPPWarmupError(message)


def _strict_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = strict_json_loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as exc:
        _fail(f"cannot read strict JSON {label}: {exc}")
    if not isinstance(value, dict):
        _fail(f"{label} must contain a JSON object")
    return value


def _strict_jsonl(path: Path, label: str) -> list[dict[str, Any]]:
    try:
        lines = [line for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    except (OSError, UnicodeError) as exc:
        _fail(f"cannot read {label}: {exc}")
    values: list[dict[str, Any]] = []
    for number, line in enumerate(lines, 1):
        try:
            value = strict_json_loads(line)
        except (ValueError, json.JSONDecodeError) as exc:
            _fail(f"{label} line {number} is not strict JSON: {exc}")
        if not isinstance(value, dict):
            _fail(f"{label} line {number} must be a JSON object")
        values.append(value)
    return values


def _finite(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _sha(value: Any, label: str) -> str:
    text = str(value or "")
    if SHA256_RE.fullmatch(text) is None:
        _fail(f"{label} must be a lowercase SHA-256")
    return text


def _identity(path: Path, declared: Mapping[str, Any] | None = None, *, label: str) -> dict[str, Any]:
    if not path.is_file():
        _fail(f"{label} is unavailable: {path}")
    result = {
        "absolute_path": str(path.resolve()),
        "sha256": file_sha256(path),
        "byte_count": path.stat().st_size,
    }
    if declared is not None:
        if _sha(declared.get("sha256"), f"{label}.sha256") != result["sha256"]:
            _fail(f"{label} SHA-256 does not match")
        size = declared.get("byte_count")
        if isinstance(size, bool) or not isinstance(size, int) or size < 0:
            _fail(f"{label}.byte_count is invalid")
        if size != result["byte_count"]:
            _fail(f"{label} byte count does not match")
    return result


def _output_path(path: Path, root: Path, label: str) -> Path:
    target = path.expanduser().resolve(strict=False)
    try:
        target.relative_to(root.resolve())
    except ValueError:
        _fail(f"{label} must be inside the S0 bundle directory")
    if target.exists():
        _fail(f"refusing to overwrite existing {label}: {target}")
    return target


def _payload_source(
    diagnostics: Path,
    materialized: Mapping[str, Any],
    sensor_id: str,
    *,
    frame_id: int,
) -> tuple[Path, tuple[str, ...]]:
    raw = materialized.get("relative_path")
    if not isinstance(raw, str) or not raw:
        _fail(f"{sensor_id} materialized payload needs relative_path")
    relative = PurePosixPath(raw)
    if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
        _fail(f"{sensor_id} relative payload path escapes diagnostics")
    positions = [index for index, part in enumerate(relative.parts) if part == "algorithm_sensor_payloads"]
    if len(positions) != 1:
        _fail(f"{sensor_id} relative payload path has no unique payload root")
    suffix = relative.parts[positions[0] :]
    if len(suffix) != 3 or not FRAME_DIR_RE.fullmatch(suffix[1]):
        _fail(f"{sensor_id} payload is not in a single frame directory")
    expected_frame = f"frame_{frame_id:08d}"
    expected_name = f"{sensor_id}.bin" if sensor_id == "lidar_top" else f"{sensor_id}.jpg"
    if suffix[1] != expected_frame or suffix[2] != expected_name:
        _fail(f"{sensor_id} payload path does not bind the NuRec trace frame and sensor")
    root = (diagnostics / "algorithm_sensor_payloads").resolve()
    path = (diagnostics / Path(*suffix)).resolve()
    try:
        path.relative_to(root)
    except ValueError:
        _fail(f"{sensor_id} payload resolves outside diagnostics")
    if not path.is_file():
        _fail(f"{sensor_id} payload is unavailable in diagnostics")
    declared_path = materialized.get("path")
    if isinstance(declared_path, str) and declared_path and Path(declared_path).exists():
        declared_resolved = Path(declared_path).resolve()
        try:
            declared_resolved.relative_to(root)
        except ValueError:
            _fail(f"{sensor_id} declared payload is outside diagnostics")
        if declared_resolved != path:
            _fail(f"{sensor_id} declared payload path diverges from relative_path")
    return path, tuple(suffix)


def _validate_output_layout(
    payload_dir: Path, observation_path: Path, provenance_path: Path
) -> None:
    """Keep immutable evidence documents separate from copied payload bytes."""

    if observation_path == provenance_path:
        _fail("observation and provenance output paths must differ")
    for path, label in (
        (observation_path, "observation output"),
        (provenance_path, "provenance output"),
    ):
        try:
            path.relative_to(payload_dir)
        except ValueError:
            pass
        else:
            _fail(f"{label} must not be inside the payload output directory")
        try:
            payload_dir.relative_to(path)
        except ValueError:
            pass
        else:
            _fail(f"payload output directory must not be inside {label}")


def _manifest(diagnostics: Path, value: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    if value.get("schema_version") != "scene0061_live_tick_artifact_manifest.v1":
        _fail("artifact manifest schema is unsupported")
    if value.get("status") != "complete" or value.get("missing_artifacts") != []:
        _fail("artifact manifest is not complete")
    rows = value.get("artifacts")
    if not isinstance(rows, list):
        _fail("artifact manifest has no artifact array")
    result: dict[str, dict[str, Any]] = {}
    for row in rows:
        if not isinstance(row, Mapping):
            _fail("artifact manifest contains a non-object row")
        name = row.get("name")
        path = row.get("path")
        if not isinstance(name, str) or not name or Path(name).name != name:
            _fail("artifact manifest artifact name is unsafe")
        if name in result:
            _fail(f"artifact manifest repeats {name}")
        if not isinstance(path, str) or Path(path).name != name:
            _fail(f"artifact manifest path does not bind {name}")
        result[name] = _identity(diagnostics / name, row, label=f"manifest artifact {name}")
    missing = REQUIRED_MANIFEST_ARTIFACTS.difference(result)
    if missing:
        _fail("artifact manifest omits required artifacts: " + ", ".join(sorted(missing)))
    return result


def _environment(
    value: Mapping[str, Any],
    formal_base: Path,
    runtime_config: Path,
    validation: Mapping[str, Any],
) -> dict[str, Any]:
    if value.get("schema_version") != "scene0061_live_tick_environment.v2" or value.get("status") != "passed":
        _fail("runtime environment must be passed scene0061_live_tick_environment.v2")
    run_id = value.get("run_id")
    if not isinstance(run_id, str) or not run_id:
        _fail("runtime environment run_id is missing")
    git_commit = str(value.get("git_commit") or "")
    if GIT_COMMIT_RE.fullmatch(git_commit) is None or value.get("execution_code_commit") != git_commit:
        _fail("runtime environment git/execution commit is invalid or divergent")
    for name, path in (("config", formal_base), ("runtime_config", runtime_config)):
        declared = value.get(name)
        if not isinstance(declared, Mapping):
            _fail(f"runtime environment {name} identity is missing")
        _identity(path, declared, label=f"runtime environment {name}")
    if _identity(formal_base, label="formal base")["sha256"] != _identity(runtime_config, label="runtime run config")["sha256"]:
        _fail("runtime run config differs from formal base config")
    environment_validation = value.get("validation")
    if (
        not isinstance(environment_validation, Mapping)
        or environment_validation.get("status") != "passed"
        or dict(environment_validation) != dict(validation)
    ):
        _fail("runtime environment has no passed validation")
    native_scan = value.get("native_scan_manifest")
    if not isinstance(native_scan, Mapping):
        _fail("runtime environment native scan manifest identity is missing")
    _sha(native_scan.get("sha256"), "runtime environment native scan manifest")
    return {"run_id": run_id, "git_commit": git_commit, "native_scan_manifest": dict(native_scan)}


def _validation(value: Mapping[str, Any]) -> None:
    expected = {
        "schema_version": "scene0061_live_tick_validation.v1",
        "status": "passed",
        "completion_class": "one_tick_physical_multimodal_smoke",
        "expected_one_tick_termination": True,
        "frame_trace_count": 1,
        "cleanup_succeeded": True,
        "problems": [],
    }
    for name, expected_value in expected.items():
        if value.get(name) != expected_value:
            _fail(f"live-tick validation {name} is not the required passed one-tick result")


def _expected_smoke(value: Mapping[str, Any]) -> None:
    detail = value.get("detail")
    if (
        value.get("status") != "failed"
        or value.get("reason") != "basic_agent_runtime_failed"
        or not isinstance(detail, str)
        or not detail.startswith("route_incomplete:")
        or "termination=max_ticks" not in detail
        or "max_ticks=1" not in detail
    ):
        _fail("runtime result is not the explicit permitted one-tick route_incomplete termination")


def _matrix(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 16 or any(not _finite(item) for item in value):
        _fail(f"{label} must be a finite 16-value matrix")
    return [float(item) for item in value]


def _sensor(config: Mapping[str, Any], collection: str, sensor_id: str) -> dict[str, Any]:
    runtime = config.get("nurec_runtime")
    rows = runtime.get(collection) if isinstance(runtime, Mapping) else None
    if not isinstance(rows, list):
        _fail(f"formal config lacks {collection}")
    matches = [row for row in rows if isinstance(row, Mapping) and row.get("sensor_id") == sensor_id]
    if len(matches) != 1:
        _fail(f"formal config must contain exactly one {sensor_id} in {collection}")
    return deepcopy(dict(matches[0]))


def _formal(acceptance: Mapping[str, Any], formal_base: Path, diagnostics: Path) -> dict[str, Any]:
    runtime = acceptance.get("nurec_runtime")
    if not isinstance(runtime, Mapping):
        _fail("formal acceptance config lacks nurec_runtime")
    if runtime.get("lidar_response_coordinate_frame") != "sensor_local" or runtime.get("lidar_axis_convention") != "carla_sensor":
        _fail("formal acceptance LiDAR coordinate contract is not sensor_local/carla_sensor")
    if runtime.get("lidar_sensor_to_ego_coordinate_frame") != "carla_x_forward_y_right_z_up":
        _fail("formal acceptance LiDAR sensor_to_ego coordinate frame is invalid")
    cameras: dict[str, dict[str, Any]] = {}
    for name in REQUIRED_CAMERAS:
        camera = _sensor(acceptance, "camera_specs", name)
        if camera.get("width") != 1600 or camera.get("height") != 900:
            _fail(f"formal acceptance {name} is not 1600x900")
        camera["sensor_to_ego"] = _matrix(camera.get("sensor_to_ego"), f"formal acceptance {name}")
        cameras[name] = camera
    lidar = _sensor(acceptance, "lidar_specs", "lidar_top")
    lidar["sensor_to_ego"] = _matrix(lidar.get("sensor_to_ego"), "formal acceptance lidar_top")
    binding = acceptance.get("formal_lidar_evidence_binding")
    if not isinstance(binding, Mapping) or binding.get("schema_version") != "scene0061_formal_lidar_evidence_binding.v1" or binding.get("evidence_status") != "passed":
        _fail("formal acceptance config lacks passed formal LiDAR evidence binding")
    base_ref = binding.get("base_run_config")
    evidence_ref = binding.get("lidar_coordinate_evidence")
    if not isinstance(base_ref, Mapping) or not isinstance(evidence_ref, Mapping):
        _fail("formal LiDAR evidence binding is incomplete")
    _identity(formal_base, base_ref, label="formal LiDAR base config")
    evidence_path = diagnostics / "lidar_axis_evidence.json"
    _identity(evidence_path, evidence_ref, label="formal LiDAR coordinate evidence")
    coordinate_validation = runtime.get("lidar_coordinate_validation")
    if not isinstance(coordinate_validation, Mapping) or coordinate_validation.get("evidence_sha256") != evidence_ref.get("sha256"):
        _fail("formal LiDAR coordinate validation is not bound to the axis evidence")
    evidence = _strict_object(evidence_path, "lidar_axis_evidence.json")
    if (
        evidence.get("schema_version") != "scene0061_lidar_coordinate_validation.v1"
        or evidence.get("status") != "passed"
        or evidence.get("sensor_id") != "lidar_top"
        or evidence.get("response_coordinate_frame") != "sensor_local"
        or evidence.get("axis_convention") != "carla_sensor"
        or evidence.get("sensor_to_ego_coordinate_frame") != "carla_x_forward_y_right_z_up"
    ):
        _fail("LiDAR axis evidence does not prove the required formal coordinate convention")
    evidence_matrix = _matrix(evidence.get("sensor_to_ego"), "LiDAR axis evidence")
    if evidence_matrix != lidar["sensor_to_ego"] or evidence.get("sensor_to_ego_sha256") != canonical_sha256(evidence_matrix):
        _fail("LiDAR axis evidence extrinsic or hash differs from formal acceptance config")
    return {"cameras": cameras, "lidar": lidar, "axis_evidence": evidence_path}


def _canonical_run_config(config: Mapping[str, Any]) -> str:
    return canonical_sha256(
        {key: value for key, value in config.items() if key not in {"config_identity", "algorithm_gpu_validation"}}
    )


def _s0_bundle(root: Path, acceptance: Mapping[str, Any], formal: Mapping[str, Any]) -> dict[str, Any]:
    run_path = root / "carla_run_config.json"
    runtime_path = root / "runtime" / "transfuserpp.runtime.json"
    bundle_path = root / "remote_run_bundle.json"
    run = _strict_object(run_path, "S0 carla_run_config.json")
    runtime_config = _strict_object(runtime_path, "S0 transfuserpp.runtime.json")
    bundle = _strict_object(bundle_path, "S0 remote_run_bundle.json")
    experiment = run.get("experiment")
    if not isinstance(experiment, Mapping) or experiment.get("case_id") != "S0_original_replay":
        _fail("warmup builder requires an S0_original_replay run config")
    if isinstance(experiment.get("seed"), bool) or not isinstance(experiment.get("seed"), int):
        _fail("S0 experiment seed is invalid")
    if experiment.get("source_run_config_sha256") != canonical_sha256(acceptance):
        _fail("S0 source_run_config_sha256 is not the formal acceptance config hash")
    run_id = run.get("run_id")
    if not isinstance(run_id, str) or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", run_id) is None:
        _fail("S0 run_id is invalid")
    canonical = _canonical_run_config(run)
    identity = run.get("config_identity")
    if (
        not isinstance(identity, Mapping)
        or identity.get("schema_version") != "closedloopbench_run_config_identity.v1"
        or identity.get("canonical_sha256") != canonical
        or identity.get("hash_scope") != "whole_run_config_excluding_config_identity_and_algorithm_gpu_validation"
    ):
        _fail("S0 run config identity is invalid")
    accepted_identity = (acceptance.get("experiment") or {}).get("identity")
    if not isinstance(accepted_identity, Mapping):
        _fail("formal acceptance experiment identity is missing")
    for name in ("artifact_sha256", "scene_package_sha256", "scenario_ir_sha256", "immutable_matrix_sha256"):
        if experiment.get(name) != accepted_identity.get(name):
            _fail(f"S0 experiment {name} differs from formal acceptance identity")
        _sha(experiment.get(name), f"S0 experiment {name}")
    _sha(experiment.get("variant_config_sha256"), "S0 variant_config_sha256")
    ego = run.get("ego")
    binding = ego.get("algorithm_sensor_binding") if isinstance(ego, Mapping) else None
    if not isinstance(binding, Mapping):
        _fail("S0 run config lacks algorithm sensor binding")
    camera = formal["cameras"]["camera_front"]
    lidar = formal["lidar"]
    if (
        binding.get("camera_sensor_id") != "camera_front"
        or binding.get("camera_source_width") != 1600
        or binding.get("camera_source_height") != 900
        or _matrix(binding.get("camera_sensor_to_ego"), "S0 camera extrinsic") != camera["sensor_to_ego"]
        or binding.get("camera_sensor_to_ego_coordinate_frame") != "carla_x_forward_y_right_z_up"
        or binding.get("camera_adaptation") != camera_adaptation_contract()
    ):
        _fail("S0 camera binding differs from formal calibration")
    if (
        binding.get("lidar_sensor_id") != "lidar_top"
        or _matrix(binding.get("lidar_sensor_to_ego"), "S0 LiDAR extrinsic") != lidar["sensor_to_ego"]
        or binding.get("lidar_axis_convention") != "carla_sensor"
        or binding.get("lidar_sensor_to_ego_coordinate_frame") != "carla_x_forward_y_right_z_up"
        or binding.get("container_payload_root") != "/sim-data"
    ):
        _fail("S0 LiDAR binding or container root differs from the formal contract")
    accepted_runtime = acceptance.get("nurec_runtime")
    s0_runtime = run.get("nurec_runtime")
    if not isinstance(accepted_runtime, Mapping) or not isinstance(s0_runtime, Mapping):
        _fail("formal acceptance or S0 config lacks NuRec runtime")
    for name in ("camera_specs", "lidar_specs", "lidar_response_coordinate_frame", "lidar_axis_convention", "lidar_sensor_to_ego_coordinate_frame"):
        if s0_runtime.get(name) != accepted_runtime.get(name):
            _fail(f"S0 NuRec {name} differs from formal acceptance config")
    runtime_experiment = runtime_config.get("experiment")
    if not isinstance(runtime_experiment, Mapping):
        _fail("S0 runtime config lacks experiment identity")
    for name in ("scene_id", "scene_version", "case_id", "seed", "artifact_sha256", "scene_package_sha256", "scenario_ir_sha256", "immutable_matrix_sha256", "source_run_config_sha256", "variant_config_sha256"):
        if runtime_experiment.get(name) != experiment.get(name):
            _fail(f"S0 runtime config experiment {name} differs from run config")
    if runtime_experiment.get("run_config_sha256") != canonical or runtime_config.get("case_id") != "S0_original_replay" or runtime_config.get("seed") != experiment.get("seed"):
        _fail("S0 runtime config run identity is invalid")
    if (
        bundle.get("schema_version") != "scene0061_transfuserpp_remote_run_bundle.v1"
        or bundle.get("status") != "remote_validation_required"
        or bundle.get("run_id") != run_id
        or bundle.get("case_id") != "S0_original_replay"
        or bundle.get("seed") != experiment.get("seed")
        or bundle.get("run_config_sha256") != canonical
        or bundle.get("runtime_config_sha256") != canonical_sha256(runtime_config)
    ):
        _fail("S0 remote run bundle identity is invalid")
    return {
        "run": run,
        "runtime": runtime_config,
        "bundle": bundle,
        "run_path": run_path,
        "runtime_path": runtime_path,
        "bundle_path": bundle_path,
        "run_id": run_id,
        "canonical_sha256": canonical,
        "experiment": deepcopy(dict(experiment)),
        "binding": deepcopy(dict(binding)),
    }


def _frame(
    diagnostics: Path,
    frame_rows: list[dict[str, Any]],
    nurec_rows: list[dict[str, Any]],
    environment: Mapping[str, Any],
    acceptance: Mapping[str, Any],
) -> dict[str, Any]:
    if len(frame_rows) != 1 or len(nurec_rows) != 1:
        _fail("warmup requires exactly one frame trace and one NuRec trace")
    frame = frame_rows[0]
    nurec = nurec_rows[0]
    if nurec.get("schema_version") != "nurec_multimodal_evidence.v1" or nurec.get("status") != "passed":
        _fail("NuRec trace is not passed multimodal evidence")
    frame_id = nurec.get("frame_id")
    if isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id < 0:
        _fail("NuRec frame_id is invalid")
    if frame.get("world_tick_frame") != frame_id or frame.get("snapshot_frame") != frame_id:
        _fail("CARLA and NuRec frame identities do not exactly match")
    sensor_summary = frame.get("multimodal_sensor")
    if (
        not isinstance(sensor_summary, Mapping)
        or sensor_summary.get("status") != "passed"
        or sensor_summary.get("frame_id") != frame_id
        or sensor_summary.get("modalities")
        != {
            "rgb": {"requested_count": 6, "passed_count": 6},
            "lidar": {"requested_count": 1, "passed_count": 1},
        }
    ):
        _fail("frame trace lacks passed multimodal sensor evidence")
    timestamp = nurec.get("simulation_time_sec")
    if not _finite(timestamp) or float(timestamp) < 0.0 or frame.get("simulation_time_sec") != timestamp:
        _fail("CARLA/NuRec simulation time is invalid or divergent")
    dynamic_sha = _sha(nurec.get("dynamic_object_sha256"), "NuRec dynamic object hash")
    dispatch = nurec.get("dispatch")
    expected_scene = (acceptance.get("experiment") or {}).get("scene_id")
    expected_runtime_scene = (acceptance.get("nurec_runtime") or {}).get("runtime_scene_id")
    if (
        not isinstance(dispatch, Mapping)
        or dispatch.get("nre_api") != "SensorsimService/26.04"
        or not isinstance(expected_scene, str)
        or not expected_scene
        or nurec.get("scene_id") != expected_scene
        or dispatch.get("canonical_scene_id") != expected_scene
        or dispatch.get("runtime_scene_id") != expected_runtime_scene
    ):
        _fail("NuRec trace is not bound to SensorsimService/26.04")
    alignment = dispatch.get("temporal_alignment")
    native_scan = environment.get("native_scan_manifest")
    if (
        not isinstance(alignment, Mapping)
        or not isinstance(native_scan, Mapping)
        or alignment.get("source") != "hashed_native_scan_manifest"
        or alignment.get("manifest_sha256") != native_scan.get("sha256")
    ):
        _fail("NuRec temporal alignment is not bound to the native scan manifest")
    midpoint = alignment.get("midpoint_error_us")
    if isinstance(midpoint, bool) or not isinstance(midpoint, int) or midpoint < 0 or midpoint > 1000:
        _fail("NuRec native scan midpoint error exceeds the 1 ms warmup limit")
    records = nurec.get("records")
    if not isinstance(records, list) or len(records) != len(REQUIRED_SENSORS):
        _fail("NuRec trace must contain exactly six RGB and one LiDAR records")
    payloads: dict[str, dict[str, Any]] = {}
    for record in records:
        if not isinstance(record, Mapping):
            _fail("NuRec trace contains a non-object record")
        sensor_id = record.get("sensor_id")
        if not isinstance(sensor_id, str) or sensor_id not in REQUIRED_SENSORS or sensor_id in payloads:
            _fail("NuRec trace has missing, duplicate, or unexpected sensor IDs")
        modality = "lidar" if sensor_id == "lidar_top" else "rgb"
        if record.get("modality") != modality or record.get("status") != "passed":
            _fail(f"NuRec {sensor_id} is not a passed {modality} response")
        _sha(record.get("payload_sha256"), f"NuRec {sensor_id} response payload")
        metadata = record.get("response_metadata")
        materialized = metadata.get("materialized_payload") if isinstance(metadata, Mapping) else None
        if not isinstance(materialized, Mapping):
            _fail(f"NuRec {sensor_id} has no materialized payload")
        source, suffix = _payload_source(
            diagnostics, materialized, sensor_id, frame_id=frame_id
        )
        identity = _identity(source, materialized, label=f"NuRec {sensor_id} materialized payload")
        if sensor_id == "lidar_top":
            if materialized.get("encoding") != "float32_xyzi_little_endian" or materialized.get("coordinate_frame") != "sensor_local" or materialized.get("axis_convention") != "carla_sensor":
                _fail("NuRec lidar_top materialized coordinate contract is invalid")
        elif (
            metadata.get("encoding") != "jpeg"
            or metadata.get("width") != 1600
            or metadata.get("height") != 900
            or materialized.get("encoding") != "jpeg"
            or materialized.get("coordinate_frame") != "camera_optical"
        ):
            _fail(f"NuRec {sensor_id} is not a 1600x900 camera-optical JPEG")
        payloads[sensor_id] = {"source": source, "suffix": suffix, "identity": identity, "materialized": deepcopy(dict(materialized))}
    if set(payloads) != REQUIRED_SENSORS:
        _fail("NuRec trace lacks the exact six-camera plus lidar_top sensor set")
    pose = frame.get("ego_pose")
    if not isinstance(pose, Mapping) or any(not _finite(pose.get(name)) for name in ("x", "y", "yaw")):
        _fail("frame trace ego pose is invalid")
    speed = frame.get("ego_speed_mps")
    if not _finite(speed) or float(speed) < 0.0:
        _fail("frame trace ego speed is invalid")
    return {
        "frame_id": frame_id,
        "timestamp": float(timestamp),
        "dynamic_sha": dynamic_sha,
        "midpoint_error_us": midpoint,
        "payloads": payloads,
        "pose": {name: float(pose[name]) for name in ("x", "y", "yaw")},
        "speed_mps": float(speed),
    }


def _route(run: Mapping[str, Any], binding: Mapping[str, Any], pose: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    ego = run.get("ego")
    waypoints = ego.get("reference_trajectory") if isinstance(ego, Mapping) else None
    if not isinstance(waypoints, list) or len(waypoints) < 2 or any(not isinstance(item, Mapping) for item in waypoints):
        _fail("S0 run config requires a two-or-more-point ego reference trajectory")
    route = [dict(item) for item in waypoints]
    for index, item in enumerate(route):
        if any(not _finite(item.get(name)) for name in ("x", "y")):
            _fail(f"S0 route point {index} has invalid x/y")
    lookahead = binding.get("route_lookahead_m", 7.5)
    if not _finite(lookahead) or float(lookahead) <= 0.0:
        _fail("S0 algorithm binding route_lookahead_m is invalid")
    nearest = _monotonic_nearest_route_index(route, dict(pose), start_index=0)
    progress = max(0, nearest)
    target_index, target_distance = _route_lookahead_index(route, start_index=progress, lookahead_m=float(lookahead))
    next_index, next_distance = _route_lookahead_index(route, start_index=target_index, lookahead_m=float(lookahead))
    target = route[target_index]
    command = _normalize_route_command(target.get("route_command", target.get("command", "LANE_FOLLOW")))
    observation = {
        "route_waypoints": route,
        "nearest_index": nearest,
        "progress_index": progress,
        "target_index": target_index,
        "target_distance_along_route_m": target_distance,
        "lookahead_m": float(lookahead),
        "target_point": target,
        "route_command": command,
        "target_point_ego_m": _world_to_ego(target, pose),
        "target_point_next_ego_m": _world_to_ego(route[next_index], pose),
        "target_point_coordinate_frame": "carla_ego",
        "gps_source": "bypassed",
    }
    derivation = {
        "route_sha256": canonical_sha256(route),
        "ego_pose": deepcopy(dict(pose)),
        "nearest_index": nearest,
        "progress_index": progress,
        "target_index": target_index,
        "next_target_index": next_index,
        "lookahead_m": float(lookahead),
        "target_distance_along_route_m": target_distance,
        "next_target_distance_along_route_m": next_distance,
        "route_command": command,
        "target_point_ego_m": list(observation["target_point_ego_m"]),
        "target_point_next_ego_m": list(observation["target_point_next_ego_m"]),
        "coordinate_transform": "agents.ros2_observation_control_driver._world_to_ego",
    }
    return observation, derivation


def _payload_ref(path: str, identity: Mapping[str, Any], *, encoding: str, coordinate_frame: str, axis_convention: str | None = None, sensor_to_ego: list[float] | None = None) -> dict[str, Any]:
    value: dict[str, Any] = {
        "path": path,
        "sha256": identity["sha256"],
        "byte_count": identity["byte_count"],
        "encoding": encoding,
        "coordinate_frame": coordinate_frame,
    }
    if axis_convention is not None:
        value["axis_convention"] = axis_convention
    if sensor_to_ego is not None:
        value["sensor_to_ego"] = list(sensor_to_ego)
    return value


def _write(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def build_scene0061_transfuserpp_warmup_observation(
    *,
    diagnostics_dir: Path,
    formal_acceptance_config: Path,
    formal_base_config: Path,
    s0_bundle_dir: Path,
    payload_output_dir: Path,
    observation_output: Path,
    provenance_output: Path,
) -> dict[str, Any]:
    """Validate r22 evidence and write a new immutable S0 warmup input."""
    diagnostics = diagnostics_dir.expanduser().resolve()
    acceptance_path = formal_acceptance_config.expanduser().resolve()
    base_path = formal_base_config.expanduser().resolve()
    s0_root = s0_bundle_dir.expanduser().resolve()
    if not diagnostics.is_dir() or not acceptance_path.is_file() or not base_path.is_file() or not s0_root.is_dir():
        _fail("diagnostics, formal configs, or S0 bundle directory is unavailable")
    payload_dir = _output_path(payload_output_dir, s0_root, "payload output directory")
    observation_path = _output_path(observation_output, s0_root, "observation output")
    provenance_path = _output_path(provenance_output, s0_root, "provenance output")

    acceptance = _strict_object(acceptance_path, "formal acceptance config")
    environment = _strict_object(diagnostics / "runtime_environment.json", "runtime_environment.json")
    validation = _strict_object(diagnostics / "live_tick_validation.json", "live_tick_validation.json")
    manifest = _strict_object(diagnostics / "artifact_manifest.json", "artifact_manifest.json")
    runtime_result = _strict_object(diagnostics / "runtime_result.json", "runtime_result.json")
    runtime_config_path = diagnostics / "runtime_run_config.json"
    manifest_artifacts = _manifest(diagnostics, manifest)
    environment_summary = _environment(environment, base_path, runtime_config_path, validation)
    _validation(validation)
    _expected_smoke(runtime_result)
    formal = _formal(acceptance, base_path, diagnostics)
    s0 = _s0_bundle(s0_root, acceptance, formal)
    frame = _frame(
        diagnostics,
        _strict_jsonl(diagnostics / "frame_trace.jsonl", "frame_trace.jsonl"),
        _strict_jsonl(diagnostics / "nurec_multimodal_trace.jsonl", "nurec_multimodal_trace.jsonl"),
        environment,
        acceptance,
    )
    route, route_derivation = _route(s0["run"], s0["binding"], frame["pose"])
    frame_name = f"frame_{frame['frame_id']:08d}"
    _validate_output_layout(payload_dir, observation_path, provenance_path)
    front_source = frame["payloads"]["camera_front"]
    lidar_source = frame["payloads"]["lidar_top"]
    front_destination = payload_dir / frame_name / "camera_front.jpg"
    lidar_destination = payload_dir / frame_name / "lidar_top.bin"
    if front_destination.exists() or lidar_destination.exists():
        _fail("refusing to overwrite an existing warmup payload")
    container_prefix = f"/sim-data/{payload_dir.relative_to(s0_root).as_posix()}/{frame_name}"
    camera_ref = _payload_ref(f"{container_prefix}/camera_front.jpg", front_source["identity"], encoding="jpeg", coordinate_frame="camera_optical")
    lidar_ref = _payload_ref(
        f"{container_prefix}/lidar_top.bin",
        lidar_source["identity"],
        encoding="float32_xyzi_little_endian",
        coordinate_frame="sensor_local",
        axis_convention="carla_sensor",
        sensor_to_ego=s0["binding"]["lidar_sensor_to_ego"],
    )
    experiment = s0["experiment"]
    run_context = {
        "run_id": s0["run_id"],
        "scene_id": experiment.get("scene_id"),
        "case_id": experiment.get("case_id"),
        "seed": experiment.get("seed"),
        "identity": {
            "artifact_sha256": experiment.get("artifact_sha256"),
            "scene_package_sha256": experiment.get("scene_package_sha256"),
            "scenario_ir_sha256": experiment.get("scenario_ir_sha256"),
            "immutable_matrix_sha256": experiment.get("immutable_matrix_sha256"),
            "source_run_config_sha256": experiment.get("source_run_config_sha256"),
            "variant_config_sha256": experiment.get("variant_config_sha256"),
            "run_config_sha256": s0["canonical_sha256"],
        },
    }
    observation: dict[str, Any] = {
        "schema_version": "transfuserpp_observation.v1",
        "observation_id": f"{s0['run_id']}-warmup-frame-{frame['frame_id']:08d}",
        "source": "scene0061_r22_one_tick_physical_multimodal_smoke",
        "frame_id": frame["frame_id"],
        "timestamp": frame["timestamp"],
        "rgb": {"camera_front": camera_ref},
        "lidar": lidar_ref,
        "sensor_validity": {"camera_front": True, "lidar": True},
        "calibration": deepcopy(s0["binding"]),
        "ego_state": {
            "pose": deepcopy(frame["pose"]),
            "speed_mps": frame["speed_mps"],
            "compass_rad": math.radians(frame["pose"]["yaw"]),
            "speed_source": "carla_actor_velocity",
            "compass_source": "carla_actor_transform_yaw",
        },
        "route": route,
        "synchronization": {
            "frame_id": frame["frame_id"],
            "clock": "carla_snapshot",
            "error_ms": frame["midpoint_error_us"] / 1000.0,
            "dynamic_object_sha256": frame["dynamic_sha"],
            "sensor_age_ticks": 0,
        },
        "run_context": run_context,
    }
    try:
        validate_observation(observation)
    except ValueError as exc:
        _fail(f"constructed warmup observation violates TF++ contract: {exc}")
    if not camera_ref["path"].startswith("/sim-data/") or not lidar_ref["path"].startswith("/sim-data/"):
        _fail("warmup observation contains a non-container payload path")

    # No mutation occurs until all source evidence, routes, hashes and contracts pass.
    front_destination.parent.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(front_source["source"], front_destination)
    shutil.copyfile(lidar_source["source"], lidar_destination)
    if _identity(front_destination, label="copied camera_front")["sha256"] != front_source["identity"]["sha256"]:
        _fail("copied camera_front SHA-256 diverges")
    if _identity(lidar_destination, label="copied lidar_top")["sha256"] != lidar_source["identity"]["sha256"]:
        _fail("copied lidar_top SHA-256 diverges")
    _write(observation_path, observation)

    coverage: dict[str, Any] = {}
    for sensor_id in sorted(REQUIRED_SENSORS):
        payload = frame["payloads"][sensor_id]
        record: dict[str, Any] = {
            "source": {"host_path": str(payload["source"].resolve()), "relative_suffix": "/".join(payload["suffix"]), **payload["identity"]},
            "materialized_contract": deepcopy(payload["materialized"]),
        }
        if sensor_id == "camera_front":
            record["bundle_copy"] = {"host_path": str(front_destination.resolve()), "container_path": camera_ref["path"], **_identity(front_destination, label="copied camera_front")}
        if sensor_id == "lidar_top":
            record["bundle_copy"] = {"host_path": str(lidar_destination.resolve()), "container_path": lidar_ref["path"], **_identity(lidar_destination, label="copied lidar_top")}
        coverage[sensor_id] = record
    provenance: dict[str, Any] = {
        "schema_version": "scene0061_transfuserpp_warmup_provenance.v1",
        "status": "passed",
        "source_diagnostics": {
            "host_path": str(diagnostics),
            "environment": _identity(diagnostics / "runtime_environment.json", label="runtime environment"),
            "validation": _identity(diagnostics / "live_tick_validation.json", label="live tick validation"),
            "artifact_manifest": _identity(diagnostics / "artifact_manifest.json", label="artifact manifest"),
            "runtime_result": _identity(diagnostics / "runtime_result.json", label="runtime result"),
            "frame_trace": _identity(diagnostics / "frame_trace.jsonl", label="frame trace"),
            "nurec_multimodal_trace": _identity(diagnostics / "nurec_multimodal_trace.jsonl", label="NuRec trace"),
            "manifest_artifacts": manifest_artifacts,
            "execution_run_id": environment_summary["run_id"],
            "execution_code_commit": environment_summary["git_commit"],
            "native_scan_manifest": deepcopy(environment_summary["native_scan_manifest"]),
        },
        "formal_inputs": {
            "formal_base_config": _identity(base_path, label="formal base config"),
            "formal_acceptance_config": _identity(acceptance_path, label="formal acceptance config"),
            "lidar_axis_evidence": _identity(formal["axis_evidence"], label="LiDAR axis evidence"),
        },
        "s0_bundle": {
            "host_path": str(s0_root),
            "carla_run_config": _identity(s0["run_path"], label="S0 run config"),
            "runtime_config": _identity(s0["runtime_path"], label="S0 runtime config"),
            "remote_run_bundle": _identity(s0["bundle_path"], label="S0 remote run bundle"),
            "run_id": s0["run_id"],
            "run_context_identity": deepcopy(run_context["identity"]),
        },
        "frame_binding": {
            "frame_id": frame["frame_id"],
            "simulation_time_sec": frame["timestamp"],
            "dynamic_object_sha256": frame["dynamic_sha"],
            "nre_api": "SensorsimService/26.04",
            "native_scan_midpoint_error_us": frame["midpoint_error_us"],
            "all_formal_sensors_passed": sorted(REQUIRED_SENSORS),
        },
        "payload_coverage": coverage,
        "route_derivation": route_derivation,
        "observation": {
            "host_path": str(observation_path.resolve()),
            "file": _identity(observation_path, label="warmup observation"),
            "canonical_sha256": canonical_sha256(observation),
            "container_payload_paths": [camera_ref["path"], lidar_ref["path"]],
        },
    }
    _write(provenance_path, provenance)
    return {
        "status": "passed",
        "frame_id": frame["frame_id"],
        "s0_run_id": s0["run_id"],
        "observation": _identity(observation_path, label="warmup observation"),
        "provenance": _identity(provenance_path, label="warmup provenance"),
        "payload_output_dir": str(payload_dir.resolve()),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build one strict scene0061 S0 TF++ warmup observation.")
    parser.add_argument("--diagnostics-dir", required=True, type=Path)
    parser.add_argument("--formal-acceptance-config", required=True, type=Path)
    parser.add_argument("--formal-base-config", required=True, type=Path)
    parser.add_argument("--s0-bundle-dir", required=True, type=Path)
    parser.add_argument("--payload-output-dir", required=True, type=Path)
    parser.add_argument("--observation-output", required=True, type=Path)
    parser.add_argument("--provenance-output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        result = build_scene0061_transfuserpp_warmup_observation(
            diagnostics_dir=args.diagnostics_dir,
            formal_acceptance_config=args.formal_acceptance_config,
            formal_base_config=args.formal_base_config,
            s0_bundle_dir=args.s0_bundle_dir,
            payload_output_dir=args.payload_output_dir,
            observation_output=args.observation_output,
            provenance_output=args.provenance_output,
        )
    except (OSError, ValueError, RuntimeError, Scene0061TransFuserPPWarmupError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
