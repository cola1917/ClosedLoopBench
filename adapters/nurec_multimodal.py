from __future__ import annotations

import hashlib
import json
import math
from copy import deepcopy
from typing import Any, Iterable, Mapping

from adapters.actor_binding import assert_actor_binding_ready, validate_actor_binding_set
from adapters.shared_protocol_validation import validate_document


class NuRecMultimodalError(ValueError):
    """Raised when RGB and LiDAR cannot be proven to use one scene state."""


def make_pose_pair(start: Mapping[str, Any], end: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Create a simulator-frame pose interval from state dictionaries."""

    return {"start": dict(start), "end": dict(end if end is not None else start)}


def sensor_pose_pair_from_ego(
    ego_pose_pair: Mapping[str, Any],
    sensor_to_ego: Iterable[float],
) -> dict[str, Any]:
    """Compose a calibrated sensor extrinsic with a simulator-frame ego pose pair.

    ``sensor_to_ego`` is a row-major transform that maps sensor-local points into
    the ego/rig frame (``T_ego_sensor`` in common robotics notation).
    """

    _validate_sim_pose_pair(ego_pose_pair, "ego")
    extrinsic = _validated_rigid_matrix(list(sensor_to_ego), "sensor_to_ego")
    return {
        endpoint: {
            "matrix_row_major_4x4": _matmul(
                _state_matrix(ego_pose_pair[endpoint]),
                extrinsic,
            )
        }
        for endpoint in ("start", "end")
    }


def build_nurec_multimodal_frame(
    scene_package: Mapping[str, Any],
    binding_set: Mapping[str, Any],
    *,
    frame_id: int,
    simulation_time_sec: float,
    interval_start_sec: float,
    camera_specs: Iterable[Mapping[str, Any]],
    lidar_specs: Iterable[Mapping[str, Any]],
    actor_samples: Mapping[str, Mapping[str, Any]],
    require_runtime_validated_alignment: bool = True,
) -> dict[str, Any]:
    """Build one synchronized, SDK-neutral NuRec RGB/LiDAR frame request.

    Sensor specs and actor samples contain poses in the declared simulator frame.
    They are transformed into nuScenes/NuRec global coordinates using the inverse
    of ``Scene Package.alignment.sim_from_log_transform``.
    """

    _require_frame(frame_id, simulation_time_sec, interval_start_sec)
    try:
        validate_document(dict(scene_package))
    except ValueError as exc:
        raise NuRecMultimodalError(str(exc)) from exc
    validate_actor_binding_set(binding_set)
    assert_actor_binding_ready(binding_set)
    scene_id = str(scene_package["scene_id"])
    if binding_set["scene_id"] != scene_id:
        raise NuRecMultimodalError("Actor Binding Set scene_id does not match Scene Package")
    alignment = scene_package["alignment"]
    if require_runtime_validated_alignment and alignment["status"] != "runtime_validated":
        raise NuRecMultimodalError(
            "NuRec closed-loop frames require runtime_validated Scene Package alignment"
        )
    sim_from_log = alignment.get("sim_from_log_transform")
    if not isinstance(sim_from_log, list) or len(sim_from_log) != 16:
        raise NuRecMultimodalError("Scene Package has no sim_from_log_transform")
    log_from_sim = _invert_rigid(sim_from_log)

    cameras = _normalize_sensors("rgb", camera_specs, log_from_sim)
    lidars = _normalize_sensors("lidar", lidar_specs, log_from_sim)
    if not cameras or not lidars:
        raise NuRecMultimodalError("a multimodal frame requires at least one RGB camera and one LiDAR")
    sensor_ids = [item["sensor_id"] for item in cameras + lidars]
    if len(sensor_ids) != len(set(sensor_ids)):
        raise NuRecMultimodalError("sensor IDs must be unique across RGB and LiDAR")

    dynamic_objects = _dynamic_objects(binding_set, actor_samples, log_from_sim)
    dynamic_digest = _digest(dynamic_objects)
    request_root = f"{scene_id}:{frame_id}"
    frame = {
        "schema_version": "nurec_multimodal_frame.v1",
        "scene_id": scene_id,
        "frame_id": frame_id,
        "simulation_time_sec": float(simulation_time_sec),
        "pose_interval_sec": {
            "start": float(interval_start_sec),
            "end": float(simulation_time_sec),
        },
        "coordinate_frame": {
            "input": "scene_local_ego_start",
            "render": "nuscenes_global",
            "transform_source": "closed_loop_scene_package.v1.alignment",
            "alignment_status": alignment["status"],
        },
        "shared_dynamic_objects": dynamic_objects,
        "shared_dynamic_object_sha256": dynamic_digest,
        "modalities": {
            "rgb": {
                "requests": [
                    _sensor_request(
                        f"{request_root}:rgb:{item['sensor_id']}", "rgb", item, dynamic_digest
                    )
                    for item in cameras
                ]
            },
            "lidar": {
                "requests": [
                    _sensor_request(
                        f"{request_root}:lidar:{item['sensor_id']}", "lidar", item, dynamic_digest
                    )
                    for item in lidars
                ]
            },
        },
        "synchronization": {
            "clock": "carla_snapshot",
            "policy": "same_frame_same_pose_interval_same_dynamic_objects",
            "fail_closed": True,
        },
    }
    validate_nurec_multimodal_frame(frame)
    return frame


def validate_nurec_multimodal_frame(frame: Mapping[str, Any]) -> None:
    """Validate the invariant that both modalities reference identical actor poses."""

    try:
        validate_document(dict(frame))
    except ValueError as exc:
        raise NuRecMultimodalError(str(exc)) from exc
    if frame.get("schema_version") != "nurec_multimodal_frame.v1":
        raise NuRecMultimodalError("unsupported NuRec multimodal frame schema")
    _require_frame(
        frame.get("frame_id"),
        frame.get("simulation_time_sec"),
        (frame.get("pose_interval_sec") or {}).get("start"),
    )
    interval = frame["pose_interval_sec"]
    if float(interval.get("end", -1.0)) != float(frame["simulation_time_sec"]):
        raise NuRecMultimodalError("pose interval must end at simulation_time_sec")
    dynamic_objects = frame.get("shared_dynamic_objects")
    if not isinstance(dynamic_objects, list):
        raise NuRecMultimodalError("shared_dynamic_objects must be a list")
    actor_ids = [str(item.get("actor_id") or "") for item in dynamic_objects]
    track_ids = [str(item.get("track_id") or "") for item in dynamic_objects]
    if not all(actor_ids) or len(actor_ids) != len(set(actor_ids)):
        raise NuRecMultimodalError("dynamic actor IDs must be non-empty and unique")
    if not all(track_ids) or len(track_ids) != len(set(track_ids)):
        raise NuRecMultimodalError("NuRec dynamic track IDs must be non-empty and unique")
    for item in dynamic_objects:
        _validate_render_pose_pair(item.get("pose_pair"), f"actor {item.get('actor_id')}")
    digest = _digest(dynamic_objects)
    if frame.get("shared_dynamic_object_sha256") != digest:
        raise NuRecMultimodalError("shared dynamic-object digest does not match payload")

    request_ids = []
    for modality in ("rgb", "lidar"):
        requests = ((frame.get("modalities") or {}).get(modality) or {}).get("requests")
        if not isinstance(requests, list) or not requests:
            raise NuRecMultimodalError(f"NuRec frame has no {modality} requests")
        for request in requests:
            if request.get("modality") != modality:
                raise NuRecMultimodalError(f"{modality} request has a mismatched modality")
            if request.get("dynamic_object_sha256") != digest:
                raise NuRecMultimodalError(f"{modality} request references different dynamic objects")
            request_id = str(request.get("request_id") or "")
            if not request_id:
                raise NuRecMultimodalError(f"{modality} request_id is required")
            request_ids.append(request_id)
            sensor = request.get("sensor")
            if not isinstance(sensor, Mapping) or not sensor.get("sensor_id") or not sensor.get("model"):
                raise NuRecMultimodalError(f"{modality} sensor identity/model is required")
            _validate_render_pose_pair(sensor.get("pose_pair"), f"sensor {sensor.get('sensor_id')}")
    if len(request_ids) != len(set(request_ids)):
        raise NuRecMultimodalError("NuRec request IDs must be unique")


def materialize_nurec_rpc_requests(frame: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Materialize per-sensor payloads for a version-specific protobuf adapter."""

    validate_nurec_multimodal_frame(frame)
    result = []
    for modality in ("rgb", "lidar"):
        for request in frame["modalities"][modality]["requests"]:
            result.append(
                {
                    "schema_version": "nurec_rpc_payload.v1",
                    "request_id": request["request_id"],
                    "scene_id": frame["scene_id"],
                    "frame_id": frame["frame_id"],
                    "simulation_time_sec": frame["simulation_time_sec"],
                    "pose_interval_sec": deepcopy(frame["pose_interval_sec"]),
                    "modality": modality,
                    "sensor": deepcopy(request["sensor"]),
                    "dynamic_objects": deepcopy(frame["shared_dynamic_objects"]),
                    "dynamic_object_sha256": frame["shared_dynamic_object_sha256"],
                }
            )
    return result


def build_nurec_multimodal_evidence(
    frame: Mapping[str, Any],
    responses: Iterable[Mapping[str, Any]],
    *,
    max_latency_ms: float | None = None,
) -> dict[str, Any]:
    """Compare locally recorded RPC responses with the exact requested frame."""

    validate_nurec_multimodal_frame(frame)
    expected = {item["request_id"]: item for item in materialize_nurec_rpc_requests(frame)}
    response_by_id: dict[str, Mapping[str, Any]] = {}
    issues: list[str] = []
    for response in responses:
        request_id = str(response.get("request_id") or "")
        if not request_id:
            issues.append("response_without_request_id")
            continue
        if request_id in response_by_id:
            issues.append(f"duplicate_response:{request_id}")
        response_by_id[request_id] = response
    for request_id in sorted(set(response_by_id) - set(expected)):
        issues.append(f"unexpected_response:{request_id}")

    records = []
    digest = frame["shared_dynamic_object_sha256"]
    for request_id, request in expected.items():
        response = response_by_id.get(request_id)
        record = {
            "request_id": request_id,
            "modality": request["modality"],
            "sensor_id": request["sensor"]["sensor_id"],
            "status": "failed",
            "latency_ms": None,
            "payload_sha256": None,
            "issues": [],
        }
        if response is None:
            record["issues"].append("missing_response")
        else:
            if response.get("status") != "ok":
                record["issues"].append("rpc_status_not_ok")
            if response.get("frame_id") != frame["frame_id"]:
                record["issues"].append("frame_id_mismatch")
            if response.get("dynamic_object_sha256") != digest:
                record["issues"].append("dynamic_object_digest_mismatch")
            payload_digest = str(response.get("payload_sha256") or "")
            if not _is_sha256(payload_digest):
                record["issues"].append("payload_sha256_invalid")
            else:
                record["payload_sha256"] = payload_digest
            latency = response.get("latency_ms")
            if not isinstance(latency, (int, float)) or isinstance(latency, bool) or not math.isfinite(float(latency)) or float(latency) < 0:
                record["issues"].append("latency_invalid")
            else:
                record["latency_ms"] = float(latency)
                if max_latency_ms is not None and float(latency) > float(max_latency_ms):
                    record["issues"].append("latency_threshold_exceeded")
        if not record["issues"]:
            record["status"] = "passed"
        else:
            issues.extend(f"{request_id}:{issue}" for issue in record["issues"])
        records.append(record)

    modality_summary = {}
    for modality in ("rgb", "lidar"):
        selected = [record for record in records if record["modality"] == modality]
        modality_summary[modality] = {
            "requested_count": len(selected),
            "passed_count": sum(record["status"] == "passed" for record in selected),
        }
    evidence = {
        "schema_version": "nurec_multimodal_evidence.v1",
        "scene_id": frame["scene_id"],
        "frame_id": frame["frame_id"],
        "simulation_time_sec": frame["simulation_time_sec"],
        "dynamic_object_sha256": digest,
        "dynamic_object_count": len(frame["shared_dynamic_objects"]),
        "records": records,
        "modalities": modality_summary,
        "max_latency_ms": float(max_latency_ms) if max_latency_ms is not None else None,
        "issues": sorted(set(issues)),
        "status": "passed" if not issues else "failed",
    }
    validate_nurec_multimodal_evidence(evidence)
    return evidence


def validate_nurec_multimodal_evidence(evidence: Mapping[str, Any]) -> None:
    try:
        validate_document(dict(evidence))
    except ValueError as exc:
        raise NuRecMultimodalError(str(exc)) from exc
    if evidence.get("schema_version") != "nurec_multimodal_evidence.v1":
        raise NuRecMultimodalError("unsupported NuRec multimodal evidence schema")
    records = evidence.get("records")
    if not isinstance(records, list) or not records:
        raise NuRecMultimodalError("NuRec multimodal evidence requires response records")
    issues = evidence.get("issues")
    if not isinstance(issues, list):
        raise NuRecMultimodalError("NuRec multimodal evidence issues must be a list")
    passed = all(record.get("status") == "passed" for record in records) and not issues
    if (evidence.get("status") == "passed") != passed:
        raise NuRecMultimodalError("NuRec multimodal evidence status is inconsistent")
    if set((evidence.get("modalities") or {})) != {"rgb", "lidar"}:
        raise NuRecMultimodalError("NuRec evidence must summarize RGB and LiDAR")
    for modality, summary in evidence["modalities"].items():
        selected = [record for record in records if record.get("modality") == modality]
        if summary.get("requested_count") != len(selected):
            raise NuRecMultimodalError(f"{modality} requested_count is inconsistent")
        if summary.get("passed_count") != sum(record.get("status") == "passed" for record in selected):
            raise NuRecMultimodalError(f"{modality} passed_count is inconsistent")


def assert_nurec_multimodal_evidence(evidence: Mapping[str, Any]) -> None:
    validate_nurec_multimodal_evidence(evidence)
    if evidence["status"] != "passed":
        raise NuRecMultimodalError(
            "NuRec multimodal frame failed: " + ", ".join(evidence["issues"])
        )


def _dynamic_objects(
    binding_set: Mapping[str, Any],
    actor_samples: Mapping[str, Mapping[str, Any]],
    log_from_sim: list[float],
) -> list[dict[str, Any]]:
    expected_ids = {item["actor_id"] for item in binding_set["bindings"]}
    extra = sorted(set(actor_samples) - expected_ids)
    if extra:
        raise NuRecMultimodalError(f"actor_samples contains unbound actors: {', '.join(extra)}")
    result = []
    for binding in binding_set["bindings"]:
        actor_id = binding["actor_id"]
        sample = actor_samples.get(actor_id)
        if not isinstance(sample, Mapping):
            raise NuRecMultimodalError(f"actor sample is missing: {actor_id}")
        expected_source = binding["sensor_sync"]["pose_source"]
        if sample.get("source") != expected_source:
            raise NuRecMultimodalError(
                f"actor {actor_id} pose source must be {expected_source}"
            )
        pose_pair = sample.get("pose_pair")
        _validate_sim_pose_pair(pose_pair, f"actor {actor_id}")
        result.append(
            {
                "actor_id": actor_id,
                "track_id": binding["nurec"]["track_id"],
                "actor_type": binding["actor_type"],
                "pose_source": expected_source,
                "pose_pair": _transform_pose_pair(pose_pair, log_from_sim),
            }
        )
    return sorted(result, key=lambda item: item["actor_id"])


def _normalize_sensors(
    modality: str,
    specs: Iterable[Mapping[str, Any]],
    log_from_sim: list[float],
) -> list[dict[str, Any]]:
    result = []
    for spec in specs:
        sensor_id = str(spec.get("sensor_id") or "")
        model = str(spec.get("model") or "")
        if not sensor_id or not model:
            raise NuRecMultimodalError(f"{modality} sensor_id and model are required")
        pose_pair = spec.get("pose_pair")
        _validate_sim_pose_pair(pose_pair, f"sensor {sensor_id}")
        parameters = {
            str(key): deepcopy(value)
            for key, value in spec.items()
            if key not in {"sensor_id", "model", "pose_pair"}
        }
        result.append(
            {
                "sensor_id": sensor_id,
                "model": model,
                "parameters": parameters,
                "pose_pair": _transform_pose_pair(pose_pair, log_from_sim),
            }
        )
    return sorted(result, key=lambda item: item["sensor_id"])


def _sensor_request(
    request_id: str,
    modality: str,
    sensor: Mapping[str, Any],
    digest: str,
) -> dict[str, Any]:
    return {
        "request_id": request_id,
        "modality": modality,
        "sensor": deepcopy(dict(sensor)),
        "dynamic_object_sha256": digest,
    }


def _transform_pose_pair(pair: Mapping[str, Any], log_from_sim: list[float]) -> dict[str, Any]:
    return {
        "start": _transform_pose(pair["start"], log_from_sim),
        "end": _transform_pose(pair["end"], log_from_sim),
    }


def _transform_pose(state: Mapping[str, Any], log_from_sim: list[float]) -> dict[str, Any]:
    sim_pose = _state_matrix(state)
    render_pose = _matmul(log_from_sim, sim_pose)
    quaternion = _rotation_to_quaternion(render_pose)
    return {
        "position_m": {"x": render_pose[3], "y": render_pose[7], "z": render_pose[11]},
        "orientation_xyzw": {
            "x": quaternion[0],
            "y": quaternion[1],
            "z": quaternion[2],
            "w": quaternion[3],
        },
    }


def _state_matrix(state: Mapping[str, Any]) -> list[float]:
    if "matrix_row_major_4x4" in state:
        matrix = state["matrix_row_major_4x4"]
        if not isinstance(matrix, list):
            raise NuRecMultimodalError("pose matrix_row_major_4x4 must be a list")
        return _validated_rigid_matrix(matrix, "pose matrix")
    values = {}
    for name in ("x", "y", "z", "roll", "pitch", "yaw"):
        default = 0.0
        value = state.get(name, default)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
            raise NuRecMultimodalError(f"pose {name} must be finite")
        values[name] = float(value)
    roll, pitch, yaw = (math.radians(values[name]) for name in ("roll", "pitch", "yaw"))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        values["x"],
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        values["y"],
        -sp,
        cp * sr,
        cp * cr,
        values["z"],
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def _invert_rigid(matrix: list[Any]) -> list[float]:
    values = _validated_rigid_matrix(matrix, "alignment matrix")
    rotation = [[values[row * 4 + col] for col in range(3)] for row in range(3)]
    transpose = [[rotation[col][row] for col in range(3)] for row in range(3)]
    translation = [values[3], values[7], values[11]]
    inverse_translation = [
        -sum(transpose[row][col] * translation[col] for col in range(3))
        for row in range(3)
    ]
    return [
        transpose[0][0], transpose[0][1], transpose[0][2], inverse_translation[0],
        transpose[1][0], transpose[1][1], transpose[1][2], inverse_translation[1],
        transpose[2][0], transpose[2][1], transpose[2][2], inverse_translation[2],
        0.0, 0.0, 0.0, 1.0,
    ]


def _validated_rigid_matrix(matrix: list[Any], label: str) -> list[float]:
    if len(matrix) != 16:
        raise NuRecMultimodalError(f"{label} must contain 16 values")
    try:
        values = [float(value) for value in matrix]
    except (TypeError, ValueError) as exc:
        raise NuRecMultimodalError(f"{label} must contain numeric values") from exc
    if any(not math.isfinite(value) for value in values):
        raise NuRecMultimodalError(f"{label} must be finite")
    if any(abs(values[index] - expected) > 1e-6 for index, expected in zip((12, 13, 14, 15), (0, 0, 0, 1))):
        raise NuRecMultimodalError(f"{label} must be a rigid homogeneous transform")
    rotation = [[values[row * 4 + col] for col in range(3)] for row in range(3)]
    for row in range(3):
        norm = sum(rotation[row][col] ** 2 for col in range(3))
        if abs(norm - 1.0) > 1e-4:
            raise NuRecMultimodalError(f"{label} rotation is not orthonormal")
    for left in range(3):
        for right in range(left + 1, 3):
            dot = sum(rotation[left][col] * rotation[right][col] for col in range(3))
            if abs(dot) > 1e-4:
                raise NuRecMultimodalError(f"{label} rotation is not orthonormal")
    return values


def _matmul(left: list[float], right: list[float]) -> list[float]:
    return [
        sum(left[row * 4 + index] * right[index * 4 + col] for index in range(4))
        for row in range(4)
        for col in range(4)
    ]


def _rotation_to_quaternion(matrix: list[float]) -> tuple[float, float, float, float]:
    m00, m11, m22 = matrix[0], matrix[5], matrix[10]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[9] - matrix[6]) / scale
        y = (matrix[2] - matrix[8]) / scale
        z = (matrix[4] - matrix[1]) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (matrix[9] - matrix[6]) / scale
        x = 0.25 * scale
        y = (matrix[1] + matrix[4]) / scale
        z = (matrix[2] + matrix[8]) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (matrix[2] - matrix[8]) / scale
        x = (matrix[1] + matrix[4]) / scale
        y = 0.25 * scale
        z = (matrix[6] + matrix[9]) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (matrix[4] - matrix[1]) / scale
        x = (matrix[2] + matrix[8]) / scale
        y = (matrix[6] + matrix[9]) / scale
        z = 0.25 * scale
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


def _validate_sim_pose_pair(value: Any, label: str) -> None:
    if not isinstance(value, Mapping) or not isinstance(value.get("start"), Mapping) or not isinstance(value.get("end"), Mapping):
        raise NuRecMultimodalError(f"{label} pose_pair requires start and end poses")
    _state_matrix(value["start"])
    _state_matrix(value["end"])


def _validate_render_pose_pair(value: Any, label: str) -> None:
    if not isinstance(value, Mapping):
        raise NuRecMultimodalError(f"{label} render pose_pair is required")
    for endpoint in ("start", "end"):
        pose = value.get(endpoint)
        position = pose.get("position_m") if isinstance(pose, Mapping) else None
        orientation = pose.get("orientation_xyzw") if isinstance(pose, Mapping) else None
        if not isinstance(position, Mapping) or set(position) != {"x", "y", "z"}:
            raise NuRecMultimodalError(f"{label} {endpoint} position is invalid")
        if not isinstance(orientation, Mapping) or set(orientation) != {"x", "y", "z", "w"}:
            raise NuRecMultimodalError(f"{label} {endpoint} orientation is invalid")
        values = [*position.values(), *orientation.values()]
        if any(not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)) for value in values):
            raise NuRecMultimodalError(f"{label} {endpoint} pose must be finite")


def _require_frame(frame_id: Any, simulation_time_sec: Any, interval_start_sec: Any) -> None:
    if not isinstance(frame_id, int) or isinstance(frame_id, bool) or frame_id < 0:
        raise NuRecMultimodalError("frame_id must be a non-negative integer")
    for label, value in (("simulation_time_sec", simulation_time_sec), ("interval_start_sec", interval_start_sec)):
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)) or float(value) < 0:
            raise NuRecMultimodalError(f"{label} must be finite and non-negative")
    if float(interval_start_sec) > float(simulation_time_sec):
        raise NuRecMultimodalError("pose interval starts after simulation time")


def _digest(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _is_sha256(value: str) -> bool:
    return len(value) == 64 and all(character in "0123456789abcdef" for character in value)
