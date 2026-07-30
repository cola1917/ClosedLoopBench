"""Bind NuRec sensor responses to physical CARLA M8 world truth."""

from __future__ import annotations

import hashlib
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from adapters.lidar_world_support import (
    LidarWorldSupportError,
    expected_lidar_support_from_physical_boxes,
    lidar_occupancy_from_xyzi,
)


class M8SensorEvidenceError(ValueError):
    """Raised when sensor output cannot be bound to one physical tick."""


def build_m8_sensor_evidence(
    runtime_row: Mapping[str, Any],
    sensor_evidence: Mapping[str, Any],
    *,
    camera_specs: Sequence[Mapping[str, Any]],
    lidar_specs: Sequence[Mapping[str, Any]],
    camera_calibration_capture: Mapping[str, Any] | None,
    max_lidar_range_m: float = 80.0,
    lidar_tolerance_m: float = 0.10,
    required_camera_ids: Sequence[str] | None = None,
) -> dict[str, dict[str, Any]]:
    """Build visibility and world-LiDAR audit payloads for one CARLA frame.

    The function deliberately reads the materialized payload bytes and checks
    their hashes.  Response ``point_count`` or an RGB RPC status alone is not
    accepted as physical-world evidence.
    """

    frame_id = _frame_id(runtime_row)
    if sensor_evidence.get("frame_id") != frame_id:
        raise M8SensorEvidenceError("sensor evidence frame does not match CARLA frame")
    if sensor_evidence.get("status") != "passed":
        raise M8SensorEvidenceError("sensor evidence frame is not passed")
    simulation_time = runtime_row.get("simulation_time_sec")
    if not _finite(simulation_time):
        raise M8SensorEvidenceError("runtime frame has invalid simulation time")

    cameras = _camera_models(
        camera_specs,
        camera_calibration_capture,
        required_camera_ids=required_camera_ids,
    )
    records = _passed_records(sensor_evidence)
    rgb_records = {
        str(record["sensor_id"]): record
        for record in records
        if record.get("modality") == "rgb"
    }
    if set(rgb_records) != set(cameras):
        missing = sorted(set(cameras) - set(rgb_records))
        raise M8SensorEvidenceError(
            "passed RGB payload set does not match calibrated cameras: "
            + ", ".join(missing)
        )

    ego_state = runtime_row.get("ego_state")
    object_states = runtime_row.get("object_states")
    if not isinstance(ego_state, Mapping) or not isinstance(object_states, list):
        raise M8SensorEvidenceError(
            "runtime frame requires ego_state and object_states"
        )

    projections: list[dict[str, Any]] = []
    payloads: list[dict[str, Any]] = []
    for camera_id in sorted(cameras):
        record = rgb_records[camera_id]
        materialized = _materialized_payload(record, "rgb")
        payloads.append(
            {
                "sensor_id": camera_id,
                "frame_id": frame_id,
                "payload_sha256": materialized["sha256"],
                "relative_path": materialized.get("relative_path"),
                "calibrated_sensor_token": cameras[camera_id][
                    "calibrated_sensor_token"
                ],
                "intrinsics_table_sha256": cameras[camera_id][
                    "intrinsics_table_sha256"
                ],
            }
        )
        for state in object_states:
            if not isinstance(state, Mapping):
                raise M8SensorEvidenceError("object state must be an object")
            object_id = _nonempty(state.get("object_id"), "object_state.object_id")
            projected = _project_box(
                state,
                ego_state,
                cameras[camera_id],
            )
            if projected is None:
                continue
            projections.append(
                {
                    "object_id": object_id,
                    "camera": camera_id,
                    "frame_id": frame_id,
                    "t_sec": float(simulation_time),
                    "expected_visible": True,
                    "observation_kind": "calibrated_3d_box_projection",
                    "projection": projected,
                    "evidence": {
                        "nre_payload_sha256": materialized["sha256"],
                        "nre_payload_relative_path": materialized.get(
                            "relative_path"
                        ),
                        "calibrated_sensor_token": cameras[camera_id][
                            "calibrated_sensor_token"
                        ],
                        "intrinsics_table_sha256": cameras[camera_id][
                            "intrinsics_table_sha256"
                        ],
                    },
                }
            )
    if not projections:
        raise M8SensorEvidenceError(
            "no physical object projects into any calibrated RGB payload"
        )

    lidar_spec = _single_lidar_spec(lidar_specs)
    lidar_records = [
        record
        for record in records
        if record.get("modality") == "lidar"
        and str(record.get("sensor_id") or "") == lidar_spec["sensor_id"]
    ]
    if len(lidar_records) != 1:
        raise M8SensorEvidenceError(
            f"expected one passed LiDAR record for {lidar_spec['sensor_id']}"
        )
    lidar_materialized = _materialized_payload(lidar_records[0], "lidar")
    metadata = lidar_records[0].get("response_metadata") or {}
    if metadata.get("coordinate_frame") != "sensor_local":
        raise M8SensorEvidenceError(
            "M8 LiDAR occupancy requires sensor_local materialized coordinates"
        )
    try:
        payload_bytes = Path(str(lidar_materialized["path"])).read_bytes()
        expected_support = expected_lidar_support_from_physical_boxes(
            ego_pose=ego_state.get("pose"),
            sensor_to_ego=list(lidar_spec["sensor_to_ego"]),
            object_states=object_states,
            max_range_m=max_lidar_range_m,
        )
        occupancy = lidar_occupancy_from_xyzi(
            payload_bytes,
            ego_pose=ego_state.get("pose"),
            sensor_to_ego=list(lidar_spec["sensor_to_ego"]),
            object_states=object_states,
            tolerance_m=lidar_tolerance_m,
        )
    except (OSError, KeyError, TypeError, ValueError, LidarWorldSupportError) as exc:
        raise M8SensorEvidenceError(f"LiDAR physical-box binding failed: {exc}") from exc
    return {
        "visibility": {
            "frame_id": frame_id,
            "simulation_time_sec": float(simulation_time),
            "projections": projections,
            "payloads": payloads,
            "payload_binding_required": True,
        },
        "lidar_world": {
            "frame_id": frame_id,
            "simulation_time_sec": float(simulation_time),
            "expected_world_objects": expected_support,
            "lidar_occupancy": occupancy,
            "sensor_id": lidar_spec["sensor_id"],
            "payload_sha256": lidar_materialized["sha256"],
            "payload_binding_required": True,
            "coordinate_frame": metadata.get("coordinate_frame"),
            "axis_convention": metadata.get("axis_convention"),
        },
    }


def load_camera_calibration_capture(
    reference: Any,
) -> Mapping[str, Any] | None:
    """Load a hash-bound calibration capture reference, or return ``None``."""

    if reference is None:
        return None
    if isinstance(reference, Mapping) and "camera_records" in reference:
        return reference
    path_value = reference.get("path") if isinstance(reference, Mapping) else reference
    if not path_value:
        return None
    path = Path(str(path_value)).expanduser()
    if not path.is_file():
        raise M8SensorEvidenceError(f"camera calibration capture does not exist: {path}")
    expected = reference.get("sha256") if isinstance(reference, Mapping) else None
    body = path.read_bytes()
    if expected is not None and hashlib.sha256(body).hexdigest() != str(expected):
        raise M8SensorEvidenceError("camera calibration capture SHA-256 mismatch")
    import json

    value = json.loads(body.decode("utf-8"))
    if not isinstance(value, Mapping):
        raise M8SensorEvidenceError("camera calibration capture must be an object")
    return value


def _camera_models(
    specs: Sequence[Mapping[str, Any]],
    capture: Mapping[str, Any] | None,
    *,
    required_camera_ids: Sequence[str] | None,
) -> dict[str, dict[str, Any]]:
    if capture is None:
        raise M8SensorEvidenceError(
            "M8 requires a source-bound camera calibration capture"
        )
    if capture.get("schema_version") != "nurec_camera_calibration_capture.v1":
        raise M8SensorEvidenceError(
            "camera calibration capture schema is unsupported"
        )
    if capture.get("intrinsics_status") != "passed":
        raise M8SensorEvidenceError(
            "camera calibration capture is not source-bound and passed"
        )
    configured = {
        str(item.get("sensor_id") or ""): item
        for item in specs
        if isinstance(item, Mapping) and item.get("sensor_id")
    }
    if not configured:
        raise M8SensorEvidenceError("M8 requires at least one configured RGB camera")
    selected = set(str(value) for value in (required_camera_ids or configured))
    if selected != set(configured):
        raise M8SensorEvidenceError("required camera set differs from configured cameras")
    captured = {
        str(item.get("sensor_id") or ""): item
        for item in (capture or {}).get("camera_records") or []
        if isinstance(item, Mapping) and item.get("sensor_id")
    }
    result: dict[str, dict[str, Any]] = {}
    for sensor_id in sorted(selected):
        spec = configured[sensor_id]
        record = captured.get(sensor_id)
        matrix = spec.get("intrinsic_matrix_3x3")
        table_hash = spec.get("intrinsics_table_sha256")
        token = spec.get("calibrated_sensor_token")
        width = spec.get("width")
        height = spec.get("height")
        if record is not None:
            configured_token = spec.get("calibrated_sensor_token")
            captured_token = record.get("calibrated_sensor_token")
            if configured_token and captured_token != configured_token:
                raise M8SensorEvidenceError(
                    f"camera {sensor_id} calibrated sensor token mismatch"
                )
            matrix = record.get("intrinsic_matrix_3x3")
            token = record.get("calibrated_sensor_token")
            source = record.get("intrinsics_source") or {}
            table_hash = source.get("table_sha256") if isinstance(source, Mapping) else None
            resolution = record.get("requested_resolution") or {}
            width = resolution.get("width", width)
            height = resolution.get("height", height)
        if not _intrinsic(matrix) or not _sha256(table_hash) or not token:
            raise M8SensorEvidenceError(
                f"camera {sensor_id} lacks source-bound intrinsic calibration"
            )
        if not _positive_int(width) or not _positive_int(height):
            raise M8SensorEvidenceError(f"camera {sensor_id} has invalid resolution")
        extrinsic = spec.get("sensor_to_ego")
        if not isinstance(extrinsic, list) or len(extrinsic) != 16:
            raise M8SensorEvidenceError(
                f"camera {sensor_id} lacks sensor_to_ego calibration"
            )
        result[sensor_id] = {
            "matrix": [[float(value) for value in row] for row in matrix],
            "sensor_to_ego": [float(value) for value in extrinsic],
            "width": int(width),
            "height": int(height),
            "calibrated_sensor_token": str(token),
            "intrinsics_table_sha256": str(table_hash),
        }
    return result


def _single_lidar_spec(specs: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    rows = [
        dict(item)
        for item in specs
        if isinstance(item, Mapping) and item.get("sensor_id")
    ]
    if len(rows) != 1:
        raise M8SensorEvidenceError("M8 requires exactly one configured LiDAR sensor")
    if not isinstance(rows[0].get("sensor_to_ego"), list) or len(
        rows[0]["sensor_to_ego"]
    ) != 16:
        raise M8SensorEvidenceError("LiDAR sensor_to_ego calibration is missing")
    return rows[0]


def _passed_records(evidence: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    records = evidence.get("records")
    if not isinstance(records, list):
        raise M8SensorEvidenceError("NuRec evidence has no records")
    result = [
        record
        for record in records
        if isinstance(record, Mapping) and record.get("status") == "passed"
    ]
    if len(result) != len(records):
        raise M8SensorEvidenceError("M8 requires every same-frame sensor response to pass")
    return result


def _materialized_payload(record: Mapping[str, Any], modality: str) -> Mapping[str, Any]:
    metadata = record.get("response_metadata")
    materialized = metadata.get("materialized_payload") if isinstance(metadata, Mapping) else None
    if not isinstance(materialized, Mapping):
        raise M8SensorEvidenceError(
            f"{modality} response has no materialized payload"
        )
    path = Path(str(materialized.get("path") or ""))
    digest = str(materialized.get("sha256") or "")
    if not path.is_file() or not _sha256(digest):
        raise M8SensorEvidenceError(
            f"{modality} materialized payload path/hash is invalid"
        )
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != digest:
        raise M8SensorEvidenceError(
            f"{modality} materialized payload hash mismatch: {path}"
        )
    return materialized


def _project_box(
    state: Mapping[str, Any],
    ego_state: Mapping[str, Any],
    camera: Mapping[str, Any],
) -> dict[str, Any] | None:
    pose = _pose(state.get("pose"), "object pose")
    ego_pose = _pose(ego_state.get("pose"), "ego pose")
    extent = state.get("extent_m")
    if not isinstance(extent, Mapping):
        raise M8SensorEvidenceError("object state has no extent_m")
    try:
        extents = [float(extent[axis]) for axis in ("x", "y", "z")]
    except (KeyError, TypeError, ValueError) as exc:
        raise M8SensorEvidenceError("object extent_m is invalid") from exc
    if any(not math.isfinite(value) or value <= 0.0 for value in extents):
        raise M8SensorEvidenceError("object extent_m must be positive")
    corners = _box_corners(pose, extents)
    world_from_ego = _pose_matrix(ego_pose)
    world_from_camera = _matmul(world_from_ego, camera["sensor_to_ego"])
    camera_from_world = _inverse_rigid(world_from_camera)
    points = [_transform_point(camera_from_world, corner) for corner in corners]
    front = [point for point in points if point[2] > 0.05]
    if not front:
        return None
    intrinsic = camera["matrix"]
    pixels = [
        [
            intrinsic[0][0] * point[0] / point[2] + intrinsic[0][2],
            intrinsic[1][1] * point[1] / point[2] + intrinsic[1][2],
        ]
        for point in front
    ]
    min_x, max_x = min(point[0] for point in pixels), max(point[0] for point in pixels)
    min_y, max_y = min(point[1] for point in pixels), max(point[1] for point in pixels)
    width, height = float(camera["width"]), float(camera["height"])
    if max_x < 0.0 or min_x > width or max_y < 0.0 or min_y > height:
        return None
    return {
        "bbox_xyxy_px": [
            max(0.0, min_x),
            max(0.0, min_y),
            min(width, max_x),
            min(height, max_y),
        ],
        "depth_range_m": [
            min(point[2] for point in front),
            max(point[2] for point in front),
        ],
    }


def _box_corners(pose: Mapping[str, float], extent: Sequence[float]) -> list[list[float]]:
    yaw = math.radians(pose["yaw"])
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [
        [
            pose["x"] + cos_yaw * dx - sin_yaw * dy,
            pose["y"] + sin_yaw * dx + cos_yaw * dy,
            pose["z"] + dz,
        ]
        for dx in (-extent[0], extent[0])
        for dy in (-extent[1], extent[1])
        for dz in (-extent[2], extent[2])
    ]


def _pose(value: Any, label: str) -> dict[str, float]:
    if not isinstance(value, Mapping):
        raise M8SensorEvidenceError(f"{label} must be an object")
    try:
        result = {axis: float(value[axis]) for axis in ("x", "y", "z")}
        result["yaw"] = float(value.get("yaw", 0.0))
    except (KeyError, TypeError, ValueError) as exc:
        raise M8SensorEvidenceError(f"{label} is invalid") from exc
    if not all(math.isfinite(item) for item in result.values()):
        raise M8SensorEvidenceError(f"{label} must be finite")
    return result


def _pose_matrix(pose: Mapping[str, float]) -> list[float]:
    yaw = math.radians(pose["yaw"])
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [
        cos_yaw,
        -sin_yaw,
        0.0,
        pose["x"],
        sin_yaw,
        cos_yaw,
        0.0,
        pose["y"],
        0.0,
        0.0,
        1.0,
        pose["z"],
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def _matmul(left: Sequence[float], right: Sequence[float]) -> list[float]:
    return [
        sum(float(left[row * 4 + index]) * float(right[index * 4 + col]) for index in range(4))
        for row in range(4)
        for col in range(4)
    ]


def _inverse_rigid(matrix: Sequence[float]) -> list[float]:
    rotation = [float(matrix[index]) for index in range(16)]
    translation = [rotation[3], rotation[7], rotation[11]]
    inverse = [
        rotation[0], rotation[4], rotation[8], 0.0,
        rotation[1], rotation[5], rotation[9], 0.0,
        rotation[2], rotation[6], rotation[10], 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]
    inverse[3] = -(inverse[0] * translation[0] + inverse[1] * translation[1] + inverse[2] * translation[2])
    inverse[7] = -(inverse[4] * translation[0] + inverse[5] * translation[1] + inverse[6] * translation[2])
    inverse[11] = -(inverse[8] * translation[0] + inverse[9] * translation[1] + inverse[10] * translation[2])
    return inverse


def _transform_point(matrix: Sequence[float], point: Sequence[float]) -> list[float]:
    return [
        sum(float(matrix[row * 4 + index]) * float(point[index]) for index in range(3))
        + float(matrix[row * 4 + 3])
        for row in range(3)
    ]


def _intrinsic(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 3
        and all(isinstance(row, list) and len(row) == 3 for row in value)
        and all(_finite(item) for row in value for item in row)
        and float(value[0][0]) > 0.0
        and float(value[1][1]) > 0.0
    )


def _frame_id(row: Mapping[str, Any]) -> int:
    value = row.get("frame_id")
    if not isinstance(value, int) or isinstance(value, bool):
        raise M8SensorEvidenceError("M8 frame_id must be an integer")
    return value


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _sha256(value: Any) -> bool:
    text = str(value or "")
    return len(text) == 64 and all(character in "0123456789abcdef" for character in text)


def _positive_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _nonempty(value: Any, label: str) -> str:
    text = str(value or "")
    if not text:
        raise M8SensorEvidenceError(f"{label} must be non-empty")
    return text
