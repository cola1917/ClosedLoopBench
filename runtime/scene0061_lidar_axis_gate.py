"""Fail-closed validation for scene-0061 live LiDAR axis evidence.

The renderer can only be admitted to a TransFuser++ acceptance run after a
remote capture binds actual little-endian XYZI bytes to CARLA-frame anchors.
The evidence deliberately contains enough independently checkable information
to reject a JSON document that merely asserts an axis convention.
"""

from __future__ import annotations

import hashlib
import itertools
import math
import re
import struct
from pathlib import Path
from typing import Any, Mapping


AXIS_EVIDENCE_SCHEMA = "scene0061_lidar_axis_alignment.v1"
XYZI_ENCODING = "float32_xyzi_little_endian"


class LiDARAxisEvidenceError(ValueError):
    """Raised when a LiDAR axis claim is not backed by immutable evidence."""


def validate_lidar_axis_evidence(
    evidence: Mapping[str, Any],
    *,
    sensor_to_ego: object,
    live_render_lidar: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate payload-bound sensor-local-to-CARLA-ego axis evidence.

    Four non-coplanar points from an actual materialized response are matched
    against CARLA-ego anchors using the run-config's rigid ``sensor_to_ego``
    transform.  Payload and native-scan manifest references are re-hashed so
    this remains evidence rather than an unchecked self-report.
    """

    axis = evidence.get("axis_validation")
    if not isinstance(axis, Mapping):
        raise LiDARAxisEvidenceError("axis_validation object is required")
    if axis.get("schema_version") != AXIS_EVIDENCE_SCHEMA:
        raise LiDARAxisEvidenceError("unsupported LiDAR axis evidence schema")
    if axis.get("status") != "passed":
        raise LiDARAxisEvidenceError("LiDAR axis evidence is not passed")

    carla_frame_id = _nonnegative_int(axis.get("carla_frame_id"), "carla_frame_id")
    if live_render_lidar.get("carla_frame_id") != carla_frame_id:
        raise LiDARAxisEvidenceError("LiDAR axis evidence frame is not bound to live response")

    payload_ref = axis.get("payload_ref")
    payload_path = _validate_file_ref(
        payload_ref,
        label="LiDAR XYZI payload",
        expected_frame_id=carla_frame_id,
        required_encoding=XYZI_ENCODING,
    )
    native_scan_ref = axis.get("native_scan_manifest_ref")
    _validate_file_ref(
        native_scan_ref,
        label="native LiDAR scan manifest",
        expected_frame_id=carla_frame_id,
    )
    if not isinstance(native_scan_ref, Mapping):  # Narrow type for static readers.
        raise LiDARAxisEvidenceError("native LiDAR scan manifest reference is required")
    _nonnegative_int(native_scan_ref.get("scan_index"), "native scan index")

    points = _read_xyzi(payload_path)
    if not points:
        raise LiDARAxisEvidenceError("LiDAR axis payload contains no points")
    live_count = live_render_lidar.get("point_count")
    if not isinstance(live_count, int) or isinstance(live_count, bool) or live_count != len(points):
        raise LiDARAxisEvidenceError("LiDAR axis payload point count disagrees with live response")

    matrix = _validated_rigid_matrix(sensor_to_ego)
    tolerance_m = _positive_finite(axis.get("tolerance_m"), "axis tolerance")
    if tolerance_m > 0.05:
        raise LiDARAxisEvidenceError("axis tolerance exceeds 5 cm")
    anchors = axis.get("anchors")
    if not isinstance(anchors, list) or len(anchors) < 4:
        raise LiDARAxisEvidenceError("at least four LiDAR axis anchors are required")

    used_indices: set[int] = set()
    sensor_points: list[tuple[float, float, float]] = []
    residuals: list[float] = []
    max_abs_error = 0.0
    for anchor_index, anchor in enumerate(anchors):
        if not isinstance(anchor, Mapping):
            raise LiDARAxisEvidenceError(f"axis anchor {anchor_index} must be an object")
        point_index = _nonnegative_int(anchor.get("source_point_index"), "axis source point index")
        if point_index >= len(points):
            raise LiDARAxisEvidenceError("axis source point index lies outside XYZI payload")
        if point_index in used_indices:
            raise LiDARAxisEvidenceError("axis anchors must reference distinct XYZI points")
        used_indices.add(point_index)
        observed_sensor = _vector3(anchor.get("sensor_local_point_m"), "sensor-local anchor")
        payload_sensor = points[point_index][:3]
        if max(abs(left - right) for left, right in zip(observed_sensor, payload_sensor)) > 1e-5:
            raise LiDARAxisEvidenceError("axis sensor-local anchor differs from immutable XYZI payload")
        expected_carla = _transform_point(matrix, observed_sensor)
        observed_carla = _vector3(anchor.get("carla_ego_point_m"), "CARLA-ego anchor")
        deltas = [left - right for left, right in zip(expected_carla, observed_carla)]
        residual = math.sqrt(sum(value * value for value in deltas))
        residuals.append(residual)
        max_abs_error = max(max_abs_error, *(abs(value) for value in deltas))
        sensor_points.append(observed_sensor)

    if not _has_non_coplanar_anchor_set(sensor_points):
        raise LiDARAxisEvidenceError("LiDAR axis anchors must include four non-coplanar points")
    if max_abs_error > tolerance_m or max(residuals) > tolerance_m:
        raise LiDARAxisEvidenceError("LiDAR axis anchors disagree with sensor_to_ego transform")

    rms_error = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    reported_max = _nonnegative_finite(axis.get("measured_max_abs_error_m"), "reported max axis error")
    reported_rms = _nonnegative_finite(axis.get("measured_rms_error_m"), "reported RMS axis error")
    if abs(reported_max - max_abs_error) > 1e-6 or abs(reported_rms - rms_error) > 1e-6:
        raise LiDARAxisEvidenceError("reported LiDAR axis residuals do not match anchors")

    return {
        "schema_version": AXIS_EVIDENCE_SCHEMA,
        "carla_frame_id": carla_frame_id,
        "point_count": len(points),
        "anchor_count": len(anchors),
        "max_abs_error_m": max_abs_error,
        "rms_error_m": rms_error,
        "tolerance_m": tolerance_m,
        "status": "passed",
    }


def _validate_file_ref(
    value: object,
    *,
    label: str,
    expected_frame_id: int,
    required_encoding: str | None = None,
) -> Path:
    if not isinstance(value, Mapping):
        raise LiDARAxisEvidenceError(f"{label} reference is required")
    path = Path(str(value.get("path") or ""))
    expected_sha = str(value.get("sha256") or "")
    byte_count = value.get("byte_count")
    if not path.is_file() or re.fullmatch(r"[0-9a-f]{64}", expected_sha) is None:
        raise LiDARAxisEvidenceError(f"{label} path or SHA-256 is invalid")
    if not isinstance(byte_count, int) or isinstance(byte_count, bool) or byte_count < 1:
        raise LiDARAxisEvidenceError(f"{label} byte count is invalid")
    if path.stat().st_size != byte_count or _file_sha256(path) != expected_sha:
        raise LiDARAxisEvidenceError(f"{label} file identity does not match evidence")
    if value.get("carla_frame_id") != expected_frame_id:
        raise LiDARAxisEvidenceError(f"{label} frame does not match CARLA frame")
    if required_encoding is not None and value.get("encoding") != required_encoding:
        raise LiDARAxisEvidenceError(f"{label} encoding is not {required_encoding}")
    return path


def _read_xyzi(path: Path) -> list[tuple[float, float, float, float]]:
    raw = path.read_bytes()
    if not raw or len(raw) % 16:
        raise LiDARAxisEvidenceError("LiDAR XYZI payload must contain complete float32 XYZI records")
    points = [tuple(float(value) for value in row) for row in struct.iter_unpack("<4f", raw)]
    if any(not all(math.isfinite(value) for value in row) for row in points):
        raise LiDARAxisEvidenceError("LiDAR XYZI payload contains non-finite values")
    return points


def _validated_rigid_matrix(value: object) -> list[float]:
    if not isinstance(value, list) or len(value) != 16:
        raise LiDARAxisEvidenceError("LiDAR sensor_to_ego must be a 16-value matrix")
    try:
        matrix = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise LiDARAxisEvidenceError("LiDAR sensor_to_ego must be numeric") from exc
    if not all(math.isfinite(item) for item in matrix):
        raise LiDARAxisEvidenceError("LiDAR sensor_to_ego contains non-finite values")
    if any(abs(matrix[index] - expected) > 1e-6 for index, expected in zip((12, 13, 14, 15), (0.0, 0.0, 0.0, 1.0))):
        raise LiDARAxisEvidenceError("LiDAR sensor_to_ego is not homogeneous")
    rotation = [matrix[0:3], matrix[4:7], matrix[8:11]]
    for row_index, row in enumerate(rotation):
        for column_index, other in enumerate(rotation):
            target = 1.0 if row_index == column_index else 0.0
            if abs(sum(left * right for left, right in zip(row, other)) - target) > 1e-4:
                raise LiDARAxisEvidenceError("LiDAR sensor_to_ego rotation is not orthonormal")
    determinant = _det3(rotation[0], rotation[1], rotation[2])
    if abs(determinant - 1.0) > 1e-4:
        raise LiDARAxisEvidenceError("LiDAR sensor_to_ego rotation determinant must be +1")
    return matrix


def _transform_point(matrix: list[float], point: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = point
    return (
        matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
        matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
        matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
    )


def _has_non_coplanar_anchor_set(points: list[tuple[float, float, float]]) -> bool:
    for first, second, third, fourth in itertools.combinations(points, 4):
        determinant = _det3(
            tuple(second[index] - first[index] for index in range(3)),
            tuple(third[index] - first[index] for index in range(3)),
            tuple(fourth[index] - first[index] for index in range(3)),
        )
        if abs(determinant) > 1e-6:
            return True
    return False


def _det3(first: tuple[float, float, float] | list[float], second: tuple[float, float, float] | list[float], third: tuple[float, float, float] | list[float]) -> float:
    return (
        first[0] * (second[1] * third[2] - second[2] * third[1])
        - first[1] * (second[0] * third[2] - second[2] * third[0])
        + first[2] * (second[0] * third[1] - second[1] * third[0])
    )


def _vector3(value: object, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise LiDARAxisEvidenceError(f"{label} must contain three numeric values")
    try:
        vector = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise LiDARAxisEvidenceError(f"{label} must contain numeric values") from exc
    if not all(math.isfinite(item) for item in vector):
        raise LiDARAxisEvidenceError(f"{label} contains non-finite values")
    return vector


def _nonnegative_int(value: object, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise LiDARAxisEvidenceError(f"{label} must be a non-negative integer")
    return value


def _positive_finite(value: object, label: str) -> float:
    result = _nonnegative_finite(value, label)
    if result <= 0:
        raise LiDARAxisEvidenceError(f"{label} must be positive")
    return result


def _nonnegative_finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) < 0:
        raise LiDARAxisEvidenceError(f"{label} must be a finite non-negative number")
    return float(value)


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
