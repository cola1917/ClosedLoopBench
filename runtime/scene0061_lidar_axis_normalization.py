"""Strict NRE-rendered-LiDAR to CARLA-sensor axis normalization.

The NRE response stream is retained byte-for-byte as the RPC artifact.  A
separate, explicitly configured proper rotation can produce a second XYZI
stream for consumers which require CARLA sensor axes.  Keeping the operation
here, instead of silently re-labelling a payload, lets the live axis collector
replay the conversion before accepting any physical-coordinate claim.
"""

from __future__ import annotations

import math
import struct
import hashlib
from pathlib import Path
from typing import Any, Mapping

from agents.plugin_contract import canonical_sha256


AXIS_NORMALIZATION_SCHEMA = "nre_lidar_axis_normalization.v1"


class LiDARAxisNormalizationError(ValueError):
    """Raised when a response-to-sensor conversion is not reproducible."""


def validate_lidar_axis_normalization(value: object) -> dict[str, Any]:
    """Validate a declared proper, origin-preserving response rotation.

    The mapping is deliberately a full 4x4 matrix, even though translation is
    prohibited.  This makes its direction unambiguous: it maps an NRE response
    point into the configured CARLA/nuScenes sensor-local basis.
    """

    if not isinstance(value, Mapping):
        raise LiDARAxisNormalizationError("LiDAR axis normalization must be an object")
    if value.get("schema_version") != AXIS_NORMALIZATION_SCHEMA:
        raise LiDARAxisNormalizationError("unsupported LiDAR axis normalization schema")
    required = {
        "source_coordinate_frame": "nre_26_04_lidar_sensor",
        "source_axis_convention": "nre_26_04_render_axes",
        "target_coordinate_frame": "sensor_local",
        "target_axis_convention": "carla_sensor",
    }
    for key, expected in required.items():
        if value.get(key) != expected:
            raise LiDARAxisNormalizationError(
                f"LiDAR axis normalization {key} must be {expected}"
            )
    matrix = _proper_origin_rotation(value.get("response_to_sensor"))
    return {
        "schema_version": AXIS_NORMALIZATION_SCHEMA,
        **required,
        "response_to_sensor": matrix,
        "response_to_sensor_sha256": canonical_sha256(matrix),
    }


def normalize_lidar_xyzi(raw: bytes, response_to_sensor: object) -> bytes:
    """Convert a complete little-endian XYZI stream without altering intensity."""

    matrix = _proper_origin_rotation(response_to_sensor)
    if not isinstance(raw, bytes) or not raw or len(raw) % 16:
        raise LiDARAxisNormalizationError(
            "LiDAR axis normalization requires complete non-empty float32 XYZI bytes"
        )
    result = bytearray(len(raw))
    for index, (x, y, z, intensity) in enumerate(struct.iter_unpack("<4f", raw)):
        if not all(math.isfinite(value) for value in (x, y, z, intensity)):
            raise LiDARAxisNormalizationError("LiDAR XYZI contains non-finite values")
        struct.pack_into(
            "<ffff",
            result,
            index * 16,
            matrix[0] * x + matrix[1] * y + matrix[2] * z,
            matrix[4] * x + matrix[5] * y + matrix[6] * z,
            matrix[8] * x + matrix[9] * y + matrix[10] * z,
            intensity,
        )
    return bytes(result)


def verify_normalized_lidar_payload(
    *,
    raw_response_payload: object,
    normalized_payload: object,
    normalization: object,
) -> dict[str, Any]:
    """Re-hash and replay a declared raw-NRE-to-sensor conversion.

    This deliberately verifies bytes rather than trusting paths or the
    materialisation client.  The caller supplies the parent frame identity,
    because NRE response metadata is intentionally frame-bound by its parent
    trace instead of repeating the field on every file reference.
    """

    contract = validate_lidar_axis_normalization(normalization)
    raw_ref, raw = _read_payload_ref(raw_response_payload, "raw NRE LiDAR payload")
    normalized_ref, normalized = _read_payload_ref(
        normalized_payload, "normalised LiDAR payload"
    )
    if raw_ref.get("encoding") != "float32_xyzi_little_endian" or normalized_ref.get(
        "encoding"
    ) != "float32_xyzi_little_endian":
        raise LiDARAxisNormalizationError(
            "LiDAR axis payloads must use float32_xyzi_little_endian"
        )
    if raw_ref.get("coordinate_frame") != contract["source_coordinate_frame"] or raw_ref.get(
        "axis_convention"
    ) != contract["source_axis_convention"]:
        raise LiDARAxisNormalizationError(
            "raw NRE LiDAR payload coordinate declaration does not match normalization"
        )
    if normalized_ref.get("coordinate_frame") != contract["target_coordinate_frame"] or normalized_ref.get(
        "axis_convention"
    ) != contract["target_axis_convention"]:
        raise LiDARAxisNormalizationError(
            "normalised LiDAR payload coordinate declaration does not match normalization"
        )
    expected = normalize_lidar_xyzi(raw, contract["response_to_sensor"])
    if expected != normalized:
        raise LiDARAxisNormalizationError(
            "normalised LiDAR payload bytes do not replay from raw NRE payload"
        )
    return {
        "normalization": contract,
        "raw_response_payload": raw_ref,
        "normalized_payload": normalized_ref,
        "point_count": len(raw) // 16,
    }


def _read_payload_ref(value: object, label: str) -> tuple[dict[str, Any], bytes]:
    if not isinstance(value, Mapping):
        raise LiDARAxisNormalizationError(f"{label} reference must be an object")
    path = Path(str(value.get("path") or "")).expanduser().resolve()
    expected_sha = str(value.get("sha256") or "")
    byte_count = value.get("byte_count")
    if not path.is_file() or len(expected_sha) != 64 or any(
        character not in "0123456789abcdef" for character in expected_sha
    ):
        raise LiDARAxisNormalizationError(f"{label} path or SHA-256 is invalid")
    if not isinstance(byte_count, int) or isinstance(byte_count, bool) or byte_count < 16:
        raise LiDARAxisNormalizationError(f"{label} byte count is invalid")
    body = path.read_bytes()
    if len(body) != byte_count or hashlib.sha256(body).hexdigest() != expected_sha:
        raise LiDARAxisNormalizationError(f"{label} bytes do not match its reference")
    if len(body) % 16:
        raise LiDARAxisNormalizationError(f"{label} does not contain complete XYZI records")
    return {
        "path": str(path),
        "sha256": expected_sha,
        "byte_count": byte_count,
        "encoding": value.get("encoding"),
        "coordinate_frame": value.get("coordinate_frame"),
        "axis_convention": value.get("axis_convention"),
    }, body


def _proper_origin_rotation(value: object) -> list[float]:
    if not isinstance(value, list) or len(value) != 16:
        raise LiDARAxisNormalizationError(
            "LiDAR response_to_sensor must be a 16-value matrix"
        )
    try:
        matrix = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise LiDARAxisNormalizationError(
            "LiDAR response_to_sensor must be numeric"
        ) from exc
    if not all(math.isfinite(item) for item in matrix):
        raise LiDARAxisNormalizationError(
            "LiDAR response_to_sensor contains non-finite values"
        )
    if any(abs(matrix[index] - expected) > 1e-6 for index, expected in zip((3, 7, 11, 12, 13, 14, 15), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))):
        raise LiDARAxisNormalizationError(
            "LiDAR response_to_sensor must be an origin-preserving homogeneous transform"
        )
    rotation = [matrix[0:3], matrix[4:7], matrix[8:11]]
    for row_index, row in enumerate(rotation):
        for other_index, other in enumerate(rotation):
            expected = 1.0 if row_index == other_index else 0.0
            if abs(sum(left * right for left, right in zip(row, other)) - expected) > 1e-4:
                raise LiDARAxisNormalizationError(
                    "LiDAR response_to_sensor rotation is not orthonormal"
                )
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > 1e-4:
        raise LiDARAxisNormalizationError(
            "LiDAR response_to_sensor rotation determinant must be +1"
        )
    return matrix
