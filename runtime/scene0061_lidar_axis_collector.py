"""Construct fail-closed Scene-0061 LiDAR axis evidence from real captures.

This module intentionally does *not* infer an axis convention from a NuRec
response or from the source calibration.  It requires a same-frame native
CARLA LiDAR capture, re-hashes both byte streams, and only emits anchors where
the configured NuRec transform agrees with independently captured CARLA points.

The native capture is expected to be written by the live CARLA capture hook as
``scene0061_carla_native_lidar_capture.v1``.  A missing capture, an unrelated
frame, or fewer than four non-coplanar matches is an error, not a partial pass.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
from pathlib import Path
from typing import Any, Mapping

from agents.plugin_contract import canonical_sha256
from runtime.scene0061_lidar_axis_gate import (
    AXIS_EVIDENCE_SCHEMA,
    XYZI_ENCODING,
    LiDARAxisEvidenceError,
    _has_non_coplanar_anchor_set,
    _observed_capture_sensor_to_ego,
    _transform_point,
    _validated_rigid_matrix,
)
from runtime.scene0061_lidar_axis_normalization import (
    LiDARAxisNormalizationError,
    validate_lidar_axis_normalization,
    verify_normalized_lidar_payload,
)


NATIVE_CAPTURE_SCHEMA = "scene0061_carla_native_lidar_capture.v1"
COORDINATE_EVIDENCE_SCHEMA = "scene0061_lidar_coordinate_validation.v1"


class LiDARAxisCollectionError(ValueError):
    """Raised when a physical axis claim cannot be reproduced from inputs."""


def collect_lidar_axis_evidence(
    *,
    run_config: Mapping[str, Any],
    nurec_evidence: Mapping[str, Any],
    frame_trace: Mapping[str, Any],
    native_capture_path: Path,
    native_scan_manifest_path: Path,
    tolerance_m: float = 0.05,
) -> dict[str, Any]:
    """Build a coordinate-evidence document from immutable physical inputs.

    Matching happens in CARLA ego coordinates.  A candidate NuRec point is first
    transformed by the *declared* ``sensor_to_ego`` matrix.  It must agree with
    a point in the independently captured CARLA native LiDAR stream.  The gate
    replays this calculation later; no field in the emitted JSON is trusted on
    its own.
    """

    if not isinstance(tolerance_m, (int, float)) or isinstance(tolerance_m, bool):
        raise LiDARAxisCollectionError("tolerance_m must be numeric")
    tolerance = float(tolerance_m)
    if not math.isfinite(tolerance) or not 0.0 < tolerance <= 0.05:
        raise LiDARAxisCollectionError("tolerance_m must be within (0, 0.05]")

    lidar_spec = _lidar_spec(run_config)
    matrix = _validated_rigid_matrix(lidar_spec.get("sensor_to_ego"))
    frame_id = _frame_id(nurec_evidence, "NuRec evidence")
    # Frame zero is a valid CARLA frame and must not be discarded by a truthy
    # fallback.  Prefer the post-tick frame when it is explicitly present.
    trace_frame = frame_trace.get("world_tick_frame")
    if not isinstance(trace_frame, int) or isinstance(trace_frame, bool):
        trace_frame = frame_trace.get("snapshot_frame")
    if trace_frame != frame_id:
        raise LiDARAxisCollectionError("CARLA frame trace does not match NuRec LiDAR frame")

    record = _live_lidar_record(nurec_evidence)
    payload_ref = _materialized_lidar_ref(record, frame_id)
    normalization = _verified_response_axis_normalization(
        run_config=run_config, record=record, normalized_payload_ref=payload_ref
    )
    nurec_points = _read_xyzi(Path(payload_ref["path"]))
    live_count = (record.get("response_metadata") or {}).get("point_count")
    if live_count != len(nurec_points):
        raise LiDARAxisCollectionError("NuRec LiDAR point count does not match materialized XYZI")

    native_capture_path = native_capture_path.expanduser().resolve()
    capture = _load_capture(native_capture_path, frame_id)
    capture_ref = _file_ref(native_capture_path, frame_id=frame_id)
    capture_points_ref = _capture_points_ref(capture, frame_id)
    capture_points = _read_xyzi(Path(capture_points_ref["path"]))
    # Do not treat a JSON relative pose as an observation.  The capture is
    # independent only when it carries both CARLA actor world transforms and
    # its recorded relative pose re-derives from those observations.
    try:
        capture_matrix = _observed_capture_sensor_to_ego(capture)
    except LiDARAxisEvidenceError as exc:
        raise LiDARAxisCollectionError(str(exc)) from exc

    anchors = _select_anchors(
        nurec_points=nurec_points,
        nurec_to_ego=matrix,
        capture_points=capture_points,
        capture_to_ego=capture_matrix,
        tolerance_m=tolerance,
    )
    if len(anchors) < 4 or not _has_non_coplanar_anchor_set(
        [tuple(anchor["sensor_local_point_m"]) for anchor in anchors]
    ):
        raise LiDARAxisCollectionError(
            "fewer than four non-coplanar NRE-to-native-CARLA LiDAR matches"
        )

    residuals = [float(anchor["match_distance_m"]) for anchor in anchors]
    max_abs = max(
        max(abs(left - right) for left, right in zip(
            _transform_point(matrix, tuple(anchor["sensor_local_point_m"])),
            tuple(anchor["carla_ego_point_m"]),
        ))
        for anchor in anchors
    )
    evidence = {
        "schema_version": COORDINATE_EVIDENCE_SCHEMA,
        "status": "passed",
        "scene_id": str((run_config.get("experiment") or {}).get("scene_id") or ""),
        "runtime_scene_id": str((run_config.get("nurec_runtime") or {}).get("runtime_scene_id") or ""),
        "artifact_sha256": str((run_config.get("experiment") or {}).get("artifact_sha256") or ""),
        "sensor_id": "lidar_top",
        "device_type": str(lidar_spec.get("model") or lidar_spec.get("device_type") or ""),
        "response_coordinate_frame": "sensor_local",
        "axis_convention": "carla_sensor",
        "sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
        "sensor_to_ego": matrix,
        "sensor_to_ego_sha256": canonical_sha256(matrix),
        "live_render_lidar": {
            "status": "passed",
            "rpc_status": "ok",
            "payload_sha256_valid": True,
            "point_count": len(nurec_points),
            "carla_frame_id": frame_id,
            "materialized_payload": payload_ref,
        },
        "axis_validation": {
            "schema_version": AXIS_EVIDENCE_SCHEMA,
            "status": "passed",
            "evidence_source": NATIVE_CAPTURE_SCHEMA,
            "carla_frame_id": frame_id,
            "tolerance_m": tolerance,
            "measured_max_abs_error_m": max_abs,
            "measured_rms_error_m": math.sqrt(sum(value * value for value in residuals) / len(residuals)),
            "payload_ref": payload_ref,
            **({"response_axis_normalization": normalization} if normalization else {}),
            "native_scan_manifest_ref": _file_ref(native_scan_manifest_path, frame_id=frame_id, scan_index=_native_scan_index(nurec_evidence)),
            "independent_carla_capture_ref": capture_ref,
            "independent_carla_points_ref": capture_points_ref,
            "anchors": anchors,
        },
    }
    return evidence


def _verified_response_axis_normalization(
    *,
    run_config: Mapping[str, Any],
    record: Mapping[str, Any],
    normalized_payload_ref: Mapping[str, Any],
) -> dict[str, Any] | None:
    runtime = run_config.get("nurec_runtime")
    declared = runtime.get("lidar_axis_normalization") if isinstance(runtime, Mapping) else None
    metadata = record.get("response_metadata")
    if declared is None:
        return None
    if not isinstance(metadata, Mapping):
        raise LiDARAxisCollectionError("NuRec LiDAR record has no response metadata")
    try:
        expected = validate_lidar_axis_normalization(declared)
        observed = metadata.get("axis_normalization")
        if observed != expected:
            raise LiDARAxisCollectionError(
                "NuRec LiDAR response normalization does not match runtime config"
            )
        verified = verify_normalized_lidar_payload(
            raw_response_payload=metadata.get("raw_response_payload"),
            normalized_payload={
                **dict(normalized_payload_ref),
                "coordinate_frame": metadata.get("materialized_payload", {}).get("coordinate_frame"),
                "axis_convention": metadata.get("materialized_payload", {}).get("axis_convention"),
            },
            normalization=observed,
        )
    except LiDARAxisNormalizationError as exc:
        raise LiDARAxisCollectionError(str(exc)) from exc
    return verified


def _select_anchors(*, nurec_points: list[tuple[float, float, float, float]], nurec_to_ego: list[float], capture_points: list[tuple[float, float, float, float]], capture_to_ego: list[float], tolerance_m: float) -> list[dict[str, Any]]:
    """Choose deterministic distinct matches using a bounded spatial index.

    The two physical streams can each contain tens or hundreds of thousands of
    points.  Scanning the entire native cloud for every NRE point makes the
    evidence collector quadratic and turns a one-tick diagnostic into an
    unbounded runtime risk.  A grid with cell width equal to the acceptance
    tolerance only needs the 27 cells surrounding each candidate point; it
    retains the previous nearest-distance, then lowest-index tie break.
    """

    capture_ego = [_transform_point(capture_to_ego, point[:3]) for point in capture_points]
    spatial_index: dict[tuple[int, int, int], list[int]] = {}
    for index, point in enumerate(capture_ego):
        spatial_index.setdefault(_spatial_cell(point, tolerance_m), []).append(index)
    selected: list[dict[str, Any]] = []
    used_capture: set[int] = set()
    for source_index, point in enumerate(nurec_points):
        expected = _transform_point(nurec_to_ego, point[:3])
        cell = _spatial_cell(expected, tolerance_m)
        nearest_index, distance = min(
            (
                (index, math.dist(expected, observed))
                for offset_x in (-1, 0, 1)
                for offset_y in (-1, 0, 1)
                for offset_z in (-1, 0, 1)
                for index in spatial_index.get(
                    (cell[0] + offset_x, cell[1] + offset_y, cell[2] + offset_z), []
                )
                if index not in used_capture
                for observed in (capture_ego[index],)
            ),
            key=lambda row: (row[1], row[0]),
            default=(-1, math.inf),
        )
        if nearest_index < 0 or distance > tolerance_m:
            continue
        candidate = {
            "source_point_index": source_index,
            "sensor_local_point_m": [float(value) for value in point[:3]],
            "carla_ego_point_m": [float(value) for value in capture_ego[nearest_index]],
            "independent_capture_point_index": nearest_index,
            "ground_truth_source": "same_frame_carla_native_lidar",
            "match_distance_m": float(distance),
        }
        selected.append(candidate)
        used_capture.add(nearest_index)
        if len(selected) >= 16:
            break
    return selected


def _spatial_cell(point: tuple[float, float, float], width: float) -> tuple[int, int, int]:
    return tuple(math.floor(value / width) for value in point)  # type: ignore[return-value]


def _lidar_spec(run_config: Mapping[str, Any]) -> Mapping[str, Any]:
    specs = (run_config.get("nurec_runtime") or {}).get("lidar_specs")
    rows = [row for row in specs or [] if isinstance(row, Mapping) and row.get("sensor_id") == "lidar_top"]
    if len(rows) != 1:
        raise LiDARAxisCollectionError("run config requires exactly one lidar_top spec")
    return rows[0]


def _live_lidar_record(evidence: Mapping[str, Any]) -> Mapping[str, Any]:
    rows = [row for row in evidence.get("records") or [] if isinstance(row, Mapping) and row.get("modality") == "lidar" and row.get("sensor_id") == "lidar_top" and row.get("status") == "passed"]
    if len(rows) != 1:
        raise LiDARAxisCollectionError("NuRec evidence requires exactly one passed lidar_top record")
    return rows[0]


def _materialized_lidar_ref(record: Mapping[str, Any], frame_id: int) -> dict[str, Any]:
    metadata = record.get("response_metadata")
    value = metadata.get("materialized_payload") if isinstance(metadata, Mapping) else None
    if not isinstance(value, Mapping):
        raise LiDARAxisCollectionError("NuRec LiDAR record has no materialized payload")
    # A materialized payload is a child of the already frame-bound NuRec
    # response record.  The production response schema intentionally keeps
    # file provenance separate from transport/frame metadata, so it need not
    # repeat ``carla_frame_id`` here.  Inherit that identity only from the
    # verified parent trace; an explicitly present, conflicting child value is
    # still a hard failure.
    result = _verified_ref(
        value,
        frame_id=frame_id,
        encoding=XYZI_ENCODING,
        require_declared_frame=False,
    )
    if value.get("coordinate_frame") not in {"unverified", "sensor_local"}:
        raise LiDARAxisCollectionError("NuRec LiDAR payload declares an unexpected coordinate frame")
    return result


def _load_capture(path: Path, frame_id: int) -> Mapping[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise LiDARAxisCollectionError(f"cannot read native CARLA LiDAR capture: {exc}") from exc
    if not isinstance(value, Mapping) or value.get("schema_version") != NATIVE_CAPTURE_SCHEMA:
        raise LiDARAxisCollectionError("unsupported native CARLA LiDAR capture schema")
    if value.get("status") != "passed" or value.get("carla_frame_id") != frame_id:
        raise LiDARAxisCollectionError("native CARLA LiDAR capture is not a passed same-frame capture")
    if value.get("coordinate_frame") != "carla_sensor":
        raise LiDARAxisCollectionError("native CARLA LiDAR capture has no CARLA sensor coordinate declaration")
    return value


def _capture_points_ref(capture: Mapping[str, Any], frame_id: int) -> dict[str, Any]:
    value = capture.get("raw_xyzi_ref")
    if not isinstance(value, Mapping):
        raise LiDARAxisCollectionError("native CARLA LiDAR capture has no raw XYZI reference")
    return _verified_ref(value, frame_id=frame_id, encoding=XYZI_ENCODING)


def _verified_ref(
    value: Mapping[str, Any],
    *,
    frame_id: int,
    encoding: str,
    require_declared_frame: bool = True,
) -> dict[str, Any]:
    path = Path(str(value.get("path") or "")).expanduser().resolve()
    expected = str(value.get("sha256") or "")
    byte_count = value.get("byte_count")
    if not path.is_file() or len(expected) != 64 or any(char not in "0123456789abcdef" for char in expected):
        raise LiDARAxisCollectionError("physical LiDAR file reference is invalid")
    if not isinstance(byte_count, int) or isinstance(byte_count, bool) or byte_count < 1:
        raise LiDARAxisCollectionError("physical LiDAR file reference has invalid byte count")
    declared_frame = value.get("carla_frame_id")
    if (
        (require_declared_frame and declared_frame != frame_id)
        or (not require_declared_frame and declared_frame is not None and declared_frame != frame_id)
        or value.get("encoding") != encoding
    ):
        raise LiDARAxisCollectionError("physical LiDAR file reference has wrong frame or encoding")
    if path.stat().st_size != byte_count or _sha256(path) != expected:
        raise LiDARAxisCollectionError("physical LiDAR file reference does not match bytes")
    return {
        "path": str(path), "sha256": expected, "byte_count": byte_count,
        "encoding": encoding, "carla_frame_id": frame_id,
    }


def _file_ref(path: Path, *, frame_id: int, scan_index: int | None = None) -> dict[str, Any]:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise LiDARAxisCollectionError(f"required evidence file is absent: {path}")
    result: dict[str, Any] = {
        "path": str(path), "sha256": _sha256(path), "byte_count": path.stat().st_size,
        "carla_frame_id": frame_id,
    }
    if scan_index is not None:
        result["scan_index"] = scan_index
    return result


def _frame_id(evidence: Mapping[str, Any], label: str) -> int:
    value = evidence.get("frame_id")
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise LiDARAxisCollectionError(f"{label} has invalid CARLA frame id")
    return value


def _native_scan_index(evidence: Mapping[str, Any]) -> int:
    alignment = (evidence.get("dispatch") or {}).get("temporal_alignment")
    value = alignment.get("native_scan_index") if isinstance(alignment, Mapping) else None
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise LiDARAxisCollectionError("NuRec evidence has no native scan index")
    return value


def _read_xyzi(path: Path) -> list[tuple[float, float, float, float]]:
    data = path.read_bytes()
    if not data or len(data) % 16:
        raise LiDARAxisCollectionError("XYZI payload does not contain complete float32 records")
    rows = [tuple(float(value) for value in row) for row in struct.iter_unpack("<4f", data)]
    if any(not all(math.isfinite(value) for value in row) for row in rows):
        raise LiDARAxisCollectionError("XYZI payload contains non-finite values")
    return rows


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
