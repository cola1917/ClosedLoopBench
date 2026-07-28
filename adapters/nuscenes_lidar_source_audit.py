"""Audit raw nuScenes LIDAR_TOP support for registered dynamic tracks.

The audit intentionally operates on the source dataset, before NCore/NuRec
conversion.  It reports both the annotation-provided ``num_lidar_pts`` and an
independent count of points inside each annotation box in the calibrated
LIDAR_TOP frame.  A non-empty source count does not prove that a renderable
artifact contains the same content; it only narrows the repair boundary.
"""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path
from typing import Any, Iterable, Mapping

try:  # numpy is present in the remote NuRec environment; keep a stdlib fallback for tests.
    import numpy as np
except ImportError:  # pragma: no cover - exercised only in minimal environments
    np = None  # type: ignore[assignment]


class NuScenesLidarAuditError(ValueError):
    """Raised when the source dataset is incomplete or inconsistent."""


def _load(path: Path) -> list[dict[str, Any]]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise NuScenesLidarAuditError(f"missing nuScenes table: {path}") from exc
    if not isinstance(value, list):
        raise NuScenesLidarAuditError(f"nuScenes table is not a list: {path}")
    return value


def _q_norm(q: Iterable[float]) -> tuple[float, float, float, float]:
    values = tuple(float(v) for v in q)
    if len(values) != 4:
        raise NuScenesLidarAuditError(f"quaternion must have four values: {q!r}")
    norm = math.sqrt(sum(v * v for v in values))
    if norm <= 1e-12:
        raise NuScenesLidarAuditError(f"zero quaternion: {q!r}")
    return tuple(v / norm for v in values)  # type: ignore[return-value]


def _q_conj(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return (q[0], -q[1], -q[2], -q[3])


def _q_mul(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _rotate(q: tuple[float, float, float, float], p: tuple[float, float, float]) -> tuple[float, float, float]:
    pure = (0.0, p[0], p[1], p[2])
    rotated = _q_mul(_q_mul(q, pure), _q_conj(q))
    return rotated[1], rotated[2], rotated[3]


def _sub(a: Iterable[float], b: Iterable[float]) -> tuple[float, float, float]:
    av = tuple(float(v) for v in a)
    bv = tuple(float(v) for v in b)
    return av[0] - bv[0], av[1] - bv[1], av[2] - bv[2]


def _sensor_box(annotation: Mapping[str, Any], ego: Mapping[str, Any], sensor: Mapping[str, Any]) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float, float]]:
    """Return box centre/half-size/orientation in the LIDAR sensor frame."""

    q_ego = _q_norm(ego["rotation"])
    q_sensor = _q_norm(sensor["rotation"])
    q_box = _q_norm(annotation["rotation"])
    center_ego = _rotate(_q_conj(q_ego), _sub(annotation["translation"], ego["translation"]))
    center_sensor = _rotate(_q_conj(q_sensor), _sub(center_ego, sensor["translation"]))
    # NuScenes size is length/width/height; the box convention is centered.
    half_size = tuple(float(v) / 2.0 for v in annotation["size"])
    q_box_sensor = _q_mul(_q_mul(_q_conj(q_sensor), _q_conj(q_ego)), q_box)
    return center_sensor, half_size, _q_norm(q_box_sensor)


def _read_points(path: Path) -> list[tuple[float, float, float]]:
    try:
        payload = path.read_bytes()
    except FileNotFoundError as exc:
        raise NuScenesLidarAuditError(f"missing LIDAR_TOP payload: {path}") from exc
    if len(payload) % 20 != 0:
        raise NuScenesLidarAuditError(f"unexpected point-cloud byte length: {path}")
    if np is not None:
        # Keep the first three fields only; the source layout is x/y/z/intensity/ring_or_time.
        return np.frombuffer(payload, dtype="<f4").reshape(-1, 5)[:, :3].copy()  # type: ignore[return-value]
    values = struct.unpack(f"<{len(payload) // 4}f", payload)
    return [(values[i], values[i + 1], values[i + 2]) for i in range(0, len(values), 5)]


def _rotation_matrix(q: tuple[float, float, float, float]):
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _count_points_in_box(
    points: Iterable[tuple[float, float, float]],
    center: tuple[float, float, float],
    half_size: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> int:
    inverse = _q_conj(orientation)
    if np is not None and isinstance(points, np.ndarray):
        shifted = points - np.asarray(center, dtype=np.float32)
        # For row vectors, local = shifted @ R_inverse.T.
        local = shifted @ np.asarray(_rotation_matrix(inverse), dtype=np.float32).T
        half = np.asarray(half_size, dtype=np.float32) + 1e-5
        return int(np.all(np.abs(local) <= half, axis=1).sum())
    count = 0
    for point in points:
        local = _rotate(inverse, _sub(point, center))
        if all(abs(local[i]) <= half_size[i] + 1e-5 for i in range(3)):
            count += 1
    return count


def _tokens(value: Any) -> list[str]:
    if isinstance(value, list):
        return [str(item) for item in value if str(item)]
    if isinstance(value, str):
        return [item for item in value.split() if item]
    return []


def _registry_tracks(registry: Mapping[str, Any]) -> list[dict[str, Any]]:
    records = registry.get("records")
    if not isinstance(records, list):
        raise NuScenesLidarAuditError("registry.records must be a list")
    tracks: list[dict[str, Any]] = []
    for record in records:
        if not isinstance(record, Mapping) or record.get("role") == "static_obstacle":
            continue
        source = record.get("source") or {}
        track_id = str(source.get("source_track_id") or (record.get("nurec") or {}).get("track_id") or "")
        if not track_id or record.get("role") == "road_boundary":
            continue
        tracks.append({
            "object_id": str(record.get("object_id") or track_id),
            "track_id": track_id,
            "semantic_class": record.get("semantic_class"),
            "category": record.get("category"),
            "annotation_tokens": _tokens(source.get("annotation_tokens")),
        })
    return tracks


def audit_nuscenes_lidar_source(
    dataset_root: str | Path,
    registry: Mapping[str, Any],
    *,
    version: str = "v1.0-mini",
) -> dict[str, Any]:
    root = Path(dataset_root).resolve()
    meta = root / version
    samples = _load(meta / "sample.json")
    sample_data = _load(meta / "sample_data.json")
    annotations = _load(meta / "sample_annotation.json")
    ego_poses = {str(row["token"]): row for row in _load(meta / "ego_pose.json")}
    calibrated = {str(row["token"]): row for row in _load(meta / "calibrated_sensor.json")}
    sensors = {str(row["token"]): row for row in _load(meta / "sensor.json")}
    lidar_sensor_tokens = {
        token for token, row in sensors.items() if row.get("channel") == "LIDAR_TOP"
    }
    lidar_by_sample: dict[str, dict[str, Any]] = {}
    for row in sample_data:
        if (
            row.get("is_key_frame")
            and row.get("sample_token")
            and str(row.get("calibrated_sensor_token")) in calibrated
            and calibrated[str(row["calibrated_sensor_token"])].get("sensor_token") in lidar_sensor_tokens
        ):
            lidar_by_sample[str(row["sample_token"])] = row
    ann_by_token = {str(row["token"]): row for row in annotations}
    sample_by_token = {str(row["token"]): row for row in samples}
    point_cache: dict[str, list[tuple[float, float, float]]] = {}

    track_rows: list[dict[str, Any]] = []
    for track in _registry_tracks(registry):
        observations: list[dict[str, Any]] = []
        missing_annotations: list[str] = []
        for annotation_token in track["annotation_tokens"]:
            annotation = ann_by_token.get(annotation_token)
            if annotation is None:
                missing_annotations.append(annotation_token)
                continue
            sample_token = str(annotation.get("sample_token") or "")
            sample = sample_by_token.get(sample_token)
            lidar = lidar_by_sample.get(sample_token)
            if sample is None or lidar is None:
                observations.append({
                    "annotation_token": annotation_token,
                    "sample_token": sample_token,
                    "timestamp": sample.get("timestamp") if sample else None,
                    "status": "missing_lidar_keyframe",
                    "metadata_num_lidar_pts": int(annotation.get("num_lidar_pts") or 0),
                })
                continue
            lidar_token = str(lidar["token"])
            payload_path = root / str(lidar["filename"])
            if lidar_token not in point_cache:
                point_cache[lidar_token] = _read_points(payload_path)
            ego = ego_poses.get(str(lidar["ego_pose_token"]))
            sensor = calibrated.get(str(lidar["calibrated_sensor_token"]))
            if ego is None or sensor is None:
                raise NuScenesLidarAuditError(f"missing pose/calibration for sample_data {lidar_token}")
            center, half_size, orientation = _sensor_box(annotation, ego, sensor)
            hit_count = _count_points_in_box(point_cache[lidar_token], center, half_size, orientation)
            observations.append({
                "annotation_token": annotation_token,
                "sample_token": sample_token,
                "timestamp": sample.get("timestamp"),
                "status": "measured",
                "lidar_sample_data_token": lidar_token,
                "lidar_filename": str(lidar["filename"]),
                "metadata_num_lidar_pts": int(annotation.get("num_lidar_pts") or 0),
                "computed_box_hit_points": hit_count,
                "total_lidar_points": len(point_cache[lidar_token]),
            })
        measured = [row for row in observations if row.get("status") == "measured"]
        nonzero = [row for row in measured if row.get("computed_box_hit_points", 0) > 0]
        metadata_nonzero = [row for row in observations if row.get("metadata_num_lidar_pts", 0) > 0]
        if missing_annotations or not observations or not measured:
            status = "source_annotation_incomplete"
        elif not nonzero:
            status = "raw_lidar_absent"
        elif len(nonzero) < len(measured):
            status = "raw_lidar_sparse"
        else:
            status = "raw_lidar_supported"
        track_rows.append({
            **track,
            "status": status,
            "annotation_count": len(track["annotation_tokens"]),
            "observation_count": len(observations),
            "measured_observation_count": len(measured),
            "nonzero_observation_count": len(nonzero),
            "metadata_nonzero_observation_count": len(metadata_nonzero),
            "max_computed_box_hit_points": max((row.get("computed_box_hit_points", 0) for row in observations), default=0),
            "sum_computed_box_hit_points": sum(row.get("computed_box_hit_points", 0) for row in observations),
            "missing_annotation_tokens": missing_annotations,
            "observations": observations,
        })

    counts: dict[str, int] = {}
    for row in track_rows:
        counts[row["status"]] = counts.get(row["status"], 0) + 1
    return {
        "schema_version": "nuscenes_lidar_source_audit.v1",
        "status": "passed" if not any(row["status"] in {"raw_lidar_absent", "source_annotation_incomplete"} for row in track_rows) else "failed",
        "scene_id": registry.get("scene_id"),
        "dataset_root": str(root),
        "version": version,
        "method": {
            "box_count": "independent point-in-oriented-box count in calibrated LIDAR_TOP frame",
            "metadata_field": "sample_annotation.num_lidar_pts",
            "keyframe_only": True,
            "point_record_layout": "float32 x,y,z,intensity,ring_or_time",
        },
        "summary": {"track_count": len(track_rows), "status_counts": counts, "keyframe_count": len(lidar_by_sample), "sample_count": len(samples)},
        "tracks": track_rows,
    }


__all__ = ["NuScenesLidarAuditError", "audit_nuscenes_lidar_source"]
