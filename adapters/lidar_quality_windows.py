"""Build editable quality windows for candidate NuRec smoke runs.

This manifest is a *selection aid*, not an M8 gate relaxation.  The complete
CARLA scene-object registry remains authoritative for physics.  An actor is
editable only inside an ``editable_quality_window``: a consecutive run of
same-tick frames that is inside the source lifetime, has a source cuboid and
enough exact/padded LiDAR occupancy.  Only ticks in that window are eligible
for a four-stream RGB/LiDAR/world/collision closed-loop claim.  Sparse or
missing returns are retained as evidence and never turned into a background
object silently.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from typing import Any


class LidarQualityWindowError(ValueError):
    """Raised when lifecycle/window evidence is malformed."""


_DYNAMIC_ROLES = {
    "background_replay",
    "controlled_lead_vehicle",
    "controlled_pedestrian",
}


def build_lidar_quality_window_manifest(
    registry: Mapping[str, Any],
    source_lidar_support: Mapping[str, Any] | Sequence[Mapping[str, Any]],
    *,
    frame_times_sec: Mapping[int | str, float] | None = None,
    candidate_object_ids: Iterable[str] | None = None,
    required_object_ids: Iterable[str] | None = None,
    min_exact_points: int = 1,
    min_padded_points: int = 1,
    min_consecutive_frames: int = 3,
    max_frame_gap_us: int | None = 100_000,
) -> dict[str, Any]:
    """Return a deterministic editable-quality-window manifest.

    ``source_lidar_support`` accepts one report or a sequence of dynamic and
    static ``ncore_*_lidar_support_audit.v2`` reports.  Reports are joined by
    native frame-end timestamp and track ID.  Frame simulation times must be
    present in each source frame row or supplied through ``frame_times_sec``;
    otherwise the result is a failed, actionable manifest rather than an
    inferred lifecycle classification.
    """

    _validate_registry(registry)
    min_exact_points = _positive_int(min_exact_points, "min_exact_points")
    min_padded_points = _positive_int(min_padded_points, "min_padded_points")
    min_consecutive_frames = _positive_int(min_consecutive_frames, "min_consecutive_frames")
    if max_frame_gap_us is not None:
        max_frame_gap_us = _positive_int(max_frame_gap_us, "max_frame_gap_us")

    records = {str(row["object_id"]): row for row in registry["records"]}
    default_candidates = {
        object_id for object_id, record in records.items() if _record_track_id(record)
    }
    candidate_ids = (
        {str(value) for value in candidate_object_ids if str(value)}
        if candidate_object_ids is not None
        else default_candidates
    )
    default_required = {
        object_id
        for object_id in candidate_ids
        if records[object_id].get("role") in {"controlled_lead_vehicle", "controlled_pedestrian"}
    }
    required_ids = (
        {str(value) for value in required_object_ids if str(value)}
        if required_object_ids is not None
        else default_required
    )
    unknown_candidates = sorted(candidate_ids - set(records))
    unknown_required = sorted(required_ids - set(records))
    if unknown_candidates:
        raise LidarQualityWindowError(
            "candidate object IDs are absent from registry: " + ", ".join(unknown_candidates)
        )
    if unknown_required:
        raise LidarQualityWindowError(
            "required object IDs are absent from registry: " + ", ".join(unknown_required)
        )
    if not required_ids.issubset(candidate_ids):
        raise LidarQualityWindowError("required_object_ids must be a subset of candidate_object_ids")

    track_to_object: dict[str, str] = {}
    object_tracks: dict[str, str] = {}
    for object_id in sorted(candidate_ids):
        track_id = _record_track_id(records[object_id])
        if not track_id:
            continue
        prior = track_to_object.get(track_id)
        if prior is not None and prior != object_id:
            raise LidarQualityWindowError(
                f"registry maps source track {track_id} to multiple objects: {prior}, {object_id}"
            )
        track_to_object[track_id] = object_id
        object_tracks[object_id] = track_id

    frames, input_issues = _merge_source_frames(source_lidar_support, frame_times_sec)
    frame_rows = []
    for frame_index, frame in enumerate(frames):
        frame_rows.append(
            {
                "frame_index": frame_index,
                "source_lidar_frame_end_us": frame["end_us"],
                "simulation_time_sec": frame.get("simulation_time_sec"),
                "track_support": frame["track_support"],
            }
        )

    track_reports: list[dict[str, Any]] = []
    all_windows: list[dict[str, Any]] = []
    for object_id in sorted(candidate_ids):
        record = records[object_id]
        track_id = object_tracks.get(object_id)
        per_frame: list[dict[str, Any]] = []
        for frame in frame_rows:
            simulation_time = frame["simulation_time_sec"]
            lifecycle = _lifecycle_state(record, simulation_time)
            source = frame["track_support"].get(track_id) if track_id else None
            per_frame.append(
                _quality_row(
                    object_id=object_id,
                    track_id=track_id,
                    frame=frame,
                    lifecycle=lifecycle,
                    source=source,
                    min_exact_points=min_exact_points,
                    min_padded_points=min_padded_points,
                )
            )
        editable = [row for row in per_frame if row["editable"]]
        windows = _editable_windows(editable, min_consecutive_frames, max_frame_gap_us)
        for window in windows:
            all_windows.append({"object_id": object_id, "track_id": track_id, **window})
        lifecycle_window = _source_window(record)
        issues = []
        if object_id in required_ids and not track_id:
            issues.append("required_object_has_no_source_track")
        if object_id in required_ids and not windows:
            issues.append("required_object_has_no_editable_quality_window")
        track_reports.append(
            {
                "object_id": object_id,
                "track_id": track_id,
                "role": record.get("role"),
                "required": object_id in required_ids,
                "source_annotation_window": lifecycle_window,
                "frame_count": len(per_frame),
                "active_frame_count": sum(row["lifecycle"] == "active" for row in per_frame),
                "editable_frame_count": len(editable),
                "editable_frame_indices": [row["frame_index"] for row in editable],
                "editable_windows": windows,
                "frames": per_frame,
                "issues": issues,
                "status": "passed" if not issues else "failed",
            }
        )

    required_failures = [row["object_id"] for row in track_reports if row["required"] and row["issues"]]
    issues = list(input_issues)
    issues.extend(
        f"{object_id}:required_object_has_no_editable_quality_window"
        for object_id in required_failures
        if f"{object_id}:required_object_has_no_editable_quality_window" not in issues
    )
    return {
        "schema_version": "lidar_quality_window_manifest.v1",
        "status": "passed" if not issues and not required_failures and frames else "failed",
        "scene_id": str(registry["scene_id"]),
        "purpose": "candidate_smoke_editable_quality_window_selection_only",
        "window_semantics": {
            "name": "editable_quality_window",
            "definition": (
                "consecutive same-tick frames inside source lifetime with a source cuboid "
                "and exact/padded LiDAR support above threshold"
            ),
            "lidar_world_closed_loop_claim_allowed_only_inside_window": True,
            "outside_window_policy": (
                "retain_carla_actor_and_collision_state_but_do_not_claim_lidar_world_closure"
            ),
            "required_streams": ["rgb", "lidar", "world", "collision"],
        },
        "policy": {
            "min_exact_points": min_exact_points,
            "min_padded_points": min_padded_points,
            "min_consecutive_frames": min_consecutive_frames,
            "max_frame_gap_us": max_frame_gap_us,
            "quality_is_not_a_carla_physics_filter": True,
            "registry_objects_preserved": True,
        },
        "candidate_object_ids": sorted(candidate_ids),
        "required_object_ids": sorted(required_ids),
        "frames": frame_rows,
        "tracks": track_reports,
        "editable_windows": all_windows,
        "editable_quality_windows": all_windows,
        "summary": {
            "candidate_object_count": len(candidate_ids),
            "required_object_count": len(required_ids),
            "source_frame_count": len(frames),
            "editable_track_count": sum(bool(row["editable_windows"]) for row in track_reports),
            "editable_window_count": len(all_windows),
            "editable_quality_window_count": len(all_windows),
            "required_failure_count": len(required_failures),
            "issue_count": len(issues),
        },
        "issues": issues,
    }


def validate_lidar_quality_window_manifest(
    manifest: Mapping[str, Any],
    *,
    scene_id: str | None = None,
    required_object_ids: Iterable[str] | None = None,
) -> None:
    """Validate a candidate manifest before handing it to a smoke runner."""

    if manifest.get("schema_version") != "lidar_quality_window_manifest.v1":
        raise LidarQualityWindowError("LiDAR quality window manifest schema is invalid")
    if scene_id is not None and str(manifest.get("scene_id") or "") != str(scene_id):
        raise LidarQualityWindowError("LiDAR quality window manifest scene_id does not match registry")
    if manifest.get("status") != "passed":
        raise LidarQualityWindowError("LiDAR quality window manifest is not passed")
    if manifest.get("policy", {}).get("quality_is_not_a_carla_physics_filter") is not True:
        raise LidarQualityWindowError("LiDAR quality manifest must declare physics separation")
    semantics = manifest.get("window_semantics")
    if not isinstance(semantics, Mapping) or semantics.get("name") != "editable_quality_window":
        raise LidarQualityWindowError("LiDAR quality manifest must declare editable_quality_window semantics")
    if semantics.get("lidar_world_closed_loop_claim_allowed_only_inside_window") is not True:
        raise LidarQualityWindowError("LiDAR-world closure must be restricted to editable quality windows")
    candidate = {str(value) for value in manifest.get("candidate_object_ids") or []}
    required = (
        {str(value) for value in required_object_ids if str(value)}
        if required_object_ids is not None
        else {str(value) for value in manifest.get("required_object_ids") or []}
    )
    if not required.issubset(candidate):
        raise LidarQualityWindowError("LiDAR quality manifest required IDs are not candidates")
    track_by_object = {str(row.get("object_id")): row for row in manifest.get("tracks") or []}
    missing = sorted(object_id for object_id in required if not track_by_object.get(object_id, {}).get("editable_windows"))
    if missing:
        raise LidarQualityWindowError(
            "required objects have no editable LiDAR quality window: " + ", ".join(missing)
        )


def _merge_source_frames(
    source_lidar_support: Mapping[str, Any] | Sequence[Mapping[str, Any]],
    frame_times_sec: Mapping[int | str, float] | None,
) -> tuple[list[dict[str, Any]], list[str]]:
    reports = [source_lidar_support] if isinstance(source_lidar_support, Mapping) else list(source_lidar_support)
    if not reports:
        raise LidarQualityWindowError("source LiDAR support reports are required")
    merged: dict[int, dict[str, Any]] = {}
    issues: list[str] = []
    for report in reports:
        if not isinstance(report, Mapping) or report.get("schema_version") not in {
            "ncore_dynamic_lidar_support_audit.v2",
            "ncore_static_lidar_support_audit.v2",
        }:
            raise LidarQualityWindowError("source LiDAR support must use an ncore *_lidar_support_audit.v2 schema")
        raw_frames = report.get("source_lidar_frames")
        if not isinstance(raw_frames, list):
            raise LidarQualityWindowError("source LiDAR support has no source_lidar_frames")
        for raw in raw_frames:
            if not isinstance(raw, Mapping):
                raise LidarQualityWindowError("source LiDAR frame record must be an object")
            end_us = _int_value(raw.get("source_lidar_frame_end_us"), "source_lidar_frame_end_us", nonnegative=True)
            rows = raw.get("track_support")
            if not isinstance(rows, list):
                raise LidarQualityWindowError("source LiDAR frame track_support must be a list")
            target = merged.setdefault(end_us, {"end_us": end_us, "track_support": {}, "simulation_time_sec": None})
            raw_time = raw.get("simulation_time_sec")
            if raw_time is not None:
                target["simulation_time_sec"] = _finite_float(raw_time, "simulation_time_sec")
            for item in rows:
                if not isinstance(item, Mapping) or not str(item.get("track_id") or ""):
                    raise LidarQualityWindowError("source LiDAR track support requires track_id")
                track_id = str(item["track_id"])
                if track_id in target["track_support"]:
                    raise LidarQualityWindowError(f"duplicate source LiDAR support for frame {end_us}, track {track_id}")
                target["track_support"][track_id] = dict(item)
    for end_us, frame in merged.items():
        if frame["simulation_time_sec"] is None and frame_times_sec is not None:
            raw_time = frame_times_sec.get(end_us)
            if raw_time is None:
                raw_time = frame_times_sec.get(str(end_us))
            if raw_time is not None:
                frame["simulation_time_sec"] = _finite_float(raw_time, "frame_times_sec")
        if frame["simulation_time_sec"] is None:
            issues.append(f"frame_{end_us}:simulation_time_sec_missing")
    ordered = sorted(merged.values(), key=lambda row: row["end_us"])
    if not ordered:
        issues.append("source_lidar_frames_empty")
    return ordered, issues


def _quality_row(*, object_id: str, track_id: str | None, frame: Mapping[str, Any], lifecycle: str, source: Mapping[str, Any] | None, min_exact_points: int, min_padded_points: int) -> dict[str, Any]:
    row: dict[str, Any] = {
        "object_id": object_id,
        "track_id": track_id,
        "frame_index": frame["frame_index"],
        "source_lidar_frame_end_us": frame["source_lidar_frame_end_us"],
        "simulation_time_sec": frame["simulation_time_sec"],
        "lifecycle": lifecycle,
        "source_cuboid_available": bool(source and source.get("source_cuboid_available") is True),
        "exact_box_hit_points": None,
        "padded_box_hit_points": None,
        "quality": "outside_lifecycle" if lifecycle != "active" else "missing_source_frame",
        "editable": False,
        "editable_quality_window": False,
        "lidar_world_closed_loop_eligible": False,
        "reason": None,
    }
    if lifecycle == "unknown":
        row["quality"], row["reason"] = "unknown_lifecycle", "simulation_time_sec_missing"
        return row
    if lifecycle != "active":
        row["reason"] = f"lifecycle_{lifecycle}"
        return row
    if source is None:
        row["reason"] = "source_track_not_observed_at_frame"
        return row
    if source.get("source_cuboid_available") is not True:
        row["quality"], row["reason"] = "missing_source_cuboid", source.get("annotation_status") or "source_cuboid_unavailable"
        return row
    exact = _int_value(source.get("exact_box_hit_points"), "exact_box_hit_points", nonnegative=True)
    padded = _int_value(source.get("padded_box_hit_points"), "padded_box_hit_points", nonnegative=True)
    if padded < exact:
        raise LidarQualityWindowError(f"padded LiDAR points are below exact points for {object_id}")
    row["exact_box_hit_points"], row["padded_box_hit_points"] = exact, padded
    if padded < min_padded_points:
        row["quality"], row["reason"] = "unsupported", "padded_points_below_threshold"
    elif exact < min_exact_points:
        row["quality"], row["reason"] = "padded_only", "exact_points_below_threshold"
    else:
        row["quality"], row["editable"], row["reason"] = "exact_supported", True, "quality_thresholds_met"
        row["editable_quality_window"] = True
        row["lidar_world_closed_loop_eligible"] = True
    return row


def _editable_windows(rows: list[Mapping[str, Any]], min_frames: int, max_gap_us: int | None) -> list[dict[str, Any]]:
    if not rows:
        return []
    runs: list[list[Mapping[str, Any]]] = []
    current: list[Mapping[str, Any]] = []
    for row in rows:
        if current:
            gap = int(row["source_lidar_frame_end_us"]) - int(current[-1]["source_lidar_frame_end_us"])
            if max_gap_us is not None and gap > max_gap_us:
                if len(current) >= min_frames:
                    runs.append(current)
                current = []
        current.append(row)
    if len(current) >= min_frames:
        runs.append(current)
    return [
        {
            "window_type": "editable_quality_window",
            "lidar_world_closed_loop_eligible": True,
            "start_frame_index": int(run[0]["frame_index"]),
            "end_frame_index": int(run[-1]["frame_index"]),
            "start_source_lidar_frame_end_us": int(run[0]["source_lidar_frame_end_us"]),
            "end_source_lidar_frame_end_us": int(run[-1]["source_lidar_frame_end_us"]),
            "start_simulation_time_sec": run[0]["simulation_time_sec"],
            "end_simulation_time_sec": run[-1]["simulation_time_sec"],
            "frame_count": len(run),
            "frame_indices": [int(row["frame_index"]) for row in run],
        }
        for run in runs
    ]


def _lifecycle_state(record: Mapping[str, Any], simulation_time: Any) -> str:
    if simulation_time is None:
        return "unknown"
    window = _source_window(record)
    if window is None:
        return "unknown"
    value = float(simulation_time)
    if value < window["start_sec"]:
        return "deferred"
    if value > window["end_sec"]:
        return "despawned"
    return "active"


def _source_window(record: Mapping[str, Any]) -> dict[str, float] | None:
    raw = record.get("time_interval")
    if not isinstance(raw, Mapping):
        return None
    start = _finite_float(raw.get("start_sec"), "time_interval.start_sec")
    end = _finite_float(raw.get("end_sec"), "time_interval.end_sec")
    if end < start:
        raise LidarQualityWindowError(f"registry object {record.get('object_id')} has inverted time_interval")
    return {"start_sec": start, "end_sec": end}


def _record_track_id(record: Mapping[str, Any]) -> str:
    nurec = record.get("nurec")
    if isinstance(nurec, Mapping) and str(nurec.get("track_id") or ""):
        return str(nurec["track_id"])
    source = record.get("source")
    if isinstance(source, Mapping) and str(source.get("source_track_id") or ""):
        return str(source["source_track_id"])
    return ""


def _validate_registry(registry: Mapping[str, Any]) -> None:
    if registry.get("schema_version") != "scene_object_registry.v1":
        raise LidarQualityWindowError("registry must use scene_object_registry.v1")
    if not str(registry.get("scene_id") or ""):
        raise LidarQualityWindowError("registry scene_id is required")
    if not isinstance(registry.get("records"), list) or not registry["records"]:
        raise LidarQualityWindowError("registry.records must be non-empty")
    ids = [str(row.get("object_id") or "") for row in registry["records"] if isinstance(row, Mapping)]
    if len(ids) != len(registry["records"]) or not all(ids) or len(ids) != len(set(ids)):
        raise LidarQualityWindowError("registry records require unique object_id values")


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise LidarQualityWindowError(f"{name} must be a positive integer")
    return int(value)


def _int_value(value: Any, name: str, *, nonnegative: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or (nonnegative and value < 0):
        raise LidarQualityWindowError(f"{name} must be a {'non-negative' if nonnegative else ''} integer")
    return int(value)


def _finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise LidarQualityWindowError(f"{name} must be finite") from exc
    if not math.isfinite(result):
        raise LidarQualityWindowError(f"{name} must be finite")
    return result


__all__ = [
    "LidarQualityWindowError",
    "build_lidar_quality_window_manifest",
    "validate_lidar_quality_window_manifest",
]
