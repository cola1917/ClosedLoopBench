"""Build an auditable NuRec render selection from the M6 registry.

The selection is intentionally separate from the CARLA scene-object registry.
CARLA keeps every physical object for collision and lane truth; this module
only decides which source tracks/static representations are eligible for a
NuRec reconstruction candidate.  A visible object with insufficient source
support is reported as a repair blocker, never silently discarded.
"""

from __future__ import annotations

import json
import statistics
from collections.abc import Iterable, Mapping
from typing import Any


class NuRecRenderSelectionError(ValueError):
    """Raised when selection evidence is malformed."""


_DYNAMIC_ROLES = {
    "background_replay",
    "controlled_lead_vehicle",
    "controlled_pedestrian",
}


def build_render_selection(
    registry: Mapping[str, Any],
    visibility_manifest: Mapping[str, Any],
    dynamic_support: Mapping[str, Any],
    static_support: Mapping[str, Any],
    *,
    mandatory_object_ids: Iterable[str] = (),
    max_corridor_range_m: float = 80.0,
    min_padded_coverage: float = 0.75,
    min_exact_coverage: float = 0.50,
) -> dict[str, Any]:
    """Return a deterministic ``nurec_render_selection.v1`` manifest.

    Visibility observations are used as a conservative corridor proxy when a
    route polyline is not available: the union of all calibrated observations
    in the required tick window.  The manifest records that limitation so a
    route-aware builder can replace it later without changing the contract.
    """

    _validate_registry(registry)
    if not isinstance(visibility_manifest, Mapping) or not isinstance(
        visibility_manifest.get("observations"), list
    ):
        raise NuRecRenderSelectionError("visibility manifest observations are required")
    if not 0 < float(max_corridor_range_m):
        raise NuRecRenderSelectionError("max_corridor_range_m must be positive")
    if not 0 <= float(min_padded_coverage) <= 1 or not 0 <= float(min_exact_coverage) <= 1:
        raise NuRecRenderSelectionError("coverage thresholds must be within [0, 1]")

    records = {str(row["object_id"]): row for row in registry["records"]}
    mandatory = {str(value) for value in mandatory_object_ids if str(value)}
    unknown_mandatory = sorted(mandatory - set(records))
    if unknown_mandatory:
        raise NuRecRenderSelectionError(
            "mandatory object IDs are absent from registry: " + ", ".join(unknown_mandatory)
        )
    for row in registry["records"]:
        if row.get("role") in {"controlled_lead_vehicle", "controlled_pedestrian", "road_boundary"}:
            mandatory.add(str(row["object_id"]))

    visibility = _visibility_by_object(visibility_manifest["observations"])
    quality_by_track = {}
    quality_by_track.update(_quality_rows(dynamic_support, "dynamic"))
    quality_by_track.update(_quality_rows(static_support, "static"))

    objects: list[dict[str, Any]] = []
    selected_ids: list[str] = []
    excluded_ids: list[str] = []
    blockers: list[str] = []
    for object_id in sorted(records):
        record = records[object_id]
        observed = visibility.get(object_id, {})
        in_corridor = bool(observed.get("observed")) and float(observed.get("min_distance_m", float("inf"))) <= float(max_corridor_range_m)
        required = object_id in mandatory
        # Dynamic records normally carry the NuRec id in ``nurec.track_id``.
        # Curated static records retain the source identity under
        # ``source.source_track_id`` because they may use generated/static
        # geometry rather than a dynamic NuRec track.  Resolve both forms so
        # static source evidence is not silently classified as missing.
        track_id = _record_track_id(record)
        quality = quality_by_track.get(track_id)
        quality_result = _quality_result(quality, min_padded_coverage, min_exact_coverage)
        if record.get("role") == "road_boundary":
            quality_result = {
                "status": "geometry_required",
                "reason": "road_boundary_is_not_a_lidar_track",
            }
        should_render = required or in_corridor
        if not should_render:
            decision = "excluded_outside_ego_corridor"
            excluded_ids.append(object_id)
        elif quality_result["status"] in {"eligible", "supported", "geometry_required"}:
            decision = "selected"
            selected_ids.append(object_id)
        else:
            decision = "repair_required"
            excluded_ids.append(object_id)
            if required or in_corridor:
                blockers.append(object_id)
        objects.append(
            {
                "object_id": object_id,
                "track_id": track_id or None,
                "source_track_id": str((record.get("source") or {}).get("source_track_id") or "") or None,
                "role": record.get("role"),
                "semantic_class": record.get("semantic_class"),
                "mandatory": required,
                "observed_in_visibility_union": bool(observed.get("observed")),
                "min_distance_m": observed.get("min_distance_m"),
                "visibility_frame_ids": observed.get("frame_ids", []),
                "quality": quality,
                "quality_decision": quality_result,
                "decision": decision,
                "exclusion_audit": None
                if decision == "selected"
                else {
                    "reason": decision,
                    "proof_scope": "three_tick_calibrated_visibility_union",
                    "required_for_carla_physics": bool(record.get("safety_relevant")),
                },
            }
        )

    return {
        "schema_version": "nurec_render_selection.v1",
        "status": "passed" if not blockers else "failed",
        "scene_id": str(registry["scene_id"]),
        "selection_method": "ego_visibility_union_corridor_plus_source_lidar_quality",
        "corridor": {
            "source": "calibrated_visibility_manifest_union",
            "max_range_m": float(max_corridor_range_m),
            "route_swept_union_required": True,
            "note": "Replace visibility proxy with full ego swept route when route samples are available.",
        },
        "quality_thresholds": {
            "min_padded_coverage": float(min_padded_coverage),
            "min_exact_coverage": float(min_exact_coverage),
            "quality_is_not_a_carlarla_physics_filter": True,
        },
        "mandatory_object_ids": sorted(mandatory),
        "selected_track_ids": sorted(
            item["track_id"] for item in objects if item["decision"] == "selected" and item["track_id"]
        ),
        "selected_object_ids": selected_ids,
        "excluded_object_ids": excluded_ids,
        "objects": objects,
        "summary": {
            "registry_object_count": len(records),
            "selected_object_count": len(selected_ids),
            "excluded_object_count": len(excluded_ids),
            "repair_blocker_count": len(blockers),
            "mandatory_count": len(mandatory),
        },
        "blockers": blockers,
    }


def build_lifecycle_quality_manifest(
    registry: Mapping[str, Any],
    dynamic_support: Mapping[str, Any],
    *,
    min_exact_points: int = 1,
    min_padded_points: int = 1,
    min_supported_ticks: int = 1,
) -> dict[str, Any]:
    """Summarize per-tick source LiDAR quality inside each track's lifetime.

    NCore's aggregate counters cannot distinguish a sparse track from a track
    that is well observed during the small interval used by a probe.  This
    manifest consumes the optional ``source_lidar_frames`` evidence emitted by
    ``audit_ncore_dynamic_lidar_points`` and makes that distinction explicit.
    It is an evidence/selection aid only: the complete CARLA object registry
    and its physical actors remain unchanged.

    A supported window is made from consecutive native frame indices whose
    track has at least ``min_exact_points`` exact and ``min_padded_points``
    padded box hits.  Frames outside ``time_interval`` are recorded as
    ``deferred`` and never count against the active quality window.
    """

    _validate_registry(registry)
    if dynamic_support.get("schema_version") != "ncore_dynamic_lidar_support_audit.v2":
        raise NuRecRenderSelectionError(
            "dynamic source support must use ncore_dynamic_lidar_support_audit.v2"
        )
    for value, name in (
        (min_exact_points, "min_exact_points"),
        (min_padded_points, "min_padded_points"),
        (min_supported_ticks, "min_supported_ticks"),
    ):
        if not isinstance(value, int) or isinstance(value, bool) or value < 1:
            raise NuRecRenderSelectionError(f"{name} must be a positive integer")
    if not isinstance(dynamic_support, Mapping):
        raise NuRecRenderSelectionError("dynamic source support report is required")

    raw_frames = dynamic_support.get("source_lidar_frames")
    if raw_frames is None:
        return {
            "schema_version": "nurec_lifecycle_quality_manifest.v1",
            "status": "evidence_unavailable",
            "scene_id": str(registry["scene_id"]),
            "evidence_source": dynamic_support.get("schema_version"),
            "reason": "source_lidar_frames_not_provided",
            "selection_does_not_change_carla_physics": True,
            "objects": [],
            "summary": {
                "registry_dynamic_object_count": 0,
                "supported_object_count": 0,
                "active_object_count": 0,
                "evidence_frame_count": 0,
            },
        }
    if not isinstance(raw_frames, list):
        raise NuRecRenderSelectionError("dynamic source support source_lidar_frames must be a list")

    frames = _lifecycle_support_frames(raw_frames)
    objects: list[dict[str, Any]] = []
    for record in sorted(registry["records"], key=lambda row: str(row["object_id"])):
        track_id = _record_track_id(record)
        # Road boundaries and records without a dynamic source identity have no
        # per-track LiDAR lifecycle to summarize.
        if not track_id or record.get("role") == "road_boundary":
            continue
        lifecycle = _record_lifecycle_window(record)
        if lifecycle is None:
            objects.append(
                {
                    "object_id": str(record["object_id"]),
                    "track_id": track_id,
                    "role": record.get("role"),
                    "semantic_class": record.get("semantic_class"),
                    "lifecycle_window": None,
                    "status": "lifecycle_metadata_missing",
                    "frames": [],
                    "supported_tick_indices": [],
                    "supported_tick_windows": [],
                }
            )
            continue

        frame_rows: list[dict[str, Any]] = []
        supported: list[dict[str, Any]] = []
        for frame in frames:
            timestamp_sec = frame["timestamp_us"] / 1_000_000.0
            active = lifecycle[0] <= timestamp_sec <= lifecycle[1]
            support = frame["tracks"].get(track_id)
            if not active:
                status = "deferred"
                exact = padded = None
                annotation_status = "outside_source_annotation_window"
            elif support is None:
                status = "active_support_missing"
                exact = padded = None
                annotation_status = None
            else:
                annotation_status = str(support.get("annotation_status") or "") or None
                exact = _optional_nonnegative_int(support.get("exact_box_hit_points"), "exact_box_hit_points")
                padded = _optional_nonnegative_int(support.get("padded_box_hit_points"), "padded_box_hit_points")
                if annotation_status == "outside_source_annotation_window":
                    status = "active_annotation_missing"
                elif not support.get("source_cuboid_available", True):
                    status = "active_source_cuboid_missing"
                elif exact is not None and padded is not None and exact >= min_exact_points and padded >= min_padded_points:
                    status = "supported"
                    supported.append(frame)
                else:
                    status = "quality_insufficient"
            frame_rows.append(
                {
                    "frame_index": frame["frame_index"],
                    "timestamp_us": frame["timestamp_us"],
                    "timestamp_sec": timestamp_sec,
                    "active": active,
                    "annotation_status": annotation_status,
                    "exact_box_hit_points": exact,
                    "padded_box_hit_points": padded,
                    "status": status,
                }
            )

        supported_windows = [
            window
            for window in _coalesce_lifecycle_windows(supported)
            if window["tick_count"] >= min_supported_ticks
        ]
        active_count = sum(int(row["active"]) for row in frame_rows)
        supported_count = len(supported)
        if active_count == 0:
            status = "deferred"
        elif supported_count >= min_supported_ticks:
            status = "supported"
        else:
            status = "quality_insufficient"
        objects.append(
            {
                "object_id": str(record["object_id"]),
                "track_id": track_id,
                "role": record.get("role"),
                "semantic_class": record.get("semantic_class"),
                "lifecycle_window": {
                    "start_sec": lifecycle[0],
                    "end_sec": None if lifecycle[1] == float("inf") else lifecycle[1],
                },
                "status": status,
                "active_tick_count": active_count,
                "supported_tick_count": supported_count,
                "supported_tick_indices": [row["frame_index"] for row in supported],
                "supported_tick_windows": supported_windows,
                "frames": frame_rows,
            }
        )

    supported_objects = sum(row["status"] == "supported" for row in objects)
    active_objects = sum(row.get("active_tick_count", 0) > 0 for row in objects)
    manifest_status = (
        "passed"
        if objects and active_objects > 0 and all(row["status"] in {"supported", "deferred"} for row in objects)
        else "deferred"
        if objects and active_objects == 0
        else "failed"
    )
    return {
        "schema_version": "nurec_lifecycle_quality_manifest.v1",
        "status": manifest_status,
        "scene_id": str(registry["scene_id"]),
        "evidence_source": dynamic_support.get("schema_version"),
        "quality_thresholds": {
            "min_exact_points": min_exact_points,
            "min_padded_points": min_padded_points,
            "min_supported_ticks": min_supported_ticks,
        },
        "selection_does_not_change_carla_physics": True,
        "objects": objects,
        "summary": {
            "registry_dynamic_object_count": len(objects),
            "supported_object_count": supported_objects,
            "active_object_count": active_objects,
            "evidence_frame_count": len(frames),
        },
    }


def _lifecycle_support_frames(raw_frames: list[Any]) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    seen: set[int] = set()
    for raw in raw_frames:
        if not isinstance(raw, Mapping):
            raise NuRecRenderSelectionError("source_lidar_frames entries must be objects")
        frame_index = raw.get("source_lidar_frame_index")
        timestamp_us = raw.get("source_lidar_frame_end_us")
        if not isinstance(frame_index, int) or isinstance(frame_index, bool) or frame_index < 0:
            raise NuRecRenderSelectionError("source_lidar_frame_index must be a non-negative integer")
        if not isinstance(timestamp_us, int) or isinstance(timestamp_us, bool) or timestamp_us < 0:
            raise NuRecRenderSelectionError("source_lidar_frame_end_us must be a non-negative integer")
        if frame_index in seen:
            raise NuRecRenderSelectionError("source_lidar_frames contain duplicate frame indices")
        tracks = raw.get("track_support")
        if not isinstance(tracks, list):
            raise NuRecRenderSelectionError("source_lidar_frame track_support must be a list")
        by_track: dict[str, Mapping[str, Any]] = {}
        for row in tracks:
            if not isinstance(row, Mapping) or not str(row.get("track_id") or ""):
                raise NuRecRenderSelectionError("track_support rows require track_id")
            track_id = str(row["track_id"])
            if track_id in by_track:
                raise NuRecRenderSelectionError("track_support contains duplicate track IDs")
            by_track[track_id] = row
        seen.add(frame_index)
        frames.append({"frame_index": frame_index, "timestamp_us": timestamp_us, "tracks": by_track})
    return sorted(frames, key=lambda row: (row["frame_index"], row["timestamp_us"]))


def _record_lifecycle_window(record: Mapping[str, Any]) -> tuple[float, float] | None:
    interval = record.get("time_interval")
    if not isinstance(interval, Mapping) or interval.get("start_sec") is None:
        return None
    try:
        start = float(interval["start_sec"])
        end = float("inf") if interval.get("end_sec") is None else float(interval["end_sec"])
    except (TypeError, ValueError):
        return None
    if start < 0 or end < start:
        return None
    return start, end


def _optional_nonnegative_int(value: Any, name: str) -> int | None:
    if value is None:
        return None
    return _nonnegative_int(value, name)


def _coalesce_lifecycle_windows(rows: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    if not rows:
        return []
    windows: list[list[Mapping[str, Any]]] = []
    for row in rows:
        if not windows or row["frame_index"] != windows[-1][-1]["frame_index"] + 1:
            windows.append([row])
        else:
            windows[-1].append(row)
    return [
        {
            "start_frame_index": group[0]["frame_index"],
            "end_frame_index": group[-1]["frame_index"],
            "start_timestamp_us": group[0]["timestamp_us"],
            "end_timestamp_us": group[-1]["timestamp_us"],
            "start_sec": group[0]["timestamp_us"] / 1_000_000.0,
            "end_sec": group[-1]["timestamp_us"] / 1_000_000.0,
            "tick_count": len(group),
        }
        for group in windows
    ]


def _quality_rows(report: Mapping[str, Any], kind: str) -> dict[str, dict[str, Any]]:
    if not isinstance(report, Mapping) or not isinstance(report.get("tracks"), list):
        raise NuRecRenderSelectionError(f"{kind} source support report tracks are required")
    result: dict[str, dict[str, Any]] = {}
    for row in report["tracks"]:
        if not isinstance(row, Mapping) or not str(row.get("track_id") or ""):
            raise NuRecRenderSelectionError(f"{kind} source support has an invalid track row")
        matched = _nonnegative_int(row.get("matched_lidar_frame_count"), "matched_lidar_frame_count")
        exact_nonzero = _nonnegative_int(row.get("nonzero_exact_box_frame_count"), "nonzero_exact_box_frame_count")
        padded_nonzero = _nonnegative_int(row.get("nonzero_padded_box_frame_count"), "nonzero_padded_box_frame_count")
        if exact_nonzero > matched or padded_nonzero > matched:
            raise NuRecRenderSelectionError(f"{kind} source support coverage exceeds frame count")
        result[str(row["track_id"])] = {
            "kind": kind,
            "status": row.get("status"),
            "matched_frame_count": matched,
            "exact_nonzero_frame_count": exact_nonzero,
            "padded_nonzero_frame_count": padded_nonzero,
            "exact_coverage": exact_nonzero / matched if matched else 0.0,
            "padded_coverage": padded_nonzero / matched if matched else 0.0,
            "sum_exact_box_hit_points": _nonnegative_int(row.get("sum_exact_box_hit_points"), "sum_exact_box_hit_points"),
            "sum_padded_box_hit_points": _nonnegative_int(row.get("sum_padded_box_hit_points"), "sum_padded_box_hit_points"),
            "max_exact_box_hit_points": _nonnegative_int(row.get("max_exact_box_hit_points"), "max_exact_box_hit_points"),
            "max_padded_box_hit_points": _nonnegative_int(row.get("max_padded_box_hit_points"), "max_padded_box_hit_points"),
            "frame_point_count_stats": {
                "min": None,
                "median": None,
                "max": _nonnegative_int(row.get("max_padded_box_hit_points"), "max_padded_box_hit_points"),
                "sum": _nonnegative_int(row.get("sum_padded_box_hit_points"), "sum_padded_box_hit_points"),
                "source": "NCore aggregate report does not expose per-frame distribution",
            },
        }
    return result


def _quality_result(quality: Mapping[str, Any] | None, min_padded: float, min_exact: float) -> dict[str, Any]:
    if quality is None:
        return {"status": "quality_insufficient", "reason": "no_source_lidar_support_report"}
    if quality["matched_frame_count"] <= 0 or quality["sum_padded_box_hit_points"] <= 0:
        return {"status": "quality_insufficient", "reason": "no_source_lidar_occupancy", "quality_status": quality["status"]}
    if quality["padded_coverage"] < min_padded or quality["exact_coverage"] < min_exact:
        return {
            "status": "quality_insufficient",
            "reason": "source_lidar_coverage_below_threshold",
            "quality_status": quality["status"],
            "padded_coverage": quality["padded_coverage"],
            "exact_coverage": quality["exact_coverage"],
        }
    return {
        "status": "supported" if quality["status"] == "ncore_dynamic_lidar_supported" else "eligible",
        "quality_status": quality["status"],
        "padded_coverage": quality["padded_coverage"],
        "exact_coverage": quality["exact_coverage"],
    }


def _record_track_id(record: Mapping[str, Any]) -> str:
    """Resolve the source identity used by either dynamic or static records."""

    nurec = record.get("nurec")
    if isinstance(nurec, Mapping):
        value = str(nurec.get("track_id") or "")
        if value:
            return value
    source = record.get("source")
    if isinstance(source, Mapping):
        return str(source.get("source_track_id") or "")
    return ""


def _visibility_by_object(observations: list[Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for row in observations:
        if not isinstance(row, Mapping) or row.get("expected_visible") is not True:
            continue
        object_id = str(row.get("object_id") or "")
        if not object_id:
            continue
        projection = row.get("projection")
        distance = projection.get("distance_to_ego_m") if isinstance(projection, Mapping) else None
        try:
            distance_value = float(distance)
        except (TypeError, ValueError):
            continue
        current = result.setdefault(object_id, {"observed": True, "min_distance_m": distance_value, "frame_ids": []})
        current["min_distance_m"] = min(float(current["min_distance_m"]), distance_value)
        frame_id = row.get("frame_id")
        if isinstance(frame_id, int) and frame_id not in current["frame_ids"]:
            current["frame_ids"].append(frame_id)
    for row in result.values():
        row["frame_ids"].sort()
    return result


def _nonnegative_int(value: Any, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise NuRecRenderSelectionError(f"{name} must be a non-negative integer")
    return int(value)


def _validate_registry(registry: Mapping[str, Any]) -> None:
    if registry.get("schema_version") != "scene_object_registry.v1":
        raise NuRecRenderSelectionError("registry must use scene_object_registry.v1")
    if not str(registry.get("scene_id") or ""):
        raise NuRecRenderSelectionError("registry scene_id is required")
    if not isinstance(registry.get("records"), list) or not registry["records"]:
        raise NuRecRenderSelectionError("registry.records must be non-empty")
    ids = []
    for row in registry["records"]:
        if not isinstance(row, Mapping) or not str(row.get("object_id") or ""):
            raise NuRecRenderSelectionError("registry records require object_id")
        ids.append(str(row["object_id"]))
    if len(ids) != len(set(ids)):
        raise NuRecRenderSelectionError("registry object IDs must be unique")


__all__ = [
    "NuRecRenderSelectionError",
    "build_lifecycle_quality_manifest",
    "build_render_selection",
]
