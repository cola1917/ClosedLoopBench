"""Fail-closed, per-tick evidence contracts for the Phase 2 M8 gate.

The module intentionally does not infer safety from a zero collision count or
from a successful NuRec request.  Callers must provide the raw CARLA and
independent sensor evidence needed to make each claim inspectable.
"""

from __future__ import annotations

from collections.abc import Mapping
import math
from typing import Any


class SceneSafetyAuditError(ValueError):
    """Raised when an M8 audit input cannot be interpreted safely."""


def audit_collision_tick(
    registry: Mapping[str, Any],
    tick: Mapping[str, Any],
) -> dict[str, Any]:
    """Audit every physically required registry object for one CARLA tick.

    ``tick.object_states`` is deliberately explicit.  It prevents a post-hoc
    audit from silently treating a source annotation or a NRE projection as a
    physical CARLA object.  A state has a CARLA runtime actor id, a centre pose
    and a positive axis-aligned bounding-box extent.
    """

    frame_id, t_sec = _tick_identity(tick)
    ego = _box(tick.get("ego_state"), "ego_state")
    states = _states_by_object(tick.get("object_states"))
    event_object_ids = _collision_event_object_ids(tick.get("collision_events"))
    rows = []
    issues = []
    for record in _active_required_records(registry, t_sec):
        object_id = str(record["object_id"])
        state = states.get(object_id)
        if state is None:
            rows.append({"object_id": object_id, "status": "failed", "issues": ["missing_carla_object_state"]})
            issues.append(f"missing_carla_object_state:{object_id}")
            continue
        obstacle = _box(state, f"object_state:{object_id}")
        clearance, overlap = _box_clearance(ego, obstacle)
        event_contact = object_id in event_object_ids
        rows.append(
            {
                "object_id": object_id,
                "carla_runtime_actor_id": state.get("carla_runtime_actor_id"),
                "clearance_m": clearance,
                "bounding_box_overlap": overlap,
                "collision_sensor_contact": event_contact,
                "status": "failed" if overlap and not event_contact else "passed",
                "issues": ["unattributed_geometric_overlap"] if overlap and not event_contact else [],
            }
        )
        if overlap and not event_contact:
            issues.append(f"unattributed_geometric_overlap:{object_id}")
    unmatched = sorted(event_object_ids - set(states))
    if unmatched:
        issues.extend(f"collision_event_without_registry_state:{object_id}" for object_id in unmatched)
    if bool(tick.get("collision_detected")) and not event_object_ids:
        issues.append("unattributed_carla_collision")
    return _result("collision_audit.v1", frame_id, t_sec, rows, issues)


def audit_lane_tick(tick: Mapping[str, Any]) -> dict[str, Any]:
    """Validate that lane topology and lane-invasion evidence exist per tick."""

    frame_id, t_sec = _tick_identity(tick)
    lane = tick.get("lane_state")
    issues = []
    if not isinstance(lane, Mapping):
        issues.append("missing_carla_lane_state")
        row = {}
    else:
        row = {
            "road_id": lane.get("road_id"),
            "section_id": lane.get("section_id"),
            "lane_id": lane.get("lane_id"),
            "lane_type": lane.get("lane_type"),
            "is_on_road": lane.get("is_on_road"),
            "signed_centerline_distance_m": lane.get("signed_centerline_distance_m"),
            "signed_boundary_distance_m": lane.get("signed_boundary_distance_m"),
            "route_progress": lane.get("route_progress"),
            "lane_invasion_events": list(lane.get("lane_invasion_events") or []),
            "lane_invasion_sensor_available": lane.get("lane_invasion_sensor_available"),
        }
        required = ("road_id", "lane_id", "is_on_road", "signed_centerline_distance_m", "signed_boundary_distance_m", "route_progress", "lane_invasion_sensor_available")
        issues.extend(f"missing_lane_field:{name}" for name in required if row.get(name) is None)
        if row.get("lane_invasion_sensor_available") is False:
            issues.append("lane_invasion_sensor_unavailable")
        if row.get("is_on_road") is False:
            issues.append("off_road")
        if row.get("lane_invasion_events"):
            issues.append("lane_invasion")
    row["status"] = "failed" if issues else "passed"
    row["issues"] = sorted(set(issues))
    return _result("lane_audit.v1", frame_id, t_sec, [row], issues)


def audit_visibility_tick(tick: Mapping[str, Any]) -> dict[str, Any]:
    """Require payload-bound, calibrated physical-box projections per tick."""

    frame_id, t_sec = _tick_identity(tick)
    projections = tick.get("projections")
    if not isinstance(projections, list):
        raise SceneSafetyAuditError("visibility audit requires a projections list")
    rows = []
    issues = []
    for projection in projections:
        if not isinstance(projection, Mapping):
            raise SceneSafetyAuditError("visibility projection must be an object")
        object_id = _nonempty(projection.get("object_id"), "projection.object_id")
        camera = _nonempty(projection.get("camera"), "projection.camera")
        expected = bool(projection.get("expected_visible", True))
        row_issues = []
        evidence = projection.get("evidence")
        calibrated = (
            projection.get("observation_kind") == "calibrated_3d_box_projection"
            and isinstance(projection.get("projection"), Mapping)
            and isinstance(evidence, Mapping)
            and len(str(evidence.get("nre_payload_sha256") or "")) == 64
            and bool(str(evidence.get("calibrated_sensor_token") or ""))
            and len(str(evidence.get("intrinsics_table_sha256") or "")) == 64
        )
        if expected and not calibrated:
            row_issues.append("missing_calibrated_same_frame_projection")
            issues.append(f"visibility_uncalibrated:{object_id}:{camera}")
        rows.append({"object_id": object_id, "camera": camera, "expected_visible": expected, "calibrated_projection": calibrated, "status": "failed" if row_issues else "passed", "issues": row_issues})
    return _result("visibility_audit.v1", frame_id, t_sec, rows, issues)


def audit_lidar_world_tick(tick: Mapping[str, Any]) -> dict[str, Any]:
    """Require declared LiDAR support for every world object expected in scan."""

    frame_id, t_sec = _tick_identity(tick)
    expected = tick.get("expected_world_objects")
    support = tick.get("lidar_occupancy")
    if not isinstance(expected, list) or not isinstance(support, list):
        raise SceneSafetyAuditError("LiDAR world audit requires expected_world_objects and lidar_occupancy lists")
    support_by_object = {
        _nonempty(item.get("object_id"), "lidar_occupancy.object_id"): item
        for item in support
        if isinstance(item, Mapping)
    }
    rows = []
    issues = []
    for item in expected:
        if not isinstance(item, Mapping):
            raise SceneSafetyAuditError("expected world object must be an object")
        object_id = _nonempty(item.get("object_id"), "expected_world_objects.object_id")
        declared = item.get("expected_lidar_support")
        if not isinstance(declared, bool):
            raise SceneSafetyAuditError(
                "expected_world_objects.expected_lidar_support must be explicit"
            )
        required = declared
        observed = support_by_object.get(object_id)
        point_count = observed.get("point_count") if isinstance(observed, Mapping) else None
        valid = isinstance(point_count, int) and not isinstance(point_count, bool) and point_count > 0
        row_issues = []
        observability = item.get("source_lidar_observability")
        if isinstance(observability, Mapping):
            source_issues = observability.get("issues")
            if isinstance(source_issues, list):
                row_issues.extend(str(issue) for issue in source_issues if issue)
                issues.extend(f"{issue}:{object_id}" for issue in row_issues)
        if required and not valid:
            row_issues.append("missing_lidar_world_occupancy")
            issues.append(f"missing_lidar_world_occupancy:{object_id}")
        rows.append({"object_id": object_id, "expected_lidar_support": required, "point_count": point_count, "status": "failed" if row_issues else "passed", "issues": row_issues})
    return _result("lidar_world_audit.v1", frame_id, t_sec, rows, issues)


def _active_required_records(registry: Mapping[str, Any], t_sec: float) -> list[Mapping[str, Any]]:
    records = registry.get("records")
    if not isinstance(records, list):
        raise SceneSafetyAuditError("registry requires a records list")
    result = []
    for record in records:
        if not isinstance(record, Mapping) or record.get("role") == "road_boundary":
            continue
        if (record.get("carla") or {}).get("collision_policy") != "required":
            continue
        interval = record.get("time_interval") or {}
        start = interval.get("start_sec", 0.0) if isinstance(interval, Mapping) else 0.0
        end = interval.get("end_sec") if isinstance(interval, Mapping) else None
        if float(start) <= t_sec and (end is None or t_sec <= float(end)):
            _nonempty(record.get("object_id"), "registry.object_id")
            result.append(record)
    return result


def _states_by_object(value: Any) -> dict[str, Mapping[str, Any]]:
    if not isinstance(value, list):
        raise SceneSafetyAuditError("collision audit requires an object_states list")
    result = {}
    for item in value:
        if not isinstance(item, Mapping):
            raise SceneSafetyAuditError("object state must be an object")
        object_id = _nonempty(item.get("object_id"), "object_state.object_id")
        if object_id in result:
            raise SceneSafetyAuditError(f"duplicate object state: {object_id}")
        if not isinstance(item.get("carla_runtime_actor_id"), int):
            raise SceneSafetyAuditError(f"object state has no CARLA runtime id: {object_id}")
        result[object_id] = item
    return result


def _collision_event_object_ids(value: Any) -> set[str]:
    if value is None:
        return set()
    if not isinstance(value, list):
        raise SceneSafetyAuditError("collision_events must be a list")
    return {_nonempty(item.get("object_id"), "collision_event.object_id") for item in value if isinstance(item, Mapping)}


def _box(value: Any, label: str) -> dict[str, float]:
    if not isinstance(value, Mapping):
        raise SceneSafetyAuditError(f"{label} must be an object")
    pose = value.get("pose")
    extent = value.get("extent_m")
    if not isinstance(pose, Mapping) or not isinstance(extent, Mapping):
        raise SceneSafetyAuditError(f"{label} requires pose and extent_m")
    try:
        result = {axis: float(pose[axis]) for axis in ("x", "y", "z")}
        result.update({f"extent_{axis}": float(extent[axis]) for axis in ("x", "y", "z")})
    except (KeyError, TypeError, ValueError) as exc:
        raise SceneSafetyAuditError(f"{label} has invalid pose or extent") from exc
    if not all(math.isfinite(item) for item in result.values()) or any(result[f"extent_{axis}"] <= 0 for axis in ("x", "y", "z")):
        raise SceneSafetyAuditError(f"{label} has non-finite pose or non-positive extent")
    return result


def _box_clearance(left: Mapping[str, float], right: Mapping[str, float]) -> tuple[float, bool]:
    gaps = [abs(left[axis] - right[axis]) - left[f"extent_{axis}"] - right[f"extent_{axis}"] for axis in ("x", "y", "z")]
    overlap = all(gap <= 0.0 for gap in gaps)
    return 0.0 if overlap else math.sqrt(sum(max(0.0, gap) ** 2 for gap in gaps)), overlap


def _tick_identity(tick: Mapping[str, Any]) -> tuple[int, float]:
    frame_id = tick.get("frame_id")
    t_sec = tick.get("simulation_time_sec")
    if not isinstance(frame_id, int) or isinstance(frame_id, bool):
        raise SceneSafetyAuditError("tick.frame_id must be an integer")
    if not isinstance(t_sec, (int, float)) or isinstance(t_sec, bool) or not math.isfinite(float(t_sec)):
        raise SceneSafetyAuditError("tick.simulation_time_sec must be finite")
    return frame_id, float(t_sec)


def _result(schema_version: str, frame_id: int, t_sec: float, records: list[dict[str, Any]], issues: list[str]) -> dict[str, Any]:
    normalized = sorted(set(str(item) for item in issues))
    return {"schema_version": schema_version, "frame_id": frame_id, "simulation_time_sec": t_sec, "records": records, "status": "failed" if normalized else "passed", "issues": normalized}


def _nonempty(value: Any, label: str) -> str:
    result = str(value or "")
    if not result:
        raise SceneSafetyAuditError(f"{label} must be non-empty")
    return result
