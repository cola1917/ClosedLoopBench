"""Fail-closed, per-tick safety evidence contracts.

The runtime collision sensor is necessary but not sufficient: callbacks can be
late, un-attributed, or absent while the physical actor boxes already overlap.
This module keeps the raw CARLA contact claim separate from geometric box
evidence and refuses to turn missing evidence into a pass.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from typing import Any


class SceneSafetyAuditError(ValueError):
    """Raised when an audit input cannot be interpreted safely."""


def audit_collision_tick(
    registry: Mapping[str, Any],
    tick: Mapping[str, Any],
) -> dict[str, Any]:
    """Audit required physical objects against the ego box for one tick.

    ``object_states`` must contain the CARLA runtime identity, bounding-box
    centre, yaw, and positive extents.  A geometric overlap without a contact
    event is deliberately a failure because it is exactly the silent miss
    this audit is intended to expose.
    """

    frame_id, simulation_time_sec = _tick_identity(tick)
    ego = _box(tick.get("ego_state"), "ego_state")
    states = _states_by_object(tick.get("object_states"))
    event_object_ids = _collision_event_object_ids(tick.get("collision_events"))
    records: list[dict[str, Any]] = []
    issues: list[str] = []

    registry_records = registry.get("records")
    if not isinstance(registry_records, list):
        raise SceneSafetyAuditError("registry requires a records list")
    registry_object_ids = {
        _nonempty(item.get("object_id"), "registry.object_id")
        for item in registry_records
        if isinstance(item, Mapping)
    }
    for object_id in sorted(set(states) - registry_object_ids):
        issues.append(f"unregistered_carla_object_state:{object_id}")
    for object_id in sorted(event_object_ids - registry_object_ids):
        issues.append(f"collision_event_without_registry_record:{object_id}")

    required_records = _active_required_records(registry, simulation_time_sec)
    for registry_record in required_records:
        object_id = str(registry_record["object_id"])
        state = states.get(object_id)
        if state is None:
            records.append(
                {
                    "object_id": object_id,
                    "status": "failed",
                    "issues": ["missing_carla_object_state"],
                }
            )
            issues.append(f"missing_carla_object_state:{object_id}")
            continue

        obstacle = _box(state, f"object_state:{object_id}")
        geometry = _box_geometry(ego, obstacle)
        event_contact = object_id in event_object_ids
        row_issues: list[str] = []
        if geometry["overlap"] and not event_contact:
            row_issues.append("unattributed_geometric_overlap")
            issues.append(f"unattributed_geometric_overlap:{object_id}")
        records.append(
            {
                "object_id": object_id,
                "carla_runtime_actor_id": state.get("carla_runtime_actor_id"),
                "minimum_clearance_m": geometry["minimum_clearance_m"],
                "horizontal_clearance_m": geometry["horizontal_clearance_m"],
                "vertical_clearance_m": geometry["vertical_clearance_m"],
                "bounding_box_overlap": geometry["overlap"],
                "collision_sensor_contact": event_contact,
                "status": "failed" if row_issues else "passed",
                "issues": row_issues,
            }
        )

    unmatched_event_ids = sorted(event_object_ids - set(states))
    for object_id in unmatched_event_ids:
        issues.append(f"collision_event_without_registry_state:{object_id}")
    if bool(tick.get("collision_detected")) and not event_object_ids:
        issues.append("unattributed_carla_collision")

    return _result("collision_audit.v1", frame_id, simulation_time_sec, records, issues)


def audit_lane_tick(tick: Mapping[str, Any]) -> dict[str, Any]:
    """Normalize and validate CARLA lane membership for one tick."""

    frame_id, simulation_time_sec = _tick_identity(tick)
    lane = tick.get("lane_state")
    issues: list[str] = []
    if not isinstance(lane, Mapping):
        issues.append("missing_carla_lane_state")
        row: dict[str, Any] = {}
    else:
        row = {
            "road_id": lane.get("road_id"),
            "section_id": lane.get("section_id"),
            "lane_id": lane.get("lane_id"),
            "lane_type": lane.get("lane_type"),
            "is_junction": lane.get("is_junction"),
            "is_on_road": lane.get(
                "is_on_road",
                bool(lane.get("inside_lane")) if lane.get("inside_lane") is not None else None,
            ),
            "inside_lane": lane.get("inside_lane"),
            "lane_width_m": lane.get("lane_width_m"),
            "center_distance_m": lane.get(
                "center_distance_m", lane.get("signed_centerline_distance_m")
            ),
            "signed_centerline_distance_m": lane.get(
                "signed_centerline_distance_m", lane.get("center_distance_m")
            ),
            "signed_boundary_distance_m": lane.get("signed_boundary_distance_m"),
            "route_progress": lane.get("route_progress"),
            "lane_invasion_events": list(lane.get("lane_invasion_events") or []),
            "lane_invasion_sensor_available": lane.get(
                "lane_invasion_sensor_available"
            ),
        }
        for field in ("road_id", "lane_id", "lane_type", "is_on_road", "route_progress"):
            if row.get(field) is None:
                issues.append(f"missing_lane_field:{field}")
        if row.get("lane_invasion_sensor_available") is not True:
            issues.append("lane_invasion_sensor_unavailable")
        if row.get("is_on_road") is False:
            issues.append("off_road")
        if row.get("inside_lane") is False:
            issues.append("outside_lane")
        if row.get("lane_invasion_events"):
            issues.append("lane_invasion")
        for field in ("route_progress", "lane_width_m", "center_distance_m"):
            value = row.get(field)
            if value is not None and not _finite_number(value):
                issues.append(f"invalid_lane_field:{field}")
        route_progress = row.get("route_progress")
        if _finite_number(route_progress) and not 0.0 <= float(route_progress) <= 1.0:
            issues.append("route_progress_out_of_range")

    row["status"] = "failed" if issues else "passed"
    row["issues"] = sorted(set(issues))
    return _result("lane_audit.v1", frame_id, simulation_time_sec, [row], issues)


def audit_visibility_tick(tick: Mapping[str, Any]) -> dict[str, Any]:
    """Require payload-bound calibrated projections when visibility is supplied."""

    frame_id, simulation_time_sec = _tick_identity(tick)
    projections = tick.get("projections")
    if not isinstance(projections, list):
        raise SceneSafetyAuditError("visibility audit requires a projections list")
    records: list[dict[str, Any]] = []
    issues: list[str] = []
    payload_by_camera: dict[str, Mapping[str, Any]] = {}
    if tick.get("payload_binding_required") is True:
        payloads = tick.get("payloads")
        if not isinstance(payloads, list) or not payloads:
            issues.append("missing_rgb_payload_binding")
        else:
            for payload in payloads:
                if not isinstance(payload, Mapping):
                    issues.append("invalid_rgb_payload_binding")
                    continue
                camera_id = _nonempty(payload.get("sensor_id"), "visibility.payload.sensor_id")
                if camera_id in payload_by_camera:
                    issues.append(f"duplicate_rgb_payload_binding:{camera_id}")
                    continue
                payload_by_camera[camera_id] = payload
                if (
                    payload.get("frame_id") != frame_id
                    or not _sha256(payload.get("payload_sha256"))
                    or not _nonempty(
                        payload.get("calibrated_sensor_token"),
                        "visibility.payload.calibrated_sensor_token",
                    )
                    or not _sha256(payload.get("intrinsics_table_sha256"))
                ):
                    issues.append(f"invalid_rgb_payload_binding:{camera_id}")
        if not projections:
            issues.append("visibility_projection_set_empty")
    for projection in projections:
        if not isinstance(projection, Mapping):
            raise SceneSafetyAuditError("visibility projection must be an object")
        object_id = _nonempty(projection.get("object_id"), "projection.object_id")
        camera = _nonempty(projection.get("camera"), "projection.camera")
        expected_visible = bool(projection.get("expected_visible", True))
        evidence = projection.get("evidence")
        calibrated = (
            projection.get("observation_kind") == "calibrated_3d_box_projection"
            and isinstance(projection.get("projection"), Mapping)
            and isinstance(evidence, Mapping)
            and len(str(evidence.get("nre_payload_sha256") or "")) == 64
            and bool(str(evidence.get("calibrated_sensor_token") or ""))
            and len(str(evidence.get("intrinsics_table_sha256") or "")) == 64
        )
        if tick.get("payload_binding_required") is True:
            bound = payload_by_camera.get(camera)
            if bound is None:
                calibrated = False
            elif (
                evidence.get("nre_payload_sha256")
                != bound.get("payload_sha256")
                or evidence.get("calibrated_sensor_token")
                != bound.get("calibrated_sensor_token")
                or evidence.get("intrinsics_table_sha256")
                != bound.get("intrinsics_table_sha256")
            ):
                calibrated = False
        row_issues = (
            ["missing_calibrated_same_frame_projection"]
            if expected_visible and not calibrated
            else []
        )
        if row_issues:
            issues.append(f"visibility_uncalibrated:{object_id}:{camera}")
        records.append(
            {
                "object_id": object_id,
                "camera": camera,
                "expected_visible": expected_visible,
                "calibrated_projection": calibrated,
                "status": "failed" if row_issues else "passed",
                "issues": row_issues,
            }
        )
    return _result("visibility_audit.v1", frame_id, simulation_time_sec, records, issues)


def audit_lidar_world_tick(tick: Mapping[str, Any]) -> dict[str, Any]:
    """Require declared LiDAR support for each expected world object."""

    frame_id, simulation_time_sec = _tick_identity(tick)
    expected = tick.get("expected_world_objects")
    support = tick.get("lidar_occupancy")
    if not isinstance(expected, list) or not isinstance(support, list):
        raise SceneSafetyAuditError(
            "LiDAR world audit requires expected_world_objects and lidar_occupancy lists"
        )
    support_by_object: dict[str, Mapping[str, Any]] = {}
    issues: list[str] = []
    for item in support:
        if not isinstance(item, Mapping):
            issues.append("invalid_lidar_occupancy_record")
            continue
        object_id = _nonempty(item.get("object_id"), "lidar_occupancy.object_id")
        if object_id in support_by_object:
            issues.append(f"duplicate_lidar_occupancy:{object_id}")
        support_by_object[object_id] = item
    if tick.get("payload_binding_required") is True:
        payload_sha256 = tick.get("payload_sha256")
        if (
            not _sha256(payload_sha256)
            or not _nonempty(tick.get("sensor_id"), "lidar.sensor_id")
            or tick.get("coordinate_frame") != "sensor_local"
        ):
            issues.append("invalid_lidar_payload_binding")
        if not support:
            issues.append("empty_lidar_occupancy")
    records: list[dict[str, Any]] = []
    for item in expected:
        if not isinstance(item, Mapping):
            raise SceneSafetyAuditError("expected world object must be an object")
        object_id = _nonempty(item.get("object_id"), "expected_world_objects.object_id")
        required = item.get("expected_lidar_support")
        if not isinstance(required, bool):
            raise SceneSafetyAuditError(
                "expected_world_objects.expected_lidar_support must be explicit"
            )
        observed = support_by_object.get(object_id)
        point_count = observed.get("point_count") if isinstance(observed, Mapping) else None
        valid = isinstance(point_count, int) and not isinstance(point_count, bool) and point_count > 0
        row_issues = ["missing_lidar_world_occupancy"] if required and not valid else []
        if row_issues:
            issues.append(f"missing_lidar_world_occupancy:{object_id}")
        records.append(
            {
                "object_id": object_id,
                "expected_lidar_support": required,
                "point_count": point_count,
                "status": "failed" if row_issues else "passed",
                "issues": row_issues,
            }
        )
    return _result("lidar_world_audit.v1", frame_id, simulation_time_sec, records, issues)


def _active_required_records(
    registry: Mapping[str, Any], simulation_time_sec: float
) -> list[Mapping[str, Any]]:
    records = registry.get("records")
    if not isinstance(records, list):
        raise SceneSafetyAuditError("registry requires a records list")
    result: list[Mapping[str, Any]] = []
    for record in records:
        if not isinstance(record, Mapping) or record.get("role") == "road_boundary":
            continue
        carla = record.get("carla")
        if not isinstance(carla, Mapping) or carla.get("collision_policy") != "required":
            continue
        object_id = _nonempty(record.get("object_id"), "registry.object_id")
        interval = record.get("time_interval")
        interval = interval if isinstance(interval, Mapping) else {}
        start = interval.get("start_sec", 0.0)
        end = interval.get("end_sec")
        if not _finite_number(start) or (end is not None and not _finite_number(end)):
            raise SceneSafetyAuditError(f"invalid registry time interval: {object_id}")
        if float(start) <= simulation_time_sec and (end is None or simulation_time_sec <= float(end)):
            result.append(record)
    return result


def _states_by_object(value: Any) -> dict[str, Mapping[str, Any]]:
    if not isinstance(value, list):
        raise SceneSafetyAuditError("collision audit requires an object_states list")
    result: dict[str, Mapping[str, Any]] = {}
    for item in value:
        if not isinstance(item, Mapping):
            raise SceneSafetyAuditError("object state must be an object")
        object_id = _nonempty(item.get("object_id"), "object_state.object_id")
        if object_id in result:
            raise SceneSafetyAuditError(f"duplicate object state: {object_id}")
        runtime_id = item.get("carla_runtime_actor_id")
        if not isinstance(runtime_id, int) or isinstance(runtime_id, bool):
            raise SceneSafetyAuditError(f"object state has no CARLA runtime id: {object_id}")
        result[object_id] = item
    return result


def _collision_event_object_ids(value: Any) -> set[str]:
    if value is None:
        return set()
    if not isinstance(value, list):
        raise SceneSafetyAuditError("collision_events must be a list")
    result: set[str] = set()
    for item in value:
        if not isinstance(item, Mapping):
            raise SceneSafetyAuditError("collision event must be an object")
        result.add(_nonempty(item.get("object_id"), "collision_event.object_id"))
    return result


def _box(value: Any, label: str) -> dict[str, float]:
    if not isinstance(value, Mapping):
        raise SceneSafetyAuditError(f"{label} must be an object")
    pose = value.get("pose")
    extent = value.get("extent_m")
    if not isinstance(pose, Mapping) or not isinstance(extent, Mapping):
        raise SceneSafetyAuditError(f"{label} requires pose and extent_m")
    try:
        result = {axis: float(pose[axis]) for axis in ("x", "y", "z")}
        result["yaw"] = float(pose.get("yaw", 0.0))
        result.update({f"extent_{axis}": float(extent[axis]) for axis in ("x", "y", "z")})
    except (KeyError, TypeError, ValueError) as exc:
        raise SceneSafetyAuditError(f"{label} has invalid pose or extent") from exc
    if not all(math.isfinite(item) for item in result.values()):
        raise SceneSafetyAuditError(f"{label} has non-finite pose or extent")
    if any(result[f"extent_{axis}"] <= 0.0 for axis in ("x", "y", "z")):
        raise SceneSafetyAuditError(f"{label} has non-positive extent")
    return result


def _box_geometry(left: Mapping[str, float], right: Mapping[str, float]) -> dict[str, Any]:
    """Return conservative OBB overlap and separation metrics."""

    horizontal_axes = _obb_axes(left, right)
    horizontal_gaps = [
        _projection_gap(left, right, axis_x, axis_y)
        for axis_x, axis_y in horizontal_axes
    ]
    vertical_gap = abs(left["z"] - right["z"]) - left["extent_z"] - right["extent_z"]
    overlap = all(gap <= 0.0 for gap in horizontal_gaps) and vertical_gap <= 0.0
    horizontal_clearance = max(0.0, max(horizontal_gaps, default=0.0))
    vertical_clearance = max(0.0, vertical_gap)
    if overlap:
        minimum_clearance = 0.0
    elif vertical_clearance > 0.0 and horizontal_clearance > 0.0:
        minimum_clearance = math.hypot(horizontal_clearance, vertical_clearance)
    else:
        minimum_clearance = max(horizontal_clearance, vertical_clearance)
    return {
        "overlap": overlap,
        "minimum_clearance_m": minimum_clearance,
        "horizontal_clearance_m": horizontal_clearance,
        "vertical_clearance_m": vertical_clearance,
    }


def _obb_axes(*boxes: Mapping[str, float]) -> list[tuple[float, float]]:
    axes: list[tuple[float, float]] = []
    for box in boxes:
        yaw = math.radians(box["yaw"])
        axes.extend(((math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))))
    return axes


def _projection_gap(
    left: Mapping[str, float],
    right: Mapping[str, float],
    axis_x: float,
    axis_y: float,
) -> float:
    center_delta = abs((right["x"] - left["x"]) * axis_x + (right["y"] - left["y"]) * axis_y)
    left_yaw = math.radians(left["yaw"])
    right_yaw = math.radians(right["yaw"])
    left_radius = left["extent_x"] * abs(math.cos(left_yaw) * axis_x + math.sin(left_yaw) * axis_y) + left["extent_y"] * abs(-math.sin(left_yaw) * axis_x + math.cos(left_yaw) * axis_y)
    right_radius = right["extent_x"] * abs(math.cos(right_yaw) * axis_x + math.sin(right_yaw) * axis_y) + right["extent_y"] * abs(-math.sin(right_yaw) * axis_x + math.cos(right_yaw) * axis_y)
    return center_delta - left_radius - right_radius


def _tick_identity(tick: Mapping[str, Any]) -> tuple[int, float]:
    frame_id = tick.get("frame_id")
    simulation_time_sec = tick.get("simulation_time_sec")
    if not isinstance(frame_id, int) or isinstance(frame_id, bool):
        raise SceneSafetyAuditError("tick.frame_id must be an integer")
    if not _finite_number(simulation_time_sec):
        raise SceneSafetyAuditError("tick.simulation_time_sec must be finite")
    return frame_id, float(simulation_time_sec)


def _result(
    schema_version: str,
    frame_id: int,
    simulation_time_sec: float,
    records: list[dict[str, Any]],
    issues: Sequence[str],
) -> dict[str, Any]:
    normalized = sorted(set(str(item) for item in issues))
    return {
        "schema_version": schema_version,
        "frame_id": frame_id,
        "simulation_time_sec": simulation_time_sec,
        "records": records,
        "status": "failed" if normalized else "passed",
        "issues": normalized,
    }


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _nonempty(value: Any, label: str) -> str:
    result = str(value or "")
    if not result:
        raise SceneSafetyAuditError(f"{label} must be non-empty")
    return result


def _sha256(value: Any) -> bool:
    text = str(value or "")
    return len(text) == 64 and all(character in "0123456789abcdef" for character in text)
