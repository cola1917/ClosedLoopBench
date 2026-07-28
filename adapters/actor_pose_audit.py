from __future__ import annotations

import math
from typing import Any, Mapping


class ActorPoseAuditError(ValueError):
    """Raised when M7 pose evidence is structurally incomplete."""


def audit_actor_pose_request(
    request: Mapping[str, Any],
    *,
    expected_actor_ids: set[str] | None = None,
) -> list[dict[str, Any]]:
    """Compare CARLA physical reference poses with the exact NuRec request."""

    if request.get("schema_version") != "nurec_dynamic_pose_request.v1":
        raise ActorPoseAuditError("unsupported pose request schema")
    contract = request.get("coordinate_contract") or {}
    transform = contract.get("carla_to_nurec_global_transform")
    if not isinstance(transform, list) or len(transform) != 16:
        raise ActorPoseAuditError("pose request has no CARLA-to-NuRec transform")
    pairs = request.get("actor_pose_pairs")
    if not isinstance(pairs, list):
        raise ActorPoseAuditError("pose request actor_pose_pairs must be a list")
    seen = set()
    rows = []
    for pair in pairs:
        if not isinstance(pair, Mapping):
            raise ActorPoseAuditError("pose request actor pair must be an object")
        actor_id = str(pair.get("actor_id") or "")
        if not actor_id or actor_id in seen:
            raise ActorPoseAuditError("pose request actor IDs must be non-empty and unique")
        seen.add(actor_id)
        rows.append(_audit_pair(request, pair, transform))
    absences = {
        str(item.get("actor_id") or ""): item
        for item in request.get("actor_absences") or []
        if isinstance(item, Mapping) and str(item.get("actor_id") or "")
    }
    for actor_id in sorted((expected_actor_ids or set()) - seen):
        absence = absences.get(actor_id)
        if absence is not None:
            rows.append(
                {
                    "schema_version": "actor_pose_audit.v1",
                    "scene_id": request.get("scene_id"),
                    "frame_id": request.get("frame_id"),
                    "tick_index": request.get("tick_index"),
                    "actor_id": actor_id,
                    "status": "not_applicable",
                    "issues": [f"outside_nurec_annotation_window:{absence.get('reason') or 'unknown'}"],
                }
            )
            continue
        rows.append(
            {
                "schema_version": "actor_pose_audit.v1",
                "scene_id": request.get("scene_id"),
                "frame_id": request.get("frame_id"),
                "tick_index": request.get("tick_index"),
                "actor_id": actor_id,
                "status": "failed",
                "issues": ["missing_nurec_pose_request"],
            }
        )
    return rows


def _audit_pair(
    request: Mapping[str, Any], pair: Mapping[str, Any], transform: list[Any]
) -> dict[str, Any]:
    actor_type = str(pair.get("actor_type") or "")
    threshold_m = 0.30 if actor_type == "pedestrian" else 0.50
    issues: list[str] = []
    source = pair.get("nurec_pose_source")
    physical_reference = pair.get("carla_physical_pose_reference")
    nurec_reference = pair.get("nurec_pose_reference")
    runtime_id = pair.get("carla_runtime_actor_id")
    if not isinstance(runtime_id, int) or isinstance(runtime_id, bool):
        issues.append("carla_runtime_actor_id_missing")
    if source != "carla_runtime_actor_pose":
        issues.append("nonphysical_nurec_pose_source")
    if physical_reference != nurec_reference:
        issues.append("physical_nurec_pose_reference_mismatch")

    physical_pair = pair.get("carla_physical_pose_pair")
    request_pair = pair.get("nurec_request_pose_pair")
    endpoint_rows = {}
    if not isinstance(physical_pair, Mapping) or not isinstance(request_pair, Mapping):
        issues.append("pose_pair_missing")
    else:
        for endpoint in ("start", "end"):
            physical = physical_pair.get(endpoint)
            rendered = request_pair.get(endpoint)
            try:
                transformed = _transform_carla_pose(physical, transform)
                request_pose = _nurec_request_pose(rendered)
            except ActorPoseAuditError as exc:
                issues.append(f"{endpoint}:{exc}")
                continue
            translation = math.dist(
                [transformed[axis] for axis in ("x", "y", "z")],
                [request_pose[axis] for axis in ("x", "y", "z")],
            )
            yaw = _angular_error_deg(transformed["yaw"], request_pose["yaw"])
            endpoint_rows[endpoint] = {
                "carla_physical_pose_nurec_global": transformed,
                "nurec_request_pose": request_pose,
                "translation_error_m": translation,
                "yaw_error_deg": yaw,
            }
            if translation > threshold_m:
                issues.append(f"{endpoint}:translation_threshold_exceeded")
            if yaw > 5.0:
                issues.append(f"{endpoint}:yaw_threshold_exceeded")
    return {
        "schema_version": "actor_pose_audit.v1",
        "scene_id": request.get("scene_id"),
        "frame_id": request.get("frame_id"),
        "tick_index": request.get("tick_index"),
        "simulation_time_sec": request.get("simulation_time_sec"),
        "pose_interval_sec": request.get("pose_interval_sec"),
        "actor_id": pair.get("actor_id"),
        "nurec_track_id": pair.get("nurec_track_id"),
        "actor_type": actor_type,
        "carla_runtime_actor_id": runtime_id,
        "carla_physical_pose_reference": physical_reference,
        "nurec_pose_source": source,
        "nurec_pose_reference": nurec_reference,
        "thresholds": {"translation_m": threshold_m, "yaw_deg": 5.0},
        "endpoints": endpoint_rows,
        "status": "passed" if not issues else "failed",
        "issues": sorted(set(issues)),
    }


def _transform_carla_pose(pose: Any, transform: list[Any]) -> dict[str, float]:
    if not isinstance(pose, Mapping):
        raise ActorPoseAuditError("carla_physical_pose_missing")
    try:
        values = {axis: float(pose.get(axis, 0.0)) for axis in ("x", "y", "z", "roll", "pitch", "yaw")}
        matrix = _matmul([float(value) for value in transform], _pose_matrix(values))
    except (TypeError, ValueError) as exc:
        raise ActorPoseAuditError("carla_physical_pose_invalid") from exc
    return {"x": matrix[3], "y": matrix[7], "z": matrix[11], "yaw": math.degrees(math.atan2(matrix[4], matrix[0]))}


def _nurec_request_pose(pose: Any) -> dict[str, float]:
    if not isinstance(pose, Mapping):
        raise ActorPoseAuditError("nurec_request_pose_missing")
    position = pose.get("position_m")
    orientation = pose.get("orientation_xyzw")
    if not isinstance(position, Mapping) or not isinstance(orientation, Mapping):
        raise ActorPoseAuditError("nurec_request_pose_invalid")
    try:
        x, y, z, w = (float(orientation[axis]) for axis in ("x", "y", "z", "w"))
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            raise ValueError("zero quaternion")
        x, y, z, w = (value / norm for value in (x, y, z, w))
        yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
        return {"x": float(position["x"]), "y": float(position["y"]), "z": float(position["z"]), "yaw": yaw}
    except (KeyError, TypeError, ValueError) as exc:
        raise ActorPoseAuditError("nurec_request_pose_invalid") from exc


def _pose_matrix(pose: Mapping[str, float]) -> list[float]:
    roll, pitch, yaw = (math.radians(pose[axis]) for axis in ("roll", "pitch", "yaw"))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, pose["x"],
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, pose["y"],
        -sp, cp * sr, cp * cr, pose["z"],
        0.0, 0.0, 0.0, 1.0,
    ]


def _matmul(left: list[float], right: list[float]) -> list[float]:
    return [
        sum(left[row * 4 + index] * right[index * 4 + column] for index in range(4))
        for row in range(4)
        for column in range(4)
    ]


def _angular_error_deg(left: float, right: float) -> float:
    return abs((left - right + 180.0) % 360.0 - 180.0)
