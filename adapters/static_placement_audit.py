from __future__ import annotations

from typing import Any, Mapping


def audit_static_placement(
    registry: Mapping[str, Any], runtime_audit: Mapping[str, Any], visibility: Mapping[str, Any]
) -> dict[str, Any]:
    """Join static collision presence with calibrated NuRec geometric placement."""

    static_records = [item for item in registry.get("records") or [] if item.get("role") == "static_obstacle"]
    runtime = {str(item.get("object_id") or ""): item for item in runtime_audit.get("static_records") or []}
    observations: dict[str, list[Mapping[str, Any]]] = {}
    for item in visibility.get("observations") or []:
        if isinstance(item, Mapping):
            observations.setdefault(str(item.get("object_id") or ""), []).append(item)
    rows = []
    issues = []
    for record in static_records:
        object_id = str(record.get("object_id") or "")
        carla = record.get("carla") or {}
        placement = carla.get("placement")
        runtime_record = runtime.get(object_id)
        row_issues = []
        if not isinstance(placement, Mapping):
            row_issues.append("registry_placement_missing")
        if carla.get("collision_policy") != "required":
            row_issues.append("collision_policy_not_required")
        if not isinstance(runtime_record, Mapping) or runtime_record.get("status") != "passed":
            row_issues.append("carla_static_runtime_missing")
        visible = observations.get(object_id, [])
        valid_observations = [
            item for item in visible
            if (item.get("evidence") or {}).get("nre_payload_sha256")
            and (item.get("evidence") or {}).get("calibrated_sensor_token")
            and item.get("observation_kind") == "calibrated_3d_box_projection"
        ]
        if visible and not valid_observations:
            row_issues.append("visible_static_projection_evidence_invalid")
        rows.append({
            "object_id": object_id,
            "semantic_class": record.get("semantic_class"),
            "declared_placement": dict(placement) if isinstance(placement, Mapping) else None,
            "carla_runtime_actor_id": (runtime_record or {}).get("carla_runtime_actor_id"),
            "geometric_observation_count": len(valid_observations),
            "status": "passed" if not row_issues else "failed",
            "issues": row_issues,
        })
        issues.extend(f"{object_id}:{issue}" for issue in row_issues)
    return {
        "schema_version": "static_placement_audit.v1",
        "scene_id": registry.get("scene_id"),
        "scope": "CARLA static collision placement plus calibrated NuRec geometric projection; semantic pixel validation is M8",
        "static_record_count": len(rows),
        "geometrically_observed_static_count": sum(row["geometric_observation_count"] > 0 for row in rows),
        "records": rows,
        "issues": sorted(set(issues)),
        "status": "passed" if not issues else "failed",
    }
