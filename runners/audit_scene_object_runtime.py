from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_object_registry import validate_scene_object_registry


def audit_scene_object_runtime(
    registry: Mapping[str, Any],
    report: Mapping[str, Any],
    frame_trace: list[Mapping[str, Any]],
) -> dict[str, Any]:
    """Correlate registry records with dynamic and static CARLA runtime identities."""

    validate_scene_object_registry(registry)
    records = list(registry["records"])
    dynamic = [
        record for record in records if record["role"] in {
            "background_replay", "controlled_lead_vehicle", "controlled_pedestrian"
        }
    ]
    static = [record for record in records if record["role"] == "static_obstacle"]
    boundaries = [record for record in records if record["role"] == "road_boundary"]
    latest = frame_trace[-1] if frame_trace else {}
    actor_states = latest.get("actor_states") if isinstance(latest, Mapping) else {}
    actor_states = actor_states if isinstance(actor_states, Mapping) else {}
    static_runtime = ((report.get("runtime") or {}).get("static_obstacle_runtime") or {})
    static_by_id = {
        str(item.get("object_id") or ""): item
        for item in static_runtime.get("records") or []
        if isinstance(item, Mapping)
    }

    dynamic_rows: list[dict[str, Any]] = []
    static_rows: list[dict[str, Any]] = []
    issues: list[str] = []
    for record in dynamic:
        object_id = str(record["object_id"])
        state = actor_states.get(object_id)
        row_issues: list[str] = []
        state = state if isinstance(state, Mapping) else {}
        runtime_id = state.get("carla_runtime_actor_id")
        if runtime_id is None:
            row_issues.append("dynamic_runtime_actor_id_missing")
        evidence = state.get("spawn_evidence") or {}
        if float(evidence.get("vertical_adjustment_m") or 0.0) > 0.5:
            row_issues.append("dynamic_vertical_spawn_adjustment_exceeds_m7_limit")
        dynamic_rows.append({
            "object_id": object_id,
            "carla_runtime_actor_id": runtime_id,
            "spawn_evidence": dict(evidence),
            "status": "passed" if not row_issues else "failed",
            "issues": row_issues,
        })
        issues.extend(f"{object_id}:{item}" for item in row_issues)
    for record in static:
        object_id = str(record["object_id"])
        runtime = static_by_id.get(object_id) or {}
        row_issues: list[str] = []
        if runtime.get("status") != "passed":
            row_issues.append("static_runtime_spawn_not_passed")
        if runtime.get("carla_runtime_actor_id") is None:
            row_issues.append("static_runtime_actor_id_missing")
        static_rows.append({
            "object_id": object_id,
            "carla_runtime_actor_id": runtime.get("carla_runtime_actor_id"),
            "collision_policy": record["carla"]["collision_policy"],
            "status": "passed" if not row_issues else "failed",
            "issues": row_issues,
        })
        issues.extend(f"{object_id}:{item}" for item in row_issues)

    runtime_ids = [
        row["carla_runtime_actor_id"]
        for row in dynamic_rows + static_rows
        if row["carla_runtime_actor_id"] is not None
    ]
    if len(runtime_ids) != len(set(runtime_ids)):
        issues.append("CARLA_runtime_actor_id_reused_across_registry_objects")
    if len(boundaries) != 1:
        issues.append("road_boundary_registry_count_invalid")
    elif boundaries[0]["carla"].get("representation") != "road_topology":
        issues.append("road_boundary_not_backed_by_carla_topology")
    return {
        "schema_version": "scene_object_runtime_audit.v1",
        "scene_id": registry["scene_id"],
        "registry_schema_version": registry["schema_version"],
        "source_carla_report_status": report.get("status"),
        "dynamic_records": dynamic_rows,
        "static_records": static_rows,
        "road_boundary_records": boundaries,
        "summary": {
            "registered_object_count": len(records),
            "dynamic_required_count": len(dynamic),
            "dynamic_runtime_count": sum(row["carla_runtime_actor_id"] is not None for row in dynamic_rows),
            "static_required_count": len(static),
            "static_runtime_count": sum(row["carla_runtime_actor_id"] is not None for row in static_rows),
            "road_boundary_count": len(boundaries),
        },
        "issues": sorted(set(issues)),
        "status": "passed" if not issues else "failed",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Audit CARLA runtime coverage against the M6 registry.")
    parser.add_argument("--scene-object-registry", required=True, type=Path)
    parser.add_argument("--closed-loop-report", required=True, type=Path)
    parser.add_argument("--frame-trace", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        registry = json.loads(args.scene_object_registry.read_text(encoding="utf-8"))
        report = json.loads(args.closed_loop_report.read_text(encoding="utf-8"))
        frames = [
            json.loads(line)
            for line in args.frame_trace.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        result = audit_scene_object_runtime(registry, report, frames)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps({"output": str(args.output), "status": result["status"], "summary": result["summary"]}, ensure_ascii=False, indent=2))
    return 0 if result["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
