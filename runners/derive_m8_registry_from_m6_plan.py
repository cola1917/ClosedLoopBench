from __future__ import annotations

import argparse
import json
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_object_registry import (
    SceneObjectRegistryError,
    build_scene_object_registry,
    validate_scene_object_registry,
)


def derive_registry(
    prior_registry: Mapping[str, Any],
    source_plan: Mapping[str, Any],
    *,
    nonreplay_static_actor_ids: set[str] | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Rebuild a registry from M6 source actors without mutating M6 evidence."""

    validate_scene_object_registry(prior_registry)
    scene_id = str(prior_registry["scene_id"])
    if str(source_plan.get("scenario_id") or "") != scene_id:
        raise SceneObjectRegistryError("M6 source plan scene_id does not match registry")
    actors = source_plan.get("actors")
    if not isinstance(actors, list):
        raise SceneObjectRegistryError("M6 source plan requires an actors list")
    prior_records = prior_registry["records"]
    prior_by_id = {str(record["object_id"]): record for record in prior_records}
    plan_actor_ids = {str(actor.get("actor_id") or "") for actor in actors if isinstance(actor, Mapping)}
    expected_actor_ids = {
        object_id
        for object_id, record in prior_by_id.items()
        if record["role"] in {"background_replay", "controlled_lead_vehicle", "controlled_pedestrian"}
    }
    if plan_actor_ids != expected_actor_ids:
        raise SceneObjectRegistryError("M6 source plan actors do not exactly match prior dynamic records")
    static_actor_ids = set(nonreplay_static_actor_ids or set())
    if not static_actor_ids.issubset(plan_actor_ids):
        raise SceneObjectRegistryError("non-replay actor IDs are absent from the M6 source plan")

    static_objects = []
    road_source = None
    role_overrides = {}
    for record in prior_records:
        role = record["role"]
        if role == "static_obstacle":
            carla = record["carla"]
            static_objects.append(
                {
                    "object_id": record["object_id"],
                    "semantic_class": record["semantic_class"],
                    "category": record["category"],
                    "source": deepcopy(dict(record["source"])),
                    "placement": deepcopy(dict(carla["placement"])),
                    "time_interval": deepcopy(dict(record["time_interval"])),
                    "blueprint_class": carla["blueprint_class"],
                    "carla_representation": carla["representation"],
                    "nurec_representation": record["nurec"]["representation"],
                    "safety_relevant": record["safety_relevant"],
                }
            )
        elif role == "road_boundary":
            road_source = deepcopy(dict(record["source"]))
        elif role in {"controlled_lead_vehicle", "controlled_pedestrian"}:
            role_overrides[str(record["object_id"])] = role

    scenario = {
        "schema_version": "scenario_ir.v1",
        "scenario_id": scene_id,
        "actors": deepcopy(actors),
    }
    rebuilt = build_scene_object_registry(
        scenario,
        static_objects=static_objects,
        role_overrides=role_overrides,
        nonreplay_static_actor_ids=static_actor_ids,
        include_road_boundary=road_source is not None,
        road_boundary_source=road_source,
    )
    rebuilt_ids = {str(record["object_id"]) for record in rebuilt["records"]}
    prior_ids = set(prior_by_id)
    if rebuilt_ids != prior_ids:
        raise SceneObjectRegistryError("derived registry object IDs do not exactly match M6")
    reclassified = [
        {
            "object_id": object_id,
            "prior_role": prior_by_id[object_id]["role"],
            "new_role": record["role"],
            "prior_nurec_representation": prior_by_id[object_id]["nurec"]["representation"],
            "new_nurec_representation": record["nurec"]["representation"],
        }
        for object_id, record in sorted(
            ((str(record["object_id"]), record) for record in rebuilt["records"]), key=lambda item: item[0]
        )
        if prior_by_id[object_id]["role"] != record["role"]
        or prior_by_id[object_id]["nurec"]["representation"] != record["nurec"]["representation"]
    ]
    manifest = {
        "schema_version": "m8_registry_derivation.v1",
        "scene_id": scene_id,
        "prior_registry_schema_version": prior_registry["schema_version"],
        "source_plan_schema_version": source_plan.get("schema_version"),
        "object_id_match": True,
        "reclassified_records": reclassified,
        "prior_summary": deepcopy(dict(prior_registry["summary"])),
        "derived_summary": deepcopy(dict(rebuilt["summary"])),
    }
    return rebuilt, manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Derive an immutable M8 registry from the M6 source actor plan.")
    parser.add_argument("--prior-registry", required=True, type=Path)
    parser.add_argument("--source-plan", required=True, type=Path)
    parser.add_argument("--nonreplay-static-actor-id", action="append", default=[])
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--manifest-output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists() or args.manifest_output.exists():
            raise ValueError("refusing to overwrite derived M8 registry evidence")
        prior = json.loads(args.prior_registry.read_text(encoding="utf-8"))
        plan = json.loads(args.source_plan.read_text(encoding="utf-8"))
        registry, manifest = derive_registry(
            prior,
            plan,
            nonreplay_static_actor_ids={str(actor_id) for actor_id in args.nonreplay_static_actor_id},
        )
        for path, value in ((args.output, registry), (args.manifest_output, manifest)):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, SceneObjectRegistryError) as exc:
        parser.error(str(exc))
    print(json.dumps({"registry": str(args.output), "summary": registry["summary"], "reclassified": manifest["reclassified_records"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
