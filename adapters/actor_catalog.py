from __future__ import annotations

import math
import re
from copy import deepcopy
from typing import Any, Iterable, Mapping

from adapters.shared_protocol_validation import validate_document


class ActorCatalogError(ValueError):
    """Raised when event references and the physical actor catalog diverge."""


_TOKEN = re.compile(r"(?<![0-9a-f])[0-9a-f]{32}(?![0-9a-f])")


def referenced_actor_ids(
    scenario_ir: Mapping[str, Any],
    *,
    known_actor_ids: Iterable[str],
) -> list[str]:
    """Find nuScenes instance tokens mentioned anywhere in event evidence."""

    known = {str(value) for value in known_actor_ids}
    found: set[str] = set()

    def visit(value: Any) -> None:
        if isinstance(value, Mapping):
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)
        elif isinstance(value, str):
            found.update(match.group(0) for match in _TOKEN.finditer(value) if match.group(0) in known)

    visit(scenario_ir.get("events") or {})
    return sorted(found)


def repair_scenario_actor_catalog(
    scenario_ir: Mapping[str, Any],
    full_scene_ir: Mapping[str, Any],
    *,
    additional_actor_ids: Iterable[str] = (),
) -> dict[str, Any]:
    """Add event-referenced/selected actors from the complete nuScenes catalog."""

    _same_scene(scenario_ir, full_scene_ir)
    current = _actor_index(scenario_ir)
    complete = _actor_index(full_scene_ir)
    referenced = referenced_actor_ids(scenario_ir, known_actor_ids=complete)
    requested = {str(value) for value in additional_actor_ids}
    unknown = sorted(requested - set(complete))
    if unknown:
        raise ActorCatalogError(
            "requested actors are absent from the full nuScenes catalog: " + ", ".join(unknown)
        )
    required = set(referenced) | requested
    added_ids = sorted(required - set(current))
    repaired = deepcopy(dict(scenario_ir))
    repaired_actors = repaired.get("actors")
    if not isinstance(repaired_actors, list):
        raise ActorCatalogError("Scenario IR actors must be a list")
    for actor_id in added_ids:
        actor = deepcopy(complete[actor_id])
        actor["catalog_inclusion"] = {
            "event_referenced": actor_id in referenced,
            "explicitly_selected": actor_id in requested,
            "source": "complete_nuscenes_scene_catalog",
        }
        repaired_actors.append(actor)
    if isinstance(repaired.get("risk_metrics"), dict):
        repaired["risk_metrics"]["actor_count"] = len(repaired_actors)
    diagnostics = repaired.setdefault("diagnostics", {})
    diagnostics["actor_catalog_repair"] = {
        "full_catalog_count": len(complete),
        "original_actor_count": len(current),
        "event_referenced_actor_count": len(referenced),
        "requested_actor_count": len(requested),
        "added_actor_count": len(added_ids),
        "added_actor_ids": added_ids,
        "unresolved_event_actor_ids": sorted(set(referenced) - set(_actor_index(repaired))),
    }
    try:
        validate_document(repaired)
    except ValueError as exc:
        raise ActorCatalogError(str(exc)) from exc
    return repaired


def rank_actor_candidates(full_scene_ir: Mapping[str, Any]) -> dict[str, Any]:
    """Rank source actors for one primary vehicle and one bounded pedestrian loop."""

    ego = (full_scene_ir.get("ego") or {}).get("reference_trajectory")
    if not isinstance(ego, list) or not ego:
        raise ActorCatalogError("Scenario IR ego reference trajectory is required")
    ego_by_time = {round(float(state["t_sec"]), 6): state for state in ego}
    records = []
    for actor in (full_scene_ir.get("actors") or []):
        actor_type = _normalized_type(actor)
        if actor_type not in {"vehicle", "pedestrian"}:
            continue
        samples = []
        for state in actor.get("reference_trajectory") or []:
            if state.get("t_sec") is None:
                continue
            ego_state = ego_by_time.get(round(float(state["t_sec"]), 6))
            if ego_state is None:
                continue
            samples.append(_relative_sample(state, ego_state))
        if not samples:
            continue
        closest = min(samples, key=lambda item: item["distance_m"])
        first = samples[0]
        coverage = len(samples) / len(ego)
        category = str(actor.get("category") or "")
        if actor_type == "vehicle":
            eligible = (
                first["longitudinal_m"] > 0.0
                and first["longitudinal_m"] <= 80.0
                and abs(first["lateral_m"]) <= 4.0
                and len(samples) >= max(10, math.ceil(len(ego) * 0.5))
            )
            score = (
                coverage * 100.0
                + max(0.0, 60.0 - first["longitudinal_m"])
                + max(0.0, 16.0 - abs(first["lateral_m"]) * 4.0)
                + max(0.0, 25.0 - closest["distance_m"])
                + (15.0 if category == "vehicle.car" else 0.0)
            )
            reasons = [
                "source_track_long_enough" if coverage >= 0.5 else "source_track_short",
                "initially_ahead_of_ego" if first["longitudinal_m"] > 0 else "not_initially_ahead",
                "same_lane_candidate" if abs(first["lateral_m"]) <= 4.0 else "lateral_offset_large",
            ]
        else:
            eligible = (
                len(samples) >= 10
                and closest["distance_m"] <= 20.0
                and closest["t_sec"] > 0.0
            )
            score = (
                coverage * 75.0
                + max(0.0, 40.0 - closest["distance_m"] * 3.0)
                + max(0.0, 20.0 - closest["t_sec"])
            )
            reasons = [
                "source_corridor_available" if len(samples) >= 10 else "source_corridor_short",
                "ego_interaction_proximity" if closest["distance_m"] <= 20.0 else "too_far_from_ego",
                "root_pose_only_closed_loop",
            ]
        records.append(
            {
                "actor_id": str(actor["actor_id"]),
                "source_track_id": str(actor.get("source_track_id") or actor["actor_id"]),
                "actor_type": actor_type,
                "category": category,
                "trajectory_point_count": len(samples),
                "trajectory_coverage": coverage,
                "initial_relative": first,
                "closest_approach": closest,
                "eligible": eligible,
                "score": score,
                "reasons": reasons,
                "closed_loop_scope": (
                    "vehicle_control_plus_nurec_root_pose"
                    if actor_type == "vehicle"
                    else "speed_pause_yield_abort_on_source_corridor"
                ),
            }
        )
    records.sort(key=lambda item: (-bool(item["eligible"]), -float(item["score"]), item["actor_id"]))
    vehicle = next((item for item in records if item["actor_type"] == "vehicle" and item["eligible"]), None)
    pedestrian = next((item for item in records if item["actor_type"] == "pedestrian" and item["eligible"]), None)
    return {
        "schema_version": "actor_candidate_audit.v1",
        "scene_id": full_scene_ir.get("scenario_id"),
        "catalog_actor_count": len(full_scene_ir.get("actors") or []),
        "candidate_count": len(records),
        "recommendations": {
            "primary_vehicle_actor_id": vehicle["actor_id"] if vehicle else None,
            "bounded_pedestrian_actor_id": pedestrian["actor_id"] if pedestrian else None,
        },
        "candidates": records,
        "limitations": [
            "ranking_uses_source_pose_geometry_not_reconstruction_visual_quality",
            "nurec_track_inventory_must_still_be_verified",
            "carla_spawn_and_map_alignment_must_still_pass_runtime_acceptance",
        ],
    }


def _relative_sample(actor: Mapping[str, Any], ego: Mapping[str, Any]) -> dict[str, float]:
    dx = float(actor["x"]) - float(ego["x"])
    dy = float(actor["y"]) - float(ego["y"])
    yaw = math.radians(float(ego.get("yaw", 0.0)))
    return {
        "t_sec": float(actor["t_sec"]),
        "distance_m": math.hypot(dx, dy),
        "longitudinal_m": math.cos(yaw) * dx + math.sin(yaw) * dy,
        "lateral_m": -math.sin(yaw) * dx + math.cos(yaw) * dy,
    }


def _actor_index(document: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    actors = document.get("actors")
    if not isinstance(actors, list):
        raise ActorCatalogError("Scenario IR actors must be a list")
    result = {}
    for actor in actors:
        actor_id = str(actor.get("actor_id") or "") if isinstance(actor, Mapping) else ""
        if not actor_id:
            raise ActorCatalogError("Scenario IR actor_id is required")
        if actor_id in result:
            raise ActorCatalogError(f"duplicate Scenario IR actor_id: {actor_id}")
        result[actor_id] = actor
    return result


def _same_scene(left: Mapping[str, Any], right: Mapping[str, Any]) -> None:
    if left.get("schema_version") != "scenario_ir.v1" or right.get("schema_version") != "scenario_ir.v1":
        raise ActorCatalogError("actor catalog repair requires scenario_ir.v1 documents")
    if left.get("scenario_id") != right.get("scenario_id"):
        raise ActorCatalogError("Scenario IR documents refer to different scenes")


def _normalized_type(actor: Mapping[str, Any]) -> str:
    value = str(actor.get("type") or actor.get("actor_type") or "object").lower()
    return {
        "walker": "pedestrian",
        "person": "pedestrian",
        "cyclist": "two_wheeler",
    }.get(value, value)
