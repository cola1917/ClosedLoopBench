from __future__ import annotations

import re
from copy import deepcopy
from typing import Any, Iterable, Mapping

from adapters.shared_protocol_validation import validate_document


class ActorBindingError(ValueError):
    """Raised when one physical actor cannot be identified across runtimes."""


_MODE_ALIASES = {
    "replay": "replay",
    "scripted": "scripted",
    "scripted_trigger": "scripted",
    "traffic_manager": "traffic_manager",
    "traffic_manager_reactive": "traffic_manager",
    "reactive_rule_based": "traffic_manager",
}

_MODE_TO_CLOSURE = {
    "replay": "replay",
    "scripted": "scripted",
    "traffic_manager": "traffic_manager_reactive",
}


def build_actor_binding_set(
    scenario_ir: Mapping[str, Any],
    *,
    selected_actor_ids: Iterable[str] | None = None,
    nurec_track_ids: Iterable[str] | None = None,
    control_modes: Mapping[str, str] | None = None,
    role_names: Mapping[str, str] | None = None,
    blueprints: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Bind selected nuScenes actors to Scenario IR, CARLA and NuRec identities.

    ``nurec_track_ids`` must come from the built USDZ/NCore asset inventory. If it
    is omitted, bindings remain blocked instead of assuming that a matching source
    token survived reconstruction.
    """

    scene_id, source = _scene_identity(scenario_ir)
    actors = scenario_ir.get("actors")
    if not isinstance(actors, list):
        raise ActorBindingError("scenario_ir.actors must be a list")
    actors_by_id: dict[str, Mapping[str, Any]] = {}
    for actor in actors:
        if not isinstance(actor, Mapping):
            raise ActorBindingError("every Scenario IR actor must be an object")
        actor_id = str(actor.get("actor_id") or "")
        if not actor_id:
            raise ActorBindingError("every Scenario IR actor requires actor_id")
        if actor_id in actors_by_id:
            raise ActorBindingError(f"duplicate Scenario IR actor_id: {actor_id}")
        actors_by_id[actor_id] = actor

    selected = list(actors_by_id) if selected_actor_ids is None else [str(value) for value in selected_actor_ids]
    if len(selected) != len(set(selected)):
        raise ActorBindingError("selected_actor_ids contains duplicates")
    missing = sorted(set(selected) - set(actors_by_id))
    if missing:
        raise ActorBindingError(f"selected actor IDs do not exist in Scenario IR: {', '.join(missing)}")

    inventory = None if nurec_track_ids is None else {str(value) for value in nurec_track_ids}
    requested_modes = {str(key): str(value) for key, value in (control_modes or {}).items()}
    requested_roles = {str(key): str(value) for key, value in (role_names or {}).items()}
    requested_blueprints = {str(key): str(value) for key, value in (blueprints or {}).items()}
    _reject_unknown_overrides(
        selected,
        control_modes=requested_modes,
        role_names=requested_roles,
        blueprints=requested_blueprints,
    )

    bindings = [
        _build_binding(
            actors_by_id[actor_id],
            inventory=inventory,
            requested_mode=requested_modes.get(actor_id),
            role_name=requested_roles.get(actor_id),
            blueprint=requested_blueprints.get(actor_id),
        )
        for actor_id in selected
    ]
    blockers = sorted(
        f"{binding['actor_id']}:{issue}"
        for binding in bindings
        for issue in binding["issues"]
    )
    if not bindings:
        blockers = ["no_actor_selected"]
        readiness = "empty"
    elif all(binding["status"] == "ready" for binding in bindings):
        readiness = "ready"
    else:
        readiness = "blocked"
    result = {
        "schema_version": "actor_binding_set.v1",
        "scene_id": scene_id,
        "source": {"dataset": "nuscenes", "scene_token": source["scene_token"]},
        "coordinate_contract": {
            "scenario_ir": "scene_local_ego_start",
            "carla": "scene_local_ego_start",
            "nurec": "nuscenes_global",
            "alignment_source": "closed_loop_scene_package.v1.alignment",
        },
        "bindings": bindings,
        "summary": {
            "selected_count": len(bindings),
            "ready_count": sum(binding["status"] == "ready" for binding in bindings),
            "interactive_count": sum(binding["control"]["ego_responsive"] for binding in bindings),
            "vehicle_count": sum(binding["actor_type"] == "vehicle" for binding in bindings),
            "pedestrian_count": sum(binding["actor_type"] == "pedestrian" for binding in bindings),
        },
        "readiness": {"status": readiness, "blockers": blockers},
    }
    validate_actor_binding_set(result)
    return result


def validate_actor_binding_set(binding_set: Mapping[str, Any]) -> None:
    """Validate schema and cross-field actor identity semantics."""

    try:
        validate_document(dict(binding_set))
    except ValueError as exc:
        raise ActorBindingError(str(exc)) from exc


def assert_actor_binding_ready(binding_set: Mapping[str, Any]) -> None:
    """Fail closed before claiming a CARLA/NuRec multimodal actor loop."""

    validate_actor_binding_set(binding_set)
    readiness = binding_set["readiness"]
    if readiness["status"] != "ready":
        detail = ", ".join(readiness["blockers"]) or readiness["status"]
        raise ActorBindingError(f"actor binding set is not multimodal-ready: {detail}")


def bind_carla_run_config(
    run_config: Mapping[str, Any],
    binding_set: Mapping[str, Any],
    *,
    require_ready: bool = True,
) -> dict[str, Any]:
    """Attach verified identities to the CARLA actors consumed by the runner."""

    validate_actor_binding_set(binding_set)
    if require_ready:
        assert_actor_binding_ready(binding_set)
    if str(run_config.get("scenario_id") or "") != binding_set["scene_id"]:
        raise ActorBindingError("CARLA run scenario_id does not match Actor Binding Set")
    actors = run_config.get("actors")
    if not isinstance(actors, list):
        raise ActorBindingError("CARLA run actors must be a list")
    run_by_id: dict[str, dict[str, Any]] = {}
    for actor in actors:
        if not isinstance(actor, dict):
            raise ActorBindingError("every CARLA run actor must be an object")
        actor_id = str(actor.get("actor_id") or "")
        if actor_id in run_by_id:
            raise ActorBindingError(f"duplicate CARLA run actor_id: {actor_id}")
        run_by_id[actor_id] = actor

    bound = deepcopy(dict(run_config))
    bound_by_id = {str(actor.get("actor_id") or ""): actor for actor in bound["actors"]}
    for binding in binding_set["bindings"]:
        actor_id = binding["actor_id"]
        actor = bound_by_id.get(actor_id)
        if actor is None:
            raise ActorBindingError(f"bound actor is missing from CARLA run config: {actor_id}")
        expected_closure = _MODE_TO_CLOSURE[binding["control"]["mode"]]
        actual_closure = str(actor.get("closed_loop_level") or "")
        if actual_closure and actual_closure != expected_closure:
            raise ActorBindingError(
                f"actor {actor_id} control mismatch: binding={expected_closure}, run={actual_closure}"
            )
        if str(actor.get("source_track_id") or "") not in {"", binding["source_track_id"]}:
            raise ActorBindingError(f"actor {actor_id} source_track_id conflicts with binding")
        actor["source_track_id"] = binding["source_track_id"]
        actor["role_name"] = binding["carla"]["role_name"]
        actor["blueprint"] = binding["carla"]["blueprint"]
        actor["binding"] = {
            "schema_version": "actor_runtime_binding.v1",
            "nurec_track_id": binding["nurec"]["track_id"],
            "sensor_pose_source": binding["sensor_sync"]["pose_source"],
            "sensor_pose_reference": binding["sensor_sync"]["pose_reference"],
            "required_modalities": list(binding["sensor_sync"]["required_modalities"]),
            "same_dynamic_object_for_all_modalities": True,
            "declared_status": binding["status"],
        }
    bound["actor_binding"] = {
        "schema_version": binding_set["schema_version"],
        "scene_id": binding_set["scene_id"],
        "readiness": deepcopy(binding_set["readiness"]),
        "selected_actor_ids": [item["actor_id"] for item in binding_set["bindings"]],
    }
    return bound


def _build_binding(
    actor: Mapping[str, Any],
    *,
    inventory: set[str] | None,
    requested_mode: str | None,
    role_name: str | None,
    blueprint: str | None,
) -> dict[str, Any]:
    actor_id = str(actor["actor_id"])
    source_track_id = str(actor.get("source_track_id") or actor_id)
    if not source_track_id:
        raise ActorBindingError(f"actor {actor_id} has no source_track_id")
    actor_type = _actor_type(actor)
    mode = _control_mode(actor, requested_mode)
    if actor_type == "pedestrian" and mode == "traffic_manager":
        raise ActorBindingError(
            f"pedestrian actor {actor_id} cannot use CARLA TrafficManager; use replay or scripted"
        )
    interactive = mode != "replay"
    issues: list[str] = []
    if actor_type == "object":
        issues.append("carla_physical_actor_type_unsupported")
    if not actor.get("initial_state"):
        issues.append("initial_state_missing")
    if not actor.get("reference_trajectory"):
        issues.append("reference_trajectory_empty")
    if inventory is None:
        inventory_verified = False
        issues.append("nurec_track_inventory_not_provided")
    else:
        inventory_verified = source_track_id in inventory
        if not inventory_verified:
            issues.append("nurec_track_not_found")

    if any(issue.endswith("unsupported") or issue.endswith("missing") or issue.endswith("empty") for issue in issues):
        status = "unsupported"
    elif not inventory_verified:
        status = "pending_nurec_track"
    else:
        status = "ready"
    return {
        "actor_id": actor_id,
        "actor_type": actor_type,
        "category": str(actor.get("category") or ""),
        "role": str(actor.get("role") or "context"),
        "source_track_id": source_track_id,
        "control": {
            "mode": mode,
            "ego_responsive": interactive,
            "capabilities": _capabilities(actor_type, mode),
            "corridor_constraint": (
                "source_reference"
                if actor_type == "pedestrian" and mode == "scripted"
                else "none"
            ),
        },
        "carla": {
            "role_name": role_name or _default_role_name(actor_id),
            "blueprint": blueprint or _default_blueprint(actor_type),
            "runtime_actor_id": None,
        },
        "nurec": {
            "track_id": source_track_id,
            "inventory_verified": inventory_verified,
            "dynamic_object_pose_supported": inventory_verified,
        },
        "sensor_sync": {
            "required_modalities": ["rgb", "lidar"],
            "pose_source": (
                "carla_runtime_actor_pose"
                if interactive
                else "scenario_ir_reference_trajectory"
            ),
            "pose_reference": (
                "source_track_frame"
                if not interactive
                else (
                    "carla_actor_origin"
                    if actor_type == "pedestrian"
                    else "carla_bounding_box_center"
                )
            ),
            "same_dynamic_object_for_all_modalities": True,
        },
        "status": status,
        "issues": sorted(set(issues)),
    }


def _scene_identity(scenario_ir: Mapping[str, Any]) -> tuple[str, Mapping[str, Any]]:
    if scenario_ir.get("schema_version") != "scenario_ir.v1":
        raise ActorBindingError("actor binding requires scenario_ir.v1")
    source = scenario_ir.get("source")
    if not isinstance(source, Mapping) or str(source.get("dataset") or "").lower() != "nuscenes":
        raise ActorBindingError("actor binding currently requires a nuScenes source")
    scene_id = str(scenario_ir.get("scenario_id") or "")
    scene_token = str(source.get("scene_token") or "")
    if not re.fullmatch(r"[0-9a-f]{32}", scene_id) or scene_token != scene_id:
        raise ActorBindingError("Scenario IR scenario_id and source.scene_token must be the same token")
    return scene_id, source


def _reject_unknown_overrides(selected: list[str], **groups: Mapping[str, str]) -> None:
    allowed = set(selected)
    for label, values in groups.items():
        unknown = sorted(set(values) - allowed)
        if unknown:
            raise ActorBindingError(f"{label} contains unselected actor IDs: {', '.join(unknown)}")


def _control_mode(actor: Mapping[str, Any], requested: str | None) -> str:
    raw = requested
    if raw is None:
        raw = str(
            actor.get("closed_loop_level")
            or actor.get("policy")
            or (actor.get("policy_hints") or {}).get("mvp")
            or "replay"
        )
    mode = _MODE_ALIASES.get(str(raw).lower())
    if mode is None:
        raise ActorBindingError(f"unsupported actor control mode: {raw}")
    return mode


def _actor_type(actor: Mapping[str, Any]) -> str:
    raw = str(actor.get("type") or actor.get("actor_type") or "object").lower()
    aliases = {"walker": "pedestrian", "person": "pedestrian", "motorcycle": "two_wheeler"}
    value = aliases.get(raw, raw)
    return value if value in {"vehicle", "pedestrian", "two_wheeler", "object"} else "object"


def _capabilities(actor_type: str, mode: str) -> list[str]:
    if mode == "replay":
        return ["trajectory_replay"]
    if actor_type == "pedestrian":
        return ["speed", "pause", "yield", "abort"]
    if mode == "traffic_manager":
        return ["speed", "throttle", "brake", "steer", "yield"]
    return ["speed", "throttle", "brake", "steer", "yield", "abort"]


def _default_blueprint(actor_type: str) -> str:
    return "walker.pedestrian.*" if actor_type == "pedestrian" else "vehicle.*"


def _default_role_name(actor_id: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_.-]", "_", actor_id)
    return f"actor.{safe[:24]}"
