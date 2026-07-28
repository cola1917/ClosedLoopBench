"""M6 safety-relevant scene object inventory and coverage audit.

The registry is deliberately broader than ``actor_binding_set.v1``.  An actor
binding identifies dynamic objects that NuRec can move; this registry also
records static visual obstacles and road boundaries that must constrain the
CARLA ego even though they are never controllable.
"""

from __future__ import annotations

from copy import deepcopy
from hashlib import sha256
from typing import Any, Iterable, Mapping


class SceneObjectRegistryError(ValueError):
    """Raised when a safety-relevant object cannot be represented unambiguously."""


_ROLES = {
    "static_obstacle",
    "background_replay",
    "controlled_lead_vehicle",
    "controlled_pedestrian",
    "road_boundary",
}
_DYNAMIC_TYPES = {"vehicle", "pedestrian", "two_wheeler"}


def build_scene_object_registry(
    scenario_ir: Mapping[str, Any],
    *,
    static_objects: Iterable[Mapping[str, Any]] = (),
    role_overrides: Mapping[str, str] | None = None,
    nonreplay_static_actor_ids: Iterable[str] = (),
    include_road_boundary: bool = True,
    road_boundary_source: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build a versioned M6 inventory from the full Scenario IR and annotations.

    ``static_objects`` is an explicitly curated inventory of objects baked into
    the NuRec reconstruction (for example, a parked roadside vehicle).  It is
    intentionally required as an input rather than inferred from RGB pixels:
    an inference cannot establish a physical collision proxy or its provenance.
    """

    scene_id = _scene_id(scenario_ir)
    roles = {str(key): str(value) for key, value in (role_overrides or {}).items()}
    nonreplay_static_ids = {str(actor_id) for actor_id in nonreplay_static_actor_ids}
    actors = scenario_ir.get("actors")
    if not isinstance(actors, list):
        raise SceneObjectRegistryError("scenario_ir.actors must be a list")

    records: list[dict[str, Any]] = []
    actor_ids: set[str] = set()
    for actor in actors:
        if not isinstance(actor, Mapping):
            raise SceneObjectRegistryError("every Scenario IR actor must be an object")
        actor_id = str(actor.get("actor_id") or "")
        record = _scenario_actor_record(
            actor,
            roles.get(actor_id),
            force_static=actor_id in nonreplay_static_ids,
        )
        if record["object_id"] in actor_ids:
            raise SceneObjectRegistryError(f"duplicate Scenario IR actor_id: {record['object_id']}")
        actor_ids.add(record["object_id"])
        records.append(record)
    unknown_roles = sorted(set(roles) - actor_ids)
    if unknown_roles:
        raise SceneObjectRegistryError(
            "role_overrides contains unknown Scenario IR actor IDs: " + ", ".join(unknown_roles)
        )
    unknown_nonreplay_static_ids = sorted(nonreplay_static_ids - actor_ids)
    if unknown_nonreplay_static_ids:
        raise SceneObjectRegistryError(
            "nonreplay_static_actor_ids contains unknown Scenario IR actor IDs: "
            + ", ".join(unknown_nonreplay_static_ids)
        )

    for raw in static_objects:
        records.append(_static_record(raw))
    if include_road_boundary:
        records.append(_road_boundary_record(road_boundary_source))

    object_ids = [record["object_id"] for record in records]
    if len(object_ids) != len(set(object_ids)):
        raise SceneObjectRegistryError("scene object registry contains duplicate object_id values")
    registry = {
        "schema_version": "scene_object_registry.v1",
        "scene_id": scene_id,
        "coordinate_frame": "scene_local_ego_start",
        "records": records,
        "summary": _summary(records),
    }
    validate_scene_object_registry(registry)
    return registry


def audit_scene_object_coverage(
    registry: Mapping[str, Any],
    visibility_manifest: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """Fail closed when a visible safety object lacks a physical counterpart."""

    validate_scene_object_registry(registry)
    records = list(registry["records"])
    by_id = {str(record["object_id"]): record for record in records}
    by_track = {
        str((record.get("nurec") or {}).get("track_id")): record
        for record in records
        if (record.get("nurec") or {}).get("track_id")
    }
    missing_policy = [
        record["object_id"]
        for record in records
        if record["safety_relevant"] and not str((record.get("carla") or {}).get("collision_policy") or "")
    ]
    missing_representation = [
        record["object_id"]
        for record in records
        if record["safety_relevant"] and not str((record.get("carla") or {}).get("representation") or "")
    ]
    observations: list[Mapping[str, Any]] = []
    manifest_status = "missing"
    if visibility_manifest is not None:
        observations = _visibility_observations(visibility_manifest, registry["scene_id"])
        manifest_status = "loaded"

    unresolved: list[dict[str, Any]] = []
    noncollidable_visible: list[dict[str, Any]] = []
    for observation in observations:
        if observation.get("safety_relevant") is not True:
            continue
        record = _resolve_observation(observation, by_id, by_track)
        if record is None:
            unresolved.append(dict(observation))
            continue
        carla = record["carla"]
        # Lane boundaries constrain the ego through OpenDRIVE/CARLA topology,
        # not an artificial collision body. Every other visible safety object
        # must have a collidable physical representation.
        if record["role"] != "road_boundary" and (
            carla["collision_policy"] != "required" or carla["representation"] in {"none", "road_topology"}
        ):
            noncollidable_visible.append(
                {
                    "object_id": record["object_id"],
                    "observation": dict(observation),
                    "carla_representation": carla["representation"],
                    "collision_policy": carla["collision_policy"],
                }
            )

    issues: list[str] = []
    if manifest_status == "missing":
        issues.append("nre_visibility_manifest_missing")
    if missing_policy:
        issues.append("required_object_missing_carla_collision_policy")
    if missing_representation:
        issues.append("required_object_missing_carla_representation")
    if unresolved:
        issues.append("unregistered_nre_visible_safety_object")
    if noncollidable_visible:
        issues.append("nre_visible_safety_object_is_not_collidable_in_carla")
    return {
        "schema_version": "scene_object_coverage_audit.v1",
        "scene_id": registry["scene_id"],
        "registry_schema_version": registry["schema_version"],
        "visibility_manifest_status": manifest_status,
        "observed_safety_object_count": sum(item.get("safety_relevant") is True for item in observations),
        "unregistered_nre_visible_objects": unresolved,
        "missing_carla_collision_policy": sorted(missing_policy),
        "missing_carla_representation": sorted(missing_representation),
        "noncollidable_nre_visible_objects": noncollidable_visible,
        "issues": issues,
        "status": "passed" if not issues else "failed",
    }


def assert_scene_object_coverage_ready(audit: Mapping[str, Any]) -> None:
    if audit.get("schema_version") != "scene_object_coverage_audit.v1":
        raise SceneObjectRegistryError("coverage audit must use scene_object_coverage_audit.v1")
    if audit.get("status") != "passed":
        detail = ", ".join(str(item) for item in audit.get("issues") or []) or "unknown"
        raise SceneObjectRegistryError(f"scene object coverage is not ready for M7: {detail}")


def attach_static_obstacles_to_carla_run(
    run_config: Mapping[str, Any],
    registry: Mapping[str, Any],
    *,
    registry_path: str | None = None,
    registry_sha256: str | None = None,
) -> dict[str, Any]:
    """Derive an M6 CARLA config without mutating a prior evidence config."""

    validate_scene_object_registry(registry)
    if str(run_config.get("scenario_id") or "") != str(registry["scene_id"]):
        raise SceneObjectRegistryError("CARLA run scenario_id does not match scene object registry")
    derived = deepcopy(dict(run_config))
    static_obstacles = []
    for record in registry["records"]:
        if record["role"] != "static_obstacle":
            continue
        carla = record["carla"]
        placement = carla.get("placement")
        if not isinstance(placement, Mapping):
            raise SceneObjectRegistryError(
                f"static obstacle {record['object_id']} lacks a CARLA placement"
            )
        static_obstacles.append(
            {
                "object_id": record["object_id"],
                "semantic_class": record["semantic_class"],
                "source": deepcopy(dict(record["source"])),
                "time_interval": deepcopy(dict(record["time_interval"])),
                "placement": deepcopy(dict(placement)),
                "blueprint": carla.get("blueprint_class"),
                "collision_policy": carla.get("collision_policy"),
            }
        )
    if not static_obstacles:
        raise SceneObjectRegistryError("M6 registry has no static collision proxies")
    derived["static_obstacles"] = static_obstacles
    derived["scene_object_registry"] = {
        "schema_version": registry["schema_version"],
        "scene_id": registry["scene_id"],
        "path": registry_path,
        "sha256": registry_sha256,
        "summary": deepcopy(dict(registry["summary"])),
    }
    derived.setdefault("runtime", {})["m6_static_obstacle_required"] = True
    return derived


def attach_dynamic_replay_to_carla_run(
    run_config: Mapping[str, Any],
    registry: Mapping[str, Any],
    scenario_ir: Mapping[str, Any],
) -> dict[str, Any]:
    """Attach every registered dynamic source track as a physical replay actor.

    The M6 probe deliberately preserves any existing bound actor declarations
    (the lead vehicle and pedestrian), while adding the rest as replay-only
    context.  This is registration and physical-presence work, not a claim
    that every added track is yet pose-bound to NuRec; that is M7.
    """

    validate_scene_object_registry(registry)
    if str(scenario_ir.get("scenario_id") or "") != str(registry["scene_id"]):
        raise SceneObjectRegistryError("Scenario IR scene_id does not match scene object registry")
    source_actors = scenario_ir.get("actors")
    if not isinstance(source_actors, list):
        raise SceneObjectRegistryError("Scenario IR actors must be a list")
    source_by_id = {
        str(actor.get("actor_id") or ""): actor
        for actor in source_actors
        if isinstance(actor, Mapping) and str(actor.get("actor_id") or "")
    }
    derived = deepcopy(dict(run_config))
    existing = {
        str(actor.get("actor_id") or ""): actor
        for actor in (derived.get("actors") or [])
        if isinstance(actor, Mapping) and str(actor.get("actor_id") or "")
    }
    replay_actors = []
    for record in registry["records"]:
        if record["role"] not in {
            "background_replay",
            "controlled_lead_vehicle",
            "controlled_pedestrian",
        }:
            continue
        actor_id = str(record["object_id"])
        source = source_by_id.get(actor_id)
        if source is None:
            raise SceneObjectRegistryError(f"registry dynamic object {actor_id} is absent from Scenario IR")
        candidate = deepcopy(dict(source))
        prior = existing.get(actor_id)
        if prior is not None:
            for key in (
                "role_name",
                "blueprint",
                "binding",
                "control_mode_contract",
                "effective_control_mode",
                "style",
                "style_profile",
                "behavior",
            ):
                if key in prior:
                    candidate[key] = deepcopy(prior[key])
        candidate["role"] = record["role"]
        candidate["policy"] = "replay"
        candidate["closed_loop_level"] = "replay"
        candidate["closed_loop"] = {"name": "replay", "ego_responsive": False}
        candidate["m6_allow_vertical_pose_calibration"] = True
        candidate["m6_max_vertical_spawn_adjustment_m"] = 0.5
        candidate.setdefault("effective_control_mode", "replay")
        candidate.setdefault("role_name", f"m6.dynamic.{actor_id[:20]}")
        candidate.setdefault("blueprint", _dynamic_blueprint(record))
        replay_actors.append(candidate)
    if not replay_actors:
        raise SceneObjectRegistryError("M6 registry has no dynamic replay actors")
    derived["actors"] = replay_actors
    derived.setdefault("runtime", {})["m6_dynamic_replay_required"] = True
    return derived


def registry_sha256(path_bytes: bytes) -> str:
    """Provide the canonical sidecar identity without silently reserializing it."""

    return sha256(path_bytes).hexdigest()


def validate_scene_object_registry(registry: Mapping[str, Any]) -> None:
    if registry.get("schema_version") != "scene_object_registry.v1":
        raise SceneObjectRegistryError("registry must use scene_object_registry.v1")
    if not str(registry.get("scene_id") or ""):
        raise SceneObjectRegistryError("registry scene_id is required")
    records = registry.get("records")
    if not isinstance(records, list) or not records:
        raise SceneObjectRegistryError("registry must contain at least one object record")
    seen: set[str] = set()
    for record in records:
        if not isinstance(record, Mapping):
            raise SceneObjectRegistryError("registry records must be objects")
        object_id = str(record.get("object_id") or "")
        if not object_id or object_id in seen:
            raise SceneObjectRegistryError("registry object_id values must be unique and non-empty")
        seen.add(object_id)
        if record.get("role") not in _ROLES:
            raise SceneObjectRegistryError(f"object {object_id} has an invalid role")
        carla = record.get("carla")
        nurec = record.get("nurec")
        if not isinstance(carla, Mapping) or not isinstance(nurec, Mapping):
            raise SceneObjectRegistryError(f"object {object_id} requires CARLA and NRE representations")
        if not str(carla.get("representation") or "") or not str(carla.get("collision_policy") or ""):
            raise SceneObjectRegistryError(f"object {object_id} requires CARLA representation and collision_policy")
        if record.get("role") == "road_boundary" and carla.get("representation") != "road_topology":
            raise SceneObjectRegistryError("road_boundary must use CARLA road_topology")
        if record.get("role") == "static_obstacle" and carla.get("collision_policy") != "required":
            raise SceneObjectRegistryError("static_obstacle must require a CARLA collision policy")


def _scene_id(scenario_ir: Mapping[str, Any]) -> str:
    if scenario_ir.get("schema_version") != "scenario_ir.v1":
        raise SceneObjectRegistryError("scene object registry requires scenario_ir.v1")
    scene_id = str(scenario_ir.get("scenario_id") or "")
    if not scene_id:
        raise SceneObjectRegistryError("Scenario IR scenario_id is required")
    return scene_id


def _scenario_actor_record(
    actor: Mapping[str, Any], role_override: str | None, *, force_static: bool = False
) -> dict[str, Any]:
    actor_id = str(actor.get("actor_id") or "")
    if not actor_id:
        raise SceneObjectRegistryError("every Scenario IR actor requires actor_id")
    actor_type = str(actor.get("type") or actor.get("actor_type") or "object").lower()
    actor_type = {"walker": "pedestrian", "person": "pedestrian", "motorcycle": "two_wheeler"}.get(actor_type, actor_type)
    is_dynamic = actor_type in _DYNAMIC_TYPES and not force_static
    points = actor.get("reference_trajectory")
    if not isinstance(points, list) or not points:
        raise SceneObjectRegistryError(f"dynamic actor {actor_id} requires reference_trajectory")
    interval = _interval(points, actor_id)
    role = role_override or ("background_replay" if is_dynamic else "static_obstacle")
    if not is_dynamic and role_override is not None:
        raise SceneObjectRegistryError(
            f"single-observation/static source object {actor_id} cannot be assigned a dynamic control role"
        )
    if is_dynamic and role not in _ROLES - {"static_obstacle", "road_boundary"}:
        raise SceneObjectRegistryError(f"dynamic actor {actor_id} has invalid role {role}")
    if role == "controlled_lead_vehicle" and actor_type != "vehicle":
        raise SceneObjectRegistryError("controlled_lead_vehicle must reference a vehicle")
    if role == "controlled_pedestrian" and actor_type != "pedestrian":
        raise SceneObjectRegistryError("controlled_pedestrian must reference a pedestrian")
    source_track = str(actor.get("source_track_id") or actor_id)
    relevant = True
    return {
        "object_id": actor_id,
        "semantic_class": actor_type,
        "category": str(actor.get("category") or "unknown"),
        "role": role,
        "safety_relevant": relevant,
        "source": {
            "kind": "nuscenes_instance_track" if is_dynamic else "nuscenes_single_observation_track",
            "source_track_id": source_track,
            "annotation_tokens": list(actor.get("source_annotation_tokens") or []),
        },
        "time_interval": interval,
        "carla": (
            {
                "representation": "physical_actor",
                "collision_policy": "required",
                "blueprint_class": "walker.pedestrian.*" if actor_type == "pedestrian" else "vehicle.*",
            }
            if is_dynamic
            else {
                "representation": "static_collision_proxy",
                "collision_policy": "required",
                "blueprint_class": (
                    "walker.pedestrian.*"
                    if actor_type == "pedestrian"
                    else "vehicle.*"
                    if actor_type in {"vehicle", "two_wheeler"}
                    else _static_blueprint(actor_type, str(actor.get("category") or ""))
                ),
                "placement": _placement(actor.get("initial_state") or {}, actor_id),
            }
        ),
        "nurec": {
            "representation": "dynamic_track" if is_dynamic else "source_scene_appearance",
            "track_id": source_track,
        },
        "control": {
            "mode": "replay" if is_dynamic else "none",
            "controllable": is_dynamic and role.startswith("controlled_"),
        },
    }


def _static_record(raw: Mapping[str, Any]) -> dict[str, Any]:
    record = deepcopy(dict(raw))
    object_id = str(record.get("object_id") or "")
    if not object_id:
        raise SceneObjectRegistryError("static object inventory entry requires object_id")
    source = record.get("source")
    placement = record.get("placement")
    if not isinstance(source, Mapping) or not str(source.get("kind") or ""):
        raise SceneObjectRegistryError(f"static object {object_id} requires source.kind provenance")
    if not isinstance(placement, Mapping) or not {"x", "y", "z", "yaw"}.issubset(placement):
        raise SceneObjectRegistryError(f"static object {object_id} requires scene-local placement x/y/z/yaw")
    return {
        "object_id": object_id,
        "semantic_class": str(record.get("semantic_class") or "static_obstacle"),
        "category": str(record.get("category") or "unknown"),
        "role": "static_obstacle",
        "safety_relevant": bool(record.get("safety_relevant", True)),
        "source": deepcopy(dict(source)),
        "time_interval": deepcopy(dict(record.get("time_interval") or {"start_sec": 0.0, "end_sec": None})),
        "carla": {
            "representation": str(record.get("carla_representation") or "static_collision_proxy"),
            "collision_policy": "required",
            "blueprint_class": str(record.get("blueprint_class") or "vehicle.*"),
            "placement": {key: float(placement[key]) for key in ("x", "y", "z", "yaw")},
        },
        "nurec": {
            "representation": str(record.get("nurec_representation") or "source_scene_appearance"),
            "track_id": None,
        },
        "control": {"mode": "none", "controllable": False},
    }


def _road_boundary_record(source: Mapping[str, Any] | None) -> dict[str, Any]:
    return {
        "object_id": "road_boundary:carla_map",
        "semantic_class": "road_boundary",
        "category": "road.lane_boundary",
        "role": "road_boundary",
        "safety_relevant": True,
        "source": deepcopy(dict(source or {"kind": "opendrive_carla_map"})),
        "time_interval": {"start_sec": 0.0, "end_sec": None},
        "carla": {"representation": "road_topology", "collision_policy": "not_applicable"},
        "nurec": {"representation": "projection_target", "track_id": None},
        "control": {"mode": "none", "controllable": False},
    }


def _interval(points: list[Any], object_id: str) -> dict[str, float]:
    try:
        times = [float(point["t_sec"]) for point in points if isinstance(point, Mapping)]
    except (KeyError, TypeError, ValueError) as exc:
        raise SceneObjectRegistryError(f"dynamic actor {object_id} has invalid trajectory time") from exc
    if not times:
        raise SceneObjectRegistryError(f"dynamic actor {object_id} has no trajectory times")
    return {"start_sec": min(times), "end_sec": max(times)}


def _placement(state: Mapping[str, Any], object_id: str) -> dict[str, float]:
    try:
        return {key: float(state[key]) for key in ("x", "y", "z", "yaw")}
    except (KeyError, TypeError, ValueError) as exc:
        raise SceneObjectRegistryError(
            f"static source object {object_id} requires initial_state x/y/z/yaw"
        ) from exc


def _static_blueprint(actor_type: str, category: str) -> str:
    if actor_type == "object" and "trafficcone" in category:
        return "static.prop.trafficcone01"
    if actor_type == "object" and "barrier" in category:
        return "static.prop.streetbarrier"
    return "static.prop.*"


def _dynamic_blueprint(record: Mapping[str, Any]) -> str:
    semantic_class = str(record.get("semantic_class") or "")
    category = str(record.get("category") or "")
    if semantic_class == "pedestrian":
        return "walker.pedestrian.*"
    if semantic_class == "two_wheeler":
        return "vehicle.bh.crossbike" if "bicycle" in category else "vehicle.yamaha.yzf"
    return "vehicle.*"


def _summary(records: list[Mapping[str, Any]]) -> dict[str, int]:
    return {
        "object_count": len(records),
        "dynamic_count": sum(record["role"] in {"background_replay", "controlled_lead_vehicle", "controlled_pedestrian"} for record in records),
        "static_obstacle_count": sum(record["role"] == "static_obstacle" for record in records),
        "road_boundary_count": sum(record["role"] == "road_boundary" for record in records),
        "controlled_actor_count": sum(bool(record["control"]["controllable"]) for record in records),
    }


def _visibility_observations(manifest: Mapping[str, Any], scene_id: str) -> list[Mapping[str, Any]]:
    if manifest.get("schema_version") != "scene_object_visibility_manifest.v1":
        raise SceneObjectRegistryError("visibility manifest must use scene_object_visibility_manifest.v1")
    if manifest.get("scene_id") != scene_id:
        raise SceneObjectRegistryError("visibility manifest scene_id does not match registry")
    observations = manifest.get("observations")
    if not isinstance(observations, list):
        raise SceneObjectRegistryError("visibility manifest observations must be a list")
    for item in observations:
        if not isinstance(item, Mapping) or not (item.get("object_id") or item.get("source_track_id")):
            raise SceneObjectRegistryError("visibility observation requires object_id or source_track_id")
    return observations


def _resolve_observation(
    observation: Mapping[str, Any], by_id: Mapping[str, Mapping[str, Any]], by_track: Mapping[str, Mapping[str, Any]],
) -> Mapping[str, Any] | None:
    object_id = observation.get("object_id")
    if object_id is not None:
        return by_id.get(str(object_id))
    return by_track.get(str(observation.get("source_track_id")))
