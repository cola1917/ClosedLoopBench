from collections import Counter
from typing import Any, Dict, Iterable, Mapping

from actors.policy_config import build_actor_policy_config


_LEVEL_TO_RUNTIME_MODE = {
    "replay": "replay",
    "scripted": "scripted",
    "traffic_manager_reactive": "traffic_manager",
}


def build_actor_runtime_plan(actor: Mapping[str, Any], policy_config: Mapping[str, Any]) -> Dict[str, Any]:
    """Build a serializable actor execution plan without touching CARLA runtime APIs."""
    closed_loop_level = str(policy_config.get("closed_loop_level", "traffic_manager_reactive"))
    runtime_mode = _LEVEL_TO_RUNTIME_MODE.get(closed_loop_level, "traffic_manager")
    closed_loop = policy_config.get("closed_loop", {})
    style_profile = dict(policy_config.get("style_profile", {}))

    return {
        "schema_version": "actor_runtime_plan.mvp.v0",
        "actor_id": _actor_id(actor, policy_config),
        "role": str(actor.get("role", policy_config.get("role", "context"))).lower(),
        "actor_type": str(actor.get("type", actor.get("actor_type", "vehicle"))),
        "runtime_mode": runtime_mode,
        "policy_mode": str(policy_config.get("policy_mode", actor.get("policy", "replay"))),
        "closed_loop_level": closed_loop_level,
        "closed_loop": dict(closed_loop),
        "interactive_candidate": bool(closed_loop.get("ego_responsive", False)),
        "requires_carla_runtime": bool(closed_loop.get("requires_carla_runtime", False)),
        "initial_state": dict(actor.get("initial_state", {})),
        "reference": _reference_summary(actor),
        "style": str(policy_config.get("style", style_profile.get("name", "normal"))),
        "style_profile": style_profile,
        "controller": _controller_plan(runtime_mode, actor, style_profile),
    }


def build_actor_runtime_plan_set(run_config: Mapping[str, Any], style: str = "normal") -> Dict[str, Any]:
    actors = list(_iter_actors(run_config))
    plans = [
        build_actor_runtime_plan(actor, build_actor_policy_config(actor, style=style))
        for actor in actors
    ]
    runtime_modes = Counter(plan["runtime_mode"] for plan in plans)

    return {
        "schema_version": "actor_runtime_plan.mvp.v0",
        "scenario_id": str(run_config.get("scenario_id", "scenario")),
        "source_schema_version": run_config.get("schema_version"),
        "actors": plans,
        "summary": {
            "actor_count": len(plans),
            "runtime_modes": dict(sorted(runtime_modes.items())),
            "interactive_candidate_count": sum(1 for plan in plans if plan["interactive_candidate"]),
            "requires_carla_runtime_count": sum(1 for plan in plans if plan["requires_carla_runtime"]),
        },
        "runtime_boundary": {
            "owns_carla_control_loop": False,
            "owns_traffic_manager_api_calls": False,
            "plan_only": True,
        },
    }


def _controller_plan(runtime_mode: str, actor: Mapping[str, Any], style_profile: Mapping[str, Any]) -> Dict[str, Any]:
    if runtime_mode == "replay":
        return {
            "type": "trajectory_replay",
            "reference_source": "scenario_ir",
            "trajectory_points": len(actor.get("reference_trajectory", [])),
            "clock": "simulation_time",
            "ego_reactive": False,
        }
    if runtime_mode == "scripted":
        return {
            "type": "scripted_reference_conditioned",
            "reference_source": "scenario_ir",
            "trajectory_points": len(actor.get("reference_trajectory", [])),
            "trigger_conditions": {
                "source": "scenario_trigger_or_runtime_ttc",
                "yield_ttc_threshold_sec": style_profile.get("yield_ttc_threshold_sec"),
                "reaction_time_sec": style_profile.get("reaction_time_sec"),
                "abort_on_low_ttc": style_profile.get("abort_on_low_ttc"),
            },
            "runtime_binding": "deferred",
        }
    return {
        "type": "carla_traffic_manager",
        "runtime_binding": "deferred",
        "parameters": {
            "desired_time_headway_sec": style_profile.get("desired_time_headway_sec"),
            "min_gap_m": style_profile.get("min_gap_m"),
            "reaction_time_sec": style_profile.get("reaction_time_sec"),
            "yield_ttc_threshold_sec": style_profile.get("yield_ttc_threshold_sec"),
            "lane_change_gap_acceptance_m": style_profile.get("lane_change_gap_acceptance_m"),
            "abort_on_low_ttc": style_profile.get("abort_on_low_ttc"),
        },
        "reference_source": "scenario_ir_initial_state_and_route_hint",
    }


def _reference_summary(actor: Mapping[str, Any]) -> Dict[str, Any]:
    trajectory = actor.get("reference_trajectory", [])
    return {
        "source": "scenario_ir",
        "trajectory_points": len(trajectory),
        "has_initial_state": bool(actor.get("initial_state")),
    }


def _actor_id(actor: Mapping[str, Any], policy_config: Mapping[str, Any]) -> str:
    for key in ("actor_id", "id", "name"):
        value = actor.get(key)
        if value:
            return str(value)
    return str(policy_config.get("actor_id", "actor"))


def _iter_actors(run_config: Mapping[str, Any]) -> Iterable[Mapping[str, Any]]:
    actors = run_config.get("actors", [])
    if not isinstance(actors, list):
        raise ValueError("run_config['actors'] must be a list")
    return actors
