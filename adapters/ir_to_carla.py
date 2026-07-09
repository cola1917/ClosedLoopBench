from __future__ import annotations

from copy import deepcopy
from typing import Any

from actors.policy_config import build_actor_policy_config


DEFAULT_METRICS = ["collision", "min_ttc", "route_progress", "comfort_jerk", "rule_violation"]


def build_carla_run_config(
    scenario_ir: dict[str, Any],
    *,
    carla_map: str = "Town04",
    carla_version: str = "0.9.16",
    fixed_delta_seconds: float = 0.05,
    reconstruction_package_path: str | None = None,
) -> dict[str, Any]:
    """Build a CARLA/ScenarioRunner MVP run config from Scenario IR."""

    actors = [
        _actor_config(actor)
        for actor in scenario_ir.get("actors", [])
        if actor.get("initial_state") is not None
    ]
    return {
        "schema_version": "carla_run_config.mvp.v0",
        "scenario_id": scenario_ir["scenario_id"],
        "scenario_type": scenario_ir.get("scenario_type", "unknown"),
        "carla": {
            "version": carla_version,
            "map": carla_map,
            "fixed_delta_seconds": fixed_delta_seconds,
            "execution": "scenario_runner_python_config",
        },
        "windows": {
            "event": deepcopy(scenario_ir["windows"]["event"]),
            "warmup": deepcopy(scenario_ir["windows"]["warmup"]),
        },
        "ego": {
            "agent": "baseline_lane_following",
            "track_id": scenario_ir.get("ego", {}).get("track_id", "ego"),
            "initial_state": _carla_state(scenario_ir.get("ego", {}).get("initial_state")),
            "reference_trajectory": [
                _carla_state(state)
                for state in scenario_ir.get("ego", {}).get("reference_trajectory", [])
            ],
        },
        "actors": actors,
        "trigger": deepcopy(scenario_ir.get("events", {}).get("trigger")),
        "metrics": list(scenario_ir.get("evaluation", {}).get("metrics", DEFAULT_METRICS)),
        "reconstruction_package": {
            "enabled": reconstruction_package_path is not None,
            "package_path": reconstruction_package_path,
        },
        "format_exports": {
            "openscenario": "optional_after_mvp",
            "opendrive": "not_required_for_mvp_existing_carla_town",
            "scenic": "optional_family_generation",
        },
    }


def _actor_config(actor: dict[str, Any]) -> dict[str, Any]:
    role = actor.get("role", "context")
    policy_config = build_actor_policy_config(actor, style=str(actor.get("style", "normal")))
    policy = str(policy_config["policy_mode"])
    return {
        "actor_id": str(actor.get("actor_id")),
        "source_track_id": actor.get("source_track_id"),
        "role": role,
        "type": actor.get("type", "vehicle"),
        "policy": policy,
        "closed_loop_level": policy_config["closed_loop_level"],
        "closed_loop": deepcopy(policy_config["closed_loop"]),
        "conditioning": policy_config["conditioning"],
        "style": policy_config["style"],
        "style_profile": deepcopy(policy_config["style_profile"]),
        "initial_state": _carla_state(actor.get("initial_state")),
        "reference_trajectory": [_carla_state(state) for state in actor.get("reference_trajectory", [])],
        "behavior": {
            "source": _behavior_source(policy),
            "reference_actor_id": str(actor.get("actor_id")),
        },
    }


def _behavior_source(policy: str) -> str:
    if policy == "replay":
        return "log_replay"
    if policy == "scripted_trigger":
        return "reference_conditioned_script"
    return "reference_conditioned_rule_based"


def _carla_state(state: dict[str, Any] | None) -> dict[str, float] | None:
    if state is None:
        return None
    return {
        "t_sec": float(state.get("t_sec", 0.0)),
        "x": float(state.get("x", 0.0)),
        "y": float(state.get("y", 0.0)),
        "z": float(state.get("z", 0.0)),
        "yaw": float(state.get("yaw", 0.0)),
        "speed_mps": float(state.get("speed_mps", 0.0)),
    }
