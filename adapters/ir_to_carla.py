from __future__ import annotations

from copy import deepcopy
from typing import Any

from actors.policy_config import build_actor_policy_config
from actors.closure_levels import actor_closure_level_for_policy


DEFAULT_METRICS = ["collision", "min_ttc", "route_progress", "comfort_jerk", "rule_violation"]


def build_carla_run_config(
    scenario_ir: dict[str, Any],
    *,
    carla_map: str = "Town04",
    carla_version: str = "0.9.16",
    fixed_delta_seconds: float = 0.05,
    reconstruction_package_path: str | None = None,
    weather: str | None = None,
    odd_id: str | None = None,
    seed: int | None = None,
    algorithm_id: str = "basic_agent",
    algorithm_version: str | None = None,
    run_id: str | None = None,
    scene_version: str | None = None,
    actor_control_mode: str = "mixed",
    actor_style: str = "normal",
) -> dict[str, Any]:
    """Build a CARLA/ScenarioRunner MVP run config from Scenario IR."""

    actors = [
        _actor_config(actor, control_mode=actor_control_mode, style=actor_style)
        for actor in scenario_ir.get("actors", [])
        if actor.get("initial_state") is not None
    ]
    return {
        "schema_version": "carla_run_config.mvp.v0",
        "run_id": run_id,
        "scenario_id": scenario_ir["scenario_id"],
        "scenario_type": scenario_ir.get("scenario_type", "unknown"),
        "carla": {
            "version": carla_version,
            "map": carla_map,
            "fixed_delta_seconds": fixed_delta_seconds,
            "execution": "scenario_runner_python_config",
            "weather": weather,
            "odd_id": odd_id,
            "seed": seed,
        },
        "windows": {
            "event": deepcopy(scenario_ir["windows"]["event"]),
            "warmup": deepcopy(scenario_ir["windows"]["warmup"]),
        },
        "ego": {
            "agent": "baseline_lane_following",
            "algorithm_id": algorithm_id,
            "algorithm_version": algorithm_version,
            "track_id": scenario_ir.get("ego", {}).get("track_id", "ego"),
            "initial_state": _carla_state(scenario_ir.get("ego", {}).get("initial_state")),
            "reference_trajectory": [
                _carla_state(state)
                for state in scenario_ir.get("ego", {}).get("reference_trajectory", [])
            ],
        },
        "experiment": {
            "run_id": run_id,
            "scene_version": scene_version,
            "algorithm_id": algorithm_id,
            "algorithm_version": algorithm_version,
            "odd_id": odd_id or weather or "default",
            "seed": seed,
        },
        "actor_control": {
            "mode": actor_control_mode,
            "style": actor_style,
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


def _actor_config(
    actor: dict[str, Any],
    *,
    control_mode: str = "mixed",
    style: str = "normal",
) -> dict[str, Any]:
    role = actor.get("role", "context")
    if control_mode not in {"replay", "scripted", "traffic_manager", "mixed"}:
        raise ValueError(f"unsupported actor control mode: {control_mode}")
    selected_style = style if style is not None else str(actor.get("style", "normal"))
    policy_config = build_actor_policy_config(actor, style=selected_style)
    policy = str(policy_config["policy_mode"])
    if control_mode != "mixed":
        policy = {
            "replay": "replay",
            "scripted": "scripted_trigger",
            "traffic_manager": "reactive_rule_based",
        }[control_mode]
        closure = actor_closure_level_for_policy(policy)
        policy_config["policy_mode"] = policy
        policy_config["closed_loop_level"] = closure.name
        policy_config["closed_loop"] = closure.to_dict()
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
