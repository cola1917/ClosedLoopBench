from dataclasses import asdict, dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class ActorClosureLevel:
    name: str
    description: str
    ego_responsive: bool
    actor_state_mutates_from_ego: bool
    requires_carla_runtime: bool

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


_CLOSURE_LEVELS: Dict[str, ActorClosureLevel] = {
    "replay": ActorClosureLevel(
        name="replay",
        description="Recorded actor trajectory is replayed; ego behavior does not change actor motion.",
        ego_responsive=False,
        actor_state_mutates_from_ego=False,
        requires_carla_runtime=False,
    ),
    "scripted": ActorClosureLevel(
        name="scripted",
        description="Actor follows a reference-conditioned maneuver and can react to ego state or trigger conditions.",
        ego_responsive=True,
        actor_state_mutates_from_ego=True,
        requires_carla_runtime=True,
    ),
    "traffic_manager_reactive": ActorClosureLevel(
        name="traffic_manager_reactive",
        description="Actor behavior is delegated to CARLA TrafficManager or an equivalent reactive controller.",
        ego_responsive=True,
        actor_state_mutates_from_ego=True,
        requires_carla_runtime=True,
    ),
}


_POLICY_TO_LEVEL = {
    "replay": "replay",
    "scripted_trigger": "scripted",
    "reactive_rule_based": "traffic_manager_reactive",
    "traffic_manager": "traffic_manager_reactive",
}


def available_actor_closure_levels() -> Tuple[str, ...]:
    return tuple(_CLOSURE_LEVELS.keys())


def get_actor_closure_level(level: str) -> ActorClosureLevel:
    try:
        return _CLOSURE_LEVELS[level]
    except KeyError as exc:
        choices = ", ".join(available_actor_closure_levels())
        raise ValueError(f"Unknown actor closure level '{level}'. Expected one of: {choices}") from exc


def actor_closure_level_for_policy(policy_mode: str) -> ActorClosureLevel:
    return get_actor_closure_level(_POLICY_TO_LEVEL.get(policy_mode, "traffic_manager_reactive"))
