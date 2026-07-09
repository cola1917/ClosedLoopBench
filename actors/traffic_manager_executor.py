from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping


TransformFactory = Callable[[Mapping[str, Any]], Any]


@dataclass(frozen=True)
class TrafficManagerExecutionConfig:
    tm_port: int = 8000
    blueprint_filter: str = "vehicle.*"
    autopilot_enabled: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "tm_port": self.tm_port,
            "blueprint_filter": self.blueprint_filter,
            "autopilot_enabled": self.autopilot_enabled,
        }


class TrafficManagerActorExecutor:
    """Bind actor runtime plans to CARLA TrafficManager-style runtime calls.

    The executor intentionally does not tick the world or implement actor logic.
    It only spawns TrafficManager actors and applies low-code TM settings.
    """

    def __init__(
        self,
        *,
        world: Any,
        client: Any | None = None,
        traffic_manager: Any | None = None,
        tm_port: int = 8000,
        blueprint_filter: str = "vehicle.*",
        transform_factory: TransformFactory | None = None,
    ) -> None:
        self.world = world
        self.client = client
        self._traffic_manager = traffic_manager
        self.config = TrafficManagerExecutionConfig(
            tm_port=int(tm_port),
            blueprint_filter=str(blueprint_filter),
        )
        self.transform_factory = transform_factory or _default_transform_factory

    def execute_plan_set(self, plan_set: Mapping[str, Any]) -> dict[str, Any]:
        actors = list(plan_set.get("actors", []))
        results = [self.execute_actor_plan(actor_plan) for actor_plan in actors]
        spawned_count = sum(1 for item in results if item["runtime_status"] == "traffic_manager_bound")
        fallback_count = sum(1 for item in results if item["runtime_status"] == "plan_only_fallback")
        failed_count = sum(1 for item in results if item["runtime_status"] == "failed")

        return {
            "schema_version": "traffic_manager_execution.mvp.v0",
            "scenario_id": str(plan_set.get("scenario_id", "scenario")),
            "source_schema_version": plan_set.get("schema_version"),
            "runtime_boundary": {
                "owns_carla_tick_loop": False,
                "owns_custom_actor_controller": False,
                "traffic_manager_only": True,
            },
            "config": self.config.to_dict(),
            "actors": results,
            "summary": {
                "actor_count": len(results),
                "spawned_count": spawned_count,
                "fallback_count": fallback_count,
                "failed_count": failed_count,
            },
        }

    def execute_actor_plan(self, actor_plan: Mapping[str, Any]) -> dict[str, Any]:
        actor_id = str(actor_plan.get("actor_id", "actor"))
        runtime_mode = str(actor_plan.get("runtime_mode", "replay"))
        if runtime_mode != "traffic_manager":
            return {
                "actor_id": actor_id,
                "runtime_mode": runtime_mode,
                "runtime_status": "plan_only_fallback",
                "reason": "only_traffic_manager_mode_is_bound_by_this_executor",
            }

        try:
            blueprint = self._select_blueprint()
            transform = self.transform_factory(dict(actor_plan.get("initial_state", {})))
            actor_handle = self.world.spawn_actor(blueprint, transform)
            self._set_autopilot(actor_handle)
            tm_settings = self._apply_traffic_manager_settings(actor_handle, actor_plan)
        except Exception as exc:  # pragma: no cover - covered by integration/debug paths
            return {
                "actor_id": actor_id,
                "runtime_mode": runtime_mode,
                "runtime_status": "failed",
                "reason": type(exc).__name__,
                "detail": str(exc),
            }

        return {
            "actor_id": actor_id,
            "runtime_mode": runtime_mode,
            "runtime_status": "traffic_manager_bound",
            "carla_actor_id": getattr(actor_handle, "id", None),
            "traffic_manager_settings": tm_settings,
        }

    def _traffic_manager_or_raise(self) -> Any:
        if self._traffic_manager is not None:
            return self._traffic_manager
        if self.client is not None and hasattr(self.client, "get_trafficmanager"):
            self._traffic_manager = self.client.get_trafficmanager(self.config.tm_port)
            return self._traffic_manager
        raise ValueError("traffic_manager or client.get_trafficmanager(tm_port) is required")

    def _select_blueprint(self) -> Any:
        library = self.world.get_blueprint_library()
        blueprints = list(library.filter(self.config.blueprint_filter))
        if not blueprints:
            raise ValueError(f"No vehicle blueprint matched {self.config.blueprint_filter}")
        return blueprints[0]

    def _set_autopilot(self, actor_handle: Any) -> None:
        if not hasattr(actor_handle, "set_autopilot"):
            raise ValueError("spawned actor does not expose set_autopilot")
        actor_handle.set_autopilot(self.config.autopilot_enabled, self.config.tm_port)

    def _apply_traffic_manager_settings(self, actor_handle: Any, actor_plan: Mapping[str, Any]) -> dict[str, Any]:
        traffic_manager = self._traffic_manager_or_raise()
        parameters = dict(actor_plan.get("controller", {}).get("parameters", {}))
        style = str(actor_plan.get("style", actor_plan.get("style_profile", {}).get("name", "normal")))

        min_gap_m = _float_or_none(parameters.get("min_gap_m"))
        speed_difference_percent = _speed_difference_percent(style, parameters)
        auto_lane_change = _auto_lane_change_enabled(style, parameters)

        if min_gap_m is not None and hasattr(traffic_manager, "distance_to_leading_vehicle"):
            traffic_manager.distance_to_leading_vehicle(actor_handle, min_gap_m)
        if hasattr(traffic_manager, "vehicle_percentage_speed_difference"):
            traffic_manager.vehicle_percentage_speed_difference(actor_handle, speed_difference_percent)
        if hasattr(traffic_manager, "auto_lane_change"):
            traffic_manager.auto_lane_change(actor_handle, auto_lane_change)

        return {
            "min_gap_m": min_gap_m,
            "speed_difference_percent": speed_difference_percent,
            "auto_lane_change": auto_lane_change,
        }


def execute_traffic_manager_plan_set(
    plan_set: Mapping[str, Any],
    *,
    world: Any,
    client: Any | None = None,
    traffic_manager: Any | None = None,
    tm_port: int = 8000,
    blueprint_filter: str = "vehicle.*",
    transform_factory: TransformFactory | None = None,
) -> dict[str, Any]:
    executor = TrafficManagerActorExecutor(
        world=world,
        client=client,
        traffic_manager=traffic_manager,
        tm_port=tm_port,
        blueprint_filter=blueprint_filter,
        transform_factory=transform_factory,
    )
    return executor.execute_plan_set(plan_set)


def _default_transform_factory(initial_state: Mapping[str, Any]) -> dict[str, float]:
    return {
        "x": float(initial_state.get("x", 0.0)),
        "y": float(initial_state.get("y", 0.0)),
        "z": float(initial_state.get("z", 0.0)),
        "pitch": float(initial_state.get("pitch", 0.0)),
        "yaw": float(initial_state.get("yaw", 0.0)),
        "roll": float(initial_state.get("roll", 0.0)),
    }


def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


def _speed_difference_percent(style: str, parameters: Mapping[str, Any]) -> float:
    if parameters.get("speed_difference_percent") is not None:
        return float(parameters["speed_difference_percent"])
    by_style = {
        "defensive": 20.0,
        "delayed": 10.0,
        "normal": 0.0,
        "assertive": -5.0,
        "aggressive": -10.0,
        "noncompliant": -15.0,
    }
    return by_style.get(style, 0.0)


def _auto_lane_change_enabled(style: str, parameters: Mapping[str, Any]) -> bool:
    if parameters.get("auto_lane_change") is not None:
        return bool(parameters["auto_lane_change"])
    if style in {"defensive", "delayed"}:
        return False
    gap = _float_or_none(parameters.get("lane_change_gap_acceptance_m"))
    if gap is not None and gap >= 10.0:
        return False
    return True
