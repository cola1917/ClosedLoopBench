from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from actors.reactive_actor import plan_reactive_actor_control
from metrics.report import build_closed_loop_report
from metrics.collector import TickMetricCollector


def build_basic_agent_plan(
    run_config: dict[str, Any],
    host: str = "127.0.0.1",
    port: int = 2000,
    max_ticks: int = 600,
    synchronous: bool = True,
    output: str | None = None,
    follow_ego: bool = False,
    debug_draw: bool = False,
) -> dict[str, Any]:
    ego = run_config.get("ego") or {}
    carla_config = run_config.get("carla") or {}
    initial_state = dict(ego.get("initial_state") or {})
    reference = list(ego.get("reference_trajectory") or [])
    destination = dict(reference[-1]) if reference else dict(initial_state)
    scenario_id = str(run_config["scenario_id"])

    return {
        "schema_version": "basic_agent_plan.mvp.v0",
        "scenario_id": scenario_id,
        "connection": {
            "host": str(host),
            "port": int(port),
        },
        "world": {
            "map": carla_config.get("map"),
            "fixed_delta_seconds": carla_config.get("fixed_delta_seconds", 0.05),
            "synchronous": bool(synchronous),
        },
        "ego": {
            "agent": "basic_agent",
            "role_name": ego.get("role_name", "ego_vehicle"),
            "spawn": _pose(initial_state),
            "destination": _pose(destination),
            "target_speed_mps": _target_speed(ego, destination),
        },
        "actors": run_config.get("actors", []),
        "metrics": run_config.get("metrics", []),
        "limits": {
            "max_ticks": int(max_ticks),
        },
        "visualization": {
            "follow_ego": bool(follow_ego),
            "debug_draw": bool(debug_draw),
            "spectator": {
                "distance_m": 8.0,
                "height_m": 4.0,
                "pitch_deg": -15.0,
            },
        },
        "artifacts": {
            "closed_loop_report": output or "closed_loop_report.json",
        },
    }


def write_basic_agent_plan(
    run_config_path: Path,
    output: Path,
    *,
    host: str = "127.0.0.1",
    port: int = 2000,
    max_ticks: int = 600,
    synchronous: bool = True,
    follow_ego: bool = False,
    debug_draw: bool = False,
) -> Path:
    run_config = json.loads(run_config_path.read_text(encoding="utf-8"))
    plan = build_basic_agent_plan(
        run_config,
        host=host,
        port=port,
        max_ticks=max_ticks,
        synchronous=synchronous,
        follow_ego=follow_ego,
        debug_draw=debug_draw,
        output=str(output.with_name("closed_loop_report.json")),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")
    return output


def run_basic_agent(plan: dict[str, Any], carla_module=None, agent_module=None) -> dict[str, Any]:
    if carla_module is None:
        try:
            import carla as carla_module  # type: ignore[no-redef]
        except Exception as exc:
            return _failed_result(plan, "missing_carla_python_api", str(exc))

    if agent_module is None:
        try:
            agent_module = _import_basic_agent_cls()
        except Exception as exc:
            return _failed_result(plan, "missing_basic_agent", str(exc))

    return _run_basic_agent_loop(plan, carla_module=carla_module, basic_agent_cls=agent_module)


def build_dry_run_report(run_config: dict[str, Any], plan: dict[str, Any]) -> dict[str, Any]:
    report = build_closed_loop_report(run_config, status="planned")
    report["artifacts"]["basic_agent_plan"] = "in_memory"
    report["runtime"] = {
        "ego_closed_loop": True,
        "interactive_actor_closed_loop": _has_interactive_actor(plan),
        "runner": "basic_agent",
    }
    return report


def _pose(state: dict[str, Any]) -> dict[str, float]:
    return {
        "x": float(state.get("x", 0.0)),
        "y": float(state.get("y", 0.0)),
        "z": float(state.get("z", 0.0)),
        "yaw": float(state.get("yaw", 0.0)),
    }


def _target_speed(ego: dict[str, Any], destination: dict[str, Any]) -> float:
    if "target_speed_mps" in ego:
        return float(ego["target_speed_mps"])
    return float(destination.get("speed_mps", 8.0))


def _has_interactive_actor(plan: dict[str, Any]) -> bool:
    interactive_levels = {"scripted", "traffic_manager_reactive"}
    for actor in plan.get("actors", []):
        if actor.get("closed_loop_level") in interactive_levels:
            return True
    return False


def _run_basic_agent_loop(
    plan: dict[str, Any],
    *,
    carla_module: Any,
    basic_agent_cls: Any,
) -> dict[str, Any]:
    client = None
    world = None
    ego_vehicle = None
    original_settings = None
    collector = TickMetricCollector()
    runtime_config = _runtime_report_config(plan)
    ticks_completed = 0

    try:
        connection = plan.get("connection") or {}
        world_config = plan.get("world") or {}
        ego_config = plan.get("ego") or {}
        limits = plan.get("limits") or {}
        visualization_config = plan.get("visualization") or {}

        client = carla_module.Client(
            connection.get("host", "127.0.0.1"),
            int(connection.get("port", 2000)),
        )
        if hasattr(client, "set_timeout"):
            client.set_timeout(float(connection.get("timeout_sec", 10.0)))

        map_name = world_config.get("map")
        if map_name and hasattr(client, "load_world"):
            world = client.load_world(str(map_name))
        elif hasattr(client, "get_world"):
            world = client.get_world()
        else:
            raise RuntimeError("CARLA client does not expose load_world or get_world")

        if hasattr(world, "get_map"):
            world.get_map()

        original_settings = world.get_settings() if hasattr(world, "get_settings") else None
        if world_config.get("synchronous", True) and original_settings is not None:
            settings = _copy_settings(original_settings)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = float(world_config.get("fixed_delta_seconds", 0.05))
            world.apply_settings(settings)

        ego_vehicle = _spawn_ego_vehicle(carla_module, world, ego_config)
        agent = basic_agent_cls(
            ego_vehicle,
            target_speed=_mps_to_kmh(float(ego_config.get("target_speed_mps", 8.0))),
        )
        _set_agent_destination(agent, carla_module, ego_config.get("destination") or {})
        if visualization_config.get("debug_draw"):
            _draw_debug_markers(carla_module, world, ego_config, plan.get("actors", []))

        max_ticks = int(limits.get("max_ticks", 600))
        dt_sec = float(world_config.get("fixed_delta_seconds", 0.05))
        for tick_index in range(max_ticks):
            if hasattr(agent, "done") and agent.done():
                break
            if hasattr(world, "tick"):
                world.tick()
            control = agent.run_step()
            ego_vehicle.apply_control(control)
            if visualization_config.get("follow_ego"):
                _follow_ego_spectator(carla_module, world, ego_vehicle, visualization_config.get("spectator") or {})
            ticks_completed += 1
            ego_pose = _vehicle_pose(ego_vehicle)
            ego_speed_mps = _vehicle_speed_mps(ego_vehicle)
            actor_distances_m, actor_decisions, min_ttc = _reactive_actor_tick(
                plan.get("actors", []),
                ego_pose=ego_pose,
                ego_speed_mps=ego_speed_mps,
            )
            collector.add_tick(
                t_sec=(tick_index + 1) * dt_sec,
                ego_pose=ego_pose,
                ego_speed_mps=ego_speed_mps,
                ego_control=_control_dict(control),
                actor_distances_m=actor_distances_m,
                ttc=min_ttc,
                collision=False,
                route_progress=_route_progress(tick_index + 1, max_ticks),
                hard_brake=_control_brake(control) > 0.6,
                jerk=None,
                actor_decisions=actor_decisions,
            )

        rows = collector.to_report_rows()
        status = "interactive_closed_loop" if _has_interactive_actor(plan) else "ego_closed_loop"
        report = build_closed_loop_report(runtime_config, tick_metrics=rows, status=status)
        _write_report_if_requested(plan, report)
        return {
            "status": status,
            "scenario_id": plan.get("scenario_id"),
            "summary": {
                "ticks": ticks_completed,
                "route_progress": report["summary"]["route_progress"],
                "collision_count": report["summary"]["collision_count"],
            },
            "report": report,
        }
    except Exception as exc:
        report = build_closed_loop_report(
            runtime_config,
            tick_metrics=collector.to_report_rows(),
            status="failed",
        )
        _write_report_if_requested(plan, report)
        result = _failed_result(plan, "basic_agent_runtime_failed", str(exc))
        result["report"] = report
        return result
    finally:
        if world is not None and original_settings is not None and hasattr(world, "apply_settings"):
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass
        if ego_vehicle is not None and hasattr(ego_vehicle, "destroy"):
            try:
                ego_vehicle.destroy()
            except Exception:
                pass


def _import_basic_agent_cls() -> Any:
    # ClosedLoopBench has a local agents package, which can shadow CARLA's
    # PythonAPI agents.navigation package. Try the normal import first, then
    # retry with the project root temporarily removed from sys.path.
    try:
        from agents.navigation.basic_agent import BasicAgent

        return BasicAgent
    except Exception as first_exc:
        original_path = list(sys.path)
        original_agents = sys.modules.get("agents")
        try:
            sys.path = [item for item in sys.path if Path(item).resolve() != PROJECT_ROOT]
            sys.modules.pop("agents", None)
            from agents.navigation.basic_agent import BasicAgent

            return BasicAgent
        except Exception:
            raise first_exc
        finally:
            sys.path = original_path
            if original_agents is not None:
                sys.modules["agents"] = original_agents


def _runtime_report_config(plan: dict[str, Any]) -> dict[str, Any]:
    return {
        "scenario_id": plan.get("scenario_id", "unknown"),
        "actors": list(plan.get("actors") or []),
        "metrics": list(plan.get("metrics") or []),
    }


def _copy_settings(settings: Any) -> Any:
    try:
        import copy

        return copy.copy(settings)
    except Exception:
        return settings


def _spawn_ego_vehicle(carla_module: Any, world: Any, ego_config: dict[str, Any]) -> Any:
    blueprint_library = world.get_blueprint_library()
    blueprint_candidates = blueprint_library.filter("vehicle.*")
    if not blueprint_candidates:
        raise RuntimeError("no CARLA vehicle blueprint matched vehicle.*")
    blueprint = blueprint_candidates[0]
    if hasattr(blueprint, "set_attribute"):
        blueprint.set_attribute("role_name", str(ego_config.get("role_name", "ego_vehicle")))

    spawn = ego_config.get("spawn") or {}
    transform = _carla_transform(carla_module, spawn)
    vehicle = _try_spawn(world, blueprint, transform)
    if vehicle is not None:
        return vehicle

    fallback_transforms = []
    if hasattr(world, "get_map"):
        world_map = world.get_map()
        if hasattr(world_map, "get_spawn_points"):
            fallback_transforms = list(world_map.get_spawn_points())

    for fallback_transform in fallback_transforms:
        vehicle = _try_spawn(world, blueprint, fallback_transform)
        if vehicle is not None:
            return vehicle

    raise RuntimeError("failed to spawn ego vehicle at planned pose or map fallback spawn points")


def _try_spawn(world: Any, blueprint: Any, transform: Any) -> Any | None:
    if hasattr(world, "try_spawn_actor"):
        return world.try_spawn_actor(blueprint, transform)
    try:
        return world.spawn_actor(blueprint, transform)
    except Exception:
        return None


def _carla_location(carla_module: Any, pose: dict[str, Any]) -> Any:
    return carla_module.Location(
        x=float(pose.get("x", 0.0)),
        y=float(pose.get("y", 0.0)),
        z=float(pose.get("z", 0.0)),
    )


def _carla_transform(carla_module: Any, pose: dict[str, Any]) -> Any:
    location = _carla_location(carla_module, pose)
    rotation = carla_module.Rotation(yaw=float(pose.get("yaw", 0.0)))
    return carla_module.Transform(location, rotation)


def _set_agent_destination(agent: Any, carla_module: Any, destination: dict[str, Any]) -> None:
    location = _carla_location(carla_module, destination)
    try:
        agent.set_destination(location)
    except TypeError:
        agent.set_destination(location, location)


def _vehicle_pose(vehicle: Any) -> dict[str, float]:
    if not hasattr(vehicle, "get_transform"):
        return {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    transform = vehicle.get_transform()
    location = getattr(transform, "location", None)
    rotation = getattr(transform, "rotation", None)
    return {
        "x": float(getattr(location, "x", 0.0)),
        "y": float(getattr(location, "y", 0.0)),
        "z": float(getattr(location, "z", 0.0)),
        "yaw": float(getattr(rotation, "yaw", 0.0)),
    }


def _vehicle_speed_mps(vehicle: Any) -> float:
    if not hasattr(vehicle, "get_velocity"):
        return 0.0
    velocity = vehicle.get_velocity()
    vx = float(getattr(velocity, "x", 0.0))
    vy = float(getattr(velocity, "y", 0.0))
    vz = float(getattr(velocity, "z", 0.0))
    return math.sqrt(vx * vx + vy * vy + vz * vz)


def _follow_ego_spectator(
    carla_module: Any,
    world: Any,
    ego_vehicle: Any,
    spectator_config: dict[str, Any],
) -> None:
    if not hasattr(world, "get_spectator") or not hasattr(ego_vehicle, "get_transform"):
        return
    spectator = world.get_spectator()
    if spectator is None or not hasattr(spectator, "set_transform"):
        return

    ego_transform = ego_vehicle.get_transform()
    ego_location = getattr(ego_transform, "location", None)
    ego_rotation = getattr(ego_transform, "rotation", None)
    yaw_deg = float(getattr(ego_rotation, "yaw", 0.0))
    yaw_rad = math.radians(yaw_deg)
    distance_m = float(spectator_config.get("distance_m", 8.0))
    height_m = float(spectator_config.get("height_m", 4.0))
    pitch_deg = float(spectator_config.get("pitch_deg", -15.0))

    location = carla_module.Location(
        x=float(getattr(ego_location, "x", 0.0)) - math.cos(yaw_rad) * distance_m,
        y=float(getattr(ego_location, "y", 0.0)) - math.sin(yaw_rad) * distance_m,
        z=float(getattr(ego_location, "z", 0.0)) + height_m,
    )
    rotation = carla_module.Rotation(pitch=pitch_deg, yaw=yaw_deg)
    spectator.set_transform(carla_module.Transform(location, rotation))


def _draw_debug_markers(carla_module: Any, world: Any, ego_config: dict[str, Any], actors: list[dict[str, Any]]) -> None:
    debug = getattr(world, "debug", None)
    if debug is None or not hasattr(debug, "draw_point"):
        return
    lifetime = 30.0
    _draw_debug_point(carla_module, debug, ego_config.get("spawn") or {}, "ego_spawn", lifetime)
    _draw_debug_point(carla_module, debug, ego_config.get("destination") or {}, "ego_destination", lifetime)
    for actor in actors:
        _draw_debug_point(
            carla_module,
            debug,
            actor.get("initial_state") or {},
            str(actor.get("actor_id", "actor")),
            lifetime,
        )


def _draw_debug_point(
    carla_module: Any,
    debug: Any,
    pose: dict[str, Any],
    label: str,
    lifetime: float,
) -> None:
    location = _carla_location(carla_module, pose)
    try:
        debug.draw_point(location, size=0.2, life_time=lifetime)
        if hasattr(debug, "draw_string"):
            debug.draw_string(location, label, life_time=lifetime)
    except TypeError:
        debug.draw_point(location)


def _reactive_actor_tick(
    actors: list[dict[str, Any]],
    *,
    ego_pose: dict[str, float],
    ego_speed_mps: float,
) -> tuple[dict[str, float], dict[str, dict[str, Any]], float | None]:
    actor_distances_m: dict[str, float] = {}
    actor_decisions: dict[str, dict[str, Any]] = {}
    finite_ttc_values: list[float] = []

    for actor in actors:
        if not _is_interactive_actor(actor):
            continue
        actor_id = str(actor.get("actor_id", "actor"))
        actor_state = dict(actor.get("initial_state") or {})
        actor_speed_mps = float(actor_state.get("speed_mps", 0.0))
        distance_m = _xy_distance(actor_state, ego_pose)
        decision = plan_reactive_actor_control(
            actor_state,
            {
                "x": ego_pose.get("x", 0.0),
                "y": ego_pose.get("y", 0.0),
                "speed_mps": ego_speed_mps,
                "distance_m": distance_m,
                "relative_speed_mps": max(0.0, ego_speed_mps - actor_speed_mps),
            },
            style=_actor_style(actor),
            reference_speed_mps=_actor_reference_speed(actor),
        )
        actor_distances_m[actor_id] = float(decision["distance_m"])
        actor_decisions[actor_id] = decision
        ttc_sec = decision.get("ttc_sec")
        if isinstance(ttc_sec, (int, float)) and math.isfinite(float(ttc_sec)):
            finite_ttc_values.append(float(ttc_sec))

    return actor_distances_m, actor_decisions, min(finite_ttc_values) if finite_ttc_values else None


def _is_interactive_actor(actor: dict[str, Any]) -> bool:
    if actor.get("closed_loop_level") in {"scripted", "traffic_manager_reactive"}:
        return True
    closed_loop = actor.get("closed_loop") or {}
    return bool(closed_loop.get("ego_responsive", False))


def _xy_distance(actor_state: dict[str, Any], ego_pose: dict[str, float]) -> float:
    dx = float(ego_pose.get("x", 0.0)) - float(actor_state.get("x", 0.0))
    dy = float(ego_pose.get("y", 0.0)) - float(actor_state.get("y", 0.0))
    return math.sqrt(dx * dx + dy * dy)


def _actor_style(actor: dict[str, Any]) -> str:
    style_profile = actor.get("style_profile") or {}
    return str(actor.get("style", style_profile.get("name", "normal")))


def _actor_reference_speed(actor: dict[str, Any]) -> float | None:
    trajectory = actor.get("reference_trajectory") or []
    if trajectory and isinstance(trajectory[-1], dict) and trajectory[-1].get("speed_mps") is not None:
        return float(trajectory[-1]["speed_mps"])
    return None


def _control_dict(control: Any) -> dict[str, float]:
    return {
        "throttle": float(getattr(control, "throttle", 0.0)),
        "brake": float(getattr(control, "brake", 0.0)),
        "steer": float(getattr(control, "steer", 0.0)),
    }


def _control_brake(control: Any) -> float:
    return float(getattr(control, "brake", 0.0))


def _route_progress(ticks_completed: int, max_ticks: int) -> float:
    if max_ticks <= 0:
        return 0.0
    return min(1.0, float(ticks_completed) / float(max_ticks))


def _mps_to_kmh(speed_mps: float) -> float:
    return speed_mps * 3.6


def _write_report_if_requested(plan: dict[str, Any], report: dict[str, Any]) -> None:
    report_path = (plan.get("artifacts") or {}).get("closed_loop_report")
    if not report_path:
        return
    path = Path(report_path)
    if not path.is_absolute():
        path = Path.cwd() / path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")


def _failed_result(plan: dict[str, Any], reason: str, detail: str) -> dict[str, Any]:
    return {
        "status": "failed",
        "reason": reason,
        "detail": detail,
        "scenario_id": plan.get("scenario_id"),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Plan or run a CARLA BasicAgent ego closed-loop evaluation.")
    parser.add_argument("--run-config", required=True, help="Path to carla_run_config.json.")
    parser.add_argument("--output", default=None, help="Path to write basic_agent_plan.json.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--max-ticks", default=600, type=int)
    parser.add_argument("--async-world", action="store_true", help="Do not request synchronous stepping.")
    parser.add_argument("--execute", action="store_true", help="Attempt real CARLA execution instead of dry-run planning.")
    parser.add_argument("--follow-ego", action="store_true", help="Move CARLA spectator behind the ego vehicle each tick.")
    parser.add_argument("--debug-draw", action="store_true", help="Draw ego and actor debug markers in the CARLA world.")
    args = parser.parse_args(argv)

    run_config_path = Path(args.run_config)
    output = Path(args.output) if args.output else run_config_path.with_name("basic_agent_plan.json")
    run_config = json.loads(run_config_path.read_text(encoding="utf-8"))
    plan = build_basic_agent_plan(
        run_config,
        host=args.host,
        port=args.port,
        max_ticks=args.max_ticks,
        synchronous=not args.async_world,
        follow_ego=args.follow_ego,
        debug_draw=args.debug_draw,
        output=str(output.with_name("closed_loop_report.json")),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")

    result = run_basic_agent(plan) if args.execute else {"status": "planned", "plan": str(output)}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["status"] in {"planned", "completed", "ego_closed_loop", "interactive_closed_loop"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
