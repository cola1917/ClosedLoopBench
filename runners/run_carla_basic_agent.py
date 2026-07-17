from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace
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
    snap_to_map: bool = False,
    actor_autopilot: bool = True,
    traffic_manager_port: int = 8000,
    ego_driver: str = "basic_agent",
    control_topic: str | None = None,
    observation_topic: str | None = None,
    control_timeout_sec: float = 0.5,
    acceptance_evidence: bool = False,
    opendrive_path: str | None = None,
    timeout_sec: float = 60.0,
    runtime_route_distance_m: float | None = None,
    physics_smoke: bool = False,
    multimodal_sensor_required: bool = False,
) -> dict[str, Any]:
    ego = run_config.get("ego") or {}
    carla_config = run_config.get("carla") or {}
    run_experiment = run_config.get("experiment") or {}
    initial_state = dict(ego.get("initial_state") or {})
    reference = list(ego.get("reference_trajectory") or [])
    destination = dict(reference[-1]) if reference else dict(initial_state)
    scenario_id = str(run_config["scenario_id"])

    return {
        "schema_version": "basic_agent_plan.mvp.v0",
        "run_id": run_config.get("run_id") or run_experiment.get("run_id"),
        "scenario_id": scenario_id,
        "connection": {
            "host": str(host),
            "port": int(port),
            "timeout_sec": float(timeout_sec),
        },
        "world": {
            "map": carla_config.get("map"),
            "opendrive_path": str(opendrive_path) if opendrive_path else None,
            "runtime_route_distance_m": (
                float(runtime_route_distance_m)
                if runtime_route_distance_m is not None
                else None
            ),
            "fixed_delta_seconds": carla_config.get("fixed_delta_seconds", 0.05),
            "synchronous": bool(synchronous),
            "weather": carla_config.get("weather"),
            "seed": carla_config.get("seed"),
        },
        "experiment": {
            "run_id": run_config.get("run_id") or run_experiment.get("run_id"),
            "scene_version": run_experiment.get("scene_version"),
            "algorithm_id": str(
                run_experiment.get("algorithm_id") or ego.get("algorithm_id") or ego_driver
            ),
            "algorithm_version": (
                run_experiment.get("algorithm_version") or ego.get("algorithm_version")
            ),
            "odd_id": str(
                run_experiment.get("odd_id")
                or carla_config.get("odd_id")
                or carla_config.get("weather")
                or "default"
            ),
            "seed": (
                run_experiment.get("seed")
                if run_experiment.get("seed") is not None
                else carla_config.get("seed")
            ),
        },
        "actor_control": dict(run_config.get("actor_control") or {}),
        "actor_binding": dict(run_config.get("actor_binding") or {}),
        "ego": {
            "agent": "basic_agent",
            "driver": str(ego_driver),
            "role_name": ego.get("role_name", "ego_vehicle"),
            "spawn": _pose(initial_state),
            "destination": _pose(destination),
            "route": [_pose(state) for state in ([initial_state] + reference)],
            "target_speed_mps": _target_speed(ego, destination),
            "control_topic": control_topic or "/carla/{}/vehicle_control_cmd".format(
                ego.get("role_name", "ego_vehicle")
            ),
            "control_timeout_sec": float(control_timeout_sec),
            "observation_topic": observation_topic or "/closed_loop/ego/observation",
        },
        "actors": run_config.get("actors", []),
        "reconstruction_package": dict(run_config.get("reconstruction_package") or {}),
        "metrics": (
            [
                metric
                for metric in run_config.get("metrics", [])
                if (
                    str(metric)
                    if isinstance(metric, str)
                    else str(metric.get("name") or metric.get("metric"))
                )
                in {"collision", "collision_count", "route_progress"}
            ]
            if physics_smoke
            else run_config.get("metrics", [])
        ),
        "evaluation": (
            {
                "criteria": [
                    {"name": "collision_count", "metric": "collision_count", "op": "==", "value": 0},
                    {"name": "route_progress", "metric": "route_progress", "op": ">=", "value": 0.95},
                ]
            }
            if physics_smoke
            else dict(run_config.get("evaluation") or {})
        ),
        "limits": {
            "max_ticks": int(max_ticks),
        },
        "runtime": {
            "snap_to_map": bool(snap_to_map),
            "actor_autopilot": bool(actor_autopilot),
            "traffic_manager_port": int(traffic_manager_port),
            "acceptance_evidence": bool(acceptance_evidence),
            "physics_smoke": bool(physics_smoke),
            "multimodal_sensor_required": bool(multimodal_sensor_required),
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
            "frame_trace": str(Path(output).with_name("frame_trace.jsonl")) if output else None,
            "metrics_trace": str(Path(output).with_name("metrics_trace.jsonl")) if output else None,
            "cleanup_audit": str(Path(output).with_name("cleanup_audit.json")) if output else None,
            "nurec_multimodal_trace": (
                str(Path(output).with_name("nurec_multimodal_trace.jsonl"))
                if output
                else None
            ),
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
    snap_to_map: bool = False,
    actor_autopilot: bool = True,
    traffic_manager_port: int = 8000,
    ego_driver: str = "basic_agent",
    control_topic: str | None = None,
    observation_topic: str | None = None,
    control_timeout_sec: float = 0.5,
    acceptance_evidence: bool = False,
    opendrive_path: str | None = None,
    timeout_sec: float = 60.0,
    runtime_route_distance_m: float | None = None,
    physics_smoke: bool = False,
    multimodal_sensor_required: bool = False,
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
        snap_to_map=snap_to_map,
        actor_autopilot=actor_autopilot,
        traffic_manager_port=traffic_manager_port,
        ego_driver=ego_driver,
        control_topic=control_topic,
        observation_topic=observation_topic,
        control_timeout_sec=control_timeout_sec,
        acceptance_evidence=acceptance_evidence,
        opendrive_path=opendrive_path,
        timeout_sec=timeout_sec,
        runtime_route_distance_m=runtime_route_distance_m,
        physics_smoke=physics_smoke,
        multimodal_sensor_required=multimodal_sensor_required,
        output=str(output.with_name("closed_loop_report.json")),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")
    return output


def run_basic_agent(
    plan: dict[str, Any],
    carla_module=None,
    agent_module=None,
    driver_factory=None,
    actor_behavior_planner=None,
    sensor_frame_handler=None,
) -> dict[str, Any]:
    if carla_module is None:
        try:
            import carla as carla_module  # type: ignore[no-redef]
        except Exception as exc:
            return _failed_result(plan, "missing_carla_python_api", str(exc))

    driver_kind = str((plan.get("ego") or {}).get("driver", "basic_agent"))
    if driver_kind == "basic_agent" and agent_module is None:
        try:
            agent_module = _import_basic_agent_cls()
        except Exception as exc:
            return _failed_result(plan, "missing_basic_agent", str(exc))

    return _run_basic_agent_loop(
        plan,
        carla_module=carla_module,
        basic_agent_cls=agent_module,
        driver_factory=driver_factory,
        actor_behavior_planner=actor_behavior_planner,
        sensor_frame_handler=sensor_frame_handler,
    )


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
    for actor in plan.get("actors", []):
        if _is_interactive_actor(actor):
            return True
    return False


def _run_basic_agent_loop(
    plan: dict[str, Any],
    *,
    carla_module: Any,
    basic_agent_cls: Any = None,
    driver_factory: Any = None,
    actor_behavior_planner: Any = None,
    sensor_frame_handler: Any = None,
) -> dict[str, Any]:
    client = None
    world = None
    ego_vehicle = None
    collision_sensor = None
    ego_driver = None
    traffic_manager = None
    actor_vehicles: dict[str, Any] = {}
    original_settings = None
    original_weather = None
    collector = TickMetricCollector()
    runtime_config = _runtime_report_config(plan)
    ticks_completed = 0
    actor_execution_evidence: dict[str, str] = {}
    actor_physical_response: dict[str, dict[str, float]] = {}
    actor_initial_poses: dict[str, dict[str, Any]] = {}
    frame_trace: list[dict[str, Any]] = []
    multimodal_trace: list[dict[str, Any]] = []
    cleanup_audit: list[dict[str, Any]] = []
    result: dict[str, Any] | None = None
    termination_reason = "max_ticks"
    acceptance_evidence = False
    multimodal_sensor_required = False
    actor_runtime_binding_evidence: dict[str, Any] = {
        "schema_version": "actor_runtime_binding_evidence.v1",
        "status": "not_configured",
        "records": [],
        "issues": [],
    }

    try:
        connection = plan.get("connection") or {}
        world_config = plan.get("world") or {}
        ego_config = plan.get("ego") or {}
        limits = plan.get("limits") or {}
        runtime_options = plan.get("runtime") or {}
        if actor_behavior_planner is None:
            actor_behavior_planner = _load_actor_behavior_planner(
                (plan.get("actor_control") or {}).get("behavior_plugin")
            )
        acceptance_evidence = bool(runtime_options.get("acceptance_evidence", False))
        multimodal_sensor_required = bool(
            runtime_options.get("multimodal_sensor_required", False)
        )
        if multimodal_sensor_required and sensor_frame_handler is None:
            raise RuntimeError(
                "multimodal sensor evidence is required but no sensor_frame_handler was provided"
            )
        visualization_config = plan.get("visualization") or {}

        client = carla_module.Client(
            connection.get("host", "127.0.0.1"),
            int(connection.get("port", 2000)),
        )
        if hasattr(client, "set_timeout"):
            client.set_timeout(float(connection.get("timeout_sec", 10.0)))

        opendrive_path = world_config.get("opendrive_path")
        map_name = world_config.get("map")
        if opendrive_path:
            if not hasattr(client, "generate_opendrive_world"):
                raise RuntimeError("CARLA client does not support generate_opendrive_world")
            xodr_path = Path(str(opendrive_path)).expanduser().resolve()
            if not xodr_path.is_file():
                raise RuntimeError(f"OpenDRIVE file does not exist: {xodr_path}")
            xodr = xodr_path.read_text(encoding="utf-8")
            if not xodr.strip():
                raise RuntimeError(f"OpenDRIVE file is empty: {xodr_path}")
            generation_parameters_type = getattr(
                carla_module, "OpendriveGenerationParameters", None
            )
            if generation_parameters_type is None:
                world = client.generate_opendrive_world(xodr)
            else:
                # Use CARLA's own version-specific defaults. Oversized road
                # chunk lengths have caused packaged 0.9.16 servers to crash
                # after navigation-mesh generation on otherwise valid XODR.
                world = client.generate_opendrive_world(
                    xodr, generation_parameters_type()
                )
        elif map_name and hasattr(client, "load_world"):
            world = client.load_world(str(map_name))
        elif hasattr(client, "get_world"):
            world = client.get_world()
        else:
            raise RuntimeError("CARLA client does not expose load_world or get_world")

        if hasattr(world, "get_map"):
            world.get_map()

        runtime_route_distance_m = world_config.get("runtime_route_distance_m")
        if runtime_route_distance_m is not None:
            _bind_runtime_topology_route(
                world,
                plan,
                distance_m=float(runtime_route_distance_m),
            )

        original_weather = world.get_weather() if hasattr(world, "get_weather") else None
        _apply_world_weather(carla_module, world, world_config.get("weather"))

        original_settings = world.get_settings() if hasattr(world, "get_settings") else None
        if world_config.get("synchronous", True) and original_settings is not None:
            settings = _copy_settings(original_settings)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = float(world_config.get("fixed_delta_seconds", 0.05))
            world.apply_settings(settings)

        if runtime_options.get("snap_to_map"):
            _snap_plan_to_map(carla_module, world, plan)

        ego_vehicle = _spawn_ego_vehicle(carla_module, world, ego_config)
        collision_tracker, collision_sensor = _spawn_collision_tracker(
            carla_module,
            world,
            ego_vehicle,
        )
        _spawn_interactive_actor_vehicles(
            carla_module,
            world,
            plan.get("actors") or [],
            actor_vehicles,
        )
        actor_initial_poses = {
            actor_id: (
                _vehicle_pose(vehicle)
                if acceptance_evidence
                else _pose(
                    next(
                        (
                            actor.get("initial_state") or {}
                            for actor in plan.get("actors") or []
                            if str(actor.get("actor_id")) == actor_id
                        ),
                        {},
                    )
                )
            )
            for actor_id, vehicle in actor_vehicles.items()
        }
        actor_runtime_binding_evidence = _actor_runtime_binding_evidence(
            plan,
            actor_vehicles,
        )
        if (
            multimodal_sensor_required
            and actor_runtime_binding_evidence["status"] != "passed"
        ):
            raise RuntimeError(
                "actor runtime binding failed: "
                + ", ".join(actor_runtime_binding_evidence["issues"])
            )
        traffic_manager_actor_ids, traffic_manager = _configure_actor_autopilot(
            client,
            actor_vehicles,
            plan.get("actors") or [],
            enabled=bool(runtime_options.get("actor_autopilot", False)),
            tm_port=int(runtime_options.get("traffic_manager_port", 8000)),
            seed=world_config.get("seed"),
        )
        actor_execution_evidence.update(
            {actor_id: "traffic_manager" for actor_id in traffic_manager_actor_ids}
        )
        ego_driver = _build_ego_driver(
            plan,
            ego_vehicle=ego_vehicle,
            carla_module=carla_module,
            basic_agent_cls=basic_agent_cls,
            driver_factory=driver_factory,
        )
        if visualization_config.get("debug_draw"):
            _draw_debug_markers(carla_module, world, ego_config, plan.get("actors", []))
            _draw_route_lane_lines(carla_module, world, ego_config.get("route") or [])

        max_ticks = int(limits.get("max_ticks", 600))
        dt_sec = float(world_config.get("fixed_delta_seconds", 0.05))
        previous_speed_mps: float | None = None
        previous_acceleration_mps2: float | None = None
        route = list(ego_config.get("route") or [ego_config.get("spawn") or {}, ego_config.get("destination") or {}])
        previous_world_frame: int | None = None
        previous_sensor_frame: int | None = None
        previous_sensor_time_sec: float | None = None
        previous_ego_render_pose = _vehicle_pose(ego_vehicle)
        previous_actor_render_poses = _initial_bound_actor_poses(
            plan.get("actors") or [],
            actor_vehicles,
        )
        for tick_index in range(max_ticks):
            if hasattr(ego_driver, "done") and ego_driver.done():
                termination_reason = "route_complete"
                break
            control = ego_driver.run_step()
            ego_vehicle.apply_control(control)
            tick_frame = None
            if hasattr(world, "tick"):
                tick_frame = world.tick()
            snapshot_frame = None
            run_time_sec = (tick_index + 1) * dt_sec
            simulation_time_sec = run_time_sec
            snapshot_delta_sec = dt_sec
            if hasattr(world, "get_snapshot"):
                snapshot = world.get_snapshot()
                snapshot_frame = getattr(snapshot, "frame", None)
                timestamp = getattr(snapshot, "timestamp", None)
                simulation_time_sec = float(
                    getattr(timestamp, "elapsed_seconds", simulation_time_sec)
                )
                snapshot_delta_sec = float(
                    getattr(timestamp, "delta_seconds", snapshot_delta_sec)
                )
            world_frame = snapshot_frame if isinstance(snapshot_frame, int) else tick_frame
            if acceptance_evidence and isinstance(world_frame, int):
                if previous_world_frame is not None and world_frame != previous_world_frame + 1:
                    raise RuntimeError(
                        f"CARLA frame discontinuity: previous={previous_world_frame}, current={world_frame}"
                    )
                previous_world_frame = world_frame
            if visualization_config.get("follow_ego"):
                _follow_ego_spectator(carla_module, world, ego_vehicle, visualization_config.get("spectator") or {})
            if visualization_config.get("debug_draw"):
                _draw_dynamic_vehicle_boxes(
                    carla_module,
                    world,
                    ego_vehicle,
                    actor_vehicles,
                    life_time=max(0.1, dt_sec * 3.0),
                )
            ticks_completed += 1
            ego_pose = _vehicle_pose(ego_vehicle)
            ego_speed_mps = _vehicle_speed_mps(ego_vehicle)
            actor_distances_m, actor_decisions, min_ttc = _reactive_actor_tick(
                plan.get("actors") or [],
                ego_pose=ego_pose,
                ego_speed_mps=ego_speed_mps,
                simulation_time_sec=simulation_time_sec,
                actor_vehicles=actor_vehicles,
                behavior_planner=actor_behavior_planner,
            )
            scripted_evidence = _apply_scripted_actor_controls(
                carla_module,
                plan.get("actors") or [],
                actor_vehicles,
                actor_decisions,
            )
            replay_evidence = _apply_replay_actor_controls(
                carla_module,
                plan.get("actors") or [],
                actor_vehicles,
                run_time_sec=run_time_sec,
            )
            actor_execution_evidence.update(scripted_evidence)
            actor_execution_evidence.update(replay_evidence)
            actor_by_id = {
                str(actor.get("actor_id", "actor")): actor
                for actor in plan.get("actors") or []
            }
            actor_states = {}
            for actor_id, vehicle in actor_vehicles.items():
                pose = _vehicle_pose(vehicle)
                speed = _vehicle_speed_mps(vehicle)
                actor = actor_by_id.get(actor_id, {})
                reference_pose = _reference_pose_at_time(
                    actor, run_time_sec
                )
                actor_states[actor_id] = {
                    "actor_type": _actor_kind(actor),
                    "carla_runtime_actor_id": getattr(vehicle, "id", None),
                    "carla_role_name": (
                        getattr(vehicle, "attributes", {}).get("role_name")
                        if isinstance(getattr(vehicle, "attributes", {}), dict)
                        else None
                    ),
                    "pose": pose,
                    "speed_mps": speed,
                    "reference_pose": reference_pose,
                    "reference_error_m": (
                        _xy_distance(pose, reference_pose)
                        if reference_pose is not None
                        else None
                    ),
                }
                initial = actor_initial_poses[actor_id]
                displacement = math.hypot(
                    float(pose["x"]) - float(initial["x"]),
                    float(pose["y"]) - float(initial["y"]),
                )
                if displacement >= 0.05:
                    actor_physical_response[actor_id] = {
                        "displacement_m": displacement,
                        "speed_mps": speed,
                    }
            multimodal_summary = None
            if sensor_frame_handler is not None:
                if not isinstance(world_frame, int):
                    raise RuntimeError(
                        "multimodal sensor handler requires an integer CARLA frame identity"
                    )
                if previous_sensor_frame is not None and world_frame <= previous_sensor_frame:
                    raise RuntimeError(
                        "multimodal CARLA frame is not strictly increasing: "
                        f"previous={previous_sensor_frame}, current={world_frame}"
                    )
                sensor_context = _build_sensor_frame_context(
                    plan,
                    frame_id=world_frame,
                    tick_index=tick_index,
                    simulation_time_sec=simulation_time_sec,
                    scenario_time_sec=run_time_sec,
                    interval_start_sec=(
                        previous_sensor_time_sec
                        if previous_sensor_time_sec is not None
                        else max(0.0, simulation_time_sec - max(0.0, snapshot_delta_sec))
                    ),
                    ego_pose=ego_pose,
                    previous_ego_pose=previous_ego_render_pose,
                    actor_states=actor_states,
                    previous_actor_poses=previous_actor_render_poses,
                )
                evidence = sensor_frame_handler(sensor_context)
                _validate_sensor_frame_evidence(evidence, world_frame)
                evidence = dict(evidence)
                multimodal_trace.append(evidence)
                multimodal_summary = {
                    "schema_version": evidence["schema_version"],
                    "status": evidence["status"],
                    "frame_id": evidence["frame_id"],
                    "dynamic_object_sha256": evidence.get("dynamic_object_sha256"),
                    "modalities": dict(evidence.get("modalities") or {}),
                    "issues": list(evidence.get("issues") or []),
                }
                if multimodal_sensor_required and evidence["status"] != "passed":
                    raise RuntimeError(
                        "required NuRec multimodal frame failed: "
                        + ", ".join(evidence.get("issues") or ["unknown_failure"])
                    )
                previous_sensor_frame = world_frame
                previous_sensor_time_sec = simulation_time_sec
                previous_ego_render_pose = dict(ego_pose)
                previous_actor_render_poses = _current_bound_actor_poses(
                    plan.get("actors") or [],
                    actor_states,
                    run_time_sec,
                )
            acceleration_mps2 = (
                (ego_speed_mps - previous_speed_mps) / dt_sec
                if previous_speed_mps is not None and dt_sec > 0.0
                else None
            )
            jerk = (
                (acceleration_mps2 - previous_acceleration_mps2) / dt_sec
                if acceleration_mps2 is not None
                and previous_acceleration_mps2 is not None
                and dt_sec > 0.0
                else None
            )
            measured_route_progress = (
                float(ego_driver.route_progress())
                if hasattr(ego_driver, "route_progress")
                else _route_progress_from_pose(route, ego_pose)
            )
            collector.add_tick(
                t_sec=(tick_index + 1) * dt_sec,
                ego_pose=ego_pose,
                ego_speed_mps=ego_speed_mps,
                ego_control=_control_dict(control),
                actor_distances_m=actor_distances_m,
                ttc=min_ttc,
                collision=collision_tracker.consume_tick() if collision_tracker is not None else None,
                route_progress=measured_route_progress,
                hard_brake=bool(acceleration_mps2 is not None and acceleration_mps2 <= -3.0),
                longitudinal_acceleration_mps2=acceleration_mps2,
                jerk=jerk,
                actor_decisions=actor_decisions,
                actor_control_evidence=dict(actor_execution_evidence),
            )
            frame_trace.append(
                {
                    "tick_index": tick_index,
                    "world_tick_frame": tick_frame,
                    "snapshot_frame": snapshot_frame,
                    "simulation_time_sec": simulation_time_sec,
                    "ego_pose": ego_pose,
                    "ego_speed_mps": ego_speed_mps,
                    "ego_control": _control_dict(control),
                    "actor_states": actor_states,
                    "multimodal_sensor": multimodal_summary,
                }
            )
            previous_speed_mps = ego_speed_mps
            if acceleration_mps2 is not None:
                previous_acceleration_mps2 = acceleration_mps2

        rows = collector.to_report_rows()
        route_progress = (
            float(ego_driver.route_progress())
            if hasattr(ego_driver, "route_progress")
            else _route_progress_from_pose(route, _vehicle_pose(ego_vehicle))
        )
        if acceptance_evidence:
            if not world_config.get("synchronous", True):
                raise RuntimeError("acceptance evidence requires synchronous CARLA stepping")
            if collision_sensor is None:
                raise RuntimeError("acceptance evidence requires a real collision sensor")
            if not frame_trace or not all(
                isinstance(row.get("world_tick_frame"), int)
                or isinstance(row.get("snapshot_frame"), int)
                for row in frame_trace
            ):
                raise RuntimeError("acceptance evidence requires CARLA frame identities")
            if route_progress < 0.95:
                raise RuntimeError(
                    f"route_incomplete: progress={route_progress:.6f}, "
                    f"termination={termination_reason}, max_ticks={max_ticks}"
                )
            if runtime_route_distance_m is not None and termination_reason != "route_complete":
                raise RuntimeError(
                    "runtime topology route did not terminate with route_complete: "
                    + termination_reason
                )
            if sum(1 for row in rows if row.get("collision") is True) != 0:
                raise RuntimeError("acceptance evidence requires a collision-free run")
            expected_interactive = {
                str(actor.get("actor_id", "actor"))
                for actor in plan.get("actors") or []
                if _is_interactive_actor(actor)
            } & set(actor_execution_evidence)
            physically_responsive = expected_interactive & set(actor_physical_response)
            if expected_interactive and physically_responsive != expected_interactive:
                missing = sorted(expected_interactive - physically_responsive)
                raise RuntimeError(
                    "interactive actors produced no physical response: " + ", ".join(missing)
                )
        if multimodal_sensor_required and (
            not multimodal_trace
            or any(item.get("status") != "passed" for item in multimodal_trace)
        ):
            raise RuntimeError("required NuRec multimodal evidence is incomplete")
        driver_diagnostics = (
            ego_driver.diagnostics()
            if hasattr(ego_driver, "diagnostics")
            else {"driver": str(ego_config.get("driver", "basic_agent"))}
        )
        if (
            ego_config.get("driver") in {"ros2_control", "ros2_observation_control"}
            and int(driver_diagnostics.get("control_count", 0)) == 0
        ):
            raise RuntimeError("ROS2 ego driver received no valid control commands")
        interactive_actor_ids = {
            str(actor.get("actor_id", "actor"))
            for actor in plan.get("actors") or []
            if _is_interactive_actor(actor)
        }
        physical_actor_evidence = (
            actor_physical_response if acceptance_evidence else actor_execution_evidence
        )
        physical_actor_evidence = {
            actor_id: evidence
            for actor_id, evidence in physical_actor_evidence.items()
            if actor_id in interactive_actor_ids
        }
        status = "interactive_closed_loop" if physical_actor_evidence else "ego_closed_loop"
        report = build_closed_loop_report(runtime_config, tick_metrics=rows, status=status)
        report["summary"]["control_timeout_count"] = int(
            driver_diagnostics.get("fallback_count", 0)
        )
        report["runtime"] = {
            "ego_driver": str(ego_config.get("driver", "basic_agent")),
            "route_binding": dict(ego_config.get("route_binding") or {}),
            "collision_sensor_available": collision_sensor is not None,
            "actor_control_evidence": dict(actor_execution_evidence),
            "actor_physical_response": dict(actor_physical_response),
            "actor_runtime_binding": actor_runtime_binding_evidence,
            "multimodal_sensor": _multimodal_runtime_summary(
                multimodal_trace,
                required=multimodal_sensor_required,
                handler_present=sensor_frame_handler is not None,
            ),
            "frame_trace_count": len(frame_trace),
            "termination_reason": termination_reason,
            "ego_driver_diagnostics": driver_diagnostics,
            "actor_behavior_plugin": (
                (plan.get("actor_control") or {}).get("behavior_plugin")
                or "actors.reactive_actor:plan_reactive_actor_control"
            ),
        }
        _write_report_if_requested(plan, report)
        result = {
            "status": status,
            "scenario_id": plan.get("scenario_id"),
            "summary": {
                "ticks": ticks_completed,
                "route_progress": report["summary"]["route_progress"],
                "collision_count": report["summary"]["collision_count"],
            },
            "report": report,
        }
        return result
    except Exception as exc:
        report = build_closed_loop_report(
            runtime_config,
            tick_metrics=collector.to_report_rows(),
            status="failed",
        )
        failure_diagnostics = (
            ego_driver.diagnostics()
            if ego_driver is not None and hasattr(ego_driver, "diagnostics")
            else {}
        )
        report["summary"]["control_timeout_count"] = int(
            failure_diagnostics.get("fallback_count", 0)
        )
        report["runtime"] = {
            "ego_driver": str((plan.get("ego") or {}).get("driver", "basic_agent")),
            "route_binding": dict((plan.get("ego") or {}).get("route_binding") or {}),
            "collision_sensor_available": collision_sensor is not None,
            "actor_control_evidence": dict(actor_execution_evidence),
            "actor_runtime_binding": actor_runtime_binding_evidence,
            "multimodal_sensor": _multimodal_runtime_summary(
                multimodal_trace,
                required=multimodal_sensor_required,
                handler_present=sensor_frame_handler is not None,
            ),
            "ego_driver_diagnostics": failure_diagnostics,
        }
        _write_report_if_requested(plan, report)
        result = _failed_result(plan, "basic_agent_runtime_failed", str(exc))
        result["report"] = report
        return result
    finally:
        if ego_driver is not None and hasattr(ego_driver, "close"):
            try:
                ego_driver.close()
                cleanup_audit.append({"action": "ego_driver.close", "status": "succeeded"})
            except Exception:
                cleanup_audit.append({"action": "ego_driver.close", "status": "failed"})
        if collision_sensor is not None:
            try:
                if hasattr(collision_sensor, "stop"):
                    collision_sensor.stop()
                if hasattr(collision_sensor, "destroy"):
                    collision_sensor.destroy()
                if acceptance_evidence and getattr(collision_sensor, "is_alive", False):
                    raise RuntimeError("collision sensor remains alive after destroy")
                cleanup_audit.append({"action": "collision_sensor.destroy", "status": "succeeded"})
            except Exception:
                cleanup_audit.append({"action": "collision_sensor.destroy", "status": "failed"})
        if traffic_manager is not None and hasattr(traffic_manager, "set_synchronous_mode"):
            try:
                traffic_manager.set_synchronous_mode(False)
                cleanup_audit.append({"action": "traffic_manager.sync_off", "status": "succeeded"})
            except Exception:
                cleanup_audit.append({"action": "traffic_manager.sync_off", "status": "failed"})
        if world is not None and original_weather is not None and hasattr(world, "set_weather"):
            try:
                world.set_weather(original_weather)
                cleanup_audit.append({"action": "world.restore_weather", "status": "succeeded"})
            except Exception:
                cleanup_audit.append({"action": "world.restore_weather", "status": "failed"})
        if world is not None and original_settings is not None and hasattr(world, "apply_settings"):
            try:
                world.apply_settings(original_settings)
                if acceptance_evidence and hasattr(world, "get_settings"):
                    restored = world.get_settings()
                    expected_signature = (
                        getattr(original_settings, "synchronous_mode", None),
                        getattr(original_settings, "fixed_delta_seconds", None),
                    )
                    actual_signature = (
                        getattr(restored, "synchronous_mode", None),
                        getattr(restored, "fixed_delta_seconds", None),
                    )
                    if actual_signature != expected_signature:
                        raise RuntimeError(
                            f"world settings were not restored: expected={expected_signature}, "
                            f"actual={actual_signature}"
                        )
                cleanup_audit.append({"action": "world.restore_settings", "status": "succeeded"})
            except Exception:
                cleanup_audit.append({"action": "world.restore_settings", "status": "failed"})
        for actor_vehicle in actor_vehicles.values():
            if acceptance_evidence and hasattr(actor_vehicle, "set_autopilot"):
                try:
                    actor_vehicle.set_autopilot(
                        False,
                        int((plan.get("runtime") or {}).get("traffic_manager_port", 8000)),
                    )
                    cleanup_audit.append(
                        {"action": "actor.autopilot_off", "status": "succeeded"}
                    )
                except Exception:
                    cleanup_audit.append(
                        {"action": "actor.autopilot_off", "status": "failed"}
                    )
            try:
                if hasattr(actor_vehicle, "destroy"):
                    actor_vehicle.destroy()
                if acceptance_evidence and getattr(actor_vehicle, "is_alive", False):
                    raise RuntimeError("actor remains alive after destroy")
                cleanup_audit.append({"action": "actor.destroy", "status": "succeeded"})
            except Exception:
                cleanup_audit.append({"action": "actor.destroy", "status": "failed"})
        if ego_vehicle is not None and hasattr(ego_vehicle, "destroy"):
            try:
                ego_vehicle.destroy()
                if acceptance_evidence and getattr(ego_vehicle, "is_alive", False):
                    raise RuntimeError("ego remains alive after destroy")
                cleanup_audit.append({"action": "ego.destroy", "status": "succeeded"})
            except Exception:
                cleanup_audit.append({"action": "ego.destroy", "status": "failed"})
        cleanup_ok = all(item["status"] == "succeeded" for item in cleanup_audit)
        if result is not None:
            result["frame_trace"] = frame_trace
            result["nurec_multimodal_trace"] = multimodal_trace
            result["cleanup_audit"] = cleanup_audit
            result["cleanup_succeeded"] = cleanup_ok
            report = result.get("report")
            if isinstance(report, dict):
                report.setdefault("runtime", {})["cleanup_succeeded"] = cleanup_ok
                if acceptance_evidence and not cleanup_ok:
                    report["status"] = "failed"
                _write_report_if_requested(plan, report)
            if acceptance_evidence and not cleanup_ok:
                result["status"] = "failed"
                result["reason"] = "cleanup_failed"
                result["detail"] = "one or more CARLA cleanup actions failed"
        _write_runtime_evidence(
            plan,
            frame_trace,
            collector.to_report_rows(),
            cleanup_audit,
            multimodal_trace,
        )


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


class _BasicAgentDriver:
    def __init__(self, agent: Any) -> None:
        self.agent = agent

    def done(self) -> bool:
        return bool(hasattr(self.agent, "done") and self.agent.done())

    def run_step(self) -> Any:
        return self.agent.run_step()

    def close(self) -> None:
        if hasattr(self.agent, "destroy"):
            self.agent.destroy()


class _TopologyFollowerDriver:
    """Small deterministic controller for CARLA topology acceptance routes."""

    def __init__(self, carla_module: Any, vehicle: Any, route: list[dict[str, Any]], target_speed_mps: float) -> None:
        if len(route) < 2:
            raise ValueError("topology follower requires at least two route points")
        self._carla = carla_module
        self._vehicle = vehicle
        self._route = route
        self._target_speed_mps = max(1.0, float(target_speed_mps))
        self._index = 1
        self._control_count = 0
        self._progress = 0.0
        self._cumulative = [0.0]
        for start, end in zip(route, route[1:]):
            self._cumulative.append(self._cumulative[-1] + _xy_distance(start, end))

    def done(self) -> bool:
        pose = _vehicle_pose(self._vehicle)
        return self._index >= len(self._route) - 1 and _xy_distance(
            pose, self._route[-1]
        ) <= 0.75

    def run_step(self) -> Any:
        pose = _vehicle_pose(self._vehicle)
        while self._index < len(self._route) - 1 and _xy_distance(
            pose, self._route[self._index]
        ) <= 1.5:
            self._index += 1
        target = self._route[self._index]
        segment_start = self._route[self._index - 1]
        segment_length = max(1e-6, _xy_distance(segment_start, target))
        remaining_to_target = _xy_distance(pose, target)
        segment_progress = min(1.0, max(0.0, 1.0 - remaining_to_target / segment_length))
        total_length = max(1e-6, self._cumulative[-1])
        self._progress = max(
            self._progress,
            (self._cumulative[self._index - 1] + segment_progress * segment_length)
            / total_length,
        )
        desired_yaw = math.degrees(
            math.atan2(
                float(target["y"]) - float(pose["y"]),
                float(target["x"]) - float(pose["x"]),
            )
        )
        yaw_error = (desired_yaw - float(pose["yaw"]) + 180.0) % 360.0 - 180.0
        steer = min(1.0, max(-1.0, yaw_error / 45.0))
        speed = _vehicle_speed_mps(self._vehicle)
        speed_error = self._target_speed_mps - speed
        throttle = min(0.65, max(0.0, 0.25 * speed_error))
        brake = min(0.7, max(0.0, -0.35 * speed_error))
        if self._index == len(self._route) - 1:
            remaining = _xy_distance(pose, target)
            if remaining < 3.0:
                throttle = min(throttle, max(0.12, remaining / 6.0))
        self._control_count += 1
        return _vehicle_control(
            self._carla,
            throttle=throttle,
            brake=brake,
            steer=steer,
        )

    def diagnostics(self) -> dict[str, Any]:
        return {
            "driver": "topology_follower",
            "control_count": self._control_count,
            "target_index": self._index,
            "route_point_count": len(self._route),
            "route_progress": self.route_progress(),
        }

    def route_progress(self) -> float:
        if self.done():
            return 1.0
        return min(1.0, max(0.0, self._progress))

    def close(self) -> None:
        return None


def _build_ego_driver(
    plan: dict[str, Any],
    *,
    ego_vehicle: Any,
    carla_module: Any,
    basic_agent_cls: Any,
    driver_factory: Any = None,
) -> Any:
    ego_config = plan.get("ego") or {}
    if driver_factory is not None:
        return driver_factory(plan, ego_vehicle, carla_module)

    driver_kind = str(ego_config.get("driver", "basic_agent"))
    if driver_kind == "basic_agent":
        if basic_agent_cls is None:
            raise RuntimeError("BasicAgent class is required for the basic_agent driver")
        agent = basic_agent_cls(
            ego_vehicle,
            target_speed=_mps_to_kmh(float(ego_config.get("target_speed_mps", 8.0))),
        )
        _set_agent_destination(agent, carla_module, ego_config.get("destination") or {})
        return _BasicAgentDriver(agent)

    if driver_kind == "topology_follower":
        return _TopologyFollowerDriver(
            carla_module,
            ego_vehicle,
            list(ego_config.get("route") or []),
            float(ego_config.get("target_speed_mps", 5.0)),
        )

    if driver_kind == "ros2_control":
        from agents.ros2_control_driver import create_ros2_control_driver

        return create_ros2_control_driver(
            carla_module=carla_module,
            control_topic=str(ego_config.get("control_topic")),
            timeout_sec=float(ego_config.get("control_timeout_sec", 0.5)),
        )

    if driver_kind == "ros2_observation_control":
        from agents.ros2_observation_control_driver import (
            create_ros2_observation_control_driver,
        )

        return create_ros2_observation_control_driver(
            carla_module=carla_module,
            vehicle=ego_vehicle,
            route=list(ego_config.get("route") or []),
            control_topic=str(ego_config.get("control_topic")),
            observation_topic=str(ego_config.get("observation_topic")),
            timeout_sec=float(ego_config.get("control_timeout_sec", 0.5)),
        )

    raise ValueError(f"unsupported ego driver: {driver_kind}")


class _CollisionTracker:
    def __init__(self) -> None:
        self._pending = 0

    def on_collision(self, _event: Any) -> None:
        self._pending += 1

    def consume_tick(self) -> bool:
        collided = self._pending > 0
        self._pending = 0
        return collided


def _spawn_collision_tracker(carla_module: Any, world: Any, ego_vehicle: Any) -> tuple[Any | None, Any | None]:
    if not hasattr(world, "get_blueprint_library") or not hasattr(world, "spawn_actor"):
        return None, None
    library = world.get_blueprint_library()
    if not hasattr(library, "find"):
        return None, None
    try:
        blueprint = library.find("sensor.other.collision")
        sensor = world.spawn_actor(
            blueprint,
            carla_module.Transform(),
            attach_to=ego_vehicle,
        )
        tracker = _CollisionTracker()
        sensor.listen(tracker.on_collision)
        return tracker, sensor
    except Exception:
        return None, None


def _runtime_report_config(plan: dict[str, Any]) -> dict[str, Any]:
    return {
        "run_id": plan.get("run_id"),
        "scenario_id": plan.get("scenario_id", "unknown"),
        "actors": list(plan.get("actors") or []),
        "metrics": list(plan.get("metrics") or []),
        "experiment": dict(plan.get("experiment") or {}),
        "evaluation": dict(plan.get("evaluation") or {}),
        "reconstruction_package": dict(plan.get("reconstruction_package") or {}),
    }


def _copy_settings(settings: Any) -> Any:
    try:
        import copy

        return copy.copy(settings)
    except Exception:
        return settings


def _apply_world_weather(carla_module: Any, world: Any, weather: Any) -> None:
    if weather is None or not hasattr(world, "set_weather"):
        return
    weather_type = getattr(carla_module, "WeatherParameters", None)
    if isinstance(weather, str):
        preset = getattr(weather_type, weather, None) if weather_type is not None else None
        if preset is None:
            raise ValueError(f"unknown CARLA weather preset: {weather}")
        world.set_weather(preset)
        return
    if isinstance(weather, dict):
        if weather_type is None:
            raise RuntimeError("CARLA WeatherParameters is unavailable")
        try:
            world.set_weather(weather_type(**weather))
        except TypeError:
            value = weather_type()
            for key, item in weather.items():
                setattr(value, key, item)
            world.set_weather(value)
        return
    raise ValueError("CARLA weather must be a preset name or an attribute mapping")


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


def _spawn_interactive_actor_vehicles(
    carla_module: Any,
    world: Any,
    actors: list[dict[str, Any]],
    actor_vehicles: dict[str, Any],
) -> None:
    for actor in actors:
        if not _should_spawn_actor(actor):
            continue
        actor_id = str(actor.get("actor_id", "actor"))
        actor_vehicles[actor_id] = _spawn_actor_vehicle(carla_module, world, actor, actor_id)


def _spawn_actor_vehicle(carla_module: Any, world: Any, actor: dict[str, Any], actor_id: str) -> Any:
    blueprint_library = world.get_blueprint_library()
    actor_kind = _actor_kind(actor)
    if (
        actor_kind == "pedestrian"
        and actor.get("closed_loop_level") == "traffic_manager_reactive"
    ):
        raise ValueError(
            f"pedestrian actor {actor_id} requires scripted WalkerControl; "
            "CARLA TrafficManager only controls vehicles"
        )
    default_filter = "walker.pedestrian.*" if actor_kind == "pedestrian" else "vehicle.*"
    blueprint_filter = str(actor.get("blueprint", default_filter))
    blueprint_candidates = blueprint_library.filter(blueprint_filter)
    if not blueprint_candidates and blueprint_filter != default_filter:
        blueprint_candidates = blueprint_library.filter(default_filter)
    if not blueprint_candidates:
        raise RuntimeError(f"no CARLA vehicle blueprint matched {blueprint_filter} for actor {actor_id}")

    blueprint = blueprint_candidates[0]
    _set_blueprint_attribute(
        blueprint, "role_name", str(actor.get("role_name", actor_id))
    )
    if actor_kind == "pedestrian":
        _set_blueprint_attribute(blueprint, "is_invincible", "false")

    transform = _carla_transform(carla_module, actor.get("initial_state") or {})
    vehicle = _try_spawn(world, blueprint, transform)
    if vehicle is not None:
        return vehicle

    if actor_kind == "vehicle" and isinstance(actor.get("binding"), dict):
        raise RuntimeError(
            f"failed to spawn bound interactive vehicle actor {actor_id} at declared pose; "
            "map fallback would invalidate actor identity and alignment evidence"
        )

    if actor_kind == "vehicle":
        for fallback_transform in _map_spawn_points(world):
            vehicle = _try_spawn(world, blueprint, fallback_transform)
            if vehicle is not None:
                return vehicle

    raise RuntimeError(f"failed to spawn interactive {actor_kind} actor {actor_id}")


def _snap_plan_to_map(carla_module: Any, world: Any, plan: dict[str, Any]) -> None:
    if not hasattr(world, "get_map"):
        return
    world_map = world.get_map()
    ego = plan.get("ego") or {}
    if ego.get("spawn"):
        ego["spawn"] = _snap_pose_to_waypoint(carla_module, world_map, ego["spawn"])
    if ego.get("destination"):
        ego["destination"] = _snap_pose_to_waypoint(carla_module, world_map, ego["destination"])
    if ego.get("route"):
        ego["route"] = [
            _snap_pose_to_waypoint(carla_module, world_map, point)
            for point in ego["route"]
        ]
    for actor in plan.get("actors") or []:
        if (
            _should_spawn_actor(actor)
            and _actor_kind(actor) == "vehicle"
            and actor.get("initial_state")
        ):
            actor["initial_state"] = _snap_pose_to_waypoint(carla_module, world_map, actor["initial_state"])


def _bind_runtime_topology_route(
    world: Any,
    plan: dict[str, Any],
    *,
    distance_m: float,
    step_m: float = 2.0,
) -> None:
    """Bind a short route that is guaranteed to stay in one CARLA map component.

    This is an explicit physics-runtime acceptance route. It must not be used as
    evidence that a source trajectory has been spatially aligned to the map.
    """
    if distance_m <= 0.0:
        raise ValueError("runtime topology route distance must be positive")
    world_map = world.get_map()
    spawn_points = list(world_map.get_spawn_points())
    if not spawn_points:
        raise RuntimeError("OpenDRIVE world exposes no vehicle spawn points")

    best_transforms: list[Any] = []
    best_distance = 0.0
    for start_transform in spawn_points:
        waypoint = _get_projected_waypoint_from_location(
            world_map, start_transform.location
        )
        if waypoint is None:
            continue
        transforms = [waypoint.transform]
        travelled_m = 0.0
        while travelled_m + 1e-6 < distance_m:
            requested_step = min(step_m, distance_m - travelled_m)
            candidates = list(waypoint.next(requested_step))
            if not candidates:
                break
            current_yaw = float(getattr(waypoint.transform.rotation, "yaw", 0.0))
            candidates.sort(
                key=lambda item: abs(
                    (float(getattr(item.transform.rotation, "yaw", 0.0)) - current_yaw + 180.0)
                    % 360.0
                    - 180.0
                )
            )
            selected = None
            for candidate in candidates:
                location = candidate.transform.location
                if all(
                    math.hypot(
                        float(location.x) - float(old.location.x),
                        float(location.y) - float(old.location.y),
                    ) >= 0.75
                    for old in transforms[:-1]
                ):
                    selected = candidate
                    break
            if selected is None:
                break
            segment = math.hypot(
                float(selected.transform.location.x) - float(waypoint.transform.location.x),
                float(selected.transform.location.y) - float(waypoint.transform.location.y),
            )
            if segment <= 0.1:
                break
            waypoint = selected
            transforms.append(waypoint.transform)
            travelled_m += segment
        if travelled_m > best_distance:
            best_transforms = transforms
            best_distance = travelled_m
        if travelled_m >= distance_m * 0.98:
            break
    transforms = best_transforms
    travelled_m = best_distance
    if travelled_m < min(distance_m * 0.8, distance_m - 0.5):
        raise RuntimeError(
            f"runtime topology route is too short: requested={distance_m}, actual={travelled_m}"
        )

    route = []
    for transform in transforms:
        pose = _pose_from_transform(transform)
        pose["z"] += 0.5
        route.append(pose)
    ego = plan.get("ego") or {}
    ego["spawn"] = dict(route[0])
    ego["destination"] = dict(route[-1])
    ego["route"] = route
    ego["route_binding"] = {
        "source": "carla_runtime_topology",
        "requested_distance_m": float(distance_m),
        "actual_distance_m": float(travelled_m),
        "source_trajectory_alignment": "not_claimed",
    }


def _get_projected_waypoint_from_location(world_map: Any, location: Any) -> Any | None:
    try:
        return world_map.get_waypoint(location, project_to_road=True)
    except TypeError:
        return world_map.get_waypoint(location)


def _snap_pose_to_waypoint(carla_module: Any, world_map: Any, pose: dict[str, Any]) -> dict[str, float]:
    if not hasattr(world_map, "get_waypoint"):
        return _pose(dict(pose))
    location = _carla_location(carla_module, pose)
    waypoint = _get_projected_waypoint(carla_module, world_map, location)
    if waypoint is None or not hasattr(waypoint, "transform"):
        return _pose(dict(pose))
    transform = waypoint.transform
    snapped = _pose(dict(pose))
    snapped.update(_pose_from_transform(transform))
    snapped["z"] = snapped.get("z", 0.0) + 0.3
    return snapped


def _get_projected_waypoint(carla_module: Any, world_map: Any, location: Any) -> Any | None:
    lane_type = getattr(getattr(carla_module, "LaneType", None), "Driving", None)
    try:
        if lane_type is not None:
            return world_map.get_waypoint(location, project_to_road=True, lane_type=lane_type)
        return world_map.get_waypoint(location, project_to_road=True)
    except TypeError:
        try:
            return world_map.get_waypoint(location, True)
        except TypeError:
            return world_map.get_waypoint(location)


def _pose_from_transform(transform: Any) -> dict[str, float]:
    location = getattr(transform, "location", None)
    rotation = getattr(transform, "rotation", None)
    return {
        "x": float(getattr(location, "x", 0.0)),
        "y": float(getattr(location, "y", 0.0)),
        "z": float(getattr(location, "z", 0.0)),
        "yaw": float(getattr(rotation, "yaw", 0.0)),
    }


def _configure_actor_autopilot(
    client: Any,
    actor_vehicles: dict[str, Any],
    actors: list[dict[str, Any]],
    *,
    enabled: bool,
    tm_port: int,
    seed: int | None = None,
) -> tuple[set[str], Any | None]:
    if not enabled:
        return set(), None
    traffic_manager = client.get_trafficmanager(tm_port) if hasattr(client, "get_trafficmanager") else None
    if traffic_manager is not None and hasattr(traffic_manager, "set_synchronous_mode"):
        try:
            traffic_manager.set_synchronous_mode(True)
        except Exception:
            pass
    if seed is not None and traffic_manager is not None and hasattr(traffic_manager, "set_random_device_seed"):
        traffic_manager.set_random_device_seed(int(seed))

    actor_by_id = {str(actor.get("actor_id", "actor")): actor for actor in actors}
    bound_actor_ids: set[str] = set()
    for actor_id, vehicle in actor_vehicles.items():
        actor = actor_by_id.get(actor_id, {})
        if _actor_kind(actor) != "vehicle":
            continue
        if actor.get("closed_loop_level") != "traffic_manager_reactive":
            continue
        if hasattr(vehicle, "set_autopilot"):
            try:
                vehicle.set_autopilot(True, tm_port)
            except TypeError:
                vehicle.set_autopilot(True)
            bound_actor_ids.add(actor_id)
        _apply_actor_traffic_manager_settings(traffic_manager, vehicle, actor)
    return bound_actor_ids, traffic_manager


def _apply_actor_traffic_manager_settings(traffic_manager: Any, vehicle: Any, actor: dict[str, Any]) -> None:
    if traffic_manager is None:
        return
    style_profile = actor.get("style_profile") or {}
    min_gap = style_profile.get("min_gap_m")
    if min_gap is not None and hasattr(traffic_manager, "distance_to_leading_vehicle"):
        traffic_manager.distance_to_leading_vehicle(vehicle, float(min_gap))
    if hasattr(traffic_manager, "auto_lane_change"):
        traffic_manager.auto_lane_change(vehicle, bool(actor.get("closed_loop_level") == "traffic_manager_reactive"))
    if hasattr(traffic_manager, "vehicle_percentage_speed_difference"):
        traffic_manager.vehicle_percentage_speed_difference(vehicle, 0.0)


def _map_spawn_points(world: Any) -> list[Any]:
    if not hasattr(world, "get_map"):
        return []
    world_map = world.get_map()
    if not hasattr(world_map, "get_spawn_points"):
        return []
    return list(world_map.get_spawn_points())


def _try_spawn(world: Any, blueprint: Any, transform: Any) -> Any | None:
    if hasattr(world, "try_spawn_actor"):
        return world.try_spawn_actor(blueprint, transform)
    try:
        return world.spawn_actor(blueprint, transform)
    except Exception:
        return None


def _set_blueprint_attribute(blueprint: Any, name: str, value: str) -> None:
    if not hasattr(blueprint, "set_attribute"):
        return
    if hasattr(blueprint, "has_attribute") and not blueprint.has_attribute(name):
        return
    try:
        blueprint.set_attribute(name, str(value))
    except (KeyError, ValueError, RuntimeError):
        return


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
        return {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
    transform = vehicle.get_transform()
    location = getattr(transform, "location", None)
    rotation = getattr(transform, "rotation", None)
    return {
        "x": float(getattr(location, "x", 0.0)),
        "y": float(getattr(location, "y", 0.0)),
        "z": float(getattr(location, "z", 0.0)),
        "roll": float(getattr(rotation, "roll", 0.0)),
        "pitch": float(getattr(rotation, "pitch", 0.0)),
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


def _draw_route_lane_lines(
    carla_module: Any,
    world: Any,
    route: list[dict[str, Any]],
    *,
    lane_width_m: float = 3.5,
    stride: int = 8,
) -> None:
    debug = getattr(world, "debug", None)
    if debug is None or not hasattr(debug, "draw_line") or len(route) < 2:
        return
    color_type = getattr(carla_module, "Color", None)
    boundary_color = color_type(235, 235, 235) if color_type is not None else None
    center_color = color_type(255, 190, 0) if color_type is not None else None
    half_width = float(lane_width_m) / 2.0
    step = max(1, int(stride))
    for start_index in range(0, len(route) - 1, step):
        end_index = min(start_index + step, len(route) - 1)
        start = route[start_index]
        end = route[end_index]
        for side in (-1.0, 1.0):
            begin = _route_offset_location(carla_module, start, side * half_width)
            finish = _route_offset_location(carla_module, end, side * half_width)
            _draw_debug_line(debug, begin, finish, boundary_color, 0.035, 60.0)
        if (start_index // step) % 2 == 0:
            begin = _route_offset_location(carla_module, start, 0.0)
            finish = _route_offset_location(carla_module, end, 0.0)
            _draw_debug_line(debug, begin, finish, center_color, 0.025, 60.0)


def _route_offset_location(carla_module: Any, pose: dict[str, Any], offset_m: float) -> Any:
    yaw_rad = math.radians(float(pose.get("yaw", 0.0)))
    return carla_module.Location(
        x=float(pose.get("x", 0.0)) - math.sin(yaw_rad) * float(offset_m),
        y=float(pose.get("y", 0.0)) + math.cos(yaw_rad) * float(offset_m),
        z=float(pose.get("z", 0.0)) + 0.08,
    )


def _draw_debug_line(
    debug: Any,
    begin: Any,
    end: Any,
    color: Any,
    thickness: float,
    lifetime: float,
) -> None:
    kwargs = {"thickness": float(thickness), "life_time": float(lifetime)}
    if color is not None:
        kwargs["color"] = color
    try:
        debug.draw_line(begin, end, **kwargs)
    except TypeError:
        debug.draw_line(begin, end)


def _draw_dynamic_vehicle_boxes(
    carla_module: Any,
    world: Any,
    ego_vehicle: Any,
    actor_vehicles: dict[str, Any],
    *,
    life_time: float,
) -> None:
    debug = getattr(world, "debug", None)
    if debug is None or not hasattr(debug, "draw_box"):
        return
    color_type = getattr(carla_module, "Color", None)
    ego_color = color_type(0, 220, 255) if color_type is not None else None
    actor_color = color_type(255, 90, 70) if color_type is not None else None
    _draw_actor_box(carla_module, debug, ego_vehicle, ego_color, life_time)
    for vehicle in actor_vehicles.values():
        _draw_actor_box(carla_module, debug, vehicle, actor_color, life_time)


def _draw_actor_box(
    carla_module: Any,
    debug: Any,
    actor: Any,
    color: Any,
    life_time: float,
) -> None:
    if actor is None or not hasattr(actor, "get_transform") or not hasattr(actor, "bounding_box"):
        return
    transform = actor.get_transform()
    source_box = actor.bounding_box
    source_location = getattr(source_box, "location", None)
    local_location = carla_module.Location(
        x=float(getattr(source_location, "x", 0.0)),
        y=float(getattr(source_location, "y", 0.0)),
        z=float(getattr(source_location, "z", 0.0)),
    )
    transformed_location = transform.transform(local_location)
    world_location = transformed_location if transformed_location is not None else local_location
    box_type = getattr(carla_module, "BoundingBox", None)
    box = box_type(world_location, source_box.extent) if box_type is not None else source_box
    rotation = getattr(transform, "rotation", None)
    box_rotation = getattr(source_box, "rotation", None)
    if box_rotation is not None and rotation is not None:
        rotation = carla_module.Rotation(
            pitch=float(getattr(rotation, "pitch", 0.0)) + float(getattr(box_rotation, "pitch", 0.0)),
            yaw=float(getattr(rotation, "yaw", 0.0)) + float(getattr(box_rotation, "yaw", 0.0)),
            roll=float(getattr(rotation, "roll", 0.0)) + float(getattr(box_rotation, "roll", 0.0)),
        )
    kwargs = {"thickness": 0.045, "life_time": float(life_time)}
    if color is not None:
        kwargs["color"] = color
    try:
        debug.draw_box(box, rotation, **kwargs)
    except TypeError:
        debug.draw_box(box, rotation)


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
    simulation_time_sec: float | None = None,
    actor_vehicles: dict[str, Any] | None = None,
    behavior_planner: Any = plan_reactive_actor_control,
) -> tuple[dict[str, float], dict[str, dict[str, Any]], float | None]:
    actor_distances_m: dict[str, float] = {}
    actor_decisions: dict[str, dict[str, Any]] = {}
    finite_ttc_values: list[float] = []
    actor_vehicles = actor_vehicles or {}

    for actor in actors:
        actor_id = str(actor.get("actor_id", "actor"))
        actor_vehicle = actor_vehicles.get(actor_id)
        if actor_vehicle is None:
            continue
        actor_state = _sample_actor_state(actor, actor_vehicle)
        actor_speed_mps = float(actor_state.get("speed_mps", 0.0))
        distance_m = _xy_distance(actor_state, ego_pose)
        actor_distances_m[actor_id] = distance_m
        relative_speed_mps = max(0.0, ego_speed_mps - actor_speed_mps)
        if relative_speed_mps > 0.0:
            finite_ttc_values.append(distance_m / relative_speed_mps)
        if not _is_interactive_actor(actor):
            continue
        decision = behavior_planner(
            actor_state,
            {
                "x": ego_pose.get("x", 0.0),
                "y": ego_pose.get("y", 0.0),
                "speed_mps": ego_speed_mps,
                "distance_m": distance_m,
                "relative_speed_mps": relative_speed_mps,
            },
            style=_actor_style(actor),
            reference_speed_mps=_actor_reference_speed(actor, simulation_time_sec),
        )
        decision = dict(decision)
        decision["target_point"] = _next_reference_target(actor, actor_state)
        if _actor_kind(actor) == "pedestrian":
            # Pedestrian closed loop is intentionally root-pose only: ego may
            # change speed/pause/yield/abort, never the recorded walking corridor.
            decision["motion_constraint"] = "source_reference_corridor"
            decision["allowed_actions"] = ["speed", "pause", "yield", "abort"]
        else:
            decision["motion_constraint"] = "carla_lane_or_reference_route"
            decision["allowed_actions"] = [
                "speed",
                "throttle",
                "brake",
                "steer",
                "yield",
                "abort",
            ]
        actor_distances_m[actor_id] = float(decision["distance_m"])
        actor_decisions[actor_id] = decision
        ttc_sec = decision.get("ttc_sec")
        if isinstance(ttc_sec, (int, float)) and math.isfinite(float(ttc_sec)):
            finite_ttc_values.append(float(ttc_sec))

    return actor_distances_m, actor_decisions, min(finite_ttc_values) if finite_ttc_values else None


def _sample_actor_state(actor: dict[str, Any], actor_vehicle: Any | None) -> dict[str, Any]:
    actor_state = dict(actor.get("initial_state") or {})
    if actor_vehicle is None:
        return actor_state
    actor_state.update(_vehicle_pose(actor_vehicle))
    actor_state["speed_mps"] = _vehicle_speed_mps(actor_vehicle)
    return actor_state


def _is_interactive_actor(actor: dict[str, Any]) -> bool:
    if actor.get("closed_loop_level") in {"scripted", "traffic_manager_reactive"}:
        return True
    closed_loop = actor.get("closed_loop") or {}
    return bool(closed_loop.get("ego_responsive", False))


def _should_spawn_actor(actor: dict[str, Any]) -> bool:
    if _is_interactive_actor(actor):
        return True
    return bool(
        actor.get("closed_loop_level") == "replay"
        and actor.get("initial_state")
        and actor.get("reference_trajectory")
    )


def _xy_distance(actor_state: dict[str, Any], ego_pose: dict[str, float]) -> float:
    dx = float(ego_pose.get("x", 0.0)) - float(actor_state.get("x", 0.0))
    dy = float(ego_pose.get("y", 0.0)) - float(actor_state.get("y", 0.0))
    return math.sqrt(dx * dx + dy * dy)


def _actor_style(actor: dict[str, Any]) -> str:
    style_profile = actor.get("style_profile") or {}
    return str(actor.get("style", style_profile.get("name", "normal")))


def _actor_reference_speed(
    actor: dict[str, Any], simulation_time_sec: float | None = None
) -> float | None:
    trajectory = actor.get("reference_trajectory") or []
    if simulation_time_sec is not None:
        timed = [
            point
            for point in trajectory
            if isinstance(point, dict)
            and point.get("t_sec") is not None
            and point.get("speed_mps") is not None
        ]
        if timed:
            point = min(
                timed,
                key=lambda sample: abs(
                    float(sample["t_sec"]) - float(simulation_time_sec)
                ),
            )
            return max(0.0, float(point["speed_mps"]))
    if trajectory and isinstance(trajectory[-1], dict) and trajectory[-1].get("speed_mps") is not None:
        return float(trajectory[-1]["speed_mps"])
    return None


def _apply_scripted_actor_controls(
    carla_module: Any,
    actors: list[dict[str, Any]],
    actor_vehicles: dict[str, Any],
    actor_decisions: dict[str, dict[str, Any]],
) -> dict[str, str]:
    evidence: dict[str, str] = {}
    actor_by_id = {str(actor.get("actor_id", "actor")): actor for actor in actors}
    for actor_id, decision in actor_decisions.items():
        actor = actor_by_id.get(actor_id, {})
        if actor.get("closed_loop_level") == "traffic_manager_reactive":
            continue
        vehicle = actor_vehicles.get(actor_id)
        if vehicle is None or not hasattr(vehicle, "apply_control"):
            continue
        if _actor_kind(actor) == "pedestrian":
            vehicle.apply_control(
                _walker_control(carla_module, actor, vehicle, decision)
            )
            evidence[actor_id] = "scripted_walker_control"
            continue
        current_speed = _vehicle_speed_mps(vehicle)
        desired_speed = float(decision.get("desired_speed_mps", current_speed))
        should_brake = bool(decision.get("brake", False))
        throttle = 0.0 if should_brake else min(1.0, max(0.0, (desired_speed - current_speed) / 5.0))
        brake = min(1.0, max(0.0, (current_speed - desired_speed) / 5.0)) if should_brake else 0.0
        steer = 0.0
        target_point = decision.get("target_point")
        if isinstance(target_point, dict):
            pose = _vehicle_pose(vehicle)
            desired_yaw = math.degrees(
                math.atan2(
                    float(target_point.get("y", pose["y"])) - pose["y"],
                    float(target_point.get("x", pose["x"])) - pose["x"],
                )
            )
            yaw_error = (desired_yaw - pose["yaw"] + 180.0) % 360.0 - 180.0
            steer = min(1.0, max(-1.0, yaw_error / 45.0))
        vehicle.apply_control(
            _vehicle_control(
                carla_module,
                throttle=throttle,
                brake=max(brake, 0.35 if should_brake else 0.0),
                steer=steer,
            )
        )
        evidence[actor_id] = "scripted_vehicle_control"
    return evidence


def _apply_replay_actor_controls(
    carla_module: Any,
    actors: list[dict[str, Any]],
    actor_vehicles: dict[str, Any],
    *,
    run_time_sec: float,
) -> dict[str, str]:
    evidence: dict[str, str] = {}
    for actor in actors:
        if actor.get("closed_loop_level") != "replay":
            continue
        actor_id = str(actor.get("actor_id", "actor"))
        entity = actor_vehicles.get(actor_id)
        if entity is None or not hasattr(entity, "apply_control"):
            continue
        target = _reference_pose_at_time(actor, run_time_sec)
        if target is None:
            continue
        target_speed = _reference_speed_at_time(actor, run_time_sec)
        if _actor_kind(actor) == "pedestrian":
            entity.apply_control(
                _walker_control(
                    carla_module,
                    {**actor, "reference_trajectory": [target]},
                    entity,
                    {"desired_speed_mps": target_speed, "should_abort": False},
                )
            )
            evidence[actor_id] = "trajectory_replay_walker_control"
            continue
        pose = _vehicle_pose(entity)
        desired_yaw = math.degrees(
            math.atan2(target["y"] - pose["y"], target["x"] - pose["x"])
        )
        yaw_error = (desired_yaw - pose["yaw"] + 180.0) % 360.0 - 180.0
        current_speed = _vehicle_speed_mps(entity)
        speed_error = target_speed - current_speed
        entity.apply_control(
            _vehicle_control(
                carla_module,
                throttle=min(0.7, max(0.0, 0.25 * speed_error)),
                brake=min(0.8, max(0.0, -0.35 * speed_error)),
                steer=min(1.0, max(-1.0, yaw_error / 45.0)),
            )
        )
        evidence[actor_id] = "trajectory_replay_vehicle_control"
    return evidence


def _actor_kind(actor: dict[str, Any]) -> str:
    value = str(actor.get("actor_type", actor.get("type", "vehicle"))).lower()
    if value in {"pedestrian", "walker", "person"}:
        return "pedestrian"
    return "vehicle"


def _walker_control(
    carla_module: Any,
    actor: dict[str, Any],
    walker: Any,
    decision: dict[str, Any],
) -> Any:
    pose = _vehicle_pose(walker)
    # Never accept a free-space pedestrian target from a behavior plugin. The
    # only editable dimensions are longitudinal progress and stop/yield state.
    target = _next_reference_target(actor, pose)
    dx = float(target.get("x", pose["x"])) - pose["x"]
    dy = float(target.get("y", pose["y"])) - pose["y"]
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        yaw = math.radians(float(pose.get("yaw", 0.0)))
        dx, dy, norm = math.cos(yaw), math.sin(yaw), 1.0
    direction_type = getattr(carla_module, "Vector3D", None)
    direction = (
        direction_type(x=dx / norm, y=dy / norm, z=0.0)
        if direction_type is not None
        else SimpleNamespace(x=dx / norm, y=dy / norm, z=0.0)
    )
    speed = float(decision.get("desired_speed_mps", 0.0))
    if bool(decision.get("should_abort", False)):
        speed = 0.0
    control_type = getattr(carla_module, "WalkerControl", None)
    values = {"direction": direction, "speed": max(0.0, speed), "jump": False}
    if control_type is None:
        return SimpleNamespace(**values)
    try:
        return control_type(**values)
    except TypeError:
        control = control_type()
        for key, value in values.items():
            setattr(control, key, value)
        return control


def _next_reference_target(
    actor: dict[str, Any], pose: dict[str, float]
) -> dict[str, Any]:
    trajectory = [
        point
        for point in actor.get("reference_trajectory") or []
        if isinstance(point, dict) and "x" in point and "y" in point
    ]
    if not trajectory:
        return pose
    nearest = min(
        range(len(trajectory)),
        key=lambda index: _xy_distance(trajectory[index], pose),
    )
    return trajectory[min(len(trajectory) - 1, nearest + 1)]


def _reference_pose_at_time(
    actor: dict[str, Any], simulation_time_sec: float
) -> dict[str, float] | None:
    trajectory = [
        point for point in actor.get("reference_trajectory") or []
        if isinstance(point, dict)
    ]
    timed = [point for point in trajectory if point.get("t_sec") is not None]
    if not timed:
        return None
    point = min(
        timed,
        key=lambda sample: abs(float(sample["t_sec"]) - float(simulation_time_sec)),
    )
    return _pose(point)


def _reference_speed_at_time(actor: dict[str, Any], run_time_sec: float) -> float:
    trajectory = [
        point for point in actor.get("reference_trajectory") or []
        if isinstance(point, dict) and point.get("t_sec") is not None
    ]
    if not trajectory:
        return max(0.0, float((actor.get("initial_state") or {}).get("speed_mps", 0.0)))
    point = min(
        trajectory,
        key=lambda sample: abs(float(sample["t_sec"]) - float(run_time_sec)),
    )
    return max(0.0, float(point.get("speed_mps", 0.0)))


def _actor_runtime_binding_evidence(
    plan: dict[str, Any],
    actor_vehicles: dict[str, Any],
) -> dict[str, Any]:
    declaration = plan.get("actor_binding") or {}
    selected = [str(value) for value in declaration.get("selected_actor_ids") or []]
    actor_by_id = {
        str(actor.get("actor_id", "actor")): actor
        for actor in plan.get("actors") or []
    }
    if not selected:
        selected = [
            actor_id
            for actor_id, actor in actor_by_id.items()
            if isinstance(actor.get("binding"), dict)
        ]
    if not selected:
        return {
            "schema_version": "actor_runtime_binding_evidence.v1",
            "status": "not_configured",
            "records": [],
            "issues": [],
        }
    issues: list[str] = []
    if (declaration.get("readiness") or {}).get("status") not in {None, "ready"}:
        issues.append("declared_actor_binding_not_ready")
    records = []
    for actor_id in selected:
        actor = actor_by_id.get(actor_id)
        record_issues = []
        binding = actor.get("binding") if isinstance(actor, dict) else None
        entity = actor_vehicles.get(actor_id)
        if actor is None:
            record_issues.append("run_actor_missing")
            binding = {}
        elif not isinstance(binding, dict):
            record_issues.append("runtime_binding_missing")
            binding = {}
        if entity is None:
            record_issues.append("carla_physical_actor_missing")
        runtime_actor_id = getattr(entity, "id", None) if entity is not None else None
        if not isinstance(runtime_actor_id, int) or isinstance(runtime_actor_id, bool):
            record_issues.append("carla_runtime_actor_id_missing")
            runtime_actor_id = None
        expected_role = str((actor or {}).get("role_name") or actor_id)
        attributes = getattr(entity, "attributes", {}) if entity is not None else {}
        actual_role = (
            str(attributes.get("role_name") or "")
            if isinstance(attributes, dict)
            else ""
        )
        if actual_role != expected_role:
            record_issues.append("carla_role_name_mismatch")
        if binding.get("declared_status") != "ready":
            record_issues.append("binding_declared_status_not_ready")
        if not binding.get("nurec_track_id"):
            record_issues.append("nurec_track_id_missing")
        if not (actor or {}).get("source_track_id"):
            record_issues.append("source_track_id_missing")
        if (
            binding.get("nurec_track_id")
            and (actor or {}).get("source_track_id")
            and binding.get("nurec_track_id") != (actor or {}).get("source_track_id")
        ):
            record_issues.append("source_nurec_track_mismatch")
        if set(binding.get("required_modalities") or []) != {"rgb", "lidar"}:
            record_issues.append("required_modalities_must_be_rgb_lidar")
        if binding.get("same_dynamic_object_for_all_modalities") is not True:
            record_issues.append("cross_modality_dynamic_object_sync_not_required")
        if binding.get("sensor_pose_source") not in {
            "carla_runtime_actor_pose",
            "scenario_ir_reference_trajectory",
        }:
            record_issues.append("sensor_pose_source_invalid")
        record = {
            "actor_id": actor_id,
            "actor_type": _actor_kind(actor or {}),
            "source_track_id": (actor or {}).get("source_track_id"),
            "carla": {
                "expected_role_name": expected_role,
                "actual_role_name": actual_role,
                "runtime_actor_id": runtime_actor_id,
            },
            "nurec_track_id": binding.get("nurec_track_id"),
            "sensor_pose_source": binding.get("sensor_pose_source"),
            "required_modalities": list(binding.get("required_modalities") or []),
            "status": "passed" if not record_issues else "failed",
            "issues": record_issues,
        }
        issues.extend(f"{actor_id}:{issue}" for issue in record_issues)
        records.append(record)
    runtime_ids = [
        record["carla"]["runtime_actor_id"]
        for record in records
        if record["carla"]["runtime_actor_id"] is not None
    ]
    if len(runtime_ids) != len(set(runtime_ids)):
        issues.append("duplicate_carla_runtime_actor_id")
    return {
        "schema_version": "actor_runtime_binding_evidence.v1",
        "status": "passed" if not issues else "failed",
        "records": records,
        "issues": sorted(set(issues)),
    }


def _initial_bound_actor_poses(
    actors: list[dict[str, Any]],
    actor_vehicles: dict[str, Any],
) -> dict[str, dict[str, float]]:
    result = {}
    for actor in actors:
        if not isinstance(actor.get("binding"), dict):
            continue
        actor_id = str(actor.get("actor_id", "actor"))
        pose_source = actor["binding"].get("sensor_pose_source")
        if pose_source == "scenario_ir_reference_trajectory":
            pose = _reference_pose_at_time(actor, 0.0)
        else:
            entity = actor_vehicles.get(actor_id)
            pose = _vehicle_pose(entity) if entity is not None else None
        if pose is not None:
            result[actor_id] = dict(pose)
    return result


def _current_bound_actor_poses(
    actors: list[dict[str, Any]],
    actor_states: dict[str, dict[str, Any]],
    simulation_time_sec: float,
) -> dict[str, dict[str, float]]:
    result = {}
    for actor in actors:
        binding = actor.get("binding")
        if not isinstance(binding, dict):
            continue
        actor_id = str(actor.get("actor_id", "actor"))
        if binding.get("sensor_pose_source") == "scenario_ir_reference_trajectory":
            pose = _reference_pose_at_time(actor, simulation_time_sec)
        else:
            state = actor_states.get(actor_id) or {}
            pose = state.get("pose")
        if isinstance(pose, dict):
            result[actor_id] = dict(pose)
    return result


def _build_sensor_frame_context(
    plan: dict[str, Any],
    *,
    frame_id: int,
    tick_index: int,
    simulation_time_sec: float,
    scenario_time_sec: float,
    interval_start_sec: float,
    ego_pose: dict[str, float],
    previous_ego_pose: dict[str, float],
    actor_states: dict[str, dict[str, Any]],
    previous_actor_poses: dict[str, dict[str, float]],
) -> dict[str, Any]:
    current_actor_poses = _current_bound_actor_poses(
        plan.get("actors") or [],
        actor_states,
        scenario_time_sec,
    )
    samples = {}
    actor_by_id = {
        str(actor.get("actor_id", "actor")): actor
        for actor in plan.get("actors") or []
    }
    selected = [str(value) for value in (plan.get("actor_binding") or {}).get("selected_actor_ids") or []]
    if not selected:
        selected = sorted(current_actor_poses)
    for actor_id in selected:
        actor = actor_by_id.get(actor_id) or {}
        binding = actor.get("binding") or {}
        current = current_actor_poses.get(actor_id)
        previous = previous_actor_poses.get(actor_id)
        if current is None or previous is None:
            raise RuntimeError(f"bound actor has no render pose pair: {actor_id}")
        samples[actor_id] = {
            "source": binding.get("sensor_pose_source"),
            "pose_pair": {"start": dict(previous), "end": dict(current)},
            "carla_runtime_actor_id": (actor_states.get(actor_id) or {}).get(
                "carla_runtime_actor_id"
            ),
            "nurec_track_id": binding.get("nurec_track_id"),
        }
    return {
        "schema_version": "carla_nurec_frame_context.v1",
        "scene_id": plan.get("scenario_id"),
        "frame_id": frame_id,
        "tick_index": tick_index,
        "simulation_time_sec": float(simulation_time_sec),
        "scenario_time_sec": float(scenario_time_sec),
        "interval_start_sec": float(interval_start_sec),
        "ego_pose_pair": {
            "start": dict(previous_ego_pose),
            "end": dict(ego_pose),
        },
        "actor_samples": samples,
        "clock": "carla_snapshot",
    }


def _validate_sensor_frame_evidence(evidence: Any, frame_id: int) -> None:
    if not isinstance(evidence, dict):
        raise RuntimeError("sensor_frame_handler must return evidence as a dictionary")
    from adapters.nurec_multimodal import (
        NuRecMultimodalError,
        validate_nurec_multimodal_evidence,
    )

    try:
        validate_nurec_multimodal_evidence(evidence)
    except NuRecMultimodalError as exc:
        raise RuntimeError(f"invalid NuRec multimodal evidence: {exc}") from exc
    if evidence.get("frame_id") != frame_id:
        raise RuntimeError(
            f"NuRec evidence frame mismatch: expected={frame_id}, actual={evidence.get('frame_id')}"
        )


def _multimodal_runtime_summary(
    trace: list[dict[str, Any]],
    *,
    required: bool,
    handler_present: bool,
) -> dict[str, Any]:
    if not handler_present:
        status = "not_configured"
    elif not trace:
        status = "no_frames"
    elif all(item.get("status") == "passed" for item in trace):
        status = "passed"
    else:
        status = "failed"
    return {
        "required": bool(required),
        "handler_present": bool(handler_present),
        "status": status,
        "sensor_closed_loop": status == "passed",
        "frame_count": len(trace),
        "passed_frame_count": sum(item.get("status") == "passed" for item in trace),
        "failed_frame_count": sum(item.get("status") != "passed" for item in trace),
        "modalities": ["rgb", "lidar"] if handler_present else [],
    }


def _load_actor_behavior_planner(plugin: Any) -> Any:
    if plugin is None or plugin == "":
        return plan_reactive_actor_control
    if not isinstance(plugin, str) or ":" not in plugin:
        raise ValueError("actor behavior_plugin must use module:function syntax")
    module_name, function_name = plugin.split(":", 1)
    function = getattr(importlib.import_module(module_name), function_name, None)
    if not callable(function):
        raise ValueError(f"actor behavior plugin is not callable: {plugin}")
    return function


def _vehicle_control(carla_module: Any, *, throttle: float, brake: float, steer: float) -> Any:
    control_type = getattr(carla_module, "VehicleControl", None)
    values = {
        "throttle": float(throttle),
        "brake": float(brake),
        "steer": float(steer),
        "hand_brake": False,
        "reverse": False,
    }
    if control_type is not None:
        try:
            return control_type(**values)
        except TypeError:
            control = control_type()
            for key, value in values.items():
                setattr(control, key, value)
            return control
    return SimpleNamespace(**values)


def _control_dict(control: Any) -> dict[str, float]:
    return {
        "throttle": float(getattr(control, "throttle", 0.0)),
        "brake": float(getattr(control, "brake", 0.0)),
        "steer": float(getattr(control, "steer", 0.0)),
    }


def _control_brake(control: Any) -> float:
    return float(getattr(control, "brake", 0.0))


def _route_progress_from_pose(route: list[dict[str, Any]], pose: dict[str, Any]) -> float:
    points = [point for point in route if isinstance(point, dict)]
    if len(points) < 2:
        return 0.0
    segment_lengths = [
        _xy_distance(points[index], points[index + 1])
        for index in range(len(points) - 1)
    ]
    total_length = sum(segment_lengths)
    if total_length <= 1e-6:
        return 1.0 if _xy_distance(points[-1], pose) <= 1.0 else 0.0

    best_distance = math.inf
    best_progress = 0.0
    distance_before = 0.0
    px = float(pose.get("x", 0.0))
    py = float(pose.get("y", 0.0))
    for index, segment_length in enumerate(segment_lengths):
        if segment_length <= 1e-6:
            continue
        start = points[index]
        end = points[index + 1]
        sx = float(start.get("x", 0.0))
        sy = float(start.get("y", 0.0))
        dx = float(end.get("x", 0.0)) - sx
        dy = float(end.get("y", 0.0)) - sy
        projection = min(1.0, max(0.0, ((px - sx) * dx + (py - sy) * dy) / (segment_length ** 2)))
        projected_x = sx + projection * dx
        projected_y = sy + projection * dy
        distance = math.hypot(px - projected_x, py - projected_y)
        progress = (distance_before + projection * segment_length) / total_length
        if distance < best_distance:
            best_distance = distance
            best_progress = progress
        distance_before += segment_length
    return min(1.0, max(0.0, best_progress))


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


def _write_runtime_evidence(
    plan: dict[str, Any],
    frame_trace: list[dict[str, Any]],
    metrics_trace: list[dict[str, Any]],
    cleanup_audit: list[dict[str, Any]],
    multimodal_trace: list[dict[str, Any]],
) -> None:
    artifacts = plan.get("artifacts") or {}
    for name, rows in (
        ("frame_trace", frame_trace),
        ("metrics_trace", metrics_trace),
        ("nurec_multimodal_trace", multimodal_trace),
    ):
        target = artifacts.get(name)
        if not target:
            continue
        path = Path(target)
        if not path.is_absolute():
            path = Path.cwd() / path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            "".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in rows),
            encoding="utf-8",
        )
    target = artifacts.get("cleanup_audit")
    if target:
        path = Path(target)
        if not path.is_absolute():
            path = Path.cwd() / path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(
                {
                    "schema_version": "carla_cleanup_audit.v1",
                    "run_id": plan.get("run_id"),
                    "succeeded": all(row.get("status") == "succeeded" for row in cleanup_audit),
                    "actions": cleanup_audit,
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )


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
    parser.add_argument("--timeout-sec", default=60.0, type=float)
    parser.add_argument("--max-ticks", default=600, type=int)
    parser.add_argument("--async-world", action="store_true", help="Do not request synchronous stepping.")
    parser.add_argument("--execute", action="store_true", help="Attempt real CARLA execution instead of dry-run planning.")
    parser.add_argument("--follow-ego", action="store_true", help="Move CARLA spectator behind the ego vehicle each tick.")
    parser.add_argument("--debug-draw", action="store_true", help="Draw ego and actor debug markers in the CARLA world.")
    parser.add_argument("--snap-to-map", action="store_true", help="Project ego and interactive actor spawns to CARLA map waypoints.")
    parser.add_argument(
        "--no-actor-autopilot",
        action="store_true",
        help="Disable CARLA TrafficManager for traffic_manager_reactive actors.",
    )
    parser.add_argument("--traffic-manager-port", default=8000, type=int)
    parser.add_argument(
        "--ego-driver",
        choices=("basic_agent", "topology_follower", "ros2_control", "ros2_observation_control"),
        default="basic_agent",
    )
    parser.add_argument("--control-topic", default=None)
    parser.add_argument("--observation-topic", default=None)
    parser.add_argument("--control-timeout-sec", default=0.5, type=float)
    parser.add_argument(
        "--acceptance-evidence",
        action="store_true",
        help="Fail closed unless frame, collision, route, actor-response, and cleanup evidence is complete.",
    )
    parser.add_argument(
        "--opendrive",
        default=None,
        help="Generate the CARLA world from this OpenDRIVE file instead of loading a native map.",
    )
    parser.add_argument(
        "--runtime-route-distance-m",
        default=None,
        type=float,
        help=(
            "Replace the source route with a connected route sampled from the loaded "
            "CARLA topology. This is only a physics-runtime acceptance route."
        ),
    )
    parser.add_argument(
        "--physics-smoke",
        action="store_true",
        help="Evaluate only collision and route-completion criteria for a no-actor physics smoke.",
    )
    parser.add_argument(
        "--multimodal-sensor-required",
        action="store_true",
        help=(
            "Mark the plan fail-closed for injected NuRec RGB/LiDAR frame evidence. "
            "Execution requires a sensor_frame_handler from the host integration."
        ),
    )
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
        snap_to_map=args.snap_to_map,
        actor_autopilot=not args.no_actor_autopilot,
        traffic_manager_port=args.traffic_manager_port,
        ego_driver=args.ego_driver,
        control_topic=args.control_topic,
        observation_topic=args.observation_topic,
        control_timeout_sec=args.control_timeout_sec,
        acceptance_evidence=args.acceptance_evidence,
        opendrive_path=args.opendrive,
        timeout_sec=args.timeout_sec,
        runtime_route_distance_m=args.runtime_route_distance_m,
        physics_smoke=args.physics_smoke,
        multimodal_sensor_required=args.multimodal_sensor_required,
        output=str(output.with_name("closed_loop_report.json")),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")

    result = run_basic_agent(plan) if args.execute else {"status": "planned", "plan": str(output)}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["status"] in {"planned", "completed", "ego_closed_loop", "interactive_closed_loop"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
