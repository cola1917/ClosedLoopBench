import unittest
import json
import tempfile
from pathlib import Path


class FakeSettings:
    def __init__(self, synchronous_mode=False, fixed_delta_seconds=None):
        self.synchronous_mode = synchronous_mode
        self.fixed_delta_seconds = fixed_delta_seconds


class FakeLocation:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeRotation:
    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


class FakeTransform:
    def __init__(self, location, rotation):
        self.location = location
        self.rotation = rotation


class FakeBlueprint:
    def __init__(self):
        self.attributes = {}

    def set_attribute(self, key, value):
        self.attributes[key] = value


class FakeBlueprintLibrary:
    def filter(self, pattern):
        return [FakeBlueprint()]


class FakeControl:
    throttle = 0.3
    brake = 0.0
    steer = 0.1


class FakeVelocity:
    def __init__(self, x=3.0, y=4.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeVehicle:
    def __init__(self, events, label="vehicle", transforms=None, velocity=None):
        self.events = events
        self.label = label
        self.controls = []
        self.transforms = transforms or [
            FakeTransform(FakeLocation(1.0, 2.0, 0.0), FakeRotation(yaw=5.0)),
        ]
        self.transform_reads = 0
        self.velocity = velocity or FakeVelocity()

    def get_transform(self):
        transform = self.transforms[min(self.transform_reads, len(self.transforms) - 1)]
        self.transform_reads += 1
        return transform

    def get_velocity(self):
        return self.velocity

    def apply_control(self, control):
        self.events.append("{}.apply_control".format(self.label))
        self.controls.append(control)

    def set_autopilot(self, enabled, tm_port=None):
        self.events.append("{}.set_autopilot.{}.{}".format(self.label, enabled, tm_port))

    def destroy(self):
        self.events.append("{}.destroy".format(self.label))


class FakeSpectator:
    def __init__(self, events):
        self.events = events
        self.transforms = []

    def set_transform(self, transform):
        self.events.append("spectator.set_transform")
        self.transforms.append(transform)


class FakeMap:
    name = "Town04"

    def __init__(self, spawn_points=None):
        self._spawn_points = spawn_points or []

    def get_spawn_points(self):
        return list(self._spawn_points)

    def get_waypoint(self, location, project_to_road=True, lane_type=None):
        return type(
            "FakeWaypoint",
            (),
            {
                "transform": FakeTransform(
                    FakeLocation(location.x + 100.0, location.y + 100.0, 0.0),
                    FakeRotation(yaw=45.0),
                )
            },
        )()


class FakeTrafficManager:
    def __init__(self, events):
        self.events = events

    def set_synchronous_mode(self, enabled):
        self.events.append("tm.set_synchronous_mode.{}".format(enabled))

    def distance_to_leading_vehicle(self, vehicle, distance):
        self.events.append("tm.distance_to_leading_vehicle.{}.{}".format(vehicle.label, distance))

    def auto_lane_change(self, vehicle, enabled):
        self.events.append("tm.auto_lane_change.{}.{}".format(vehicle.label, enabled))

    def vehicle_percentage_speed_difference(self, vehicle, percentage):
        self.events.append("tm.vehicle_percentage_speed_difference.{}.{}".format(vehicle.label, percentage))


class FakeWorld:
    def __init__(self, events):
        self.events = events
        self.settings = FakeSettings(synchronous_mode=False, fixed_delta_seconds=None)
        self.vehicle = FakeVehicle(events)
        self.actor_vehicles = {}
        self.spectator = FakeSpectator(events)
        self.spawn_points = [
            FakeTransform(FakeLocation(50.0, 0.0, 0.0), FakeRotation(yaw=0.0)),
        ]

    def get_settings(self):
        self.events.append("world.get_settings")
        return self.settings

    def apply_settings(self, settings):
        self.events.append(
            "world.apply_settings.sync={}.dt={}".format(
                settings.synchronous_mode,
                settings.fixed_delta_seconds,
            )
        )
        self.settings = settings

    def get_map(self):
        self.events.append("world.get_map")
        return FakeMap(self.spawn_points)

    def get_spectator(self):
        self.events.append("world.get_spectator")
        return self.spectator

    def get_blueprint_library(self):
        self.events.append("world.get_blueprint_library")
        return FakeBlueprintLibrary()

    def spawn_actor(self, blueprint, transform):
        self.events.append("world.spawn_actor")
        self.spawn_blueprint = blueprint
        self.spawn_transform = transform
        return self._vehicle_for_blueprint(blueprint)

    def try_spawn_actor(self, blueprint, transform):
        self.events.append(
            "world.try_spawn_actor.x={}".format(getattr(transform.location, "x", None))
        )
        role_name = blueprint.attributes.get("role_name", "vehicle")
        self.events.append(
            "world.try_spawn_actor.role={}.x={}".format(
                role_name,
                getattr(transform.location, "x", None),
            )
        )
        self.spawn_blueprint = blueprint
        self.spawn_transform = transform
        return self._vehicle_for_blueprint(blueprint)

    def tick(self):
        self.events.append("world.tick")
        return 1

    def _vehicle_for_blueprint(self, blueprint):
        role_name = blueprint.attributes.get("role_name", "vehicle")
        if role_name == "ego_vehicle":
            return self.vehicle
        if role_name not in self.actor_vehicles:
            self.actor_vehicles[role_name] = FakeVehicle(
                self.events,
                label="actor.{}".format(role_name),
                transforms=[
                    FakeTransform(FakeLocation(6.0, 2.0, 0.0), FakeRotation(yaw=0.0)),
                    FakeTransform(FakeLocation(8.0, 2.0, 0.0), FakeRotation(yaw=0.0)),
                ],
                velocity=FakeVelocity(x=1.0, y=0.0, z=0.0),
            )
        return self.actor_vehicles[role_name]


class FakeClient:
    def __init__(self, events, host, port):
        self.events = events
        self.host = host
        self.port = port
        self.world = FakeWorld(events)
        events.append("client.init")

    def set_timeout(self, timeout):
        self.events.append("client.set_timeout")
        self.timeout = timeout

    def load_world(self, map_name):
        self.events.append("client.load_world.{}".format(map_name))
        return self.world

    def get_world(self):
        self.events.append("client.get_world")
        return self.world

    def get_trafficmanager(self, port):
        self.events.append("client.get_trafficmanager.{}".format(port))
        return FakeTrafficManager(self.events)


class FakeCarlaModule:
    def __init__(self, events):
        self.events = events
        self.Location = FakeLocation
        self.Rotation = FakeRotation
        self.Transform = FakeTransform
        self.LaneType = type("LaneType", (), {"Driving": "Driving"})

    def Client(self, host, port):
        return FakeClient(self.events, host, port)


class FakeBasicAgent:
    def __init__(self, vehicle, target_speed=20.0):
        vehicle.events.append("agent.init")
        self.vehicle = vehicle
        self.target_speed = target_speed

    def set_destination(self, destination):
        self.vehicle.events.append("agent.set_destination")
        self.destination = destination

    def done(self):
        return False

    def run_step(self):
        self.vehicle.events.append("agent.run_step")
        return FakeControl()


class BasicAgentRuntimeLoopTests(unittest.TestCase):
    def test_carla_boundary_reflects_scene_pose_and_round_trips(self):
        from runners.run_carla_basic_agent import _carla_transform, _vehicle_pose

        events = []
        carla = FakeCarlaModule(events)
        scene_pose = {
            "x": 3.0,
            "y": 4.0,
            "z": 1.0,
            "roll": 5.0,
            "pitch": 6.0,
            "yaw": 30.0,
        }
        transform = _carla_transform(carla, scene_pose)

        self.assertEqual(transform.location.x, 3.0)
        self.assertEqual(transform.location.y, -4.0)
        self.assertEqual(transform.rotation.roll, -5.0)
        self.assertEqual(transform.rotation.pitch, 6.0)
        self.assertEqual(transform.rotation.yaw, -30.0)

        vehicle = FakeVehicle(events, transforms=[transform])
        self.assertEqual(_vehicle_pose(vehicle), scene_pose)

    def _plan(self):
        from runners.run_carla_basic_agent import build_basic_agent_plan

        run_config = {
            "schema_version": "carla_run_config.mvp.v0",
            "scenario_id": "scene-basic-agent-runtime",
            "carla": {"map": "Town04", "fixed_delta_seconds": 0.05},
            "ego": {
                "initial_state": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 5.0, "speed_mps": 3.0},
                "reference_trajectory": [
                    {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 5.0, "speed_mps": 3.0},
                    {"x": 10.0, "y": 2.0, "z": 0.0, "yaw": 5.0, "speed_mps": 6.0},
                ],
            },
            "actors": [],
            "metrics": ["collision", "route_progress"],
        }
        plan = build_basic_agent_plan(run_config, max_ticks=2)
        plan["artifacts"]["closed_loop_report"] = None
        return plan

    def _interactive_plan(self):
        from runners.run_carla_basic_agent import build_basic_agent_plan

        run_config = {
            "schema_version": "carla_run_config.mvp.v0",
            "scenario_id": "scene-basic-agent-interactive-runtime",
            "carla": {"map": "Town04", "fixed_delta_seconds": 0.05},
            "ego": {
                "initial_state": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 5.0, "speed_mps": 3.0},
                "reference_trajectory": [
                    {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 5.0, "speed_mps": 3.0},
                    {"x": 10.0, "y": 2.0, "z": 0.0, "yaw": 5.0, "speed_mps": 6.0},
                ],
            },
            "actors": [
                {
                    "actor_id": "trigger",
                    "policy": "scripted_trigger",
                    "closed_loop_level": "scripted",
                    "style": "defensive",
                    "style_profile": {"min_gap_m": 5.0},
                    "initial_state": {"x": 3.0, "y": 2.0, "z": 0.0, "speed_mps": 1.0},
                }
            ],
            "metrics": ["collision", "route_progress", "min_ttc"],
        }
        plan = build_basic_agent_plan(run_config, max_ticks=2)
        plan["artifacts"]["closed_loop_report"] = None
        return plan

    def test_real_loop_skeleton_accepts_fake_carla_and_agent(self):
        from runners.run_carla_basic_agent import run_basic_agent

        events = []
        result = run_basic_agent(
            self._plan(),
            carla_module=FakeCarlaModule(events),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "ego_closed_loop")
        self.assertEqual(result["scenario_id"], "scene-basic-agent-runtime")
        self.assertEqual(result["summary"]["ticks"], 2)
        self.assertEqual(result["report"]["status"], "ego_closed_loop")
        self.assertEqual(len(result["report"]["metrics"]), 2)
        self.assertIn("world.apply_settings.sync=True.dt=0.05", events)
        self.assertIn("world.try_spawn_actor.x=1.0", events)
        self.assertIn("agent.set_destination", events)
        self.assertEqual(events.count("world.tick"), 2)
        self.assertEqual(events.count("agent.run_step"), 2)
        self.assertEqual(events.count("vehicle.apply_control"), 2)
        self.assertTrue(events[-2].startswith("world.apply_settings.sync=False"))
        self.assertEqual(events[-1], "vehicle.destroy")

    def test_acceptance_mode_fails_closed_without_real_collision_sensor(self):
        from runners.run_carla_basic_agent import run_basic_agent

        plan = self._plan()
        plan["runtime"]["acceptance_evidence"] = True
        plan["limits"]["max_ticks"] = 1
        result = run_basic_agent(
            plan,
            carla_module=FakeCarlaModule([]),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "failed")
        self.assertIn("collision sensor", result["detail"])
        self.assertTrue(result["cleanup_succeeded"])

    def test_runtime_writes_frame_metrics_and_cleanup_evidence(self):
        from runners.run_carla_basic_agent import run_basic_agent

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = self._plan()
            plan["artifacts"] = {
                "closed_loop_report": str(root / "closed_loop_report.json"),
                "frame_trace": str(root / "frame_trace.jsonl"),
                "metrics_trace": str(root / "metrics_trace.jsonl"),
                "cleanup_audit": str(root / "cleanup_audit.json"),
            }
            result = run_basic_agent(
                plan,
                carla_module=FakeCarlaModule([]),
                agent_module=FakeBasicAgent,
            )

            self.assertEqual(len((root / "frame_trace.jsonl").read_text().splitlines()), 2)
            self.assertEqual(len((root / "metrics_trace.jsonl").read_text().splitlines()), 2)
            audit = json.loads((root / "cleanup_audit.json").read_text())
            self.assertTrue(audit["succeeded"])
            self.assertTrue(result["cleanup_succeeded"])

    def test_real_loop_records_reactive_actor_decisions_for_interactive_actors(self):
        from runners.run_carla_basic_agent import run_basic_agent

        events = []
        result = run_basic_agent(
            self._interactive_plan(),
            carla_module=FakeCarlaModule(events),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "interactive_closed_loop")
        self.assertEqual(result["report"]["status"], "interactive_closed_loop")
        self.assertIn("world.try_spawn_actor.role=trigger.x=3.0", events)
        first_tick = result["report"]["metrics"][0]
        second_tick = result["report"]["metrics"][1]
        self.assertIn("trigger", first_tick["actor_decisions"])
        self.assertEqual(first_tick["actor_distances_m"]["trigger"], 5.0)
        self.assertEqual(second_tick["actor_distances_m"]["trigger"], 7.0)
        self.assertAlmostEqual(first_tick["min_ttc"], 1.25)
        self.assertTrue(first_tick["actor_decisions"]["trigger"]["should_yield"])
        self.assertEqual(
            first_tick["actor_control_evidence"]["trigger"],
            "scripted_vehicle_control",
        )
        self.assertEqual(events.count("actor.trigger.apply_control"), 2)
        self.assertTrue(events[-2].startswith("actor.trigger.destroy"))
        self.assertEqual(events[-1], "vehicle.destroy")

    def test_real_loop_does_not_spawn_replay_actors(self):
        from runners.run_carla_basic_agent import run_basic_agent

        plan = self._interactive_plan()
        plan["actors"][0]["closed_loop_level"] = "replay"
        plan["actors"][0].pop("closed_loop", None)
        events = []
        result = run_basic_agent(
            plan,
            carla_module=FakeCarlaModule(events),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "ego_closed_loop")
        self.assertNotIn("world.try_spawn_actor.role=trigger.x=3.0", events)
        self.assertEqual(result["report"]["metrics"][0]["actor_distances_m"], {})
        self.assertNotIn("actor.trigger.destroy", events)

    def test_real_loop_spawns_ego_responsive_actor(self):
        from runners.run_carla_basic_agent import run_basic_agent

        plan = self._interactive_plan()
        plan["actors"][0]["closed_loop_level"] = "replay"
        plan["actors"][0]["closed_loop"] = {"ego_responsive": True}
        events = []
        result = run_basic_agent(
            plan,
            carla_module=FakeCarlaModule(events),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "interactive_closed_loop")
        self.assertIn("world.try_spawn_actor.role=trigger.x=3.0", events)
        self.assertEqual(result["report"]["metrics"][0]["actor_distances_m"]["trigger"], 5.0)
        self.assertIn("actor.trigger.destroy", events)

    def test_snap_to_map_projects_ego_and_interactive_actor_spawns_before_runtime(self):
        from runners.run_carla_basic_agent import run_basic_agent

        plan = self._interactive_plan()
        plan["runtime"]["snap_to_map"] = True
        events = []
        result = run_basic_agent(
            plan,
            carla_module=FakeCarlaModule(events),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "interactive_closed_loop")
        self.assertIn("world.try_spawn_actor.role=ego_vehicle.x=101.0", events)
        self.assertIn("world.try_spawn_actor.role=trigger.x=103.0", events)
        self.assertEqual(plan["ego"]["spawn"]["yaw"], -45.0)
        self.assertEqual(plan["ego"]["route"][0]["x"], 101.0)
        self.assertEqual(plan["ego"]["route"][-1]["x"], 110.0)
        self.assertEqual(plan["actors"][0]["initial_state"]["yaw"], -45.0)

    def test_actor_autopilot_binds_interactive_actor_to_traffic_manager(self):
        from runners.run_carla_basic_agent import run_basic_agent

        plan = self._interactive_plan()
        plan["actors"][0]["closed_loop_level"] = "traffic_manager_reactive"
        plan["runtime"]["actor_autopilot"] = True
        plan["runtime"]["traffic_manager_port"] = 8100
        events = []
        result = run_basic_agent(
            plan,
            carla_module=FakeCarlaModule(events),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "interactive_closed_loop")
        self.assertIn("client.get_trafficmanager.8100", events)
        self.assertIn("tm.set_synchronous_mode.True", events)
        self.assertIn("actor.trigger.set_autopilot.True.8100", events)
        self.assertIn("tm.distance_to_leading_vehicle.actor.trigger.5.0", events)
        self.assertIn("tm.auto_lane_change.actor.trigger.True", events)
        self.assertIn("tm.vehicle_percentage_speed_difference.actor.trigger.0.0", events)
        self.assertEqual(
            result["report"]["runtime"]["actor_control_evidence"],
            {"trigger": "traffic_manager"},
        )

    def test_declared_tm_actor_without_autopilot_is_not_reported_as_interactive(self):
        from runners.run_carla_basic_agent import run_basic_agent

        plan = self._interactive_plan()
        plan["actors"][0]["closed_loop_level"] = "traffic_manager_reactive"
        plan["runtime"]["actor_autopilot"] = False
        result = run_basic_agent(
            plan,
            carla_module=FakeCarlaModule([]),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "ego_closed_loop")
        self.assertEqual(result["report"]["runtime"]["actor_control_evidence"], {})

    def test_interactive_actor_spawn_collision_falls_back_to_map_spawn_point(self):
        from runners.run_carla_basic_agent import run_basic_agent

        class ActorCollisionAtPlannedPoseWorld(FakeWorld):
            def __init__(self, events):
                super().__init__(events)
                self.actor_try_count = 0

            def try_spawn_actor(self, blueprint, transform):
                role_name = blueprint.attributes.get("role_name", "vehicle")
                self.events.append(
                    "world.try_spawn_actor.role={}.x={}".format(
                        role_name,
                        getattr(transform.location, "x", None),
                    )
                )
                if role_name != "ego_vehicle":
                    self.actor_try_count += 1
                    if self.actor_try_count == 1:
                        return None
                return self._vehicle_for_blueprint(blueprint)

        class ActorCollisionFallbackClient(FakeClient):
            def __init__(self, events, host, port):
                self.events = events
                self.host = host
                self.port = port
                self.world = ActorCollisionAtPlannedPoseWorld(events)
                events.append("client.init")

        class ActorCollisionFallbackCarla(FakeCarlaModule):
            def Client(self, host, port):
                return ActorCollisionFallbackClient(self.events, host, port)

        events = []
        result = run_basic_agent(
            self._interactive_plan(),
            carla_module=ActorCollisionFallbackCarla(events),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "interactive_closed_loop")
        self.assertIn("world.try_spawn_actor.role=trigger.x=3.0", events)
        self.assertIn("world.try_spawn_actor.role=trigger.x=50.0", events)
        self.assertIn("actor.trigger.destroy", events)

    def test_bound_actor_spawn_collision_fails_without_random_fallback(self):
        from runners.run_carla_basic_agent import run_basic_agent

        class BoundActorCollisionWorld(FakeWorld):
            def try_spawn_actor(self, blueprint, transform):
                role_name = blueprint.attributes.get("role_name", "vehicle")
                self.events.append(
                    "world.try_spawn_actor.role={}.x={}".format(
                        role_name,
                        getattr(transform.location, "x", None),
                    )
                )
                if role_name != "ego_vehicle":
                    return None
                return self._vehicle_for_blueprint(blueprint)

        class BoundActorCollisionClient(FakeClient):
            def __init__(self, events, host, port):
                self.events = events
                self.host = host
                self.port = port
                self.world = BoundActorCollisionWorld(events)

        class BoundActorCollisionCarla(FakeCarlaModule):
            def Client(self, host, port):
                return BoundActorCollisionClient(self.events, host, port)

        plan = self._interactive_plan()
        plan["actors"][0]["binding"] = {
            "schema_version": "actor_runtime_binding.v1"
        }
        events = []
        result = run_basic_agent(
            plan,
            carla_module=BoundActorCollisionCarla(events),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "failed")
        self.assertIn("map fallback would invalidate actor identity", result["detail"])
        self.assertNotIn("world.try_spawn_actor.role=trigger.x=50.0", events)

    def test_scripted_vehicle_uses_time_sampled_speed_and_next_reference_target(self):
        from runners.run_carla_basic_agent import _reactive_actor_tick

        captured = {}

        def planner(actor_state, ego_state, *, style, reference_speed_mps):
            captured["reference_speed_mps"] = reference_speed_mps
            return {
                "distance_m": ego_state["distance_m"],
                "desired_speed_mps": reference_speed_mps,
                "brake": False,
                "ttc_sec": float("inf"),
            }

        actor = {
            "actor_id": "trigger",
            "closed_loop_level": "scripted",
            "initial_state": {"x": 1.0, "y": 2.0, "speed_mps": 2.0},
            "reference_trajectory": [
                {"t_sec": 0.0, "x": 1.0, "y": 2.0, "speed_mps": 2.0},
                {"t_sec": 5.0, "x": 10.0, "y": 2.0, "speed_mps": 7.0},
            ],
        }
        _, decisions, _ = _reactive_actor_tick(
            [actor],
            ego_pose={"x": 20.0, "y": 2.0},
            ego_speed_mps=5.0,
            simulation_time_sec=5.0,
            actor_vehicles={
                "trigger": FakeVehicle(
                    [],
                    label="actor.trigger",
                    velocity=FakeVelocity(x=1.0, y=0.0, z=0.0),
                )
            },
            behavior_planner=planner,
        )

        self.assertEqual(captured["reference_speed_mps"], 7.0)
        self.assertEqual(decisions["trigger"]["target_point"]["x"], 10.0)

    def test_follow_ego_moves_spectator_each_tick(self):
        from runners.run_carla_basic_agent import run_basic_agent

        plan = self._plan()
        plan["visualization"]["follow_ego"] = True
        events = []
        result = run_basic_agent(
            plan,
            carla_module=FakeCarlaModule(events),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "ego_closed_loop")
        self.assertEqual(events.count("world.get_spectator"), 2)
        self.assertEqual(events.count("spectator.set_transform"), 2)

    def test_real_loop_reports_structured_failure_and_restores_settings(self):
        from runners.run_carla_basic_agent import run_basic_agent

        class FailingAgent(FakeBasicAgent):
            def run_step(self):
                self.vehicle.events.append("agent.run_step")
                raise RuntimeError("planner failed")

        events = []
        result = run_basic_agent(
            self._plan(),
            carla_module=FakeCarlaModule(events),
            agent_module=FailingAgent,
        )

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["reason"], "basic_agent_runtime_failed")
        self.assertIn("planner failed", result["detail"])
        self.assertTrue(events[-2].startswith("world.apply_settings.sync=False"))
        self.assertEqual(events[-1], "vehicle.destroy")

    def test_ros2_driver_factory_controls_ego_without_basic_agent_import(self):
        from runners.run_carla_basic_agent import run_basic_agent

        class FakeRosDriver:
            def __init__(self):
                self.calls = 0

            def done(self):
                return False

            def run_step(self):
                self.calls += 1
                return FakeControl()

            def diagnostics(self):
                return {"driver": "ros2_control", "control_count": self.calls, "fallback_count": 0}

            def close(self):
                pass

        plan = self._plan()
        plan["ego"]["driver"] = "ros2_control"
        result = run_basic_agent(
            plan,
            carla_module=FakeCarlaModule([]),
            driver_factory=lambda *_args: FakeRosDriver(),
        )

        self.assertEqual(result["status"], "ego_closed_loop")
        self.assertEqual(
            result["report"]["runtime"]["ego_driver_diagnostics"]["control_count"],
            2,
        )

    def test_spawn_collision_falls_back_to_map_spawn_point(self):
        from runners.run_carla_basic_agent import run_basic_agent

        class CollisionAtPlannedPoseWorld(FakeWorld):
            def __init__(self, events):
                super().__init__(events)
                self.try_count = 0

            def try_spawn_actor(self, blueprint, transform):
                self.try_count += 1
                self.events.append(
                    "world.try_spawn_actor.x={}".format(getattr(transform.location, "x", None))
                )
                if self.try_count == 1:
                    return None
                return self.vehicle

        class CollisionFallbackClient(FakeClient):
            def __init__(self, events, host, port):
                self.events = events
                self.host = host
                self.port = port
                self.world = CollisionAtPlannedPoseWorld(events)
                events.append("client.init")

        class CollisionFallbackCarla(FakeCarlaModule):
            def Client(self, host, port):
                return CollisionFallbackClient(self.events, host, port)

        events = []
        result = run_basic_agent(
            self._plan(),
            carla_module=CollisionFallbackCarla(events),
            agent_module=FakeBasicAgent,
        )

        self.assertEqual(result["status"], "ego_closed_loop")
        self.assertIn("world.try_spawn_actor.x=1.0", events)
        self.assertIn("world.try_spawn_actor.x=50.0", events)


if __name__ == "__main__":
    unittest.main()
