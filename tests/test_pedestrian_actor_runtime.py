import math
import unittest


class Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Rotation:
    def __init__(self, yaw=0.0):
        self.yaw = yaw


class Transform:
    def __init__(self, location=None, rotation=None):
        self.location = location or Vector()
        self.rotation = rotation or Rotation()


class Blueprint:
    def __init__(self):
        self.attributes = {}

    def set_attribute(self, name, value):
        self.attributes[name] = value


class BlueprintLibrary:
    def __init__(self):
        self.patterns = []

    def filter(self, pattern):
        self.patterns.append(pattern)
        return [Blueprint()]


class Walker:
    def __init__(self, yaw=0.0):
        self.controls = []
        self.yaw = yaw

    def get_transform(self):
        return Transform(Vector(1.0, 2.0, 0.0), Rotation(self.yaw))

    def get_velocity(self):
        return Vector(0.0, 0.0, 0.0)

    def apply_control(self, control):
        self.controls.append(control)


class World:
    def __init__(self):
        self.library = BlueprintLibrary()
        self.walker = Walker()
        self.spawn_transform = None

    def get_blueprint_library(self):
        return self.library

    def try_spawn_actor(self, _blueprint, transform):
        self.spawn_transform = transform
        return self.walker


class Carla:
    Location = Vector
    Vector3D = Vector
    Rotation = Rotation
    Transform = Transform

    class WalkerControl:
        def __init__(self, **kwargs):
            for key, value in kwargs.items():
                setattr(self, key, value)


class PedestrianActorRuntimeTests(unittest.TestCase):
    def test_behavior_plugin_can_change_vehicle_track_target(self):
        from runners.run_carla_basic_agent import _apply_scripted_actor_controls

        vehicle = Walker()
        actor = {
            "actor_id": "vehicle-1",
            "type": "vehicle",
            "closed_loop_level": "scripted",
        }
        evidence = _apply_scripted_actor_controls(
            Carla,
            [actor],
            {"vehicle-1": vehicle},
            {
                "vehicle-1": {
                    "desired_speed_mps": 2.0,
                    "brake": False,
                    "target_point": {"x": 1.0, "y": 12.0},
                }
            },
        )

        self.assertEqual(evidence, {"vehicle-1": "scripted_vehicle_control"})
        self.assertGreater(vehicle.controls[0].steer, 0.0)

    def test_behavior_plugin_loader_requires_callable_module_function(self):
        from runners.run_carla_basic_agent import _load_actor_behavior_planner

        planner = _load_actor_behavior_planner(
            "actors.reactive_actor:plan_reactive_actor_control"
        )
        self.assertTrue(callable(planner))
        with self.assertRaisesRegex(ValueError, "module:function"):
            _load_actor_behavior_planner("invalid")
        with self.assertRaisesRegex(ValueError, "not callable"):
            _load_actor_behavior_planner("actors.reactive_actor:SAFE_DEFAULT_SPEED_MPS")

    def test_replay_pedestrian_is_physical_but_not_labeled_interactive(self):
        from runners.run_carla_basic_agent import build_basic_agent_plan, run_basic_agent
        from tests.test_basic_agent_runtime_loop import FakeBasicAgent, FakeCarlaModule

        events = []
        carla = FakeCarlaModule(events)
        carla.Vector3D = Vector
        carla.WalkerControl = Carla.WalkerControl
        plan = build_basic_agent_plan(
            {
                "schema_version": "carla_run_config.mvp.v0",
                "scenario_id": "pedestrian-replay-runtime",
                "carla": {"map": "Town04", "fixed_delta_seconds": 0.05},
                "ego": {
                    "initial_state": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
                    "reference_trajectory": [
                        {"x": 10.0, "y": 2.0, "z": 0.0, "yaw": 0.0}
                    ],
                },
                "actors": [
                    {
                        "actor_id": "ped-replay",
                        "type": "pedestrian",
                        "closed_loop_level": "replay",
                        "initial_state": {"x": 5.0, "y": 2.0, "z": 0.2, "yaw": 90.0, "speed_mps": 1.0},
                        "reference_trajectory": [
                            {"t_sec": 0.05, "x": 5.0, "y": 3.0, "z": 0.2, "yaw": 90.0, "speed_mps": 1.0},
                            {"t_sec": 0.10, "x": 5.0, "y": 4.0, "z": 0.2, "yaw": 90.0, "speed_mps": 1.0},
                        ],
                    }
                ],
                "metrics": ["collision", "route_progress", "min_ttc"],
            },
            max_ticks=2,
        )
        plan["artifacts"]["closed_loop_report"] = None
        result = run_basic_agent(plan, carla_module=carla, agent_module=FakeBasicAgent)

        self.assertEqual(result["status"], "ego_closed_loop")
        self.assertIn("world.try_spawn_actor.role=ped-replay.x=5.0", events)
        self.assertEqual(events.count("actor.ped-replay.apply_control"), 2)
        self.assertEqual(
            result["report"]["runtime"]["actor_control_evidence"],
            {"ped-replay": "trajectory_replay_walker_control"},
        )
        self.assertIn(
            "ped-replay", result["report"]["metrics"][0]["actor_distances_m"]
        )
        self.assertIsNotNone(result["report"]["metrics"][0]["min_ttc"])

    def test_full_runner_spawns_controls_and_traces_pedestrian(self):
        from runners.run_carla_basic_agent import build_basic_agent_plan, run_basic_agent
        from tests.test_basic_agent_runtime_loop import FakeBasicAgent, FakeCarlaModule

        events = []
        carla = FakeCarlaModule(events)
        carla.Vector3D = Vector
        carla.WalkerControl = Carla.WalkerControl
        run_config = {
            "schema_version": "carla_run_config.mvp.v0",
            "scenario_id": "pedestrian-runtime",
            "carla": {"map": "Town04", "fixed_delta_seconds": 0.05},
            "ego": {
                "initial_state": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
                "reference_trajectory": [
                    {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
                    {"x": 10.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
                ],
            },
            "actors": [
                {
                    "actor_id": "ped-1",
                    "type": "pedestrian",
                    "closed_loop_level": "scripted",
                    "style": "normal",
                    "initial_state": {
                        "x": 3.0,
                        "y": 4.0,
                        "z": 0.2,
                        "yaw": 90.0,
                        "speed_mps": 1.0,
                    },
                    "reference_trajectory": [
                        {"t_sec": 0.05, "x": 3.0, "y": 4.0, "z": 0.2, "yaw": 90.0, "speed_mps": 1.0},
                        {"t_sec": 0.10, "x": 3.0, "y": 5.0, "z": 0.2, "yaw": 90.0, "speed_mps": 1.0},
                    ],
                }
            ],
            "metrics": ["collision", "route_progress", "min_ttc"],
        }
        plan = build_basic_agent_plan(
            run_config, max_ticks=2, snap_to_map=True
        )
        plan["artifacts"]["closed_loop_report"] = None
        result = run_basic_agent(
            plan, carla_module=carla, agent_module=FakeBasicAgent
        )

        self.assertEqual(result["status"], "interactive_closed_loop")
        self.assertIn("world.try_spawn_actor.role=ped-1.x=3.0", events)
        self.assertNotIn("world.try_spawn_actor.role=ped-1.x=103.0", events)
        self.assertEqual(events.count("actor.ped-1.apply_control"), 2)
        self.assertEqual(
            result["report"]["runtime"]["actor_control_evidence"],
            {"ped-1": "scripted_walker_control"},
        )
        actor_state = result["frame_trace"][0]["actor_states"]["ped-1"]
        self.assertEqual(actor_state["actor_type"], "pedestrian")
        self.assertIsNotNone(actor_state["reference_pose"])
        self.assertIsInstance(actor_state["reference_error_m"], float)
        self.assertIn("pose", actor_state)
        decision = result["report"]["metrics"][0]["actor_decisions"]["ped-1"]
        self.assertEqual(decision["motion_constraint"], "source_reference_corridor")
        self.assertEqual(decision["allowed_actions"], ["speed", "pause", "yield", "abort"])

    def test_spawns_pedestrian_blueprint_at_declared_pose(self):
        from runners.run_carla_basic_agent import _spawn_actor_vehicle

        world = World()
        actor = {
            "actor_id": "ped-1",
            "type": "pedestrian",
            "closed_loop_level": "scripted",
            "initial_state": {"x": 4.0, "y": 5.0, "z": 0.3, "yaw": 90.0},
        }
        spawned = _spawn_actor_vehicle(Carla, world, actor, "ped-1")

        self.assertIs(spawned, world.walker)
        self.assertEqual(world.library.patterns, ["walker.pedestrian.*"])
        self.assertEqual(world.spawn_transform.location.x, 4.0)
        self.assertEqual(world.spawn_transform.rotation.yaw, 90.0)

    def test_scripted_walker_follows_reference_and_emits_evidence(self):
        from runners.run_carla_basic_agent import _apply_scripted_actor_controls

        walker = Walker()
        actor = {
            "actor_id": "ped-1",
            "actor_type": "pedestrian",
            "closed_loop_level": "scripted",
            "reference_trajectory": [
                {"x": 1.0, "y": 2.0, "speed_mps": 1.2},
                {"x": 1.0, "y": 5.0, "speed_mps": 1.2},
            ],
        }
        evidence = _apply_scripted_actor_controls(
            Carla,
            [actor],
            {"ped-1": walker},
            {"ped-1": {"desired_speed_mps": 1.2, "should_abort": False}},
        )

        control = walker.controls[0]
        self.assertEqual(evidence, {"ped-1": "scripted_walker_control"})
        self.assertAlmostEqual(control.speed, 1.2)
        self.assertAlmostEqual(control.direction.x, 0.0)
        self.assertAlmostEqual(control.direction.y, 1.0)
        self.assertFalse(control.jump)

    def test_pedestrian_plugin_cannot_send_walker_off_source_corridor(self):
        from runners.run_carla_basic_agent import _apply_scripted_actor_controls

        walker = Walker()
        actor = {
            "actor_id": "ped-1",
            "actor_type": "pedestrian",
            "closed_loop_level": "scripted",
            "reference_trajectory": [
                {"x": 1.0, "y": 2.0, "speed_mps": 1.0},
                {"x": 1.0, "y": 5.0, "speed_mps": 1.0},
            ],
        }
        _apply_scripted_actor_controls(
            Carla,
            [actor],
            {"ped-1": walker},
            {
                "ped-1": {
                    "desired_speed_mps": 1.0,
                    "should_abort": False,
                    "target_point": {"x": 100.0, "y": 2.0},
                }
            },
        )

        self.assertAlmostEqual(walker.controls[0].direction.x, 0.0)
        self.assertAlmostEqual(walker.controls[0].direction.y, 1.0)

    def test_abort_stops_walker_and_tm_mode_is_rejected(self):
        from runners.run_carla_basic_agent import (
            _apply_scripted_actor_controls,
            _spawn_actor_vehicle,
        )

        walker = Walker(yaw=180.0)
        actor = {
            "actor_id": "ped-1",
            "type": "person",
            "closed_loop_level": "scripted",
            "initial_state": {"x": 1.0, "y": 2.0, "yaw": 180.0},
        }
        _apply_scripted_actor_controls(
            Carla,
            [actor],
            {"ped-1": walker},
            {"ped-1": {"desired_speed_mps": 1.0, "should_abort": True}},
        )
        self.assertEqual(walker.controls[0].speed, 0.0)
        self.assertAlmostEqual(walker.controls[0].direction.x, -1.0)
        self.assertAlmostEqual(walker.controls[0].direction.y, 0.0, places=7)

        actor["closed_loop_level"] = "traffic_manager_reactive"
        with self.assertRaisesRegex(ValueError, "TrafficManager only controls vehicles"):
            _spawn_actor_vehicle(Carla, World(), actor, "ped-1")

    def test_runtime_trace_keeps_reference_pose_separate(self):
        from runners.run_carla_basic_agent import _reference_pose_at_time

        actor = {
            "reference_trajectory": [
                {"t_sec": 0.0, "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
                {"t_sec": 1.0, "x": 3.0, "y": 4.0, "z": 0.0, "yaw": 10.0},
            ]
        }
        self.assertEqual(_reference_pose_at_time(actor, 0.8)["x"], 3.0)


if __name__ == "__main__":
    unittest.main()
