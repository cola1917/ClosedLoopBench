import unittest


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
    def __init__(self, yaw=0.0):
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
    x = 3.0
    y = 4.0
    z = 0.0


class FakeVehicle:
    def __init__(self, events):
        self.events = events
        self.controls = []

    def get_transform(self):
        return FakeTransform(FakeLocation(1.0, 2.0, 0.0), FakeRotation(5.0))

    def get_velocity(self):
        return FakeVelocity()

    def apply_control(self, control):
        self.events.append("vehicle.apply_control")
        self.controls.append(control)

    def destroy(self):
        self.events.append("vehicle.destroy")


class FakeMap:
    name = "Town04"


class FakeWorld:
    def __init__(self, events):
        self.events = events
        self.settings = FakeSettings(synchronous_mode=False, fixed_delta_seconds=None)
        self.vehicle = FakeVehicle(events)

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
        return FakeMap()

    def get_blueprint_library(self):
        self.events.append("world.get_blueprint_library")
        return FakeBlueprintLibrary()

    def spawn_actor(self, blueprint, transform):
        self.events.append("world.spawn_actor")
        self.spawn_blueprint = blueprint
        self.spawn_transform = transform
        return self.vehicle

    def tick(self):
        self.events.append("world.tick")
        return 1


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


class FakeCarlaModule:
    def __init__(self, events):
        self.events = events
        self.Location = FakeLocation
        self.Rotation = FakeRotation
        self.Transform = FakeTransform

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
        return build_basic_agent_plan(run_config, max_ticks=2)

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
        self.assertIn("world.spawn_actor", events)
        self.assertIn("agent.set_destination", events)
        self.assertEqual(events.count("world.tick"), 2)
        self.assertEqual(events.count("agent.run_step"), 2)
        self.assertEqual(events.count("vehicle.apply_control"), 2)
        self.assertTrue(events[-2].startswith("world.apply_settings.sync=False"))
        self.assertEqual(events[-1], "vehicle.destroy")

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


if __name__ == "__main__":
    unittest.main()
