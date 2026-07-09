import unittest


class FakeActorHandle:
    def __init__(self, actor_id):
        self.id = actor_id
        self.calls = []

    def set_autopilot(self, enabled, tm_port=None):
        self.calls.append(("set_autopilot", enabled, tm_port))


class FakeBlueprintLibrary:
    def __init__(self):
        self.calls = []

    def filter(self, pattern):
        self.calls.append(("filter", pattern))
        return ["vehicle.mock"]


class FakeWorld:
    def __init__(self):
        self.blueprints = FakeBlueprintLibrary()
        self.spawned = []

    def get_blueprint_library(self):
        return self.blueprints

    def spawn_actor(self, blueprint, transform):
        actor = FakeActorHandle(len(self.spawned) + 100)
        self.spawned.append((blueprint, transform, actor))
        return actor


class FakeTrafficManager:
    def __init__(self):
        self.calls = []

    def distance_to_leading_vehicle(self, actor, distance):
        self.calls.append(("distance_to_leading_vehicle", actor.id, distance))

    def vehicle_percentage_speed_difference(self, actor, percentage):
        self.calls.append(("vehicle_percentage_speed_difference", actor.id, percentage))

    def auto_lane_change(self, actor, enabled):
        self.calls.append(("auto_lane_change", actor.id, enabled))


class TrafficManagerExecutorTests(unittest.TestCase):
    def _plan_set(self):
        return {
            "schema_version": "actor_runtime_plan.mvp.v0",
            "scenario_id": "tm-test",
            "actors": [
                {
                    "actor_id": "background-replay",
                    "runtime_mode": "replay",
                    "initial_state": {"x": 1.0, "y": 2.0, "yaw": 0.0},
                    "controller": {"type": "trajectory_replay"},
                },
                {
                    "actor_id": "reactive-actor",
                    "runtime_mode": "traffic_manager",
                    "initial_state": {"x": 10.0, "y": 3.0, "z": 0.2, "yaw": 90.0, "speed_mps": 7.0},
                    "style": "defensive",
                    "controller": {
                        "type": "carla_traffic_manager",
                        "parameters": {
                            "min_gap_m": 7.5,
                            "desired_time_headway_sec": 2.2,
                            "lane_change_gap_acceptance_m": 12.0,
                        },
                    },
                },
            ],
        }

    def test_execute_traffic_manager_plan_spawns_only_tm_actors_and_sets_autopilot(self):
        from actors.traffic_manager_executor import execute_traffic_manager_plan_set

        world = FakeWorld()
        tm = FakeTrafficManager()

        result = execute_traffic_manager_plan_set(self._plan_set(), world=world, traffic_manager=tm, tm_port=8100)

        self.assertEqual(result["scenario_id"], "tm-test")
        self.assertEqual(result["summary"]["spawned_count"], 1)
        self.assertEqual(result["summary"]["fallback_count"], 1)
        self.assertEqual(world.blueprints.calls, [("filter", "vehicle.*")])
        self.assertEqual(len(world.spawned), 1)
        spawned_actor = world.spawned[0][2]
        self.assertEqual(spawned_actor.calls, [("set_autopilot", True, 8100)])
        self.assertEqual(result["actors"][1]["runtime_status"], "traffic_manager_bound")

    def test_executor_applies_distance_speed_and_lane_change_style_parameters(self):
        from actors.traffic_manager_executor import execute_traffic_manager_plan_set

        world = FakeWorld()
        tm = FakeTrafficManager()

        execute_traffic_manager_plan_set(self._plan_set(), world=world, traffic_manager=tm, tm_port=8100)

        self.assertIn(("distance_to_leading_vehicle", 100, 7.5), tm.calls)
        self.assertIn(("vehicle_percentage_speed_difference", 100, 20.0), tm.calls)
        self.assertIn(("auto_lane_change", 100, False), tm.calls)

    def test_closed_loop_level_drives_tm_style_configuration_for_reactive_and_scripted_plans(self):
        from actors.traffic_manager_executor import build_traffic_manager_actor_settings

        reactive = {
            "actor_id": "reactive",
            "runtime_mode": "traffic_manager",
            "closed_loop_level": "traffic_manager_reactive",
            "style": "aggressive",
            "controller": {"parameters": {}},
        }
        scripted = {
            "actor_id": "scripted",
            "runtime_mode": "scripted",
            "closed_loop_level": "scripted",
            "style": "defensive",
            "controller": {"parameters": {}},
        }

        reactive_settings = build_traffic_manager_actor_settings(reactive)
        scripted_settings = build_traffic_manager_actor_settings(scripted)

        self.assertLess(reactive_settings["min_gap_m"], scripted_settings["min_gap_m"])
        self.assertLess(reactive_settings["speed_difference_percent"], scripted_settings["speed_difference_percent"])
        self.assertTrue(reactive_settings["auto_lane_change"])
        self.assertFalse(scripted_settings["auto_lane_change"])

    def test_scripted_and_replay_are_plan_only_fallbacks(self):
        from actors.traffic_manager_executor import execute_traffic_manager_plan_set

        plan_set = self._plan_set()
        plan_set["actors"].append({"actor_id": "scripted", "runtime_mode": "scripted", "controller": {}})

        result = execute_traffic_manager_plan_set(plan_set, world=FakeWorld(), traffic_manager=FakeTrafficManager())

        statuses = {actor["actor_id"]: actor["runtime_status"] for actor in result["actors"]}
        self.assertEqual(statuses["background-replay"], "plan_only_fallback")
        self.assertEqual(statuses["scripted"], "plan_only_fallback")
        self.assertEqual(statuses["reactive-actor"], "traffic_manager_bound")

        scripted_result = next(actor for actor in result["actors"] if actor["actor_id"] == "scripted")
        self.assertIn("traffic_manager_settings", scripted_result)

    def test_missing_traffic_manager_can_be_resolved_from_client(self):
        from actors.traffic_manager_executor import execute_traffic_manager_plan_set

        class FakeClient:
            def __init__(self):
                self.tm = FakeTrafficManager()
                self.calls = []

            def get_trafficmanager(self, port):
                self.calls.append(("get_trafficmanager", port))
                return self.tm

        client = FakeClient()
        result = execute_traffic_manager_plan_set(self._plan_set(), world=FakeWorld(), client=client, tm_port=8200)

        self.assertEqual(client.calls, [("get_trafficmanager", 8200)])
        self.assertEqual(result["summary"]["spawned_count"], 1)

    def test_transform_factory_can_be_injected_for_real_carla_debug(self):
        from actors.traffic_manager_executor import execute_traffic_manager_plan_set

        def transform_factory(initial_state):
            return (initial_state["x"], initial_state["y"], initial_state.get("yaw", 0.0))

        world = FakeWorld()
        execute_traffic_manager_plan_set(
            self._plan_set(),
            world=world,
            traffic_manager=FakeTrafficManager(),
            transform_factory=transform_factory,
        )

        self.assertEqual(world.spawned[0][1], (10.0, 3.0, 90.0))


if __name__ == "__main__":
    unittest.main()
