import unittest


def minimal_scenario_ir():
    return {
        "schema_version": "scenario_ir.mvp.v0",
        "scenario_id": "scene-test-1077",
        "windows": {
            "event": {"start_sec": 0.0, "end_sec": 5.0},
            "warmup": {"start_sec": -2.0, "end_sec": 0.0},
        },
        "ego": {
            "initial_state": {"t_sec": 0.0, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "speed_mps": 4.0},
            "reference_trajectory": [
                {"t_sec": 0.0, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "speed_mps": 4.0},
                {"t_sec": 5.0, "x": 20.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "speed_mps": 8.0},
            ],
        },
        "actors": [
            {
                "actor_id": "actor-trigger",
                "role": "trigger",
                "type": "vehicle",
                "initial_state": {"t_sec": 0.0, "x": 8.0, "y": 1.5, "z": 0.0, "yaw": 0.0, "speed_mps": 5.0},
                "reference_trajectory": [],
            }
        ],
        "evaluation": {"metrics": ["collision", "min_ttc", "route_progress"]},
    }


class CarlaAdapterContractTests(unittest.TestCase):
    def test_builds_carla_run_config_from_scenario_ir(self):
        from adapters.ir_to_carla import build_carla_run_config

        scenario_ir = minimal_scenario_ir()
        config = build_carla_run_config(scenario_ir, carla_map="Town04")

        self.assertEqual(config["schema_version"], "carla_run_config.mvp.v0")
        self.assertEqual(config["scenario_id"], scenario_ir["scenario_id"])
        self.assertEqual(config["carla"]["version"], "0.9.16")
        self.assertEqual(config["carla"]["map"], "Town04")
        self.assertEqual(config["windows"]["event"], scenario_ir["windows"]["event"])
        self.assertEqual(config["ego"]["agent"], "baseline_lane_following")
        self.assertIsNotNone(config["ego"]["initial_state"])
        self.assertGreaterEqual(len(config["actors"]), 1)
        self.assertIn(config["actors"][0]["policy"], {"replay", "reactive_rule_based"})
        self.assertIn("collision", config["metrics"])
        self.assertEqual(config["reconstruction_package"]["enabled"], False)


if __name__ == "__main__":
    unittest.main()
