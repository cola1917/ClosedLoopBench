import json
import unittest
from pathlib import Path


class CarlaAdapterContractTests(unittest.TestCase):
    def test_builds_carla_run_config_from_scenario_ir(self):
        from adapters.ir_to_carla import build_carla_run_config

        scenario_ir = json.loads(
            Path("E:/code/TriggerEngine/outputs/scenario_ir/scene-1077.mvp.json").read_text(encoding="utf-8")
        )

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
