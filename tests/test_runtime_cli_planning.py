import json
import tempfile
import types
import unittest
from pathlib import Path


class RuntimeCliPlanningTests(unittest.TestCase):
    def test_scenario_runner_dry_run_returns_planned_command(self):
        from runtime.scenario_runner import run_scenario_runner

        result = run_scenario_runner(
            {
                "python": "python",
                "scenario_runner_root": "E:/tools/scenario_runner",
                "openscenario": "E:/code/ClosedLoopBench/outputs/scene/scenario.xosc",
                "host": "127.0.0.1",
                "port": 2000,
                "output": True,
            },
            dry_run=True,
        )

        self.assertEqual(result["status"], "planned")
        self.assertIn("--openscenario", result["command"])
        self.assertIn("--output", result["command"])

    def test_basic_agent_plan_cli_writer_outputs_json(self):
        from runners.run_carla_basic_agent import write_basic_agent_plan

        run_config = {
            "schema_version": "carla_run_config.mvp.v0",
            "scenario_id": "scene-cli-plan",
            "carla": {"map": "Town04", "fixed_delta_seconds": 0.05},
            "ego": {
                "initial_state": {"x": 0.0, "y": 1.0, "z": 0.0, "yaw": 0.0, "speed_mps": 2.0},
                "reference_trajectory": [
                    {"x": 0.0, "y": 1.0, "z": 0.0, "yaw": 0.0, "speed_mps": 2.0},
                    {"x": 10.0, "y": 1.0, "z": 0.0, "yaw": 0.0, "speed_mps": 6.0},
                ],
            },
            "actors": [],
            "metrics": ["collision", "route_progress"],
        }

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            config_path = root / "carla_run_config.json"
            plan_path = root / "basic_agent_plan.json"
            config_path.write_text(json.dumps(run_config), encoding="utf-8")

            written = write_basic_agent_plan(config_path, plan_path, host="localhost", port=2000)

            self.assertEqual(written, plan_path)
            plan = json.loads(plan_path.read_text(encoding="utf-8"))
            self.assertEqual(plan["schema_version"], "basic_agent_plan.mvp.v0")
            self.assertEqual(plan["scenario_id"], "scene-cli-plan")
            self.assertEqual(plan["connection"]["host"], "localhost")
            self.assertEqual(plan["ego"]["destination"]["x"], 10.0)

    def test_basic_agent_execute_without_carla_fails_structurally(self):
        from runners.run_carla_basic_agent import run_basic_agent

        fake_carla_without_client = types.SimpleNamespace()
        result = run_basic_agent(
            {"scenario_id": "scene-no-cclient"},
            carla_module=fake_carla_without_client,
            agent_module=None,
        )

        self.assertEqual(result["status"], "failed")
        self.assertIn(result["reason"], {"missing_basic_agent", "basic_agent_runtime_failed"})
        self.assertEqual(result["scenario_id"], "scene-no-cclient")


if __name__ == "__main__":
    unittest.main()
