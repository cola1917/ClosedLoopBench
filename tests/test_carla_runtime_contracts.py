import importlib
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _import_or_skip(module_name):
    try:
        return importlib.import_module(module_name)
    except ModuleNotFoundError as exc:
        if exc.name == module_name or exc.name == module_name.split(".")[0]:
            raise unittest.SkipTest("{} is not implemented yet".format(module_name))
        raise


def _minimal_run_config():
    return {
        "schema_version": "carla_run_config.mvp.v0",
        "scenario_id": "scene-runtime-contract",
        "carla": {
            "version": "0.9.16",
            "map": "Town04",
            "fixed_delta_seconds": 0.05,
        },
        "ego": {
            "agent": "baseline_lane_following",
            "initial_state": {
                "t_sec": 0.0,
                "x": 1.0,
                "y": 2.0,
                "z": 0.0,
                "yaw": 0.5,
                "speed_mps": 4.0,
            },
            "reference_trajectory": [
                {"t_sec": 0.0, "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5, "speed_mps": 4.0},
                {"t_sec": 4.0, "x": 20.0, "y": 8.0, "z": 0.0, "yaw": 0.5, "speed_mps": 8.0},
            ],
        },
        "actors": [],
        "metrics": ["collision", "min_ttc", "route_progress"],
    }


class CarlaRuntimeContractTests(unittest.TestCase):
    def test_carla_probe_module_imports_without_installed_carla(self):
        with mock.patch.dict(sys.modules, {"carla": None}):
            module = _import_or_skip("runtime.carla_probe")

        self.assertTrue(hasattr(module, "build_probe_config"))
        self.assertTrue(hasattr(module, "probe_carla"))

    def test_carla_probe_accepts_injected_fake_carla_module(self):
        module = _import_or_skip("runtime.carla_probe")

        class FakeMap:
            name = "Town04"

        class FakeWorld:
            def get_map(self):
                return FakeMap()

        class FakeClient:
            def __init__(self, host, port):
                self.host = host
                self.port = port
                self.timeout = None

            def set_timeout(self, timeout):
                self.timeout = timeout

            def get_world(self):
                return FakeWorld()

            def get_server_version(self):
                return "0.9.16"

        fake_carla = types.SimpleNamespace(Client=FakeClient)
        config = module.build_probe_config(
            host="127.0.0.1",
            port=2000,
            timeout_sec=1.0,
            map_name="Town04",
        )

        result = module.probe_carla(config, carla_module=fake_carla)

        self.assertEqual(result["status"], "available")
        self.assertEqual(result["host"], "127.0.0.1")
        self.assertEqual(result["port"], 2000)
        self.assertEqual(result["map"], "Town04")
        self.assertEqual(result["carla_version"], "0.9.16")

    def test_scenario_runner_command_is_argument_list(self):
        module = _import_or_skip("runtime.scenario_runner")
        xosc_path = PROJECT_ROOT / "outputs" / "scene-1077-integrated" / "scenario.latest.xosc"
        config = {
            "python": "python",
            "scenario_runner_root": "E:/tools/scenario_runner",
            "openscenario": str(xosc_path),
            "host": "127.0.0.1",
            "port": 2000,
            "output": True,
        }

        command = module.build_scenario_runner_command(config)

        self.assertIsInstance(command, list)
        self.assertIn("scenario_runner.py", command[1])
        self.assertIn("--openscenario", command)
        self.assertIn(str(xosc_path), command)
        self.assertIn("--host", command)
        self.assertIn("127.0.0.1", command)
        self.assertIn("--port", command)
        self.assertIn("2000", command)
        self.assertIn("--output", command)

    def test_basic_agent_plan_is_constructed_without_importing_carla(self):
        with mock.patch.dict(sys.modules, {"carla": None}):
            module = _import_or_skip("runners.run_carla_basic_agent")

        plan = module.build_basic_agent_plan(
            _minimal_run_config(),
            host="127.0.0.1",
            port=2000,
            max_ticks=300,
            synchronous=True,
        )

        self.assertEqual(plan["scenario_id"], "scene-runtime-contract")
        self.assertEqual(plan["connection"]["host"], "127.0.0.1")
        self.assertEqual(plan["connection"]["port"], 2000)
        self.assertEqual(plan["world"]["map"], "Town04")
        self.assertTrue(plan["world"]["synchronous"])
        self.assertEqual(plan["ego"]["agent"], "basic_agent")
        self.assertEqual(plan["ego"]["spawn"]["x"], 1.0)
        self.assertEqual(plan["ego"]["destination"]["x"], 20.0)
        self.assertEqual(plan["limits"]["max_ticks"], 300)
        self.assertIn("closed_loop_report", plan["artifacts"])


if __name__ == "__main__":
    unittest.main()

