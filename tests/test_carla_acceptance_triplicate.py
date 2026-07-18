import copy
import tempfile
import unittest
from pathlib import Path


def _result(status="interactive_closed_loop"):
    return {
        "status": status,
        "cleanup_succeeded": True,
        "report": {
            "summary": {"route_progress": 0.99, "collision_count": 0},
            "runtime": {
                "collision_sensor_available": True,
                "frame_trace_count": 600,
                "actor_physical_response": {
                    "trigger": {"displacement_m": 3.0, "speed_mps": 2.0}
                },
            },
        },
    }


class CarlaAcceptanceTriplicateTests(unittest.TestCase):
    def test_triplicate_carries_formal_map_driver_and_injected_handler(self):
        from runners.run_carla_acceptance_triplicate import run_acceptance_triplicate

        plans = []
        handlers = []

        class Handler:
            def __init__(self):
                self.closed = False

            def __call__(self, context):
                return {"frame": context.get("frame_id")}

            def close(self):
                self.closed = True

        def handler_factory(config, run_dir):
            handler = Handler()
            handlers.append((config["run_id"], run_dir, handler))
            return handler

        def execute(plan, *, sensor_frame_handler):
            plans.append((plan, sensor_frame_handler))
            return _result()

        config = {
            "scenario_id": "scene-triplicate",
            "run_id": "formal",
            "ego": {"initial_state": {"x": 0.0, "y": 0.0}},
            "actors": [],
            "metrics": ["collision", "route_progress"],
        }
        with tempfile.TemporaryDirectory() as directory:
            result = run_acceptance_triplicate(
                config,
                Path(directory),
                max_ticks=1200,
                opendrive_path="/runtime/scene0061-v7.xodr",
                ego_driver="topology_follower",
                sensor_frame_handler_factory=handler_factory,
                execute=execute,
            )

        self.assertEqual(result["run_count"], 3)
        self.assertEqual(len(handlers), 3)
        self.assertEqual(len(plans), 3)
        self.assertTrue(all(handler.closed for _, _, handler in handlers))
        self.assertTrue(
            all(plan["world"]["opendrive_path"].endswith("scene0061-v7.xodr") for plan, _ in plans)
        )
        self.assertTrue(all(plan["ego"]["driver"] == "topology_follower" for plan, _ in plans))
        self.assertTrue(all(plan["limits"]["max_ticks"] == 1200 for plan, _ in plans))

    def test_real_multimodal_triplicate_requires_handler_factory(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            run_acceptance_triplicate,
        )

        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(CarlaAcceptanceError, "real sensor frame handler"):
                run_acceptance_triplicate(
                    {"scenario_id": "scene-triplicate"},
                    Path(directory),
                    require_multimodal=True,
                )

    def test_requires_three_complete_physical_runs(self):
        from runners.run_carla_acceptance_triplicate import validate_acceptance_runs

        validated = validate_acceptance_runs([_result(), _result(), _result()])
        self.assertEqual(len(validated), 3)
        self.assertTrue(all(item["cleanup_succeeded"] for item in validated))

    def test_rejects_control_only_actor_claim_and_unknown_collision_sensor(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            validate_acceptance_runs,
        )

        control_only = _result()
        control_only["report"]["runtime"]["actor_physical_response"] = {}
        with self.assertRaisesRegex(CarlaAcceptanceError, "physical Actor"):
            validate_acceptance_runs([_result(), control_only, _result()])

        no_sensor = _result()
        no_sensor["report"]["runtime"]["collision_sensor_available"] = False
        with self.assertRaisesRegex(CarlaAcceptanceError, "collision sensor"):
            validate_acceptance_runs([_result(), no_sensor, _result()])

    def test_rejects_route_cleanup_and_run_count_gaps(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            validate_acceptance_runs,
        )

        with self.assertRaisesRegex(CarlaAcceptanceError, "exactly three"):
            validate_acceptance_runs([_result(), _result()])
        route = _result()
        route["report"]["summary"]["route_progress"] = 0.94
        with self.assertRaisesRegex(CarlaAcceptanceError, "route progress"):
            validate_acceptance_runs([_result(), route, _result()])
        cleanup = _result()
        cleanup["cleanup_succeeded"] = False
        with self.assertRaisesRegex(CarlaAcceptanceError, "cleanup"):
            validate_acceptance_runs([_result(), cleanup, _result()])

    def test_optional_multimodal_gate_requires_complete_actor_sensor_evidence(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            validate_acceptance_runs,
        )
        from tests.test_multimodal_closed_loop_acceptance import _result as multimodal_result

        valid = multimodal_result()
        valid["cleanup_succeeded"] = True
        valid["sensor_handler_cleanup_succeeded"] = True
        valid["report"]["summary"] = {"route_progress": 0.99, "collision_count": 0}
        valid["report"]["runtime"].update(
            collision_sensor_available=True,
            frame_trace_count=2,
        )
        accepted = validate_acceptance_runs(
            [copy.deepcopy(valid), copy.deepcopy(valid), copy.deepcopy(valid)],
            require_multimodal=True,
        )
        self.assertEqual(accepted[0]["multimodal_closed_loop"]["status"], "passed")

        invalid = copy.deepcopy(valid)
        invalid["nurec_multimodal_trace"][0]["modalities"]["rgb"]["passed_count"] = 0
        with self.assertRaisesRegex(CarlaAcceptanceError, "multimodal"):
            validate_acceptance_runs(
                [copy.deepcopy(valid), invalid, copy.deepcopy(valid)],
                require_multimodal=True,
            )

    def test_multimodal_handler_cleanup_is_fail_closed(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            run_acceptance_triplicate,
        )

        class Handler:
            def __call__(self, _context):
                return {}

            def close(self):
                raise RuntimeError("grpc close failed")

        def execute(_plan, *, sensor_frame_handler):
            self.assertIsInstance(sensor_frame_handler, Handler)
            return _result()

        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(CarlaAcceptanceError, "cleanup failed"):
                run_acceptance_triplicate(
                    {
                        "scenario_id": "scene-triplicate",
                        "run_id": "cleanup",
                        "ego": {"initial_state": {"x": 0.0, "y": 0.0}},
                        "actors": [],
                    },
                    Path(directory),
                    require_multimodal=True,
                    sensor_frame_handler_factory=lambda _config, _run_dir: Handler(),
                    execute=execute,
                )


if __name__ == "__main__":
    unittest.main()
