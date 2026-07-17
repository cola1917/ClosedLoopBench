import copy
import unittest


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


if __name__ == "__main__":
    unittest.main()
