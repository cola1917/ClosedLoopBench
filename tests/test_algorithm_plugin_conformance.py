import json
import unittest
from pathlib import Path


class AlgorithmPluginConformanceTests(unittest.TestCase):
    def test_guard_suite_covers_and_passes_required_failures(self):
        from runners.run_algorithm_plugin_conformance import run_contract_guard_suite

        cases = run_contract_guard_suite()
        by_name = {case["name"]: case for case in cases}
        expected = {
            "normal_control",
            "missing_required_camera",
            "missing_lidar",
            "stale_frame",
            "observation_frame_mismatch",
            "control_frame_mismatch",
            "inference_timeout",
            "backend_exception",
            "invalid_control_range",
            "nan_inf_control",
            "health_check_failure",
            "reset_close_lifecycle",
            "checkpoint_not_found",
            "capability_input_mismatch",
            "safe_stop_policy",
        }
        self.assertTrue(expected.issubset(by_name))
        self.assertTrue(all(case["execution_status"] == "passed" for case in cases))

    def test_reference_plugin_passes_machine_readable_conformance(self):
        from runners.run_algorithm_plugin_conformance import run_plugin_conformance

        report = run_plugin_conformance(
            "agents.reference_pure_pursuit:create_plugin",
            {"repo_path": str(Path(__file__).resolve().parents[1]), "profile": "long"},
        )
        self.assertEqual(report["execution_status"], "passed")
        self.assertEqual(report["evidence_classification"], "offline_conformance")
        self.assertTrue(report["remote_validation_queue_eligible"])
        self.assertFalse(report["real_carla_nurec_closed_loop"])
        json.dumps(report, sort_keys=True)


if __name__ == "__main__":
    unittest.main()
