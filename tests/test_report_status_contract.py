import unittest


class ReportStatusContractTests(unittest.TestCase):
    def test_runtime_status_values_are_explicit(self):
        from metrics.report import RUNTIME_STATUSES

        self.assertEqual(
            RUNTIME_STATUSES,
            {
                "not_run",
                "planned",
                "ego_closed_loop",
                "interactive_closed_loop",
                "failed",
                "completed",
            },
        )

    def test_report_accepts_runtime_handoff_statuses(self):
        from metrics.report import build_closed_loop_report

        run_config = {"scenario_id": "scene-status", "actors": []}
        for status in ["planned", "ego_closed_loop", "interactive_closed_loop", "failed"]:
            with self.subTest(status=status):
                report = build_closed_loop_report(run_config, status=status)
                self.assertEqual(report["status"], status)

    def test_report_rejects_unknown_status(self):
        from metrics.report import build_closed_loop_report

        with self.assertRaises(ValueError):
            build_closed_loop_report({"scenario_id": "scene-status", "actors": []}, status="maybe")


if __name__ == "__main__":
    unittest.main()
