import unittest


def _run_config(criteria=None):
    config = {
        "scenario_id": "scene-criteria",
        "actors": [],
        "metrics": ["collision", "min_ttc", "route_progress"],
    }
    if criteria is not None:
        config["evaluation"] = {"criteria": criteria}
    return config


class EvaluationCriteriaTests(unittest.TestCase):
    def test_default_criteria_pass_when_metrics_clear_thresholds(self):
        from metrics.report import build_closed_loop_report

        report = build_closed_loop_report(
            _run_config(),
            tick_metrics=[
                {"collision": False, "route_progress": 0.25},
                {"collision": False, "route_progress": 1.0, "min_ttc": 3.2},
            ],
            status="completed",
        )

        self.assertEqual(report["evaluation"]["overall_result"], "pass")
        by_name = {
            criterion["name"]: criterion
            for criterion in report["evaluation"]["criteria"]
        }
        self.assertEqual(by_name["collision_count"]["result"], "pass")
        self.assertEqual(by_name["route_progress"]["result"], "pass")
        self.assertEqual(by_name["min_ttc"]["result"], "pass")

    def test_criteria_failures_make_overall_fail(self):
        from metrics.report import build_closed_loop_report

        report = build_closed_loop_report(
            _run_config(
                [
                    {"metric": "collision_count", "op": "==", "value": 0},
                    {"metric": "route_progress", "op": ">=", "value": 0.95},
                ]
            ),
            tick_metrics=[
                {"collision": True, "route_progress": 0.5},
            ],
            status="completed",
        )

        self.assertEqual(report["evaluation"]["overall_result"], "fail")
        results = [
            criterion["result"]
            for criterion in report["evaluation"]["criteria"]
        ]
        self.assertEqual(results, ["fail", "fail"])

    def test_missing_ttc_is_unknown_not_pass(self):
        from metrics.report import build_closed_loop_report

        report = build_closed_loop_report(
            _run_config(
                [
                    {"metric": "collision_count", "op": "==", "value": 0},
                    {"metric": "route_progress", "op": ">=", "value": 0.95},
                    {"metric": "min_ttc", "op": ">=", "value": 1.0},
                ]
            ),
            tick_metrics=[
                {"collision": False, "route_progress": 1.0},
            ],
            status="completed",
        )

        self.assertEqual(report["evaluation"]["overall_result"], "unknown")
        by_name = {
            criterion["name"]: criterion
            for criterion in report["evaluation"]["criteria"]
        }
        self.assertEqual(by_name["collision_count"]["result"], "pass")
        self.assertEqual(by_name["route_progress"]["result"], "pass")
        self.assertEqual(by_name["min_ttc"]["result"], "unknown")
        self.assertIsNone(by_name["min_ttc"]["actual"])

    def test_not_run_progress_is_unknown_instead_of_fail(self):
        from metrics.report import build_closed_loop_report

        report = build_closed_loop_report(_run_config(), status="not_run")

        by_name = {
            criterion["name"]: criterion
            for criterion in report["evaluation"]["criteria"]
        }
        self.assertEqual(report["evaluation"]["overall_result"], "unknown")
        self.assertEqual(by_name["collision_count"]["result"], "pass")
        self.assertEqual(by_name["route_progress"]["result"], "unknown")
        self.assertEqual(by_name["min_ttc"]["result"], "unknown")


if __name__ == "__main__":
    unittest.main()
