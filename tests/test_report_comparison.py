import unittest


class ReportComparisonTests(unittest.TestCase):
    def test_groups_runs_by_algorithm_and_odd(self):
        from metrics.comparison import compare_closed_loop_reports

        reports = [
            {
                "scenario_id": "scene-a",
                "experiment": {"algorithm_id": "tcp", "algorithm_version": "v1", "odd_id": "clear", "seed": 41},
                "summary": {"collision_count": 0, "route_progress": 1.0, "control_timeout_count": 0},
                "evaluation": {"overall_result": "pass"},
            },
            {
                "scenario_id": "scene-a",
                "experiment": {"algorithm_id": "tcp", "algorithm_version": "v1", "odd_id": "rain", "seed": 42},
                "summary": {"collision_count": 1, "route_progress": 0.5, "control_timeout_count": 2},
                "evaluation": {"overall_result": "fail"},
            },
            {
                "scenario_id": "scene-a",
                "experiment": {"algorithm_id": "basic_agent", "odd_id": "clear"},
                "summary": {"collision_count": 0, "route_progress": 0.8},
                "evaluation": {"overall_result": "pass"},
            },
        ]

        comparison = compare_closed_loop_reports(reports)

        self.assertEqual(comparison["run_count"], 3)
        self.assertEqual(comparison["algorithms"]["tcp"]["run_count"], 2)
        self.assertEqual(comparison["algorithms"]["tcp"]["odd_ids"], ["clear", "rain"])
        self.assertEqual(comparison["algorithms"]["tcp"]["mean"]["collision_count"], 0.5)
        self.assertEqual(comparison["algorithms"]["tcp"]["mean"]["route_progress"], 0.75)
        self.assertEqual(comparison["algorithms"]["tcp"]["algorithm_versions"], ["v1"])
        self.assertEqual(comparison["algorithms"]["tcp"]["seeds"], [41, 42])
        self.assertEqual(comparison["algorithms"]["tcp"]["mean"]["control_timeout_count"], 1.0)


if __name__ == "__main__":
    unittest.main()
