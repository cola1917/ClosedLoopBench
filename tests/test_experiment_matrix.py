import copy
import unittest


def _matrix():
    return {
        "schema_version": "closed_loop_experiment_matrix.v0",
        "matrix_id": "core-baselines-v1",
        "producer": {
            "project": "ClosedLoopBench",
            "component": "matrix-planner",
            "version": "1.0.0",
        },
        "scenes": [
            {
                "scene_id": "cc8c0bf57f984915a77078b10eb33198",
                "scene_version": "v001",
                "correlation_id": "flow-scene-0061",
                "root_message_id": "msg-selection-001",
                "scene_result_message_id": "msg-reconstruction-result-001",
                "scene_package": {
                    "schema_version": "shared_artifact_ref.v1",
                    "path": "scenes/cc8c0bf57f984915a77078b10eb33198/v001/scene_package.json",
                    "role": "scene_package",
                    "media_type": "application/json",
                    "sha256": "a" * 64,
                    "size_bytes": 2048,
                    "immutable": True,
                },
            }
        ],
        "algorithms": [
            {
                "algorithm_id": "basic_agent",
                "algorithm_version": "carla-0.9.16",
                "driver": "basic_agent",
            },
            {
                "algorithm_id": "tcp",
                "algorithm_version": "commit-abc123",
                "driver": "ros2",
            },
        ],
        "odds": [
            {"odd_id": "clear", "weather": "ClearNoon"},
            {"odd_id": "rain", "weather": "HardRainNoon"},
        ],
        "seeds": [41, 42, 43],
        "simulator": {
            "name": "carla",
            "version": "0.9.16",
            "synchronous_mode": True,
            "fixed_delta_seconds": 0.05,
        },
        "actor_control": {"mode": "traffic_manager", "style": "normal"},
        "metrics": ["collision_count", "min_ttc", "route_progress", "hard_brake_count", "max_jerk"],
        "timeout_sec": 120,
    }


def _report(run):
    request = run["request"]
    payload = request["payload"]
    return {
        "schema_version": "closed_loop_report.mvp.v0",
        "scenario_id": payload["scene_id"],
        "status": "interactive_closed_loop",
        "experiment": {
            "scene_version": payload["scene_version"],
            "algorithm_id": payload["algorithm"]["algorithm_id"],
            "algorithm_version": payload["algorithm"]["algorithm_version"],
            "odd_id": payload["odd"]["odd_id"],
            "seed": payload["seed"],
        },
        "summary": {
            "collision_count": 0,
            "min_ttc": 2.0,
            "route_progress": 1.0,
            "hard_brake_count": 0,
            "max_jerk": 3.0,
        },
        "evaluation": {"overall_result": "pass"},
    }


class ExperimentMatrixTests(unittest.TestCase):
    def test_builds_deterministic_cartesian_requests(self):
        from runtime.experiment_matrix import build_experiment_plan

        plan = build_experiment_plan(_matrix(), created_at="2026-07-14T00:00:00Z")
        self.assertEqual(plan["expected_run_count"], 12)
        self.assertEqual(plan["dimensions"], {"scenes": 1, "algorithms": 2, "odds": 2, "seeds": 3})
        self.assertEqual(len({run["run_id"] for run in plan["runs"]}), 12)
        self.assertTrue(all(run["request"]["schema_version"] == "evaluation_run_request.v1" for run in plan["runs"]))

    def test_rejects_duplicate_or_ambiguous_dimensions(self):
        from runtime.experiment_matrix import ExperimentMatrixError, build_experiment_plan

        matrix = _matrix()
        matrix["seeds"] = [42, 42]
        with self.assertRaisesRegex(ExperimentMatrixError, "seeds must be unique"):
            build_experiment_plan(matrix)

        matrix = _matrix()
        matrix["algorithms"].append(copy.deepcopy(matrix["algorithms"][0]))
        with self.assertRaisesRegex(ExperimentMatrixError, "duplicate algorithm_id"):
            build_experiment_plan(matrix)

    def test_complete_successful_reports_open_comparison_gate(self):
        from runtime.experiment_matrix import build_experiment_plan, evaluate_experiment_coverage

        plan = build_experiment_plan(_matrix(), created_at="2026-07-14T00:00:00Z")
        coverage = evaluate_experiment_coverage(plan, [_report(run) for run in plan["runs"]])
        self.assertTrue(coverage["ready_for_comparison"])
        self.assertEqual(coverage["coverage_ratio"], 1.0)

    def test_missing_duplicate_unknown_metric_and_noninteractive_runs_block_gate(self):
        from runtime.experiment_matrix import build_experiment_plan, evaluate_experiment_coverage

        plan = build_experiment_plan(_matrix(), created_at="2026-07-14T00:00:00Z")
        reports = [_report(run) for run in plan["runs"][:-1]]
        reports.append(copy.deepcopy(reports[0]))
        reports[1]["summary"]["min_ttc"] = None
        reports[2]["status"] = "ego_closed_loop"
        coverage = evaluate_experiment_coverage(plan, reports)
        self.assertFalse(coverage["ready_for_comparison"])
        self.assertEqual(len(coverage["missing"]), 1)
        self.assertEqual(len(coverage["duplicates"]), 1)
        reasons = [reason for item in coverage["invalid_runs"] for reason in item["reasons"]]
        self.assertTrue(any("min_ttc" in reason for reason in reasons))
        self.assertTrue(any("interactive_closed_loop" in reason for reason in reasons))


if __name__ == "__main__":
    unittest.main()
