import copy
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path


def _matrix():
    from runtime.scene0061_counterfactual import build_scene0061_counterfactual_matrix

    return build_scene0061_counterfactual_matrix()


def _report(matrix, algorithm, case, seed, *, evidence_class=None):
    identity = matrix["scene_identity"]
    stress = case["quality_stress_only"]
    outcomes = {name: 1 for name in case.get("required_actor_outcomes", [])}
    return {
        "scenario_id": identity["scene_id"],
        "status": "completed",
        "evidence_classification": evidence_class or ("quality_stress" if stress else "control_only"),
        "experiment": {
            "algorithm_id": algorithm["algorithm_id"],
            "algorithm_version": algorithm["algorithm_version"],
            "algorithm_config_sha256": algorithm["config_sha256"],
            "checkpoint_sha256": algorithm["checkpoint_sha256"],
            "case_id": case["case_id"],
            "seed": seed,
            "scene_version": identity["scene_version"],
            "identity": {
                "artifact_sha256": identity["artifact_sha256"],
                "scene_package_sha256": identity["scene_package_sha256"],
                "scenario_ir_sha256": identity["scenario_ir_sha256"],
            },
        },
        "summary": {
            "collision_count": 0,
            "route_progress": 1.0,
            "min_ttc": 2.0 + seed / 1000.0,
            "route_completion_time_sec": 10.0 + seed / 100.0,
            "actor_outcomes": outcomes or None,
            "metric_availability": {
                "collision_count": True,
                "route_progress": True,
                "min_ttc": True,
                "actor_outcomes": bool(outcomes),
            },
        },
        "evaluation": {"overall_result": "pass"},
    }


def _complete_inputs(matrix):
    reports = []
    quality = []
    for algorithm in matrix["algorithms"]:
        for case in matrix["cases"]:
            for seed in matrix["seeds"]:
                reports.append(_report(matrix, algorithm, case, seed))
                if case["quality_stress_only"]:
                    quality.append(
                        {
                            "experiment": {
                                "algorithm_id": algorithm["algorithm_id"],
                                "case_id": case["case_id"],
                                "seed": seed,
                                "identity": {
                                    "artifact_sha256": matrix["scene_identity"]["artifact_sha256"],
                                    "scene_package_sha256": matrix["scene_identity"]["scene_package_sha256"],
                                    "scenario_ir_sha256": matrix["scene_identity"]["scenario_ir_sha256"],
                                },
                            },
                            "classification": "quality_stress",
                        }
                    )
    return reports, quality


class CounterfactualSuiteTests(unittest.TestCase):
    def test_complete_triplicate_is_comparable_but_keeps_quality_stress_unranked(self):
        from metrics.counterfactual_suite import evaluate_counterfactual_suite

        matrix = _matrix()
        reports, quality = _complete_inputs(matrix)
        result = evaluate_counterfactual_suite(matrix, reports, quality)

        self.assertTrue(result["ready_for_formal_comparison"])
        self.assertTrue(result["acceptance_passed"])
        self.assertEqual(result["expected_run_count"], 48)
        self.assertEqual(result["accepted_run_count"], 48)
        self.assertTrue(all(row["triplicate_complete"] for row in result["coverage"]))
        self.assertEqual(len(result["rankings"]["control_only"]), 2)
        self.assertEqual(result["rankings"]["perception_eligible"], [])
        self.assertEqual(len(result["quality_stress_results"]), 6)
        self.assertTrue(all(not row["ranked"] for row in result["quality_stress_results"]))
        self.assertEqual(len(result["baseline_deltas"]), 36)

    def test_missing_duplicate_unexpected_and_malformed_reports_fail_closed(self):
        from metrics.counterfactual_suite import evaluate_counterfactual_suite

        matrix = _matrix()
        reports, quality = _complete_inputs(matrix)
        removed = reports.pop()
        reports.append(copy.deepcopy(reports[0]))
        unexpected = copy.deepcopy(reports[1])
        unexpected["experiment"]["case_id"] = "S99_unknown"
        reports.append(unexpected)
        reports.append({"status": "completed"})
        result = evaluate_counterfactual_suite(matrix, reports, quality)

        self.assertFalse(result["ready_for_formal_comparison"])
        self.assertEqual(len(result["missing"]), 1)
        self.assertEqual(len(result["duplicates"]), 1)
        self.assertEqual(len(result["unexpected"]), 1)
        self.assertEqual(len(result["malformed"]), 1)
        self.assertEqual(result["missing"][0]["algorithm_id"], removed["experiment"]["algorithm_id"])

    def test_offline_identity_mismatch_and_missing_quality_cannot_enter_formal_buckets(self):
        from metrics.counterfactual_suite import evaluate_counterfactual_suite

        matrix = _matrix()
        reports, quality = _complete_inputs(matrix)
        reports[0]["status"] = "offline_conformance"
        reports[0]["evidence_classification"] = "offline_conformance"
        reports[1]["experiment"]["identity"]["artifact_sha256"] = "0" * 64
        quality.pop()
        result = evaluate_counterfactual_suite(matrix, reports, quality)

        self.assertFalse(result["ready_for_formal_comparison"])
        reasons = [reason for row in result["invalid_runs"] for reason in row["reasons"]]
        self.assertTrue(any("offline_conformance" in reason for reason in reasons))
        self.assertTrue(any("artifact_sha256" in reason for reason in reasons))
        self.assertTrue(any("quality-stress" in reason for reason in reasons))
        self.assertEqual(len(result["evidence_buckets"]["offline_conformance"]), 1)

    def test_missing_required_pedestrian_outcome_is_fail_closed(self):
        from metrics.counterfactual_suite import evaluate_counterfactual_suite

        matrix = _matrix()
        reports, quality = _complete_inputs(matrix)
        target = next(
            report
            for report in reports
            if report["experiment"]["case_id"] == "S5_pedestrian_yield"
        )
        target["summary"]["actor_outcomes"] = None
        target["summary"]["metric_availability"]["actor_outcomes"] = False
        result = evaluate_counterfactual_suite(matrix, reports, quality)
        self.assertFalse(result["ready_for_formal_comparison"])
        reasons = [reason for row in result["invalid_runs"] for reason in row["reasons"]]
        self.assertIn("required actor outcome is unavailable: yield", reasons)

    def test_failure_rate_and_baseline_delta_preserve_failed_real_runs(self):
        from metrics.counterfactual_suite import evaluate_counterfactual_suite

        matrix = _matrix()
        reports, quality = _complete_inputs(matrix)
        target = next(
            report for report in reports
            if report["experiment"]["algorithm_id"] == "reference_pure_pursuit_short"
            and report["experiment"]["case_id"] == "S1_lead_slowdown"
            and report["experiment"]["seed"] == 41
        )
        target["evaluation"]["overall_result"] = "fail"
        target["summary"]["route_completion_time_sec"] = 15.0
        result = evaluate_counterfactual_suite(matrix, reports, quality)

        self.assertTrue(result["ready_for_formal_comparison"])
        self.assertFalse(result["acceptance_passed"])
        short = next(row for row in result["rankings"]["control_only"] if row["algorithm_id"] == "reference_pure_pursuit_short")
        self.assertGreater(short["failure_rate"], 0.0)
        delta = next(row for row in result["baseline_deltas"] if row["algorithm_id"] == "reference_pure_pursuit_short" and row["case_id"] == "S1_lead_slowdown" and row["seed"] == 41)
        self.assertGreater(delta["edited_minus_baseline"]["route_completion_time_sec"], 0.0)

    def test_case_level_render_quality_report_schema_is_reusable_across_algorithms_and_seeds(self):
        from metrics.counterfactual_suite import evaluate_counterfactual_suite

        matrix = _matrix()
        reports, quality = _complete_inputs(matrix)
        quality = [
            {
                "schema_version": "render_quality_report.v1",
                "status": "offline_quality_evaluation",
                "scene_id": matrix["scene_identity"]["scene_id"],
                "case_id": "S7_lead_removed_quality_stress",
                "artifact": {"sha256": matrix["scene_identity"]["artifact_sha256"]},
                "evidence_classification": "quality_stress",
            }
        ]
        result = evaluate_counterfactual_suite(matrix, reports, quality)
        self.assertTrue(result["ready_for_formal_comparison"])
        self.assertEqual(len(result["quality_stress_results"]), 6)

    def test_canonical_evidence_classification_precedes_legacy_alias(self):
        from metrics.counterfactual_suite import evaluate_counterfactual_suite

        matrix = _matrix()
        reports, quality = _complete_inputs(matrix)
        reports[0]["evidence_class"] = "offline_conformance"
        result = evaluate_counterfactual_suite(matrix, reports, quality)
        self.assertTrue(result["ready_for_formal_comparison"])
        self.assertEqual(result["evidence_buckets"]["offline_conformance"], [])

    def test_cli_writes_fail_closed_empty_coverage(self):
        from runners.evaluate_counterfactual_suite import main

        matrix = _matrix()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            matrix_path = root / "matrix.json"
            output = root / "evaluation.json"
            matrix_path.write_text(json.dumps(matrix), encoding="utf-8")
            self.assertEqual(main(["--matrix", str(matrix_path), "--output", str(output)]), 0)
            payload = json.loads(output.read_text(encoding="utf-8"))
            self.assertFalse(payload["ready_for_formal_comparison"])
            self.assertEqual(len(payload["missing"]), 48)
            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main(["--matrix", str(matrix_path), "--output", str(output)])


if __name__ == "__main__":
    unittest.main()
