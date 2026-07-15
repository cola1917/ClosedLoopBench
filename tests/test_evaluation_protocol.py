import copy
import json
import unittest
from pathlib import Path


SCHEMA = (
    Path(__file__).parents[2]
    / "SceneExchangeContracts"
    / "src"
    / "scene_exchange_contracts"
    / "schemas"
    / "shared_exchange_protocol"
    / "evaluation_run_request.schema.json"
)


def _request():
    return copy.deepcopy(json.loads(SCHEMA.read_text(encoding="utf-8"))["examples"][0])


def _report(request, status="interactive_closed_loop"):
    payload = request["payload"]
    return {
        "schema_version": "closed_loop_report.mvp.v0",
        "run_id": payload["run_id"],
        "scenario_id": payload["scene_id"],
        "status": status,
        "experiment": {
            "run_id": payload["run_id"],
            "scene_version": payload["scene_version"],
            "algorithm_id": payload["algorithm"]["algorithm_id"],
            "algorithm_version": payload["algorithm"]["algorithm_version"],
            "odd_id": payload["odd"]["odd_id"],
            "seed": payload["seed"],
        },
        "summary": {
            "collision_count": 0,
            "min_ttc": 2.8,
            "route_progress": 1.0,
            "hard_brake_count": 0,
            "max_jerk": 3.1,
        },
        "evaluation": {"overall_result": "pass"},
        "runtime": {"ego_driver_diagnostics": {"fallback_count": 2}},
    }


def _report_ref():
    return {
        "schema_version": "shared_artifact_ref.v1",
        "path": "runs/run-cc8c0bf57f984915a77078b10eb33198-basic-agent-clear-42/closed_loop_report.json",
        "role": "closed_loop_report",
        "media_type": "application/json",
        "sha256": "f" * 64,
        "size_bytes": 8192,
        "immutable": True,
    }


class EvaluationProtocolTests(unittest.TestCase):
    def test_success_report_becomes_schema_valid_result_without_losing_identity(self):
        from adapters.evaluation_protocol import build_evaluation_result_message
        from adapters.shared_protocol_validation import validate_shared_document

        request = _request()
        result = build_evaluation_result_message(
            request,
            _report(request),
            report_reference=_report_ref(),
            started_at="2026-07-14T00:00:00Z",
            finished_at="2026-07-14T00:01:00Z",
            producer={
                "project": "ClosedLoopBench",
                "component": "test-adapter",
                "version": "test",
            },
        )
        validate_shared_document(result)
        self.assertEqual(result["payload"]["status"], "succeeded")
        self.assertEqual(result["payload"]["run_id"], request["payload"]["run_id"])
        self.assertEqual(result["payload"]["summary"]["control_timeout_count"], 2)

    def test_failed_runtime_has_structured_retryable_error(self):
        from adapters.evaluation_protocol import build_evaluation_result_message

        request = _request()
        result = build_evaluation_result_message(
            request,
            _report(request, status="failed"),
            report_reference=_report_ref(),
            started_at="2026-07-14T00:00:00Z",
            finished_at="2026-07-14T00:00:02Z",
            producer={"project": "ClosedLoopBench", "component": "test", "version": "test"},
        )
        self.assertEqual(result["payload"]["status"], "failed")
        self.assertTrue(result["payload"]["error"]["retryable"])

    def test_mismatched_report_is_rejected_instead_of_misattributed(self):
        from adapters.evaluation_protocol import EvaluationProtocolError, build_evaluation_result_message

        request = _request()
        report = _report(request)
        report["experiment"]["seed"] = 999
        with self.assertRaisesRegex(EvaluationProtocolError, "seed"):
            build_evaluation_result_message(
                request,
                report,
                report_reference=_report_ref(),
                started_at="2026-07-14T00:00:00Z",
                finished_at="2026-07-14T00:01:00Z",
                producer={"project": "ClosedLoopBench", "component": "test", "version": "test"},
            )


if __name__ == "__main__":
    unittest.main()
