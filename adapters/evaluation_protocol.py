from __future__ import annotations

from copy import deepcopy
from datetime import datetime
from typing import Any

from adapters.shared_protocol_validation import (
    validate_artifact_reference,
    validate_shared_document,
)


class EvaluationProtocolError(ValueError):
    """Raised when a report cannot be attributed to an evaluation request."""


def build_evaluation_result_message(
    request: dict[str, Any],
    report: dict[str, Any],
    *,
    report_reference: dict[str, Any],
    started_at: str,
    finished_at: str,
    producer: dict[str, Any],
    artifact_references: list[dict[str, Any]] | None = None,
    warnings: list[str] | None = None,
) -> dict[str, Any]:
    validate_shared_document(request)
    if request.get("schema_version") != "evaluation_run_request.v1":
        raise EvaluationProtocolError("request must use evaluation_run_request.v1")
    validate_artifact_reference(report_reference)
    if report_reference.get("role") != "closed_loop_report":
        raise EvaluationProtocolError("report_reference role must be closed_loop_report")
    if report_reference.get("media_type") != "application/json":
        raise EvaluationProtocolError("closed-loop report must use application/json")
    _validate_report_identity(request, report)

    request_payload = request["payload"]
    runtime_status = str(report.get("status") or "failed")
    succeeded = runtime_status in {
        "completed",
        "ego_closed_loop",
        "interactive_closed_loop",
    }
    status = "succeeded" if succeeded else "failed"
    evaluation = report.get("evaluation") or {}
    outcome = str(evaluation.get("overall_result") or "unknown")
    if outcome not in {"pass", "fail", "unknown"}:
        outcome = "unknown"
    summary = report.get("summary") or {}
    runtime = report.get("runtime") or {}
    diagnostics = runtime.get("ego_driver_diagnostics") or {}
    timeout_count = summary.get("control_timeout_count")
    if timeout_count is None:
        timeout_count = diagnostics.get("fallback_count")

    artifacts = [deepcopy(report_reference)]
    for reference in artifact_references or []:
        validate_artifact_reference(reference)
        if not any(
            existing["path"] == reference["path"] and existing["role"] == reference["role"]
            for existing in artifacts
        ):
            artifacts.append(deepcopy(reference))
    payload: dict[str, Any] = {
        "run_id": request_payload["run_id"],
        "scene_id": request_payload["scene_id"],
        "scene_version": request_payload["scene_version"],
        "status": status,
        "runtime_status": runtime_status,
        "outcome": outcome,
        "algorithm": {
            "algorithm_id": request_payload["algorithm"]["algorithm_id"],
            "algorithm_version": request_payload["algorithm"]["algorithm_version"],
        },
        "odd": {
            key: deepcopy(request_payload["odd"][key])
            for key in ("odd_id", "weather")
            if key in request_payload["odd"]
        },
        "started_at": started_at,
        "finished_at": finished_at,
        "duration_sec": _duration(started_at, finished_at),
        "summary": {
            "collision_count": _number_or_none(summary.get("collision_count"), integer=True),
            "min_ttc": _number_or_none(summary.get("min_ttc")),
            "route_progress": _number_or_none(summary.get("route_progress")),
            "hard_brake_count": _number_or_none(summary.get("hard_brake_count"), integer=True),
            "max_jerk": _number_or_none(summary.get("max_jerk")),
            "control_timeout_count": _number_or_none(timeout_count, integer=True),
        },
        "report": deepcopy(report_reference),
        "artifacts": artifacts,
        "warnings": list(warnings or []),
    }
    if not succeeded:
        payload["error"] = {
            "code": "EVALUATION_RUNTIME_FAILED",
            "message": str(
                (report.get("runtime") or {}).get("failure_reason")
                or f"Closed-loop runtime ended with status {runtime_status!r}."
            ),
            "retryable": runtime_status in {"not_run", "planned", "failed"},
        }
    message = {
        "protocol_version": "shared_exchange_protocol.v1",
        "schema_version": "evaluation_run_result.v1",
        "message_id": f"msg-result-{request_payload['run_id']}",
        "message_type": "evaluation.run.result",
        "created_at": finished_at,
        "producer": deepcopy(producer),
        "correlation": {
            "correlation_id": request["correlation"]["correlation_id"],
            "root_message_id": request["correlation"]["root_message_id"],
            "causation_message_id": request["message_id"],
        },
        "idempotency": {
            "key": f"evaluation-result/{request_payload['run_id']}",
            "scope": "run",
        },
        "payload": payload,
    }
    validate_shared_document(message)
    return message


def _validate_report_identity(request: dict[str, Any], report: dict[str, Any]) -> None:
    payload = request["payload"]
    experiment = report.get("experiment") or {}
    expected = {
        "run_id": payload["run_id"],
        "scenario_id": payload["scene_id"],
        "scene_version": payload["scene_version"],
        "algorithm_id": payload["algorithm"]["algorithm_id"],
        "algorithm_version": payload["algorithm"]["algorithm_version"],
        "odd_id": payload["odd"]["odd_id"],
        "seed": payload["seed"],
    }
    actual = {
        "run_id": report.get("run_id") or experiment.get("run_id"),
        "scenario_id": report.get("scenario_id"),
        "scene_version": experiment.get("scene_version"),
        "algorithm_id": experiment.get("algorithm_id"),
        "algorithm_version": experiment.get("algorithm_version"),
        "odd_id": experiment.get("odd_id"),
        "seed": experiment.get("seed"),
    }
    mismatched = [name for name in expected if actual[name] != expected[name]]
    if mismatched:
        raise EvaluationProtocolError(
            "report identity does not match request: " + ", ".join(mismatched)
        )


def _duration(started_at: str, finished_at: str) -> float:
    parse = lambda value: datetime.fromisoformat(value.replace("Z", "+00:00"))
    duration = (parse(finished_at) - parse(started_at)).total_seconds()
    if duration < 0:
        raise EvaluationProtocolError("finished_at precedes started_at")
    return duration


def _number_or_none(value: Any, *, integer: bool = False) -> int | float | None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    return int(value) if integer else float(value)
