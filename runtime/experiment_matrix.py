from __future__ import annotations

import re
from copy import deepcopy
from datetime import datetime, timezone
from typing import Any

from adapters.shared_protocol_validation import (
    validate_artifact_reference,
    validate_shared_document,
)


MATRIX_SCHEMA_VERSION = "closed_loop_experiment_matrix.v0"
PLAN_SCHEMA_VERSION = "closed_loop_experiment_plan.v0"
_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_SUCCESS_STATUSES = {"completed", "ego_closed_loop", "interactive_closed_loop"}
_REQUIRED_SUMMARY = (
    "collision_count",
    "min_ttc",
    "route_progress",
    "hard_brake_count",
    "max_jerk",
)


class ExperimentMatrixError(ValueError):
    """Raised when an experiment matrix is ambiguous or incomparable."""


def build_experiment_plan(
    matrix: dict[str, Any],
    *,
    created_at: str | None = None,
) -> dict[str, Any]:
    _validate_matrix(matrix)
    timestamp = created_at or datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    runs = []
    for scene in matrix["scenes"]:
        for algorithm in matrix["algorithms"]:
            for odd in matrix["odds"]:
                for seed in matrix["seeds"]:
                    run_id = _run_id(scene, algorithm, odd, seed)
                    request = _request(
                        matrix,
                        scene,
                        algorithm,
                        odd,
                        seed,
                        run_id,
                        timestamp,
                    )
                    validate_shared_document(request)
                    runs.append(
                        {
                            "run_id": run_id,
                            "key": list(_run_key_from_request(request)),
                            "request": request,
                        }
                    )
    return {
        "schema_version": PLAN_SCHEMA_VERSION,
        "matrix_id": matrix["matrix_id"],
        "created_at": timestamp,
        "expected_run_count": len(runs),
        "dimensions": {
            "scenes": len(matrix["scenes"]),
            "algorithms": len(matrix["algorithms"]),
            "odds": len(matrix["odds"]),
            "seeds": len(matrix["seeds"]),
        },
        "runs": runs,
    }


def evaluate_experiment_coverage(
    plan: dict[str, Any],
    reports: list[dict[str, Any]],
) -> dict[str, Any]:
    if plan.get("schema_version") != PLAN_SCHEMA_VERSION:
        raise ExperimentMatrixError("unsupported experiment plan schema")
    expected = {tuple(run["key"]): run for run in plan.get("runs", [])}
    observed: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    malformed = []
    for index, report in enumerate(reports):
        try:
            key = _run_key_from_report(report)
        except ExperimentMatrixError as exc:
            malformed.append({"index": index, "reason": str(exc)})
            continue
        observed.setdefault(key, []).append(report)

    missing = [list(key) for key in expected if key not in observed]
    unexpected = [list(key) for key in observed if key not in expected]
    duplicates = [list(key) for key, rows in observed.items() if len(rows) > 1]
    invalid_runs = []
    for key, run in expected.items():
        rows = observed.get(key, [])
        if len(rows) != 1:
            continue
        report = rows[0]
        reasons = _report_reasons(run["request"], report)
        if reasons:
            invalid_runs.append({"key": list(key), "reasons": reasons})

    matched = sum(1 for key in expected if len(observed.get(key, [])) == 1)
    total = len(expected)
    ready = not any((missing, unexpected, duplicates, malformed, invalid_runs))
    return {
        "schema_version": "closed_loop_experiment_coverage.v0",
        "matrix_id": plan.get("matrix_id"),
        "expected_run_count": total,
        "observed_report_count": len(reports),
        "matched_run_count": matched,
        "coverage_ratio": matched / total if total else 0.0,
        "ready_for_comparison": ready,
        "missing": missing,
        "unexpected": unexpected,
        "duplicates": duplicates,
        "malformed": malformed,
        "invalid_runs": invalid_runs,
    }


def _validate_matrix(matrix: dict[str, Any]) -> None:
    if matrix.get("schema_version") != MATRIX_SCHEMA_VERSION:
        raise ExperimentMatrixError("unsupported experiment matrix schema")
    _safe(matrix.get("matrix_id"), "matrix_id")
    for name in ("scenes", "algorithms", "odds", "seeds", "metrics"):
        if not isinstance(matrix.get(name), list) or not matrix[name]:
            raise ExperimentMatrixError(f"{name} must be a non-empty list")
    _unique(matrix["scenes"], lambda item: (item.get("scene_id"), item.get("scene_version")), "scene")
    _unique(matrix["algorithms"], lambda item: item.get("algorithm_id"), "algorithm_id")
    _unique(matrix["odds"], lambda item: item.get("odd_id"), "odd_id")
    if len(matrix["seeds"]) != len(set(matrix["seeds"])):
        raise ExperimentMatrixError("seeds must be unique")
    if any(not isinstance(seed, int) or isinstance(seed, bool) or seed < 0 for seed in matrix["seeds"]):
        raise ExperimentMatrixError("seeds must be non-negative integers")
    if len(matrix["metrics"]) != len(set(matrix["metrics"])):
        raise ExperimentMatrixError("metrics must be unique")
    for metric in matrix["metrics"]:
        _safe(metric, "metric")
    for scene in matrix["scenes"]:
        _safe(scene.get("scene_id"), "scene_id")
        _safe(scene.get("scene_version"), "scene_version")
        _safe(scene.get("correlation_id"), "correlation_id")
        _safe(scene.get("root_message_id"), "root_message_id")
        validate_artifact_reference(scene.get("scene_package") or {})
    for algorithm in matrix["algorithms"]:
        _safe(algorithm.get("algorithm_id"), "algorithm_id")
        if not str(algorithm.get("algorithm_version") or ""):
            raise ExperimentMatrixError("algorithm_version is required")
        if algorithm.get("driver") not in {"basic_agent", "ros2", "external_plugin"}:
            raise ExperimentMatrixError("unsupported algorithm driver")
    for odd in matrix["odds"]:
        _safe(odd.get("odd_id"), "odd_id")
    simulator = matrix.get("simulator") or {}
    if simulator.get("name") != "carla" or simulator.get("synchronous_mode") is not True:
        raise ExperimentMatrixError("matrix requires synchronous CARLA")
    delta = simulator.get("fixed_delta_seconds")
    if not isinstance(delta, (int, float)) or isinstance(delta, bool) or not 0 < delta <= 1:
        raise ExperimentMatrixError("fixed_delta_seconds must be in (0, 1]")
    actor = matrix.get("actor_control") or {}
    if actor.get("mode") not in {"replay", "scripted", "traffic_manager", "mixed"}:
        raise ExperimentMatrixError("unsupported actor control mode")
    if actor.get("style") not in {None, "cautious", "normal", "aggressive"}:
        raise ExperimentMatrixError("unsupported actor style")
    timeout = matrix.get("timeout_sec")
    if not isinstance(timeout, (int, float)) or isinstance(timeout, bool) or timeout <= 0:
        raise ExperimentMatrixError("timeout_sec must be positive")


def _request(matrix, scene, algorithm, odd, seed, run_id, created_at):
    payload = {
        "run_id": run_id,
        "scene_id": scene["scene_id"],
        "scene_version": scene["scene_version"],
        "scene_package": deepcopy(scene["scene_package"]),
        "algorithm": {
            "algorithm_id": algorithm["algorithm_id"],
            "algorithm_version": algorithm["algorithm_version"],
            "driver": algorithm["driver"],
        },
        "odd": {"odd_id": odd["odd_id"]},
        "simulator": deepcopy(matrix["simulator"]),
        "actor_control": deepcopy(matrix["actor_control"]),
        "metrics": list(matrix["metrics"]),
        "seed": seed,
        "timeout_sec": matrix["timeout_sec"],
        "output_prefix": f"runs/{run_id}",
    }
    for field in ("checkpoint", "parameters"):
        if field in algorithm:
            payload["algorithm"][field] = deepcopy(algorithm[field])
    for field in ("weather", "parameters"):
        if field in odd:
            payload["odd"][field] = deepcopy(odd[field])
    return {
        "protocol_version": "shared_exchange_protocol.v1",
        "schema_version": "evaluation_run_request.v1",
        "message_id": f"msg-{run_id}",
        "message_type": "evaluation.run.request",
        "created_at": created_at,
        "producer": deepcopy(matrix["producer"]),
        "correlation": {
            "correlation_id": scene["correlation_id"],
            "root_message_id": scene["root_message_id"],
            "causation_message_id": scene["scene_result_message_id"],
        },
        "idempotency": {
            "key": f"evaluation/{scene['scene_id']}/{scene['scene_version']}/{algorithm['algorithm_id']}/{odd['odd_id']}/seed-{seed}",
            "scope": "run",
        },
        "payload": payload,
    }


def _run_id(scene, algorithm, odd, seed):
    value = "run-{}-{}-{}-s{}".format(
        scene["scene_id"], algorithm["algorithm_id"], odd["odd_id"], seed
    )
    return _safe(value, "run_id")


def _safe(value, label):
    normalized = str(value or "")
    if not _SAFE_ID.fullmatch(normalized):
        raise ExperimentMatrixError(f"invalid {label}: {normalized!r}")
    return normalized


def _unique(rows, key, label):
    values = [key(row) for row in rows]
    if len(values) != len(set(values)):
        raise ExperimentMatrixError(f"duplicate {label}")


def _run_key_from_request(request):
    payload = request["payload"]
    return (
        payload["scene_id"],
        payload["scene_version"],
        payload["algorithm"]["algorithm_id"],
        payload["algorithm"]["algorithm_version"],
        payload["odd"]["odd_id"],
        payload["seed"],
    )


def _run_key_from_report(report):
    experiment = report.get("experiment") or {}
    required = {
        "scenario_id": report.get("scenario_id"),
        "scene_version": experiment.get("scene_version"),
        "algorithm_id": experiment.get("algorithm_id"),
        "algorithm_version": experiment.get("algorithm_version"),
        "odd_id": experiment.get("odd_id"),
        "seed": experiment.get("seed"),
    }
    missing = [name for name, value in required.items() if value is None]
    if missing:
        raise ExperimentMatrixError("report identity is missing: " + ", ".join(missing))
    return tuple(required.values())


def _report_reasons(request, report):
    reasons = []
    status = report.get("status")
    if status not in _SUCCESS_STATUSES:
        reasons.append(f"runtime status is not successful: {status!r}")
    mode = request["payload"]["actor_control"]["mode"]
    if mode in {"scripted", "traffic_manager", "mixed"} and status != "interactive_closed_loop":
        reasons.append("interactive actor mode lacks interactive_closed_loop evidence")
    summary = report.get("summary") or {}
    for metric in _REQUIRED_SUMMARY:
        if summary.get(metric) is None:
            reasons.append(f"required summary metric is unknown: {metric}")
    evaluation = report.get("evaluation") or {}
    if evaluation.get("overall_result") not in {"pass", "fail"}:
        reasons.append("overall_result is unknown")
    return reasons
