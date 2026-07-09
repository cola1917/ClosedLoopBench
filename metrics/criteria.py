from __future__ import annotations

from collections.abc import Mapping
from typing import Any


DEFAULT_CRITERIA = [
    {"name": "collision_count", "metric": "collision_count", "op": "==", "value": 0},
    {"name": "route_progress", "metric": "route_progress", "op": ">=", "value": 0.95},
    {"name": "min_ttc", "metric": "min_ttc", "op": ">=", "value": 1.0},
]

PASS = "pass"
FAIL = "fail"
UNKNOWN = "unknown"


def evaluate_report(
    run_config: dict[str, Any],
    summary: dict[str, Any],
    metric_rows: list[dict[str, Any]],
    status: str,
) -> dict[str, Any]:
    criteria = [
        _evaluate_criterion(criterion, summary, metric_rows, status)
        for criterion in _criteria_from_run_config(run_config)
    ]
    return {
        "overall_result": _overall_result(criteria),
        "criteria": criteria,
    }


def _criteria_from_run_config(run_config: dict[str, Any]) -> list[dict[str, Any]]:
    config = run_config.get("evaluation") or {}
    criteria = config.get("criteria") if isinstance(config, Mapping) else None
    if criteria is None:
        metrics_config = run_config.get("metrics")
        if isinstance(metrics_config, Mapping):
            criteria = metrics_config.get("criteria") or metrics_config.get("thresholds")

    if criteria is None:
        return [dict(criterion) for criterion in DEFAULT_CRITERIA]
    if isinstance(criteria, Mapping):
        return _criteria_from_mapping(criteria)
    if isinstance(criteria, list):
        return [_normalize_criterion(criterion) for criterion in criteria]
    raise ValueError("evaluation criteria must be a list or mapping")


def _criteria_from_mapping(criteria: Mapping[str, Any]) -> list[dict[str, Any]]:
    normalized: list[dict[str, Any]] = []
    for metric, config in criteria.items():
        if isinstance(config, Mapping):
            criterion = {"metric": metric, **dict(config)}
        else:
            criterion = {"metric": metric, "value": config}
        normalized.append(_normalize_criterion(criterion))
    return normalized


def _normalize_criterion(criterion: Any) -> dict[str, Any]:
    if not isinstance(criterion, Mapping):
        raise ValueError("each evaluation criterion must be a mapping")
    normalized = dict(criterion)
    metric = normalized.get("metric") or normalized.get("name")
    if not metric:
        raise ValueError("evaluation criterion requires a metric")
    normalized["metric"] = str(metric)
    normalized.setdefault("name", str(metric))
    normalized.setdefault("op", _default_operator(str(metric)))
    if "value" not in normalized:
        raise ValueError(f"evaluation criterion {normalized['name']} requires a value")
    return normalized


def _evaluate_criterion(
    criterion: dict[str, Any],
    summary: dict[str, Any],
    metric_rows: list[dict[str, Any]],
    status: str,
) -> dict[str, Any]:
    metric = criterion["metric"]
    actual = summary.get(metric)
    result = _criterion_result(
        metric,
        actual,
        criterion["op"],
        criterion["value"],
        metric_rows,
        status,
    )
    evaluated = {
        "name": criterion["name"],
        "metric": metric,
        "op": criterion["op"],
        "expected": criterion["value"],
        "actual": actual,
        "result": result,
    }
    if result == UNKNOWN:
        evaluated["reason"] = _unknown_reason(metric, status)
    return evaluated


def _criterion_result(
    metric: str,
    actual: Any,
    op: str,
    expected: Any,
    metric_rows: list[dict[str, Any]],
    status: str,
) -> str:
    if not _is_metric_known(metric, actual, metric_rows, status):
        return UNKNOWN
    if not isinstance(actual, (int, float)) or not isinstance(expected, (int, float)):
        return UNKNOWN
    return PASS if _compare(float(actual), op, float(expected)) else FAIL


def _is_metric_known(
    metric: str,
    actual: Any,
    metric_rows: list[dict[str, Any]],
    status: str,
) -> bool:
    if actual is None:
        return False
    if metric == "route_progress":
        return any(isinstance(row.get("route_progress"), (int, float)) for row in metric_rows)
    if metric == "min_ttc":
        return any(
            isinstance(row.get("min_ttc"), (int, float)) or isinstance(row.get("ttc"), (int, float))
            for row in metric_rows
        )
    if status == "not_run" and metric not in {"collision_count"}:
        return False
    return True


def _compare(actual: float, op: str, expected: float) -> bool:
    if op == "==":
        return actual == expected
    if op == "!=":
        return actual != expected
    if op == ">=":
        return actual >= expected
    if op == ">":
        return actual > expected
    if op == "<=":
        return actual <= expected
    if op == "<":
        return actual < expected
    raise ValueError(f"unknown evaluation criterion operator: {op}")


def _overall_result(criteria: list[dict[str, Any]]) -> str:
    results = [criterion["result"] for criterion in criteria]
    if any(result == FAIL for result in results):
        return FAIL
    if results and all(result == PASS for result in results):
        return PASS
    return UNKNOWN


def _default_operator(metric: str) -> str:
    return "==" if metric == "collision_count" else ">="


def _unknown_reason(metric: str, status: str) -> str:
    if status == "not_run" and metric != "collision_count":
        return "run has not produced metric samples"
    return "metric is missing or not sampled"
