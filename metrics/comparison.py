from __future__ import annotations

from collections import defaultdict
from typing import Any, Iterable


_SUMMARY_METRICS = (
    "collision_count",
    "min_ttc",
    "route_progress",
    "hard_brake_count",
    "max_jerk",
    "control_timeout_count",
)


def compare_closed_loop_reports(reports: Iterable[dict[str, Any]]) -> dict[str, Any]:
    rows = list(reports)
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for report in rows:
        experiment = report.get("experiment") or {}
        algorithm_id = str(experiment.get("algorithm_id", "unknown"))
        groups[algorithm_id].append(report)

    return {
        "schema_version": "closed_loop_comparison.v0",
        "run_count": len(rows),
        "algorithms": {
            algorithm_id: _summarize_algorithm(algorithm_reports)
            for algorithm_id, algorithm_reports in sorted(groups.items())
        },
    }


def _summarize_algorithm(reports: list[dict[str, Any]]) -> dict[str, Any]:
    results = [str((report.get("evaluation") or {}).get("overall_result", "unknown")) for report in reports]
    return {
        "run_count": len(reports),
        "scenario_ids": sorted({str(report.get("scenario_id", "unknown")) for report in reports}),
        "odd_ids": sorted({str((report.get("experiment") or {}).get("odd_id", "default")) for report in reports}),
        "algorithm_versions": sorted(
            {
                str((report.get("experiment") or {}).get("algorithm_version"))
                for report in reports
                if (report.get("experiment") or {}).get("algorithm_version") is not None
            }
        ),
        "seeds": sorted(
            {
                int((report.get("experiment") or {})["seed"])
                for report in reports
                if isinstance((report.get("experiment") or {}).get("seed"), int)
            }
        ),
        "result_counts": {result: results.count(result) for result in sorted(set(results))},
        "mean": {
            metric: _mean(report.get("summary", {}).get(metric) for report in reports)
            for metric in _SUMMARY_METRICS
        },
    }


def _mean(values: Iterable[Any]) -> float | None:
    numeric = [float(value) for value in values if isinstance(value, (int, float))]
    return sum(numeric) / len(numeric) if numeric else None
