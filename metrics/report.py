from __future__ import annotations

from collections import Counter
from copy import deepcopy
from typing import Any


SCHEMA_VERSION = "closed_loop_report.mvp.v0"
RUNTIME_STATUSES = {
    "not_run",
    "planned",
    "ego_closed_loop",
    "interactive_closed_loop",
    "failed",
    "completed",
}


def build_closed_loop_report(
    run_config: dict[str, Any],
    tick_metrics: list[dict[str, Any]] | None = None,
    status: str = "not_run",
) -> dict[str, Any]:
    """Build the MVP closed-loop report from a CARLA run config and optional tick metrics."""

    if status not in RUNTIME_STATUSES:
        raise ValueError(f"unknown closed-loop report status: {status}")

    metric_rows = list(tick_metrics or [])
    return {
        "schema_version": SCHEMA_VERSION,
        "scenario_id": run_config["scenario_id"],
        "status": status,
        "summary": {
            "collision_count": _collision_count(metric_rows),
            "min_ttc": _min_numeric(metric_rows, ("min_ttc", "ttc")),
            "route_progress": _route_progress(metric_rows),
            "hard_brake_count": _hard_brake_count(metric_rows),
            "max_jerk": _max_abs_numeric(metric_rows, ("max_jerk", "jerk")),
            "actor_policy_modes": _actor_policy_modes(run_config),
            "actor_closed_loop_levels": _actor_closed_loop_levels(run_config),
        },
        "metrics": deepcopy(metric_rows),
        "artifacts": _artifacts(run_config),
    }


def _collision_count(metric_rows: list[dict[str, Any]]) -> int:
    count = 0
    for row in metric_rows:
        if isinstance(row.get("collision_count"), (int, float)):
            count += int(row["collision_count"])
        elif row.get("collision") is True:
            count += 1
    return count


def _min_numeric(metric_rows: list[dict[str, Any]], keys: tuple[str, ...]) -> float | None:
    values: list[float] = []
    for row in metric_rows:
        for key in keys:
            value = row.get(key)
            if isinstance(value, (int, float)):
                values.append(float(value))
                break
    return min(values) if values else None


def _route_progress(metric_rows: list[dict[str, Any]]) -> float:
    values = [
        float(row["route_progress"])
        for row in metric_rows
        if isinstance(row.get("route_progress"), (int, float))
    ]
    return max(values) if values else 0.0


def _hard_brake_count(metric_rows: list[dict[str, Any]]) -> int:
    count = 0
    for row in metric_rows:
        if isinstance(row.get("hard_brake_count"), (int, float)):
            count += int(row["hard_brake_count"])
        elif row.get("hard_brake") is True:
            count += 1
    return count


def _max_abs_numeric(metric_rows: list[dict[str, Any]], keys: tuple[str, ...]) -> float | None:
    values: list[float] = []
    for row in metric_rows:
        for key in keys:
            value = row.get(key)
            if isinstance(value, (int, float)):
                values.append(abs(float(value)))
                break
    return max(values) if values else None


def _actor_policy_modes(run_config: dict[str, Any]) -> dict[str, int]:
    policies = Counter(
        actor.get("policy", "unknown")
        for actor in run_config.get("actors", [])
    )
    return dict(policies)


def _actor_closed_loop_levels(run_config: dict[str, Any]) -> dict[str, int]:
    levels = Counter(
        actor.get("closed_loop_level", "unknown")
        for actor in run_config.get("actors", [])
    )
    return dict(levels)


def _artifacts(run_config: dict[str, Any]) -> dict[str, str]:
    artifacts = {"run_config": "in_memory"}
    reconstruction_package = run_config.get("reconstruction_package") or {}
    if reconstruction_package.get("enabled") and reconstruction_package.get("package_path"):
        artifacts["reconstruction_package"] = str(reconstruction_package["package_path"])
    return artifacts
