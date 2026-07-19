from __future__ import annotations

from collections import Counter
from copy import deepcopy
import math
from typing import Any

from metrics.criteria import evaluate_report


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
    jerk_values = _jerk_values(metric_rows)
    timeout_count = _count_boolean_or_numeric(metric_rows, "control_timeout", "control_timeout_count")
    fallback_count = _count_boolean_or_numeric(metric_rows, "control_fallback", "control_fallback_count")
    summary = {
        "collision_count": _collision_count(metric_rows),
        "min_ttc": _min_numeric(metric_rows, ("min_ttc", "ttc")),
        "route_progress": _route_progress(metric_rows),
        "route_completion_time_sec": _route_completion_time(metric_rows),
        "average_speed_mps": _average_ego_speed(metric_rows),
        "stopped_time_sec": _duration_for_flag(metric_rows, _is_stopped),
        "following_time_sec": _duration_for_flag(metric_rows, _is_following),
        "min_distance_m": _min_actor_distance(metric_rows),
        "min_pet_sec": _min_numeric(metric_rows, ("min_pet_sec", "pet_sec", "post_encroachment_time_sec")),
        "max_drac_mps2": _max_numeric(metric_rows, ("max_drac_mps2", "drac_mps2", "drac")),
        "hard_brake_count": _hard_brake_count(metric_rows),
        "max_jerk": max((abs(value) for value in jerk_values), default=None),
        "jerk_p50_mps3": _percentile([abs(value) for value in jerk_values], 50.0),
        "jerk_p95_mps3": _percentile([abs(value) for value in jerk_values], 95.0),
        "jerk_p99_mps3": _percentile([abs(value) for value in jerk_values], 99.0),
        "control_latency_p50_ms": _numeric_percentile(metric_rows, ("control_latency_ms", "inference_latency_ms"), 50.0),
        "control_latency_p95_ms": _numeric_percentile(metric_rows, ("control_latency_ms", "inference_latency_ms"), 95.0),
        "control_latency_p99_ms": _numeric_percentile(metric_rows, ("control_latency_ms", "inference_latency_ms"), 99.0),
        "control_timeout_count": timeout_count,
        "control_timeout_rate": _rate(timeout_count, metric_rows, ("control_timeout", "control_timeout_count")),
        "control_fallback_count": fallback_count,
        "control_fallback_rate": _rate(fallback_count, metric_rows, ("control_fallback", "control_fallback_count")),
        "actor_outcomes": _actor_outcomes(metric_rows),
        "sensor_dropped_frame_count": _sensor_dropped_frame_count(metric_rows),
        "max_synchronization_error_ms": _sync_percentile(metric_rows, 100.0),
        "synchronization_error_p95_ms": _sync_percentile(metric_rows, 95.0),
        "actor_policy_modes": _actor_policy_modes(run_config),
        "actor_closed_loop_levels": _actor_closed_loop_levels(run_config),
    }
    summary["metric_availability"] = _metric_availability(summary, metric_rows)
    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_config.get("run_id") or (run_config.get("experiment") or {}).get("run_id"),
        "scenario_id": run_config["scenario_id"],
        "status": status,
        "experiment": deepcopy(run_config.get("experiment") or {}),
        "summary": summary,
        "evaluation": evaluate_report(run_config, summary, metric_rows, status),
        "metrics": deepcopy(metric_rows),
        "artifacts": _artifacts(run_config),
    }


def _collision_count(metric_rows: list[dict[str, Any]]) -> int | None:
    if not metric_rows or not any(
        isinstance(row.get("collision"), bool)
        or isinstance(row.get("collision_count"), (int, float))
        for row in metric_rows
    ):
        return None
    count = 0
    in_event = False
    for row in metric_rows:
        if isinstance(row.get("collision_count"), (int, float)):
            count += int(row["collision_count"])
            in_event = False
        elif isinstance(row.get("collision"), bool):
            active = row["collision"] is True
            if active and not in_event:
                count += 1
            in_event = active
        else:
            in_event = False
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
    in_event = False
    for row in metric_rows:
        if isinstance(row.get("hard_brake_count"), (int, float)):
            count += int(row["hard_brake_count"])
            in_event = False
        elif isinstance(row.get("hard_brake"), bool):
            active = row["hard_brake"] is True
            if active and not in_event:
                count += 1
            in_event = active
        else:
            in_event = False
    return count


def _route_completion_time(metric_rows: list[dict[str, Any]], threshold: float = 0.95) -> float | None:
    timed = [row for row in metric_rows if _finite(row.get("t_sec"))]
    if not timed:
        return None
    start = float(timed[0]["t_sec"])
    for row in timed:
        if _finite(row.get("route_progress")) and float(row["route_progress"]) >= threshold:
            return max(0.0, float(row["t_sec"]) - start)
    return None


def _average_ego_speed(metric_rows: list[dict[str, Any]]) -> float | None:
    values = []
    for row in metric_rows:
        ego = row.get("ego") or {}
        value = ego.get("speed_mps") if isinstance(ego, dict) else None
        if value is None:
            value = row.get("ego_speed_mps")
        if _finite(value):
            values.append(float(value))
    return sum(values) / len(values) if values else None


def _is_stopped(row: dict[str, Any]) -> bool | None:
    ego = row.get("ego") or {}
    speed = ego.get("speed_mps") if isinstance(ego, dict) else None
    if speed is None:
        speed = row.get("ego_speed_mps")
    return abs(float(speed)) <= 0.1 if _finite(speed) else None


def _is_following(row: dict[str, Any]) -> bool | None:
    value = row.get("following")
    if isinstance(value, bool):
        return value
    state = row.get("interaction_state")
    return state == "following" if isinstance(state, str) else None


def _duration_for_flag(metric_rows, predicate) -> float | None:
    known = False
    total = 0.0
    for current, following in zip(metric_rows, metric_rows[1:]):
        active = predicate(current)
        if active is None or not _finite(current.get("t_sec")) or not _finite(following.get("t_sec")):
            continue
        known = True
        delta = float(following["t_sec"]) - float(current["t_sec"])
        if active and 0.0 < delta <= 1.0:
            total += delta
    return total if known else None


def _min_actor_distance(metric_rows: list[dict[str, Any]]) -> float | None:
    values = []
    for row in metric_rows:
        if _finite(row.get("min_distance_m")):
            values.append(float(row["min_distance_m"]))
        distances = row.get("actor_distances_m")
        if isinstance(distances, dict):
            values.extend(float(value) for value in distances.values() if _finite(value))
    return min(values) if values else None


def _max_numeric(metric_rows: list[dict[str, Any]], keys: tuple[str, ...]) -> float | None:
    values = []
    for row in metric_rows:
        for key in keys:
            if _finite(row.get(key)):
                values.append(float(row[key]))
                break
    return max(values) if values else None


def _jerk_values(metric_rows: list[dict[str, Any]]) -> list[float]:
    explicit = []
    previous_t = None
    for row in metric_rows:
        value = row.get("max_jerk") if _finite(row.get("max_jerk")) else row.get("jerk")
        timestamp = row.get("t_sec")
        if _finite(value):
            if previous_t is None or not _finite(timestamp) or float(timestamp) > previous_t:
                explicit.append(float(value))
        if _finite(timestamp):
            previous_t = float(timestamp)
    if explicit:
        # Long traces commonly contain artificial jerk at controller startup and
        # shutdown.  Preserve short unit/smoke traces, but trim those two boundary
        # samples for real runs before calculating max and percentiles.
        return explicit[1:-1] if len(explicit) >= 5 else explicit
    derived = []
    previous = None
    for row in metric_rows:
        acceleration = row.get("longitudinal_acceleration_mps2")
        timestamp = row.get("t_sec")
        if not (_finite(acceleration) and _finite(timestamp)):
            continue
        sample = (float(timestamp), float(acceleration))
        if previous is not None:
            delta = sample[0] - previous[0]
            if 1e-4 <= delta <= 1.0:
                derived.append((sample[1] - previous[1]) / delta)
        previous = sample
    return derived


def _numeric_percentile(metric_rows, keys, percentile):
    values = []
    for row in metric_rows:
        for key in keys:
            if _finite(row.get(key)):
                values.append(float(row[key]))
                break
    return _percentile(values, percentile)


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _count_boolean_or_numeric(metric_rows, bool_key, count_key):
    if not any(isinstance(row.get(bool_key), bool) or _finite(row.get(count_key)) for row in metric_rows):
        return None
    return sum(
        int(row[count_key]) if _finite(row.get(count_key)) else int(row.get(bool_key) is True)
        for row in metric_rows
    )


def _rate(count, metric_rows, keys):
    if count is None:
        return None
    samples = sum(1 for row in metric_rows if any(isinstance(row.get(key), bool) or _finite(row.get(key)) for key in keys))
    return float(count) / samples if samples else None


def _actor_outcomes(metric_rows):
    outcomes = Counter()
    known = False
    previous_by_actor = {}
    for row in metric_rows:
        payload = row.get("actor_outcomes")
        if isinstance(payload, dict):
            known = known or bool(payload)
            for actor_id, value in payload.items():
                normalized = str(value)
                if previous_by_actor.get(actor_id) != normalized:
                    outcomes[normalized] += 1
                previous_by_actor[actor_id] = normalized
        decisions = row.get("actor_decisions")
        if isinstance(decisions, dict):
            for actor_id, decision in decisions.items():
                outcome = decision.get("outcome") if isinstance(decision, dict) else None
                if outcome is None and isinstance(decision, dict):
                    action = decision.get("action")
                    if action in {"yield", "abort", "crossing"}:
                        outcome = action
                    elif decision.get("should_yield") is True:
                        outcome = "yield"
                    elif decision.get("should_abort") is True:
                        outcome = "abort"
                normalized = str(outcome) if outcome else None
                if normalized and previous_by_actor.get(actor_id) != normalized:
                    known = True
                    outcomes[normalized] += 1
                    previous_by_actor[actor_id] = normalized
    return dict(outcomes) if known else None


def _sum_numeric(metric_rows, keys, *, integer=False):
    values = []
    for row in metric_rows:
        for key in keys:
            if _finite(row.get(key)):
                values.append(float(row[key]))
                break
    if not values:
        return None
    total = sum(values)
    return int(total) if integer else total


def _sensor_dropped_frame_count(metric_rows):
    keys = (
        "sensor_dropped_frames",
        "dropped_frame_count",
        "dropped_camera_frames",
        "dropped_lidar_frames",
    )
    known = False
    total = 0
    for row in metric_rows:
        for key in keys:
            if _finite(row.get(key)):
                known = True
                total += int(row[key])
    return total if known else None


def _sync_percentile(metric_rows, percentile):
    values = []
    for row in metric_rows:
        if _finite(row.get("synchronization_error_ms")):
            values.append(abs(float(row["synchronization_error_ms"])))
        elif _finite(row.get("synchronization_error_us")):
            values.append(abs(float(row["synchronization_error_us"])) / 1000.0)
        elif _finite(row.get("max_sync_error_us")):
            values.append(abs(float(row["max_sync_error_us"])) / 1000.0)
    return _percentile(values, percentile)


def _metric_availability(summary, metric_rows):
    availability = {}
    for key, value in summary.items():
        if key in {"actor_policy_modes", "actor_closed_loop_levels", "metric_availability"}:
            continue
        if key == "collision_count":
            availability[key] = any(isinstance(row.get("collision"), bool) or _finite(row.get("collision_count")) for row in metric_rows)
        elif key == "route_progress":
            availability[key] = any(_finite(row.get("route_progress")) for row in metric_rows)
        elif key == "hard_brake_count":
            availability[key] = any(isinstance(row.get("hard_brake"), bool) or _finite(row.get("hard_brake_count")) for row in metric_rows)
        else:
            availability[key] = value is not None
    return availability


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


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
