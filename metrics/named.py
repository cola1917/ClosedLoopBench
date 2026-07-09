from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from math import inf
from typing import Any


def time_to_collision(distance_m: float, closing_speed_mps: float) -> float | None:
    """Return TTC in seconds when objects are closing, otherwise None."""

    if closing_speed_mps <= 0:
        return None
    if distance_m <= 0:
        return 0.0
    return float(distance_m) / float(closing_speed_mps)


def drac(distance_m: float, closing_speed_mps: float) -> float:
    """Return deceleration rate to avoid crash in m/s^2."""

    if closing_speed_mps <= 0:
        return 0.0
    if distance_m <= 0:
        return inf
    return float(closing_speed_mps) ** 2 / (2.0 * float(distance_m))


def time_exposed_ttc(
    ticks: Iterable[float | None | Mapping[str, Any]],
    threshold_s: float,
    dt_s: float | None = None,
) -> float:
    """Return total time where TTC is finite and below threshold."""

    return _integrate_ttc_deficit(ticks, threshold_s, dt_s, deficit=False)


def time_integrated_ttc(
    ticks: Iterable[float | None | Mapping[str, Any]],
    threshold_s: float,
    dt_s: float | None = None,
) -> float:
    """Return integral of threshold - TTC where TTC is below threshold."""

    return _integrate_ttc_deficit(ticks, threshold_s, dt_s, deficit=True)


def jerk_from_acceleration(acceleration_mps2: Sequence[float], dt_s: float) -> list[float]:
    """Return finite-difference jerk values from longitudinal acceleration."""

    if dt_s <= 0:
        raise ValueError("dt_s must be positive")
    return [
        (float(curr) - float(prev)) / float(dt_s)
        for prev, curr in zip(acceleration_mps2, acceleration_mps2[1:])
    ]


def hard_brake_count(
    longitudinal_acceleration_mps2: Iterable[float],
    threshold_mps2: float = -3.0,
) -> int:
    """Count ticks at or below the hard-braking acceleration threshold."""

    return sum(
        1
        for acceleration in longitudinal_acceleration_mps2
        if float(acceleration) <= threshold_mps2
    )


def _integrate_ttc_deficit(
    ticks: Iterable[float | None | Mapping[str, Any]],
    threshold_s: float,
    dt_s: float | None,
    deficit: bool,
) -> float:
    rows = list(ticks)
    if dt_s is not None and dt_s <= 0:
        raise ValueError("dt_s must be positive")

    total = 0.0
    for index, tick in enumerate(rows):
        ttc = _ttc_value(tick)
        if ttc is None or ttc >= threshold_s:
            continue
        duration = dt_s if dt_s is not None else _duration_from_timestamps(rows, index)
        total += (threshold_s - ttc) * duration if deficit else duration
    return total


def _ttc_value(tick: float | None | Mapping[str, Any]) -> float | None:
    if tick is None:
        return None
    if isinstance(tick, Mapping):
        value = tick.get("min_ttc", tick.get("ttc"))
    else:
        value = tick
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _duration_from_timestamps(
    rows: list[float | None | Mapping[str, Any]],
    index: int,
) -> float:
    if index + 1 >= len(rows):
        return 0.0

    current = rows[index]
    next_tick = rows[index + 1]
    if not isinstance(current, Mapping) or not isinstance(next_tick, Mapping):
        return 0.0

    current_t = current.get("t_sec")
    next_t = next_tick.get("t_sec")
    if not isinstance(current_t, (int, float)) or not isinstance(next_t, (int, float)):
        return 0.0
    return max(0.0, float(next_t) - float(current_t))
