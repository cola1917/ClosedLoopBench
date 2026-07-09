from __future__ import annotations

from math import inf
from typing import Any, Mapping

from actors.style_profiles import ActorStyleProfile


SAFE_DEFAULT_SPEED_MPS = 0.0


def plan_reactive_actor_control(
    actor_state: Mapping[str, Any],
    ego_state: Mapping[str, Any] | None,
    *,
    style: str = "normal",
    reference_speed_mps: float | None = None,
) -> dict[str, Any]:
    """Plan one explainable actor control decision from ego proximity.

    This intentionally returns a serializable decision instead of touching CARLA.
    Runners can translate it to TrafficManager parameters, VehicleControl, or a
    scripted maneuver controller.
    """
    profile = ActorStyleProfile.for_style(style)
    current_speed_mps = _float(actor_state.get("speed_mps"), SAFE_DEFAULT_SPEED_MPS)

    if not ego_state:
        fallback_speed = _fallback_speed(reference_speed_mps, actor_state)
        reason = (
            "no_ego_state_reference_fallback"
            if reference_speed_mps is not None
            else "no_ego_state_safe_default"
        )
        return _decision(
            profile=profile,
            desired_speed_mps=fallback_speed,
            brake=False,
            should_yield=False,
            should_abort=False,
            lane_change_enabled=_lane_change_enabled(profile),
            ttc_sec=None,
            distance_m=None,
            reason=reason,
        )

    distance_m = _ego_distance_m(actor_state, ego_state)
    relative_speed_mps = max(0.0, _float(ego_state.get("relative_speed_mps"), 0.0))
    ttc_sec = distance_m / relative_speed_mps if relative_speed_mps > 0.0 else inf

    gap_too_small = distance_m <= profile.min_gap_m
    ttc_too_low = ttc_sec <= profile.yield_ttc_threshold_sec
    should_yield = gap_too_small or ttc_too_low
    should_abort = bool(should_yield and profile.abort_on_low_ttc)
    desired_speed_mps = _yield_speed(current_speed_mps, profile) if should_yield else current_speed_mps

    return _decision(
        profile=profile,
        desired_speed_mps=desired_speed_mps,
        brake=should_yield,
        should_yield=should_yield,
        should_abort=should_abort,
        lane_change_enabled=(not should_yield) and _lane_change_enabled(profile),
        ttc_sec=ttc_sec,
        distance_m=distance_m,
        reason="ego_gap_or_ttc_reactive" if should_yield else "ego_state_within_style_gap",
    )


def _decision(
    *,
    profile: ActorStyleProfile,
    desired_speed_mps: float,
    brake: bool,
    should_yield: bool,
    should_abort: bool,
    lane_change_enabled: bool,
    ttc_sec: float | None,
    distance_m: float | None,
    reason: str,
) -> dict[str, Any]:
    return {
        "style": profile.name,
        "desired_speed_mps": float(max(0.0, desired_speed_mps)),
        "brake": bool(brake),
        "should_yield": bool(should_yield),
        "should_abort": bool(should_abort),
        "lane_change_enabled": bool(lane_change_enabled),
        "min_gap_m": float(profile.min_gap_m),
        "yield_ttc_threshold_sec": float(profile.yield_ttc_threshold_sec),
        "ttc_sec": None if ttc_sec is None else float(ttc_sec),
        "distance_m": None if distance_m is None else float(distance_m),
        "reason": reason,
    }


def _ego_distance_m(actor_state: Mapping[str, Any], ego_state: Mapping[str, Any]) -> float:
    if ego_state.get("distance_m") is not None:
        return max(0.0, _float(ego_state.get("distance_m"), 0.0))
    dx = _float(ego_state.get("x"), 0.0) - _float(actor_state.get("x"), 0.0)
    dy = _float(ego_state.get("y"), 0.0) - _float(actor_state.get("y"), 0.0)
    return max(0.0, (dx * dx + dy * dy) ** 0.5)


def _fallback_speed(reference_speed_mps: float | None, actor_state: Mapping[str, Any]) -> float:
    if reference_speed_mps is not None:
        return float(reference_speed_mps)
    if actor_state.get("reference_speed_mps") is not None:
        return _float(actor_state.get("reference_speed_mps"), SAFE_DEFAULT_SPEED_MPS)
    return SAFE_DEFAULT_SPEED_MPS


def _yield_speed(current_speed_mps: float, profile: ActorStyleProfile) -> float:
    if profile.name == "defensive":
        return min(current_speed_mps * 0.35, 2.0)
    if profile.name == "normal":
        return min(current_speed_mps * 0.5, 4.0)
    return min(current_speed_mps * 0.75, current_speed_mps)


def _lane_change_enabled(profile: ActorStyleProfile) -> bool:
    return profile.lane_change_gap_acceptance_m < 10.0


def _float(value: Any, default: float) -> float:
    if value is None:
        return float(default)
    return float(value)
