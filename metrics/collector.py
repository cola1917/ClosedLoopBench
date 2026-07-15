from __future__ import annotations

from copy import deepcopy
from typing import Any


def build_tick_row(
    *,
    t_sec: float,
    ego_pose: dict[str, float],
    ego_speed_mps: float,
    ego_control: dict[str, float],
    actor_distances_m: dict[str, float],
    ttc: float | None,
    collision: bool | None,
    route_progress: float,
    hard_brake: bool,
    jerk: float | None,
    longitudinal_acceleration_mps2: float | None = None,
    actor_decisions: dict[str, dict[str, Any]] | None = None,
    actor_control_evidence: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Build the minimal per-tick metric row shared by CARLA and dry-run tests."""

    return {
        "t_sec": float(t_sec),
        "ego": {
            "pose": deepcopy(ego_pose),
            "speed_mps": float(ego_speed_mps),
            "control": deepcopy(ego_control),
        },
        "actor_distances_m": {
            actor_id: float(distance_m)
            for actor_id, distance_m in actor_distances_m.items()
        },
        "actor_decisions": deepcopy(actor_decisions or {}),
        "actor_control_evidence": deepcopy(actor_control_evidence or {}),
        "ttc": float(ttc) if isinstance(ttc, (int, float)) else None,
        "min_ttc": float(ttc) if isinstance(ttc, (int, float)) else None,
        "collision": bool(collision) if isinstance(collision, bool) else None,
        "route_progress": float(route_progress),
        "hard_brake": bool(hard_brake),
        "longitudinal_acceleration_mps2": (
            float(longitudinal_acceleration_mps2)
            if isinstance(longitudinal_acceleration_mps2, (int, float))
            else None
        ),
        "jerk": float(jerk) if isinstance(jerk, (int, float)) else None,
    }


class TickMetricCollector:
    """Collect runtime tick rows without owning CARLA sensor callbacks."""

    def __init__(self) -> None:
        self._rows: list[dict[str, Any]] = []

    def add_tick(
        self,
        *,
        t_sec: float,
        ego_pose: dict[str, float],
        ego_speed_mps: float,
        ego_control: dict[str, float],
        actor_distances_m: dict[str, float],
        ttc: float | None,
        collision: bool | None,
        route_progress: float,
        hard_brake: bool,
        jerk: float | None,
        longitudinal_acceleration_mps2: float | None = None,
        actor_decisions: dict[str, dict[str, Any]] | None = None,
        actor_control_evidence: dict[str, str] | None = None,
    ) -> dict[str, Any]:
        row = build_tick_row(
            t_sec=t_sec,
            ego_pose=ego_pose,
            ego_speed_mps=ego_speed_mps,
            ego_control=ego_control,
            actor_distances_m=actor_distances_m,
            actor_decisions=actor_decisions,
            actor_control_evidence=actor_control_evidence,
            ttc=ttc,
            collision=collision,
            route_progress=route_progress,
            hard_brake=hard_brake,
            longitudinal_acceleration_mps2=longitudinal_acceleration_mps2,
            jerk=jerk,
        )
        self._rows.append(row)
        return deepcopy(row)

    def to_report_rows(self) -> list[dict[str, Any]]:
        return deepcopy(self._rows)
