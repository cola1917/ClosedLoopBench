from __future__ import annotations

import math
from copy import deepcopy
from typing import Any, Mapping


PROFILES = {
    "short": {"lookahead_m": 4.0, "target_speed_mps": 6.0},
    "long": {"lookahead_m": 9.0, "target_speed_mps": 8.0},
}


class ReferencePurePursuitPlugin:
    """Deterministic geometry-only baseline; it is not a perception algorithm."""

    def __init__(self) -> None:
        self.capability = {
            "algorithm_id": "reference_pure_pursuit",
            "uses_route": True,
            "uses_ego_state": True,
            "required_rgb_cameras": [],
            "requires_lidar": False,
            "is_perception_algorithm": False,
            "requires_gpu": False,
            "checkpoint_identity": "not_applicable",
            "supported_control_hz": 20.0,
            "timeout_sec": 0.05,
        }
        self._initialized = False
        self._closed = False
        self._route_index = 0
        self._scene_context: dict[str, Any] = {}
        self._diagnostics: list[dict[str, Any]] = []

    def initialize(self, config: Mapping[str, Any]) -> None:
        profile = str(config.get("profile", "short"))
        if profile not in PROFILES:
            raise ValueError(f"unknown pure-pursuit profile: {profile}")
        defaults = PROFILES[profile]
        self.profile = profile
        self.lookahead_m = _positive(config.get("lookahead_m", defaults["lookahead_m"]), "lookahead_m")
        self.target_speed_mps = _nonnegative(
            config.get("target_speed_mps", defaults["target_speed_mps"]), "target_speed_mps"
        )
        self.wheelbase_m = _positive(config.get("wheelbase_m", 2.875), "wheelbase_m")
        self.max_steer_rad = _positive(config.get("max_steer_rad", 0.7), "max_steer_rad")
        self.speed_kp = _positive(config.get("speed_kp", 0.35), "speed_kp")
        self.brake_kp = _positive(config.get("brake_kp", 0.5), "brake_kp")
        self.waypoint_reached_m = _positive(
            config.get("waypoint_reached_m", 1.5), "waypoint_reached_m"
        )
        self.capability["supported_control_hz"] = _positive(
            config.get("supported_control_hz", 20.0), "supported_control_hz"
        )
        self.capability["timeout_sec"] = _positive(
            config.get("timeout_sec", 0.05), "timeout_sec"
        )
        self._initialized = True
        self._closed = False

    def reset(self, scene_context: Mapping[str, Any]) -> None:
        if not self._initialized or self._closed:
            raise RuntimeError("pure-pursuit plugin is not initialized")
        self._route_index = 0
        self._diagnostics = []
        self._scene_context = deepcopy(dict(scene_context))

    def predict_control(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        if not self._initialized or self._closed:
            raise RuntimeError("pure-pursuit plugin is not ready")
        frame_id = int(observation["frame_id"])
        ego_state = observation["ego_state"]
        pose = ego_state.get("pose", {})
        ego_x = _finite(pose.get("x"), "ego pose x")
        ego_y = _finite(pose.get("y"), "ego pose y")
        yaw_rad = _yaw_radians(pose)
        speed_mps = _nonnegative(ego_state.get("speed_mps"), "speed_mps")
        waypoints = _waypoints(observation["route"])

        nearest_index = min(
            range(self._route_index, len(waypoints)),
            key=lambda index: _distance(ego_x, ego_y, *waypoints[index]),
        )
        self._route_index = max(self._route_index, nearest_index)
        while (
            self._route_index < len(waypoints) - 1
            and _distance(ego_x, ego_y, *waypoints[self._route_index]) <= self.waypoint_reached_m
        ):
            self._route_index += 1

        target_index = self._route_index
        for index in range(self._route_index, len(waypoints)):
            target_index = index
            if _distance(ego_x, ego_y, *waypoints[index]) >= self.lookahead_m:
                break
        target_x, target_y = waypoints[target_index]
        target_distance = max(_distance(ego_x, ego_y, target_x, target_y), 1e-6)
        heading = math.atan2(target_y - ego_y, target_x - ego_x)
        alpha = _wrap_angle(heading - yaw_rad)
        curvature = 2.0 * math.sin(alpha) / target_distance
        steer_rad = math.atan(self.wheelbase_m * curvature)
        steer = max(-1.0, min(1.0, steer_rad / self.max_steer_rad))

        speed_error = self.target_speed_mps - speed_mps
        throttle = max(0.0, min(1.0, self.speed_kp * speed_error))
        brake = max(0.0, min(1.0, self.brake_kp * -speed_error))
        if brake > 0.0:
            throttle = 0.0

        progress = target_index / max(1, len(waypoints) - 1)
        diagnostic = {
            "frame_id": frame_id,
            "profile": self.profile,
            "route_index": self._route_index,
            "target_index": target_index,
            "route_progress": progress,
            "lookahead_m": self.lookahead_m,
            "target_distance_m": target_distance,
            "heading_error_rad": alpha,
            "speed_error_mps": speed_error,
        }
        self._diagnostics.append(deepcopy(diagnostic))
        return {
            "throttle": throttle,
            "steer": steer,
            "brake": brake,
            "hand_brake": False,
            "reverse": False,
            "source_frame_id": frame_id,
            "inference_ms": 0.0,
            "status": "ok",
            "reason": None,
            "diagnostics": diagnostic,
        }

    def health_check(self) -> dict[str, Any]:
        return {
            "status": "ready" if self._initialized and not self._closed else "not_ready",
            "is_perception_algorithm": False,
            "profile": getattr(self, "profile", None),
        }

    def close(self) -> None:
        self._closed = True
        self._scene_context = {}

    @property
    def diagnostics(self) -> list[dict[str, Any]]:
        return deepcopy(self._diagnostics)


def create_plugin(config: Mapping[str, Any]) -> ReferencePurePursuitPlugin:
    plugin = ReferencePurePursuitPlugin()
    plugin.initialize(config)
    return plugin


def _waypoints(route: Mapping[str, Any]) -> list[tuple[float, float]]:
    raw = route.get("route_waypoints")
    if not isinstance(raw, list) or not raw:
        raise ValueError("route_waypoints must be a non-empty list")
    result = []
    for waypoint in raw:
        if isinstance(waypoint, Mapping):
            x, y = waypoint.get("x"), waypoint.get("y")
        elif isinstance(waypoint, (list, tuple)) and len(waypoint) >= 2:
            x, y = waypoint[:2]
        else:
            raise ValueError("route waypoint must contain x/y")
        result.append((_finite(x, "waypoint x"), _finite(y, "waypoint y")))
    return result


def _yaw_radians(pose: Mapping[str, Any]) -> float:
    if "yaw_rad" in pose:
        return _finite(pose["yaw_rad"], "yaw_rad")
    return math.radians(_finite(pose.get("yaw", 0.0), "yaw"))


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _distance(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(x1 - x0, y1 - y0)


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _positive(value: Any, name: str) -> float:
    result = _finite(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative(value: Any, name: str) -> float:
    result = _finite(value, name)
    if result < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return result
