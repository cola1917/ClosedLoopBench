from __future__ import annotations

import math
from copy import deepcopy
from typing import Any


EGO_STATE_FIELDS = ("speed_mps", "pose", "velocity", "acceleration")
ROUTE_FIELDS = ("route_waypoints", "route_command", "target_point")

_CAMERA_TRANSFORMS = {
    "rgb_front": {"x": 1.5, "y": 0.0, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "rgb_front_left": {"x": 1.3, "y": -0.5, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": -55.0},
    "rgb_front_right": {"x": 1.3, "y": 0.5, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": 55.0},
    "rgb_back": {"x": -1.5, "y": 0.0, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": 180.0},
    "rgb_back_left": {"x": -1.3, "y": -0.5, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": -125.0},
    "rgb_back_right": {"x": -1.3, "y": 0.5, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": 125.0},
}


def build_camera_sensor_specs(
    camera_names: list[str] | tuple[str, ...],
    *,
    width: int = 900,
    height: int = 256,
    fov_deg: float = 100.0,
    sensor_tick_sec: float = 0.05,
) -> dict[str, dict[str, Any]]:
    """Build explicit CARLA blueprint and nominal mount specifications."""
    if width <= 0 or height <= 0 or not 1.0 < float(fov_deg) < 179.0:
        raise ValueError("camera width/height must be positive and fov_deg must be in (1, 179)")
    if sensor_tick_sec <= 0.0:
        raise ValueError("sensor_tick_sec must be positive")
    unknown = sorted(set(camera_names) - set(_CAMERA_TRANSFORMS))
    if unknown:
        raise ValueError(f"unknown CARLA camera roles: {unknown}")
    return {
        name: {
            "blueprint": "sensor.camera.rgb",
            "attributes": {
                "image_size_x": str(int(width)),
                "image_size_y": str(int(height)),
                "fov": str(float(fov_deg)),
                "sensor_tick": str(float(sensor_tick_sec)),
            },
            "ego_mount": deepcopy(_CAMERA_TRANSFORMS[name]),
            "runtime_calibration": {
                "required": True,
                "intrinsic_shape": [3, 3],
                "sensor_to_ego_shape": [4, 4],
                "source": "spawned_carla_sensor_and_transform",
            },
        }
        for name in camera_names
    }


def build_pinhole_calibration(
    *, width: int, height: int, fov_deg: float, sensor_to_ego: list[list[float]]
) -> dict[str, Any]:
    """Create a serializable calibration record from the spawned CARLA sensor."""
    if width <= 0 or height <= 0 or not 1.0 < float(fov_deg) < 179.0:
        raise ValueError("invalid pinhole camera dimensions or field of view")
    focal = float(width) / (2.0 * math.tan(math.radians(float(fov_deg)) / 2.0))
    calibration = {
        "width": int(width),
        "height": int(height),
        "fov_deg": float(fov_deg),
        "intrinsic": [
            [focal, 0.0, float(width) / 2.0],
            [0.0, focal, float(height) / 2.0],
            [0.0, 0.0, 1.0],
        ],
        "sensor_to_ego": deepcopy(sensor_to_ego),
        "coordinate_system": "carla_ego_frame",
    }
    error = validate_camera_calibration(calibration)
    if error:
        raise ValueError(error)
    return calibration


def validate_camera_calibration(calibration: Any) -> str | None:
    if not isinstance(calibration, dict):
        return "calibration must be an object"
    if not isinstance(calibration.get("width"), int) or calibration["width"] <= 0:
        return "calibration width must be a positive integer"
    if not isinstance(calibration.get("height"), int) or calibration["height"] <= 0:
        return "calibration height must be a positive integer"
    if not _finite_matrix(calibration.get("intrinsic"), 3, 3):
        return "calibration intrinsic must be a finite 3x3 matrix"
    if not _finite_matrix(calibration.get("sensor_to_ego"), 4, 4):
        return "calibration sensor_to_ego must be a finite 4x4 matrix"
    return None


class TickObservationAggregator:
    """Fail-closed aggregation of camera, ego state, and route for one CARLA tick."""

    def __init__(
        self,
        *,
        required_cameras: list[str] | tuple[str, ...],
        timeout_sec: float = 0.5,
        max_skew_sec: float = 0.001,
        require_calibration: bool = True,
    ) -> None:
        if not required_cameras:
            raise ValueError("at least one required camera is needed")
        if timeout_sec <= 0.0 or max_skew_sec < 0.0:
            raise ValueError("timeout_sec must be positive and max_skew_sec non-negative")
        self.required_cameras = tuple(str(name) for name in required_cameras)
        self.timeout_sec = float(timeout_sec)
        self.max_skew_sec = float(max_skew_sec)
        self.require_calibration = bool(require_calibration)
        self._channels: dict[str, dict[str, Any]] = {}

    def receive_camera(
        self,
        camera: str,
        data: Any,
        *,
        frame_id: int | str,
        t_sec: float,
        calibration: dict[str, Any] | None,
    ) -> None:
        name = str(camera)
        if name not in self.required_cameras:
            raise ValueError(f"unexpected camera channel: {name}")
        self._channels[f"camera:{name}"] = _sample(data, frame_id, t_sec, calibration=calibration)

    def receive_ego_state(self, data: Any, *, frame_id: int | str, t_sec: float) -> None:
        self._channels["ego_state"] = _sample(data, frame_id, t_sec)

    def receive_route(self, data: Any, *, frame_id: int | str, t_sec: float) -> None:
        self._channels["route"] = _sample(data, frame_id, t_sec)

    def build(self, *, now_sec: float, expected_frame_id: int | str | None = None) -> dict[str, Any]:
        required = [f"camera:{name}" for name in self.required_cameras] + ["ego_state", "route"]
        missing = [name for name in required if name not in self._channels]
        if missing:
            return _blocked("missing_channel", missing=missing)
        samples = {name: self._channels[name] for name in required}

        frame_ids = {sample["frame_id"] for sample in samples.values()}
        if len(frame_ids) != 1:
            return _blocked(
                "tick_mismatch",
                frames={name: sample["frame_id"] for name, sample in samples.items()},
            )
        frame_id = next(iter(frame_ids))
        if expected_frame_id is not None and frame_id != expected_frame_id:
            return _blocked("unexpected_tick", expected=expected_frame_id, actual=frame_id)

        timestamps = [sample["t_sec"] for sample in samples.values()]
        oldest = min(timestamps)
        newest = max(timestamps)
        age = float(now_sec) - oldest
        if age < -self.max_skew_sec:
            return _blocked("future_observation", age_sec=age)
        if age > self.timeout_sec:
            return _blocked("stale_observation", age_sec=age, timeout_sec=self.timeout_sec)
        if newest - oldest > self.max_skew_sec:
            return _blocked(
                "timestamp_skew",
                skew_sec=newest - oldest,
                max_skew_sec=self.max_skew_sec,
            )

        ego_state = samples["ego_state"]["data"]
        route = samples["route"]["data"]
        missing_state = _missing_fields(ego_state, EGO_STATE_FIELDS)
        if missing_state:
            return _blocked("invalid_ego_state", missing=missing_state)
        missing_route = _missing_fields(route, ROUTE_FIELDS)
        if missing_route:
            return _blocked("invalid_route", missing=missing_route)

        calibration = {}
        sensors = {}
        for camera in self.required_cameras:
            sample = samples[f"camera:{camera}"]
            if sample["data"] is None:
                return _blocked("invalid_sensor", camera=camera)
            calibration[camera] = sample.get("calibration")
            if self.require_calibration:
                error = validate_camera_calibration(calibration[camera])
                if error:
                    return _blocked("invalid_calibration", camera=camera, detail=error)
            sensors[camera] = deepcopy(sample["data"])

        return {
            "status": "ready",
            "observation": {
                "frame_id": frame_id,
                "t_sec": newest,
                "source": "carla_current_tick",
                "sensors": sensors,
                "calibration": deepcopy(calibration),
                "ego_state": deepcopy(ego_state),
                "route": deepcopy(route),
            },
        }


def _sample(data: Any, frame_id: int | str, t_sec: float, **extra: Any) -> dict[str, Any]:
    if (
        isinstance(frame_id, bool)
        or not isinstance(frame_id, (int, str))
        or (isinstance(frame_id, int) and frame_id < 0)
        or (isinstance(frame_id, str) and not frame_id)
    ):
        raise ValueError("frame_id must be a non-negative integer or non-empty string")
    timestamp = float(t_sec)
    if not math.isfinite(timestamp) or timestamp < 0.0:
        raise ValueError("t_sec must be finite and non-negative")
    return {"data": deepcopy(data), "frame_id": frame_id, "t_sec": timestamp, **deepcopy(extra)}


def _missing_fields(value: Any, required: tuple[str, ...]) -> list[str]:
    if not isinstance(value, dict):
        return list(required)
    return [field for field in required if field not in value or value[field] is None]


def _finite_matrix(value: Any, rows: int, columns: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == rows
        and all(
            isinstance(row, list)
            and len(row) == columns
            and all(isinstance(item, (int, float)) and math.isfinite(float(item)) for item in row)
            for row in value
        )
    )


def _blocked(reason: str, **detail: Any) -> dict[str, Any]:
    return {"status": "blocked", "reason": reason, "detail": detail}
