"""Physical CARLA-box support checks for materialized LiDAR payloads."""

from __future__ import annotations

import math
import struct
from collections.abc import Mapping
from typing import Any


class LidarWorldSupportError(ValueError):
    """Raised when a LiDAR payload cannot be bound to CARLA world state."""


def expected_lidar_support_from_physical_boxes(
    *,
    ego_pose: Mapping[str, Any],
    sensor_to_ego: list[Any],
    object_states: list[Mapping[str, Any]],
    max_range_m: float,
    min_visible_surface_samples: int = 1,
) -> list[dict[str, Any]]:
    """Derive conservative per-object LiDAR expectations from physical boxes."""

    _positive_number(max_range_m, "max_range_m")
    if not isinstance(min_visible_surface_samples, int) or isinstance(
        min_visible_surface_samples, bool
    ) or min_visible_surface_samples < 1:
        raise LidarWorldSupportError(
            "min_visible_surface_samples must be a positive integer"
        )
    matrix = _matrix(sensor_to_ego)
    ego = _pose(ego_pose, "ego_pose")
    boxes = [_box(item) for item in object_states]
    sensor_origin = _sensor_point_to_scene((0.0, 0.0, 0.0, 0.0), matrix, ego)
    result = []
    for box in boxes:
        visible_samples = 0
        in_range_samples = 0
        nearest_distance = None
        for point in _box_surface_samples(box):
            distance = math.dist(sensor_origin, point)
            if distance > float(max_range_m):
                continue
            in_range_samples += 1
            target_distance = _ray_box_distance(sensor_origin, point, box)
            if target_distance is None:
                continue
            hits = [
                distance_hit
                for candidate in boxes
                if (distance_hit := _ray_box_distance(sensor_origin, point, candidate))
                is not None
            ]
            if hits and target_distance <= min(hits) + 0.05:
                visible_samples += 1
                nearest_distance = (
                    distance
                    if nearest_distance is None
                    else min(nearest_distance, distance)
                )
        expected = visible_samples >= min_visible_surface_samples
        result.append(
            {
                "object_id": box["object_id"],
                "carla_runtime_actor_id": box["carla_runtime_actor_id"],
                "expected_lidar_support": expected,
                "reason": (
                    "carla_first_hit_surface"
                    if expected
                    else (
                        "outside_declared_lidar_range"
                        if not in_range_samples
                        else "carla_occluded"
                    )
                ),
                "visible_surface_sample_count": visible_samples,
                "in_range_surface_sample_count": in_range_samples,
                "nearest_visible_surface_distance_m": nearest_distance,
                "max_range_m": float(max_range_m),
                "source": "carla_physical_box_occlusion.v1",
            }
        )
    return result


def lidar_occupancy_from_xyzi(
    xyzi: bytes,
    *,
    ego_pose: Mapping[str, Any],
    sensor_to_ego: list[Any],
    object_states: list[Mapping[str, Any]],
    tolerance_m: float = 0.10,
) -> list[dict[str, Any]]:
    """Count materialized sensor-local XYZI points inside physical OBBs."""

    if not isinstance(xyzi, bytes) or not xyzi or len(xyzi) % 16:
        raise LidarWorldSupportError(
            "LiDAR payload must be non-empty float32 XYZI"
        )
    if not isinstance(tolerance_m, (int, float)) or isinstance(
        tolerance_m, bool
    ) or not math.isfinite(float(tolerance_m)) or tolerance_m < 0.0:
        raise LidarWorldSupportError("tolerance_m must be non-negative")
    matrix = _matrix(sensor_to_ego)
    ego = _pose(ego_pose, "ego_pose")
    boxes = [_box(item) for item in object_states]
    points = [
        _sensor_point_to_scene(point, matrix, ego)
        for point in struct.iter_unpack("<4f", xyzi)
    ]
    if not all(math.isfinite(value) for point in points for value in point):
        raise LidarWorldSupportError("LiDAR XYZI contains non-finite values")
    return [
        {
            "object_id": box["object_id"],
            "carla_runtime_actor_id": box["carla_runtime_actor_id"],
            "point_count": sum(
                _inside_box(point, box, float(tolerance_m)) for point in points
            ),
            "source": "nurec_materialized_lidar_sensor_local",
        }
        for box in boxes
    ]


def _box(state: Mapping[str, Any]) -> dict[str, float | str | int]:
    if not isinstance(state, Mapping):
        raise LidarWorldSupportError("object state must be an object")
    object_id = str(state.get("object_id") or "")
    runtime_id = state.get("carla_runtime_actor_id")
    if not object_id or not isinstance(runtime_id, int) or isinstance(runtime_id, bool):
        raise LidarWorldSupportError(
            "object state requires object_id and CARLA runtime id"
        )
    pose = _pose(state.get("pose"), f"object {object_id} pose")
    extent = state.get("extent_m")
    if not isinstance(extent, Mapping):
        raise LidarWorldSupportError(f"object {object_id} has no extent_m")
    try:
        values = {
            f"extent_{axis}": float(extent[axis]) for axis in ("x", "y", "z")
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise LidarWorldSupportError(
            f"object {object_id} has invalid extent_m"
        ) from exc
    if any(not math.isfinite(value) or value <= 0.0 for value in values.values()):
        raise LidarWorldSupportError(
            f"object {object_id} has non-positive extent_m"
        )
    return {
        "object_id": object_id,
        "carla_runtime_actor_id": runtime_id,
        **pose,
        **values,
    }


def _pose(value: Any, label: str) -> dict[str, float]:
    if not isinstance(value, Mapping):
        raise LidarWorldSupportError(f"{label} must be an object")
    try:
        pose = {axis: float(value[axis]) for axis in ("x", "y", "z")}
        pose["yaw"] = float(value.get("yaw", 0.0))
    except (KeyError, TypeError, ValueError) as exc:
        raise LidarWorldSupportError(f"{label} is invalid") from exc
    if not all(math.isfinite(item) for item in pose.values()):
        raise LidarWorldSupportError(f"{label} must be finite")
    return pose


def _matrix(value: Any) -> list[float]:
    if not isinstance(value, list) or len(value) != 16:
        raise LidarWorldSupportError("sensor_to_ego must be a 4x4 matrix")
    try:
        matrix = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise LidarWorldSupportError("sensor_to_ego must be numeric") from exc
    if not all(math.isfinite(item) for item in matrix):
        raise LidarWorldSupportError("sensor_to_ego must be finite")
    return matrix


def _sensor_point_to_scene(
    point: tuple[float, float, float, float],
    matrix: list[float],
    ego: Mapping[str, float],
) -> tuple[float, float, float]:
    x, y, z, _intensity = point
    ego_x = matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3]
    ego_y = matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7]
    ego_z = matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11]
    yaw = math.radians(ego["yaw"])
    return (
        ego["x"] + math.cos(yaw) * ego_x - math.sin(yaw) * ego_y,
        ego["y"] + math.sin(yaw) * ego_x + math.cos(yaw) * ego_y,
        ego["z"] + ego_z,
    )


def _inside_box(
    point: tuple[float, float, float],
    box: Mapping[str, float | str | int],
    tolerance: float,
) -> bool:
    yaw = math.radians(float(box["yaw"]))
    dx, dy = point[0] - float(box["x"]), point[1] - float(box["y"])
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return (
        abs(local_x) <= float(box["extent_x"]) + tolerance
        and abs(local_y) <= float(box["extent_y"]) + tolerance
        and abs(point[2] - float(box["z"]))
        <= float(box["extent_z"]) + tolerance
    )


def _box_surface_samples(
    box: Mapping[str, float | str | int],
) -> list[tuple[float, float, float]]:
    yaw = math.radians(float(box["yaw"]))
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    samples = []
    for local_x, local_y in (
        (float(box["extent_x"]), 0.0),
        (-float(box["extent_x"]), 0.0),
        (0.0, float(box["extent_y"])),
        (0.0, -float(box["extent_y"])),
    ):
        for vertical_fraction in (-0.5, 0.0, 0.5):
            samples.append(
                (
                    float(box["x"])
                    + cos_yaw * local_x
                    - sin_yaw * local_y,
                    float(box["y"])
                    + sin_yaw * local_x
                    + cos_yaw * local_y,
                    float(box["z"])
                    + vertical_fraction * float(box["extent_z"]),
                )
            )
    return samples


def _ray_box_distance(
    origin: tuple[float, float, float],
    point: tuple[float, float, float],
    box: Mapping[str, float | str | int],
) -> float | None:
    direction = tuple(point[index] - origin[index] for index in range(3))
    length = math.sqrt(sum(value * value for value in direction))
    if length <= 1e-9:
        return 0.0
    unit = tuple(value / length for value in direction)
    yaw = math.radians(float(box["yaw"]))
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    dx, dy = origin[0] - float(box["x"]), origin[1] - float(box["y"])
    local_origin = (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
        origin[2] - float(box["z"]),
    )
    local_direction = (
        cos_yaw * unit[0] + sin_yaw * unit[1],
        -sin_yaw * unit[0] + cos_yaw * unit[1],
        unit[2],
    )
    near, far = -math.inf, math.inf
    for coordinate, direction_component, extent in zip(
        local_origin,
        local_direction,
        (
            float(box["extent_x"]),
            float(box["extent_y"]),
            float(box["extent_z"]),
        ),
    ):
        if abs(direction_component) <= 1e-9:
            if coordinate < -extent or coordinate > extent:
                return None
            continue
        first = (-extent - coordinate) / direction_component
        second = (extent - coordinate) / direction_component
        near = max(near, min(first, second))
        far = min(far, max(first, second))
        if near > far:
            return None
    if far < 0.0:
        return None
    return max(0.0, near)


def _positive_number(value: Any, label: str) -> float:
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
        or float(value) <= 0.0
    ):
        raise LidarWorldSupportError(f"{label} must be a positive finite number")
    return float(value)
