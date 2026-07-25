"""Capture a same-frame native CARLA LiDAR stream for axis validation.

This probe deliberately lives outside the NuRec client.  It attaches a real
``sensor.lidar.ray_cast`` actor to the CARLA ego vehicle and persists the
callback's unmodified float32 XYZI bytes.  The output can therefore be used as
the independent CARLA-side source required by
``scene0061_lidar_axis_collector``; it is not derived from an NRE response.

The caller must arm the probe before ``world.tick()`` and call
``persist_frame`` after that tick.  Both the CARLA callback frame and the
requested frame must match exactly.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
import threading
import time
from pathlib import Path
from typing import Any, Mapping

from runtime.scene0061_lidar_axis_gate import XYZI_ENCODING, _validated_rigid_matrix


NATIVE_CAPTURE_SCHEMA = "scene0061_carla_native_lidar_capture.v1"


class CarlaNativeLidarProbeError(RuntimeError):
    """Raised when a native CARLA LiDAR capture cannot be reproduced."""


class CarlaNativeLidarProbe:
    """One sensor actor that retains only raw data from its CARLA callback.

    ``sensor_to_ego`` is only the requested CARLA attachment transform.  The
    transform persisted beside a point cloud is separately observed from the
    CARLA sensor and ego actors after the matching world tick; configuration is
    never used as the coordinate-evidence truth value.
    """

    def __init__(
        self,
        *,
        carla_module: Any,
        world: Any,
        ego_vehicle: Any,
        output_dir: Path,
        sensor_to_ego: object,
        blueprint_attributes: Mapping[str, object] | None = None,
        callback_wait_sec: float = 2.0,
    ) -> None:
        self._carla = carla_module
        self._world = world
        self._ego = ego_vehicle
        self._output_dir = Path(output_dir).expanduser().resolve()
        self._requested_sensor_to_ego = _validated_rigid_matrix(sensor_to_ego)
        self._attributes = {str(key): str(value) for key, value in (blueprint_attributes or {}).items()}
        if not isinstance(callback_wait_sec, (int, float)) or isinstance(callback_wait_sec, bool):
            raise CarlaNativeLidarProbeError("CARLA LiDAR callback wait must be numeric")
        self._callback_wait_sec = float(callback_wait_sec)
        if not math.isfinite(self._callback_wait_sec) or not 0.0 <= self._callback_wait_sec <= 10.0:
            raise CarlaNativeLidarProbeError("CARLA LiDAR callback wait must be within [0, 10] seconds")
        self._sensor: Any | None = None
        self._frames: dict[int, bytes] = {}
        self._frame_condition = threading.Condition()
        self._closed = False

    def start(self) -> None:
        """Spawn and listen to the independent CARLA LiDAR sensor once."""

        if self._closed:
            raise CarlaNativeLidarProbeError("cannot start a closed CARLA LiDAR probe")
        if self._sensor is not None:
            raise CarlaNativeLidarProbeError("CARLA LiDAR probe is already started")
        library = self._world.get_blueprint_library()
        choices = library.filter("sensor.lidar.ray_cast")
        if not choices:
            raise CarlaNativeLidarProbeError("CARLA sensor.lidar.ray_cast blueprint is unavailable")
        blueprint = choices[0]
        # A physical sensor must tick with the synchronous world.  Defaults
        # intentionally stay conservative and can be overridden only explicitly.
        attributes = {
            "channels": "128",
            "range": "120",
            "points_per_second": "250000",
            "rotation_frequency": "20",
            "upper_fov": "10",
            "lower_fov": "-30",
            "horizontal_fov": "360",
            "sensor_tick": "0.0",
            **self._attributes,
        }
        for key, value in attributes.items():
            if hasattr(blueprint, "has_attribute") and not blueprint.has_attribute(key):
                continue
            blueprint.set_attribute(key, value)
        transform = _matrix_to_carla_transform(self._carla, self._requested_sensor_to_ego)
        try:
            self._sensor = self._world.spawn_actor(blueprint, transform, attach_to=self._ego)
            self._sensor.listen(self._on_measurement)
        except Exception as exc:
            self.close()
            raise CarlaNativeLidarProbeError(f"cannot start CARLA native LiDAR sensor: {exc}") from exc

    def persist_frame(self, frame_id: int) -> Path:
        """Persist an exact callback frame and its independently recorded transform."""

        if not isinstance(frame_id, int) or isinstance(frame_id, bool) or frame_id < 0:
            raise CarlaNativeLidarProbeError("CARLA capture frame id must be a non-negative integer")
        if self._sensor is None:
            raise CarlaNativeLidarProbeError("CARLA LiDAR probe has not been started")
        deadline = time.monotonic() + self._callback_wait_sec
        with self._frame_condition:
            raw = self._frames.pop(frame_id, None)
            while raw is None and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._frame_condition.wait(remaining)
                raw = self._frames.pop(frame_id, None)
        if raw is None:
            with self._frame_condition:
                available = sorted(self._frames)
            raise CarlaNativeLidarProbeError(
                f"native CARLA LiDAR callback did not produce requested frame {frame_id}; available={available}"
            )
        _validate_raw_xyzi(raw)
        observed = self._observed_sensor_to_ego()
        directory = self._output_dir / "carla_native_lidar"
        points_path = directory / f"frame_{frame_id:08d}.xyzi"
        capture_path = directory / f"frame_{frame_id:08d}.json"
        if points_path.exists() or capture_path.exists():
            raise FileExistsError(f"refusing to overwrite native CARLA LiDAR evidence: {capture_path}")
        directory.mkdir(parents=True, exist_ok=True)
        points_path.write_bytes(raw)
        points_ref = _file_ref(points_path, frame_id)
        capture = {
            "schema_version": NATIVE_CAPTURE_SCHEMA,
            "status": "passed",
            "capture_source": "carla_sensor.lidar.ray_cast_callback_raw_data",
            "carla_frame_id": frame_id,
            "coordinate_frame": "carla_sensor",
            "axis_convention": "carla_x_forward_y_right_z_up",
            # This must originate from CARLA actor APIs, rather than from the
            # requested transform above.  The complete two world transforms
            # let a later validator independently reproduce the derivation.
            "sensor_to_ego": observed["sensor_to_ego"],
            "sensor_to_ego_observation": "carla_actor_get_transform",
            "observed_sensor_world_transform": observed["sensor_world_transform"],
            "observed_ego_world_transform": observed["ego_world_transform"],
            "requested_sensor_to_ego": self._requested_sensor_to_ego,
            "raw_xyzi_ref": points_ref,
            "point_count": len(raw) // 16,
        }
        capture_path.write_text(json.dumps(capture, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return capture_path

    def close(self) -> None:
        """Stop and destroy only the sensor actor owned by this probe."""

        if self._closed:
            return
        self._closed = True
        sensor, self._sensor = self._sensor, None
        with self._frame_condition:
            self._frames.clear()
            self._frame_condition.notify_all()
        if sensor is None:
            return
        try:
            if hasattr(sensor, "stop"):
                sensor.stop()
        finally:
            if hasattr(sensor, "destroy"):
                sensor.destroy()

    def _on_measurement(self, measurement: Any) -> None:
        frame = getattr(measurement, "frame", None)
        raw = getattr(measurement, "raw_data", None)
        if not isinstance(frame, int) or isinstance(frame, bool) or frame < 0:
            return
        if not isinstance(raw, (bytes, bytearray, memoryview)):
            return
        body = bytes(raw)
        try:
            _validate_raw_xyzi(body)
        except CarlaNativeLidarProbeError:
            return
        # CARLA can emit later frames while the caller handles a prior tick.
        # Retaining them by frame preserves the exact matching requirement.
        with self._frame_condition:
            self._frames.setdefault(frame, body)
            self._frame_condition.notify_all()

    def _observed_sensor_to_ego(self) -> dict[str, list[float]]:
        """Read CARLA's current actor poses and derive the relative rig pose.

        The capture is fail-closed when CARLA cannot provide either observation.
        It is deliberately taken only after the exact callback frame is
        available, so recorded raw bytes, frame ID and transforms belong to the
        same tick boundary controlled by the synchronous runner.
        """

        if self._sensor is None:
            raise CarlaNativeLidarProbeError("CARLA LiDAR probe has not been started")
        try:
            sensor_get_transform = getattr(self._sensor, "get_transform")
            ego_get_transform = getattr(self._ego, "get_transform")
            if not callable(sensor_get_transform) or not callable(ego_get_transform):
                raise TypeError("actor has no callable get_transform()")
            sensor_world = _carla_transform_to_matrix(sensor_get_transform())
            ego_world = _carla_transform_to_matrix(ego_get_transform())
            sensor_to_ego = _matrix_multiply(_invert_rigid_matrix(ego_world), sensor_world)
            return {
                "sensor_world_transform": sensor_world,
                "ego_world_transform": ego_world,
                "sensor_to_ego": _validated_rigid_matrix(sensor_to_ego),
            }
        except (AttributeError, TypeError, ValueError, CarlaNativeLidarProbeError) as exc:
            raise CarlaNativeLidarProbeError(
                f"cannot independently observe CARLA sensor/ego transforms: {exc}"
            ) from exc


def _file_ref(path: Path, frame_id: int) -> dict[str, Any]:
    body = path.read_bytes()
    return {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(body).hexdigest(),
        "byte_count": len(body),
        "encoding": XYZI_ENCODING,
        "carla_frame_id": frame_id,
    }


def _validate_raw_xyzi(raw: bytes) -> None:
    if not raw or len(raw) % 16:
        raise CarlaNativeLidarProbeError("CARLA native LiDAR raw_data is not complete float32 XYZI")
    try:
        rows = struct.iter_unpack("<4f", raw)
        if any(not all(math.isfinite(value) for value in row) for row in rows):
            raise CarlaNativeLidarProbeError("CARLA native LiDAR raw_data contains non-finite XYZI")
    except struct.error as exc:
        raise CarlaNativeLidarProbeError("CARLA native LiDAR raw_data cannot be decoded as XYZI") from exc


def _matrix_to_carla_transform(carla_module: Any, matrix: list[float]) -> Any:
    """Convert a proper row-major CARLA sensor-to-ego matrix to Transform."""

    # For CARLA's roll(X), pitch(Y), yaw(Z) convention: R = Rz(yaw) Ry(pitch) Rx(roll).
    pitch = math.asin(max(-1.0, min(1.0, -matrix[8])))
    cosine = math.cos(pitch)
    if abs(cosine) > 1e-7:
        yaw = math.atan2(matrix[4], matrix[0])
        roll = math.atan2(matrix[9], matrix[10])
    else:
        # Gimbal lock has an infinity of roll/yaw decompositions.  A zero roll
        # is deterministic and exactly reconstructs the declared orientation.
        yaw = math.atan2(-matrix[1], matrix[5])
        roll = 0.0
    return carla_module.Transform(
        carla_module.Location(x=matrix[3], y=matrix[7], z=matrix[11]),
        carla_module.Rotation(
            roll=math.degrees(roll), pitch=math.degrees(pitch), yaw=math.degrees(yaw)
        ),
    )


def _carla_transform_to_matrix(transform: Any) -> list[float]:
    """Normalize a CARLA Transform returned by an actor API to 4x4 row-major.

    CARLA's native ``Transform.get_matrix`` is preferred.  The attribute-based
    form supports CARLA-compatible test doubles while using the documented
    roll(X), pitch(Y), yaw(Z) convention.
    """

    if transform is None:
        raise CarlaNativeLidarProbeError("CARLA get_transform() returned None")
    get_matrix = getattr(transform, "get_matrix", None)
    if callable(get_matrix):
        try:
            rows = get_matrix()
            matrix = [float(value) for row in rows for value in row]
            return _validated_rigid_matrix(matrix)
        except (TypeError, ValueError, CarlaNativeLidarProbeError) as exc:
            raise CarlaNativeLidarProbeError(
                f"CARLA Transform.get_matrix() is not a rigid 4x4 matrix: {exc}"
            ) from exc
    try:
        location = transform.location
        rotation = transform.rotation
        x, y, z = float(location.x), float(location.y), float(location.z)
        roll = math.radians(float(rotation.roll))
        pitch = math.radians(float(rotation.pitch))
        yaw = math.radians(float(rotation.yaw))
    except (AttributeError, TypeError, ValueError) as exc:
        raise CarlaNativeLidarProbeError(
            f"CARLA Transform has no usable location/rotation: {exc}"
        ) from exc
    if not all(math.isfinite(value) for value in (x, y, z, roll, pitch, yaw)):
        raise CarlaNativeLidarProbeError("CARLA Transform contains non-finite values")
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return _validated_rigid_matrix([
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y,
        -sp, cp * sr, cp * cr, z,
        0.0, 0.0, 0.0, 1.0,
    ])


def _invert_rigid_matrix(matrix: object) -> list[float]:
    value = _validated_rigid_matrix(matrix)
    rotation = [value[0], value[1], value[2], value[4], value[5], value[6], value[8], value[9], value[10]]
    translation = [value[3], value[7], value[11]]
    inverse_rotation = [
        rotation[0], rotation[3], rotation[6],
        rotation[1], rotation[4], rotation[7],
        rotation[2], rotation[5], rotation[8],
    ]
    inverse_translation = [
        -sum(inverse_rotation[index * 3 + column] * translation[column] for column in range(3))
        for index in range(3)
    ]
    return [
        inverse_rotation[0], inverse_rotation[1], inverse_rotation[2], inverse_translation[0],
        inverse_rotation[3], inverse_rotation[4], inverse_rotation[5], inverse_translation[1],
        inverse_rotation[6], inverse_rotation[7], inverse_rotation[8], inverse_translation[2],
        0.0, 0.0, 0.0, 1.0,
    ]


def _matrix_multiply(left: object, right: object) -> list[float]:
    lhs = _validated_rigid_matrix(left)
    rhs = _validated_rigid_matrix(right)
    return _validated_rigid_matrix([
        sum(lhs[row * 4 + index] * rhs[index * 4 + column] for index in range(4))
        for row in range(4)
        for column in range(4)
    ])
