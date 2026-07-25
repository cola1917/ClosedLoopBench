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
from pathlib import Path
from typing import Any, Mapping

from runtime.scene0061_lidar_axis_gate import XYZI_ENCODING, _validated_rigid_matrix


NATIVE_CAPTURE_SCHEMA = "scene0061_carla_native_lidar_capture.v1"


class CarlaNativeLidarProbeError(RuntimeError):
    """Raised when a native CARLA LiDAR capture cannot be reproduced."""


class CarlaNativeLidarProbe:
    """One sensor actor that retains only raw data from its CARLA callback.

    ``sensor_to_ego`` is both the requested CARLA attachment transform and the
    immutable transform recorded beside the raw point cloud.  It must be a
    proper CARLA-coordinate rigid transform; no NRE calibration is consulted.
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
    ) -> None:
        self._carla = carla_module
        self._world = world
        self._ego = ego_vehicle
        self._output_dir = Path(output_dir).expanduser().resolve()
        self._sensor_to_ego = _validated_rigid_matrix(sensor_to_ego)
        self._attributes = {str(key): str(value) for key, value in (blueprint_attributes or {}).items()}
        self._sensor: Any | None = None
        self._frames: dict[int, bytes] = {}
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
        transform = _matrix_to_carla_transform(self._carla, self._sensor_to_ego)
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
        raw = self._frames.pop(frame_id, None)
        if raw is None:
            available = sorted(self._frames)
            raise CarlaNativeLidarProbeError(
                f"native CARLA LiDAR callback did not produce requested frame {frame_id}; available={available}"
            )
        _validate_raw_xyzi(raw)
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
            "sensor_to_ego": self._sensor_to_ego,
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
        self._frames.clear()
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
        self._frames.setdefault(frame, body)


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
