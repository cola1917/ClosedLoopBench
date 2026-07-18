from __future__ import annotations

import importlib
import json
import struct
import sys
from pathlib import Path
from typing import Any, Mapping

from adapters.nurec_grpc_dispatch import dispatch_nurec_multimodal_frame
from adapters.nurec_multimodal import NuRecMultimodalError
from adapters.nurec_runtime_handler import make_nurec_sensor_frame_handler


class NuRec260Client:
    """Concrete NRE 26.04 SensorsimService RGB/LiDAR adapter.

    The adapter imports NVIDIA's generated protobuf package from the installed
    CARLA NuRec runtime. Generated protobuf sources are not copied into this
    repository. ``runtime_scene_id`` is deliberately separate from the
    canonical nuScenes scene token used by Scene Exchange contracts.
    """

    def __init__(
        self,
        *,
        target: str,
        runtime_scene_id: str,
        scene_start_us: int,
        timeout_sec: float = 60.0,
        max_message_bytes: int = 1024 * 1024 * 1024,
        protobuf_module: Any | None = None,
        common_protobuf_module: Any | None = None,
        stub: Any | None = None,
        channel: Any | None = None,
        camera_specs: Mapping[str, Any] | None = None,
    ) -> None:
        if not target or not runtime_scene_id:
            raise NuRecMultimodalError("NRE target and runtime_scene_id are required")
        if int(scene_start_us) < 0:
            raise NuRecMultimodalError("scene_start_us must be non-negative")
        self.target = str(target)
        self.runtime_scene_id = str(runtime_scene_id)
        self.scene_start_us = int(scene_start_us)
        self.timeout_sec = float(timeout_sec)
        self._pb = protobuf_module or importlib.import_module(
            "nre.grpc.protos.sensorsim_pb2"
        )
        self._common_pb = common_protobuf_module or importlib.import_module(
            "nre.grpc.protos.common_pb2"
        )
        self._channel = channel
        if stub is None:
            grpc = importlib.import_module("grpc")
            stub_module = importlib.import_module(
                "nre.grpc.protos.sensorsim_pb2_grpc"
            )
            self._channel = grpc.insecure_channel(
                self.target,
                options=[
                    ("grpc.max_send_message_length", int(max_message_bytes)),
                    ("grpc.max_receive_message_length", int(max_message_bytes)),
                ],
            )
            stub = stub_module.SensorsimServiceStub(self._channel)
        self.stub = stub
        self._camera_specs = dict(camera_specs or self._load_camera_specs())

    def close(self) -> None:
        if self._channel is not None and hasattr(self._channel, "close"):
            self._channel.close()

    def dispatch_frame(self, frame: Mapping[str, Any]) -> dict[str, Any]:
        evidence = dispatch_nurec_multimodal_frame(
            frame,
            encode_rgb=self.encode_rgb,
            encode_lidar=self.encode_lidar,
            render_rgb=self.render_rgb,
            render_lidar=self.render_lidar,
            response_bytes=self.response_bytes,
            response_inspector=self.inspect_response,
        )
        evidence["dispatch"]["runtime_scene_id"] = self.runtime_scene_id
        evidence["dispatch"]["canonical_scene_id"] = frame.get("scene_id")
        evidence["dispatch"]["nre_api"] = "SensorsimService/26.04"
        return evidence

    def query_runtime_inventory(self) -> dict[str, Any]:
        """Query the live service before accepting any render evidence."""

        empty = self._common_pb.Empty()
        version = self.stub.get_version(empty, timeout=self.timeout_sec)
        scenes = self.stub.get_available_scenes(empty, timeout=self.timeout_sec)
        cameras = self.stub.get_available_cameras(
            self._pb.AvailableCamerasRequest(scene_id=self.runtime_scene_id),
            timeout=self.timeout_sec,
        )
        scene_ids = sorted(str(value) for value in scenes.scene_ids)
        if self.runtime_scene_id not in scene_ids:
            raise NuRecMultimodalError(
                f"configured runtime_scene_id is unavailable: {self.runtime_scene_id}"
            )
        camera_rows = []
        for item in cameras.available_cameras:
            logical_id = str(item.logical_id)
            if not logical_id:
                raise NuRecMultimodalError("NRE advertised a camera without logical_id")
            camera_rows.append(
                {
                    "logical_id": logical_id,
                    "trajectory_idx": int(item.trajectory_idx),
                    "resolution_w": int(item.intrinsics.resolution_w),
                    "resolution_h": int(item.intrinsics.resolution_h),
                }
            )
        if not camera_rows:
            raise NuRecMultimodalError(
                f"NRE scene has no available cameras: {self.runtime_scene_id}"
            )
        api = version.grpc_api_version
        return {
            "schema_version": "nurec_260_runtime_inventory.v1",
            "target": self.target,
            "runtime_scene_id": self.runtime_scene_id,
            "available_scene_ids": scene_ids,
            "renderer": {
                "version_id": str(version.version_id),
                "git_hash": str(version.git_hash),
                "grpc_api_version": {
                    "major": int(api.major),
                    "minor": int(api.minor),
                    "patch": int(api.patch),
                },
            },
            "cameras": sorted(camera_rows, key=lambda item: item["logical_id"]),
            "lidar": {
                "supported_device_types": ["PANDAR128", "AT128"],
                "parameterization": "device_type_only",
                "capability_source": "nre_26_04_protobuf_api_boundary",
                "render_verified": False,
            },
            "status": "capability_only",
        }

    def encode_rgb(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        sensor = payload["sensor"]
        sensor_id = str(sensor["sensor_id"])
        camera_spec = self._camera_specs.get(sensor_id)
        if camera_spec is None:
            raise NuRecMultimodalError(
                f"NRE runtime did not advertise recorded camera: {sensor_id}"
            )
        parameters = sensor.get("parameters") or {}
        width = int(parameters.get("width") or camera_spec.resolution_w)
        height = int(parameters.get("height") or camera_spec.resolution_h)
        frame_start_us, frame_end_us = self._time_window_us(payload)
        request = self._pb.RGBRenderRequest(
            scene_id=self.runtime_scene_id,
            resolution_h=height,
            resolution_w=width,
            camera_intrinsics=camera_spec,
            frame_start_us=frame_start_us,
            frame_end_us=frame_end_us,
            sensor_pose=self._pose_pair(sensor["pose_pair"]),
            dynamic_objects=self._dynamic_objects(payload["dynamic_objects"]),
            image_format=self._pb.JPEG,
            image_quality=float(parameters.get("image_quality", 95.0)),
        )
        return self._encoded(payload, request)

    def encode_lidar(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        sensor = payload["sensor"]
        parameters = sensor.get("parameters") or {}
        device_name = str(parameters.get("device_type") or "PANDAR128").upper()
        if device_name not in {"PANDAR128", "AT128"}:
            raise NuRecMultimodalError(
                "NRE 26.04 LiDAR supports only PANDAR128 or AT128"
            )
        frame_start_us, frame_end_us = self._time_window_us(payload)
        request = self._pb.LidarRenderRequest(
            scene_id=self.runtime_scene_id,
            lidar_config=self._pb.LidarSpec(
                lidar_type=getattr(self._pb, device_name)
            ),
            frame_start_us=frame_start_us,
            frame_end_us=frame_end_us,
            sensor_pose=self._pose_pair(sensor["pose_pair"]),
            dynamic_objects=self._dynamic_objects(payload["dynamic_objects"]),
        )
        return self._encoded(payload, request)

    def render_rgb(self, request: Any) -> Any:
        return self.stub.render_rgb(request, timeout=self.timeout_sec)

    def render_lidar(self, request: Any) -> Any:
        return self.stub.render_lidar(request, timeout=self.timeout_sec)

    @staticmethod
    def response_bytes(response: Any) -> bytes:
        serializer = getattr(response, "SerializeToString", None)
        if not callable(serializer):
            raise NuRecMultimodalError("NRE response is not a protobuf message")
        body = serializer()
        if not isinstance(body, bytes) or not body:
            raise NuRecMultimodalError("NRE response protobuf is empty")
        return body

    def inspect_response(
        self, payload: Mapping[str, Any], response: Any, body: bytes
    ) -> Mapping[str, Any]:
        del body
        if payload["modality"] == "rgb":
            image = bytes(getattr(response, "image_bytes", b""))
            if not image:
                raise NuRecMultimodalError("NRE RGB response image_bytes is empty")
            width, height = _jpeg_dimensions(image)
            parameters = payload["sensor"].get("parameters") or {}
            expected_width = int(parameters.get("width") or width)
            expected_height = int(parameters.get("height") or height)
            if (width, height) != (expected_width, expected_height):
                raise NuRecMultimodalError(
                    f"NRE RGB dimensions {(width, height)} != {(expected_width, expected_height)}"
                )
            return {"width": width, "height": height, "encoding": "jpeg"}

        xyz = getattr(response, "point_xyzs", ())
        intensities = getattr(response, "point_intensities", ())
        if not xyz or len(xyz) % 3:
            raise NuRecMultimodalError(
                "NRE LiDAR response must contain non-empty XYZ triples"
            )
        point_count = len(xyz) // 3
        if len(intensities) != point_count:
            raise NuRecMultimodalError(
                "NRE LiDAR intensity count does not match XYZ point count"
            )
        return {"point_count": point_count, "encoding": "float_xyz_intensity"}

    def _load_camera_specs(self) -> dict[str, Any]:
        response = self.stub.get_available_cameras(
            self._pb.AvailableCamerasRequest(scene_id=self.runtime_scene_id),
            timeout=self.timeout_sec,
        )
        return {
            str(item.logical_id): item.intrinsics
            for item in response.available_cameras
        }

    def _time_window_us(self, payload: Mapping[str, Any]) -> tuple[int, int]:
        interval = payload["pose_interval_sec"]
        start = self.scene_start_us + int(round(float(interval["start"]) * 1_000_000))
        end = self.scene_start_us + int(round(float(interval["end"]) * 1_000_000))
        if start < self.scene_start_us:
            raise NuRecMultimodalError("NRE pose interval starts before scene_start_us")
        return start, max(start + 1, end)

    def _pose_pair(self, pair: Mapping[str, Any]) -> Any:
        return self._pb.PosePair(
            start_pose=_pose_mapping(pair["start"]),
            end_pose=_pose_mapping(pair["end"]),
        )

    def _dynamic_objects(self, objects: Any) -> list[Any]:
        return [
            self._pb.DynamicObject(
                track_id=str(item["track_id"]),
                pose_pair=self._pose_pair(item["pose_pair"]),
            )
            for item in objects
        ]

    @staticmethod
    def _encoded(payload: Mapping[str, Any], request: Any) -> dict[str, Any]:
        return {
            "wire_request": request,
            "frame_id": payload["frame_id"],
            "modality": payload["modality"],
            "dynamic_object_sha256": payload["dynamic_object_sha256"],
        }


def build_nurec_260_client(run_config: Mapping[str, Any]) -> NuRec260Client:
    """Build the concrete client from the shared ``nurec_runtime`` config."""
    config = run_config.get("nurec_runtime")
    if not isinstance(config, Mapping):
        raise NuRecMultimodalError("run config requires nurec_runtime")
    if "scene_start_us" not in config:
        raise NuRecMultimodalError("nurec_runtime.scene_start_us is required")
    runtime_path = config.get("python_api_path")
    if runtime_path:
        resolved = str(Path(str(runtime_path)).resolve())
        if resolved not in sys.path:
            sys.path.insert(0, resolved)
    return NuRec260Client(
        target=str(config.get("target") or "127.0.0.1:46435"),
        runtime_scene_id=str(config.get("runtime_scene_id") or ""),
        scene_start_us=int(config["scene_start_us"]),
        timeout_sec=float(config.get("timeout_sec") or 60.0),
    )


def build_nurec_260_handler(
    run_config: Mapping[str, Any], attempt_dir: Path
) -> Any:
    """Triplicate sensor-handler factory configured by ``nurec_runtime``."""

    config = run_config.get("nurec_runtime")
    if not isinstance(config, Mapping):
        raise NuRecMultimodalError("run config requires nurec_runtime")
    client = build_nurec_260_client(run_config)
    scene_package = _load_json(config, "scene_package")
    binding_set = _load_json(config, "actor_bindings")
    handler = make_nurec_sensor_frame_handler(
        scene_package,
        binding_set,
        camera_specs=config.get("camera_specs") or [],
        lidar_specs=config.get("lidar_specs") or [],
        dispatch_frame=client.dispatch_frame,
    )
    handler.close = client.close  # type: ignore[attr-defined]
    handler.attempt_dir = str(attempt_dir)  # type: ignore[attr-defined]
    return handler


def _load_json(config: Mapping[str, Any], name: str) -> dict[str, Any]:
    path = Path(str(config.get(name) or ""))
    if not path.is_file():
        raise NuRecMultimodalError(f"nurec_runtime.{name} does not exist: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise NuRecMultimodalError(f"nurec_runtime.{name} must contain a JSON object")
    return value


def _pose_mapping(pose: Mapping[str, Any]) -> dict[str, Any]:
    position = pose["position_m"]
    orientation = pose["orientation_xyzw"]
    return {
        "vec": {
            "x": float(position["x"]),
            "y": float(position["y"]),
            "z": float(position["z"]),
        },
        "quat": {
            "w": float(orientation["w"]),
            "x": float(orientation["x"]),
            "y": float(orientation["y"]),
            "z": float(orientation["z"]),
        },
    }


def _jpeg_dimensions(data: bytes) -> tuple[int, int]:
    if len(data) < 4 or data[:2] != b"\xff\xd8":
        raise NuRecMultimodalError("NRE RGB response is not a JPEG image")
    offset = 2
    sof_markers = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
    while offset + 4 <= len(data):
        if data[offset] != 0xFF:
            offset += 1
            continue
        marker = data[offset + 1]
        offset += 2
        if marker in {0xD8, 0xD9}:
            continue
        if offset + 2 > len(data):
            break
        segment_length = struct.unpack(">H", data[offset : offset + 2])[0]
        if segment_length < 2 or offset + segment_length > len(data):
            break
        if marker in sof_markers and segment_length >= 7:
            height, width = struct.unpack(">HH", data[offset + 3 : offset + 7])
            return int(width), int(height)
        offset += segment_length
    raise NuRecMultimodalError("NRE JPEG response has no supported SOF dimensions")
