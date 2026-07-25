from __future__ import annotations

import importlib
import hashlib
import json
import re
import struct
import sys
from pathlib import Path
from typing import Any, Mapping

from adapters.nurec_grpc_dispatch import dispatch_nurec_multimodal_frame
from adapters.nurec_multimodal import (
    NuRecMultimodalError,
    validate_nurec_multimodal_evidence,
)
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
        payload_output_dir: str | Path | None = None,
        payload_reference_root: str | Path | None = None,
        lidar_response_coordinate_frame: str = "unverified",
        lidar_axis_convention: str = "unverified",
        native_scan_manifest: Mapping[str, Any] | None = None,
        native_scan_manifest_sha256: str | None = None,
        native_scan_max_midpoint_error_us: int = 25_000,
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
        self._payload_output_dir = (
            Path(payload_output_dir).expanduser()
            if payload_output_dir is not None
            else None
        )
        self._payload_reference_root = (
            Path(payload_reference_root).expanduser().resolve()
            if payload_reference_root is not None
            else None
        )
        self._lidar_response_coordinate_frame = str(lidar_response_coordinate_frame)
        self._lidar_axis_convention = str(lidar_axis_convention)
        self._native_scan_alignment = _validate_native_scan_manifest(
            native_scan_manifest,
            manifest_sha256=native_scan_manifest_sha256,
            runtime_scene_id=self.runtime_scene_id,
            scene_start_us=self.scene_start_us,
            max_midpoint_error_us=native_scan_max_midpoint_error_us,
        )
        self._active_temporal_alignment: dict[str, Any] | None = None

    def close(self) -> None:
        if self._channel is not None and hasattr(self._channel, "close"):
            self._channel.close()

    def dispatch_frame(self, frame: Mapping[str, Any]) -> dict[str, Any]:
        if self._active_temporal_alignment is not None:
            raise NuRecMultimodalError("NuRec client does not support concurrent dispatch")
        alignment = self._select_native_scan_alignment(frame)
        self._active_temporal_alignment = alignment
        try:
            evidence = dispatch_nurec_multimodal_frame(
                frame,
                encode_rgb=self.encode_rgb,
                encode_lidar=self.encode_lidar,
                render_rgb=self.render_rgb,
                render_lidar=self.render_lidar,
                response_bytes=self.response_bytes,
                response_inspector=self.inspect_response,
            )
        finally:
            self._active_temporal_alignment = None
        evidence["dispatch"]["runtime_scene_id"] = self.runtime_scene_id
        evidence["dispatch"]["canonical_scene_id"] = frame.get("scene_id")
        evidence["dispatch"]["nre_api"] = "SensorsimService/26.04"
        if alignment is not None:
            evidence["dispatch"]["temporal_alignment"] = alignment
        validate_nurec_multimodal_evidence(evidence)
        return evidence

    def _select_native_scan_alignment(
        self, frame: Mapping[str, Any]
    ) -> dict[str, Any] | None:
        config = self._native_scan_alignment
        if config is None:
            return None
        interval = frame.get("pose_interval_sec")
        if not isinstance(interval, Mapping):
            raise NuRecMultimodalError("NuRec frame requires pose_interval_sec")
        logical_start = self.scene_start_us + int(
            round(float(interval["start"]) * 1_000_000)
        )
        logical_end = self.scene_start_us + int(
            round(float(interval["end"]) * 1_000_000)
        )
        if logical_start >= logical_end:
            raise NuRecMultimodalError("NuRec logical frame window is not positive")
        logical_midpoint = (logical_start + logical_end) // 2
        index, window = min(
            enumerate(config["scan_windows_us"]),
            key=lambda item: abs((item[1][0] + item[1][1]) // 2 - logical_midpoint),
        )
        wire_start, wire_end = window
        midpoint_error = abs((wire_start + wire_end) // 2 - logical_midpoint)
        max_error = config["max_midpoint_error_us"]
        if midpoint_error > max_error:
            raise NuRecMultimodalError(
                "nearest native LiDAR scan exceeds midpoint threshold: "
                f"error_us={midpoint_error}, max_us={max_error}"
            )
        return {
            "policy": "nearest_native_lidar_scan_midpoint",
            "source": "hashed_native_scan_manifest",
            "manifest_sha256": config["manifest_sha256"],
            "artifact_sha256": config["artifact_sha256"],
            "native_scan_index": index,
            "logical_start_us": logical_start,
            "logical_end_us": logical_end,
            "wire_start_us": wire_start,
            "wire_end_us": wire_end,
            "midpoint_error_us": midpoint_error,
            "max_midpoint_error_us": max_error,
        }

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
        image_quality = float(parameters.get("image_quality", 0.95))
        if not 0.0 <= image_quality <= 1.0:
            raise NuRecMultimodalError(
                "NRE 26.04 RGB image_quality must be between 0.0 and 1.0"
            )
        # The 26.04.146 service forwards this float to nvJPEG after casting it
        # to an integer.  Sending the documented normalized value (for
        # example 0.95) therefore becomes quality 0 and emits nvJPEG error #2.
        # Keep the public/client parameter normalized as documented, but adapt
        # it to the percent value consumed by this deployed service boundary.
        wire_image_quality = image_quality * 100.0
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
            image_quality=wire_image_quality,
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
            metadata = {"width": width, "height": height, "encoding": "jpeg"}
            materialized = self._materialize_payload(payload, image, suffix=".jpg")
            if materialized is not None:
                metadata["materialized_payload"] = {
                    **materialized,
                    "encoding": "jpeg",
                    "coordinate_frame": "camera_optical",
                }
            return metadata

        metadata = _inspect_lidar_response(response, body)
        xyzi = _lidar_xyzi_bytes(response, body)
        materialized = self._materialize_payload(payload, xyzi, suffix=".bin")
        if materialized is not None:
            metadata["materialized_payload"] = {
                **materialized,
                "encoding": "float32_xyzi_little_endian",
                "coordinate_frame": self._lidar_response_coordinate_frame,
                "axis_convention": self._lidar_axis_convention,
            }
        return metadata

    def _materialize_payload(
        self, payload: Mapping[str, Any], data: bytes, *, suffix: str
    ) -> dict[str, Any] | None:
        if self._payload_output_dir is None:
            return None
        frame_id = int(payload["frame_id"])
        sensor_id = re.sub(
            r"[^A-Za-z0-9_.-]+", "_", str(payload["sensor"]["sensor_id"])
        ).strip("_")
        if not sensor_id:
            raise NuRecMultimodalError("cannot materialize payload with empty sensor_id")
        directory = self._payload_output_dir / f"frame_{frame_id:08d}"
        directory.mkdir(parents=True, exist_ok=True)
        target = directory / f"{sensor_id}{suffix}"
        if target.exists():
            raise NuRecMultimodalError(
                f"refusing to overwrite materialized NuRec payload: {target}"
            )
        target.write_bytes(data)
        result = {
            "path": str(target),
            "sha256": hashlib.sha256(data).hexdigest(),
            "byte_count": len(data),
        }
        if self._payload_reference_root is not None:
            try:
                result["relative_path"] = target.resolve().relative_to(
                    self._payload_reference_root
                ).as_posix()
            except ValueError as exc:
                raise NuRecMultimodalError(
                    "materialized payload is outside payload_reference_root"
                ) from exc
        return result

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
        alignment = self._active_temporal_alignment
        if alignment is not None:
            if (start, end) != (
                alignment["logical_start_us"],
                alignment["logical_end_us"],
            ):
                raise NuRecMultimodalError(
                    "NuRec frame sensors do not share one logical time window"
                )
            return alignment["wire_start_us"], alignment["wire_end_us"]
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


def build_nurec_260_client(
    run_config: Mapping[str, Any],
    *,
    payload_output_dir: str | Path | None = None,
) -> NuRec260Client:
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
    manifest, manifest_sha256, max_midpoint_error_us = _load_native_scan_reference(
        config.get("native_scan_manifest")
    )
    return NuRec260Client(
        target=str(config.get("target") or "127.0.0.1:46435"),
        runtime_scene_id=str(config.get("runtime_scene_id") or ""),
        scene_start_us=int(config["scene_start_us"]),
        timeout_sec=float(config.get("timeout_sec") or 60.0),
        payload_output_dir=payload_output_dir,
        payload_reference_root=config.get("payload_reference_root"),
        lidar_response_coordinate_frame=str(
            config.get("lidar_response_coordinate_frame") or "unverified"
        ),
        lidar_axis_convention=str(config.get("lidar_axis_convention") or "unverified"),
        native_scan_manifest=manifest,
        native_scan_manifest_sha256=manifest_sha256,
        native_scan_max_midpoint_error_us=max_midpoint_error_us,
    )


def build_nurec_260_handler(
    run_config: Mapping[str, Any], attempt_dir: Path
) -> Any:
    """Triplicate sensor-handler factory configured by ``nurec_runtime``."""

    config = run_config.get("nurec_runtime")
    if not isinstance(config, Mapping):
        raise NuRecMultimodalError("run config requires nurec_runtime")
    client = build_nurec_260_client(run_config)
    client._payload_output_dir = Path(attempt_dir) / "algorithm_sensor_payloads"
    # The sidecar mounts the triplicate output root once at /sim-data. Keep
    # each attempt directory in the relative path so repeated CARLA frame IDs
    # remain unambiguous across attempt-01/02/03.
    client._payload_reference_root = Path(attempt_dir).resolve().parent
    scene_package = _load_json(config, "scene_package")
    binding_set = _load_json(config, "actor_bindings")
    _validate_runtime_actor_binding_contract(run_config, config, binding_set)
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


def _validate_runtime_actor_binding_contract(
    run_config: Mapping[str, Any],
    nurec_runtime: Mapping[str, Any],
    binding_set: Mapping[str, Any],
) -> None:
    """Reject a NuRec sidecar that diverges from the CARLA actor contract.

    A run config holds the actor binding used to create CARLA actors, while the
    NuRec handler reads the sidecar named by ``nurec_runtime.actor_bindings``.
    Both must identify the same physical object and pose reference before a
    sensor transaction can be sent.
    """

    sidecar_path = Path(str(nurec_runtime.get("actor_bindings") or ""))
    expected_sha256 = nurec_runtime.get("actor_bindings_sha256")
    if expected_sha256 is not None:
        expected_sha256 = str(expected_sha256)
        if not _is_sha256(expected_sha256):
            raise NuRecMultimodalError(
                "nurec_runtime.actor_bindings_sha256 must be a lowercase SHA-256"
            )
        actual_sha256 = hashlib.sha256(sidecar_path.read_bytes()).hexdigest()
        if actual_sha256 != expected_sha256:
            raise NuRecMultimodalError(
                "nurec_runtime.actor_bindings SHA-256 mismatch"
            )

    sidecar_by_id = {
        str(item.get("actor_id") or ""): item
        for item in binding_set.get("bindings") or []
        if isinstance(item, Mapping)
    }
    embedded_by_id = {
        str(actor.get("actor_id") or ""): actor
        for actor in run_config.get("actors") or []
        if isinstance(actor, Mapping) and isinstance(actor.get("binding"), Mapping)
    }
    declared = (run_config.get("actor_binding") or {}).get("selected_actor_ids")
    actor_ids = [str(value) for value in declared] if isinstance(declared, list) else list(embedded_by_id)
    for actor_id in actor_ids:
        actor = embedded_by_id.get(actor_id)
        sidecar = sidecar_by_id.get(actor_id)
        if actor is None or sidecar is None:
            raise NuRecMultimodalError(
                f"NuRec actor binding sidecar does not match configured actor: {actor_id}"
            )
        embedded = actor["binding"]
        sync = sidecar.get("sensor_sync") or {}
        nurec = sidecar.get("nurec") or {}
        if (
            embedded.get("nurec_track_id") != nurec.get("track_id")
            or embedded.get("sensor_pose_source") != sync.get("pose_source")
            or embedded.get("sensor_pose_reference") != sync.get("pose_reference")
            or list(embedded.get("required_modalities") or [])
            != list(sync.get("required_modalities") or [])
        ):
            raise NuRecMultimodalError(
                f"NuRec actor binding sidecar contract mismatch for actor {actor_id}"
            )


def _load_native_scan_reference(
    reference: Any,
) -> tuple[dict[str, Any] | None, str | None, int]:
    if reference is None:
        return None, None, 25_000
    if not isinstance(reference, Mapping):
        raise NuRecMultimodalError("nurec_runtime.native_scan_manifest must be an object")
    path = Path(str(reference.get("path") or ""))
    expected_sha256 = str(reference.get("sha256") or "")
    if not path.is_file() or not _is_sha256(expected_sha256):
        raise NuRecMultimodalError(
            "native_scan_manifest requires an existing path and lowercase SHA-256"
        )
    body = path.read_bytes()
    if hashlib.sha256(body).hexdigest() != expected_sha256:
        raise NuRecMultimodalError("native_scan_manifest SHA-256 mismatch")
    try:
        value = json.loads(body)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise NuRecMultimodalError("native_scan_manifest is not strict JSON") from exc
    if not isinstance(value, dict):
        raise NuRecMultimodalError("native_scan_manifest must contain a JSON object")
    max_error = int(reference.get("max_midpoint_error_us", 25_000))
    return value, expected_sha256, max_error


def _validate_native_scan_manifest(
    manifest: Mapping[str, Any] | None,
    *,
    manifest_sha256: str | None,
    runtime_scene_id: str,
    scene_start_us: int,
    max_midpoint_error_us: int,
) -> dict[str, Any] | None:
    if manifest is None:
        return None
    if manifest.get("schema_version") != "nurec_native_lidar_scan_manifest.v1":
        raise NuRecMultimodalError("unsupported native LiDAR scan manifest")
    if manifest.get("runtime_scene_id") != runtime_scene_id:
        raise NuRecMultimodalError("native scan manifest runtime_scene_id mismatch")
    if int(manifest.get("scene_start_us", -1)) != scene_start_us:
        raise NuRecMultimodalError("native scan manifest scene_start_us mismatch")
    artifact_sha256 = str(manifest.get("artifact_sha256") or "")
    if not _is_sha256(str(manifest_sha256 or "")) or not _is_sha256(artifact_sha256):
        raise NuRecMultimodalError("native scan manifest identities are invalid")
    if max_midpoint_error_us < 0:
        raise NuRecMultimodalError("native scan midpoint threshold must be non-negative")
    raw_windows = manifest.get("scan_windows_us")
    if not isinstance(raw_windows, list) or not raw_windows:
        raise NuRecMultimodalError("native scan manifest has no scan windows")
    windows: list[tuple[int, int]] = []
    previous_start = -1
    for raw in raw_windows:
        if not isinstance(raw, list) or len(raw) != 2:
            raise NuRecMultimodalError("native scan window must contain start/end")
        start, end = int(raw[0]), int(raw[1])
        if start < scene_start_us or start >= end or start <= previous_start:
            raise NuRecMultimodalError("native scan windows are invalid or unsorted")
        windows.append((start, end))
        previous_start = start
    return {
        "manifest_sha256": str(manifest_sha256),
        "artifact_sha256": artifact_sha256,
        "scan_windows_us": windows,
        "max_midpoint_error_us": max_midpoint_error_us,
    }


def _is_sha256(value: str) -> bool:
    return len(value) == 64 and all(character in "0123456789abcdef" for character in value)


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


def _inspect_lidar_response(response: Any, body: bytes) -> dict[str, Any]:
    """Inspect both legacy and NRE 26.04 buffered LiDAR responses.

    CARLA 0.9.16 ships an older ``LidarRenderReturn`` descriptor whose fields
    1/2 are repeated floats.  The NRE 26.04 server keeps those legacy fields
    but normally returns the efficient layout below instead::

        uint32 num_points = 3;
        bytes point_xyzs_buffer = 4;
        bytes point_intensities_buffer = 5;

    Protobuf preserves those fields as unknown data when the response is
    parsed with CARLA's older generated module.  Decode only the three exact
    top-level fields needed for validation; the full serialized response stays
    unchanged and remains the source of the acceptance SHA-256.
    """

    xyz = getattr(response, "point_xyzs", ())
    intensities = getattr(response, "point_intensities", ())
    if xyz or intensities:
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

    fields = _protobuf_wire_field_records(body)
    point_counts = fields.get(3, [])
    xyz_buffers = fields.get(4, [])
    intensity_buffers = fields.get(5, [])
    if not point_counts and not xyz_buffers and not intensity_buffers:
        raise NuRecMultimodalError(
            "NRE LiDAR response contains neither legacy points nor 26.04 buffers"
        )
    if (
        len(point_counts) != 1
        or len(xyz_buffers) != 1
        or len(intensity_buffers) != 1
        or point_counts[0][0] != 0
        or xyz_buffers[0][0] != 2
        or intensity_buffers[0][0] != 2
        or not isinstance(point_counts[0][1], int)
        or not isinstance(xyz_buffers[0][1], bytes)
        or not isinstance(intensity_buffers[0][1], bytes)
    ):
        raise NuRecMultimodalError(
            "NRE 26.04 LiDAR response buffer fields are missing, duplicated, "
            "or use the wrong protobuf wire type"
        )
    point_count = int(point_counts[0][1])
    xyz_buffer = xyz_buffers[0][1]
    intensity_buffer = intensity_buffers[0][1]
    if point_count < 1:
        raise NuRecMultimodalError(
            "NRE 26.04 LiDAR response num_points must be positive"
        )
    expected_xyz_bytes = point_count * 3 * 4
    expected_intensity_bytes = point_count * 4
    if len(xyz_buffer) != expected_xyz_bytes:
        raise NuRecMultimodalError(
            "NRE 26.04 LiDAR XYZ buffer size does not match num_points"
        )
    if len(intensity_buffer) != expected_intensity_bytes:
        raise NuRecMultimodalError(
            "NRE 26.04 LiDAR intensity buffer size does not match num_points"
        )
    # Buffer hashes are deliberately covered by the caller's hash of the full
    # serialized protobuf.  Keep response_metadata within the canonical Scene
    # Exchange contract shared by legacy and buffered layouts.
    return {"point_count": point_count, "encoding": "float_xyz_intensity"}


def _lidar_xyzi_bytes(response: Any, body: bytes) -> bytes:
    """Return a canonical little-endian float32 XYZI stream for inference."""

    xyz = getattr(response, "point_xyzs", ())
    intensities = getattr(response, "point_intensities", ())
    if xyz or intensities:
        if not xyz or len(xyz) % 3 or len(intensities) != len(xyz) // 3:
            raise NuRecMultimodalError("cannot materialize malformed legacy LiDAR fields")
        point_count = len(intensities)
        result = bytearray(point_count * 16)
        for index in range(point_count):
            struct.pack_into(
                "<ffff",
                result,
                index * 16,
                float(xyz[index * 3]),
                float(xyz[index * 3 + 1]),
                float(xyz[index * 3 + 2]),
                float(intensities[index]),
            )
        return bytes(result)

    fields = _protobuf_wire_field_records(body)
    point_counts = fields.get(3, [])
    xyz_buffers = fields.get(4, [])
    intensity_buffers = fields.get(5, [])
    if (
        len(point_counts) != 1
        or len(xyz_buffers) != 1
        or len(intensity_buffers) != 1
        or not isinstance(point_counts[0][1], int)
        or not isinstance(xyz_buffers[0][1], bytes)
        or not isinstance(intensity_buffers[0][1], bytes)
    ):
        raise NuRecMultimodalError("cannot materialize malformed NRE 26.04 LiDAR buffers")
    point_count = int(point_counts[0][1])
    xyz_buffer = xyz_buffers[0][1]
    intensity_buffer = intensity_buffers[0][1]
    if len(xyz_buffer) != point_count * 12 or len(intensity_buffer) != point_count * 4:
        raise NuRecMultimodalError("cannot materialize inconsistent NRE LiDAR buffer sizes")
    result = bytearray(point_count * 16)
    for index in range(point_count):
        x, y, z = struct.unpack_from("<fff", xyz_buffer, index * 12)
        intensity = struct.unpack_from("<f", intensity_buffer, index * 4)[0]
        struct.pack_into("<ffff", result, index * 16, x, y, z, intensity)
    return bytes(result)


def _protobuf_wire_fields(data: bytes) -> dict[int, list[int | bytes]]:
    """Return top-level varint and length-delimited protobuf fields."""

    return {
        field_number: [value for _, value in records]
        for field_number, records in _protobuf_wire_field_records(data).items()
    }


def _protobuf_wire_field_records(
    data: bytes,
) -> dict[int, list[tuple[int, int | bytes]]]:
    """Return top-level protobuf fields without discarding their wire types."""

    fields: dict[int, list[tuple[int, int | bytes]]] = {}
    offset = 0
    while offset < len(data):
        key, offset = _read_protobuf_varint(data, offset)
        field_number = key >> 3
        wire_type = key & 7
        if field_number < 1 or field_number > (1 << 29) - 1:
            raise NuRecMultimodalError(
                f"NRE response contains invalid protobuf field {field_number}"
            )
        if wire_type == 0:
            value, offset = _read_protobuf_varint(data, offset)
        elif wire_type == 1:
            end = offset + 8
            if end > len(data):
                raise NuRecMultimodalError("NRE response has a truncated fixed64 field")
            value = data[offset:end]
            offset = end
        elif wire_type == 2:
            length, offset = _read_protobuf_varint(data, offset)
            end = offset + length
            if end > len(data):
                raise NuRecMultimodalError(
                    "NRE response has a truncated length-delimited field"
                )
            value = data[offset:end]
            offset = end
        elif wire_type == 5:
            end = offset + 4
            if end > len(data):
                raise NuRecMultimodalError("NRE response has a truncated fixed32 field")
            value = data[offset:end]
            offset = end
        else:
            raise NuRecMultimodalError(
                f"NRE response uses unsupported protobuf wire type {wire_type}"
            )
        fields.setdefault(field_number, []).append((wire_type, value))
    return fields


def _read_protobuf_varint(data: bytes, offset: int) -> tuple[int, int]:
    value = 0
    for index in range(10):
        if offset >= len(data):
            raise NuRecMultimodalError(
                "NRE response contains a truncated protobuf varint"
            )
        byte = data[offset]
        offset += 1
        if index == 9 and byte > 1:
            raise NuRecMultimodalError(
                "NRE response contains a protobuf varint larger than uint64"
            )
        value |= (byte & 0x7F) << (index * 7)
        if not byte & 0x80:
            return value, offset
    raise NuRecMultimodalError(
        "NRE response contains a protobuf varint larger than uint64"
    )
