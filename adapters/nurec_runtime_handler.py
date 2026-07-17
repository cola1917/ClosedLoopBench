from __future__ import annotations

from copy import deepcopy
from typing import Any, Callable, Iterable, Mapping

from adapters.nurec_multimodal import (
    NuRecMultimodalError,
    build_nurec_multimodal_frame,
    sensor_pose_pair_from_ego,
)


FrameDispatcher = Callable[[Mapping[str, Any]], Mapping[str, Any]]


def make_nurec_sensor_frame_handler(
    scene_package: Mapping[str, Any],
    binding_set: Mapping[str, Any],
    *,
    camera_specs: Iterable[Mapping[str, Any]],
    lidar_specs: Iterable[Mapping[str, Any]],
    dispatch_frame: FrameDispatcher,
    require_runtime_validated_alignment: bool = True,
) -> Callable[[Mapping[str, Any]], dict[str, Any]]:
    """Adapt CARLA runner frame contexts to synchronized NuRec dispatch calls."""

    cameras = _freeze_specs("rgb", camera_specs)
    lidars = _freeze_specs("lidar", lidar_specs)
    if not callable(dispatch_frame):
        raise NuRecMultimodalError("dispatch_frame must be callable")

    def handle(context: Mapping[str, Any]) -> dict[str, Any]:
        if context.get("schema_version") != "carla_nurec_frame_context.v1":
            raise NuRecMultimodalError("handler requires carla_nurec_frame_context.v1")
        if context.get("scene_id") != scene_package.get("scene_id"):
            raise NuRecMultimodalError("CARLA frame context scene_id does not match Scene Package")
        ego_pose_pair = context.get("ego_pose_pair")
        prepared_cameras = _prepare_sensors(cameras, ego_pose_pair)
        prepared_lidars = _prepare_sensors(lidars, ego_pose_pair)
        frame = build_nurec_multimodal_frame(
            scene_package,
            binding_set,
            frame_id=context["frame_id"],
            simulation_time_sec=context["simulation_time_sec"],
            interval_start_sec=context["interval_start_sec"],
            camera_specs=prepared_cameras,
            lidar_specs=prepared_lidars,
            actor_samples=context.get("actor_samples") or {},
            require_runtime_validated_alignment=require_runtime_validated_alignment,
        )
        evidence = dispatch_frame(frame)
        if not isinstance(evidence, Mapping):
            raise NuRecMultimodalError("dispatch_frame must return evidence")
        return dict(evidence)

    handle.runtime_contract = {  # type: ignore[attr-defined]
        "schema_version": "nurec_runtime_handler.v1",
        "scene_id": scene_package.get("scene_id"),
        "camera_ids": [item["sensor_id"] for item in cameras],
        "lidar_ids": [item["sensor_id"] for item in lidars],
        "clock": "carla_snapshot",
        "dynamic_objects": "actor_binding_set.v1",
    }
    return handle


def _freeze_specs(modality: str, specs: Iterable[Mapping[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for spec in specs:
        item = deepcopy(dict(spec))
        if not item.get("sensor_id") or not item.get("model"):
            raise NuRecMultimodalError(f"{modality} sensor_id and model are required")
        extrinsic = item.get("sensor_to_ego")
        if not isinstance(extrinsic, list) or len(extrinsic) != 16:
            raise NuRecMultimodalError(
                f"{modality} sensor {item['sensor_id']} requires sensor_to_ego 4x4 calibration"
            )
        result.append(item)
    if not result:
        raise NuRecMultimodalError(f"at least one {modality} sensor spec is required")
    ids = [str(item["sensor_id"]) for item in result]
    if len(ids) != len(set(ids)):
        raise NuRecMultimodalError(f"duplicate {modality} sensor IDs")
    return sorted(result, key=lambda item: str(item["sensor_id"]))


def _prepare_sensors(
    specs: list[dict[str, Any]],
    ego_pose_pair: Any,
) -> list[dict[str, Any]]:
    if not isinstance(ego_pose_pair, Mapping):
        raise NuRecMultimodalError("CARLA frame context has no ego pose pair")
    result = []
    for frozen in specs:
        item = deepcopy(frozen)
        extrinsic = item.pop("sensor_to_ego")
        item["pose_pair"] = sensor_pose_pair_from_ego(ego_pose_pair, extrinsic)
        result.append(item)
    return result
