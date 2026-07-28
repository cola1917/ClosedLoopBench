from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping

from adapters.nurec_multimodal import NuRecMultimodalError, validate_nurec_multimodal_frame


def build_nurec_dynamic_pose_request_record(
    context: Mapping[str, Any],
    frame: Mapping[str, Any],
    *,
    carla_to_nurec_global_transform: list[float],
) -> dict[str, Any]:
    """Freeze the exact dynamic-object payload before dispatching it to NuRec."""

    validate_nurec_multimodal_frame(frame)
    if context.get("schema_version") != "carla_nurec_frame_context.v1":
        raise NuRecMultimodalError("pose request trace requires carla_nurec_frame_context.v1")
    if context.get("scene_id") != frame.get("scene_id"):
        raise NuRecMultimodalError("pose request trace scene_id does not match NuRec frame")
    transform = list(carla_to_nurec_global_transform)
    if len(transform) != 16:
        raise NuRecMultimodalError("pose request trace requires a 4x4 CARLA-to-NuRec transform")
    samples = context.get("actor_samples")
    if not isinstance(samples, Mapping):
        raise NuRecMultimodalError("pose request trace requires actor_samples")

    physical_records = []
    for dynamic in frame["shared_dynamic_objects"]:
        actor_id = str(dynamic["actor_id"])
        sample = samples.get(actor_id)
        if not isinstance(sample, Mapping):
            raise NuRecMultimodalError(f"pose request trace is missing actor sample: {actor_id}")
        physical_pair = sample.get("carla_physical_pose_pair")
        if not isinstance(physical_pair, Mapping):
            raise NuRecMultimodalError(
                f"pose request trace is missing CARLA physical pose pair: {actor_id}"
            )
        physical_records.append(
            {
                "actor_id": actor_id,
                "nurec_track_id": dynamic["track_id"],
                "actor_type": dynamic["actor_type"],
                "carla_runtime_actor_id": sample.get("carla_runtime_actor_id"),
                "carla_physical_pose_reference": sample.get("carla_physical_pose_reference"),
                "carla_physical_pose_pair": deepcopy(dict(physical_pair)),
                "nurec_pose_source": dynamic["pose_source"],
                "nurec_pose_reference": dynamic["pose_reference"],
                "nurec_request_pose_pair": deepcopy(dict(dynamic["pose_pair"])),
                "extent_m": deepcopy(sample.get("extent_m")),
            }
        )
    absences = []
    for actor_id, sample in samples.items():
        if not isinstance(sample, Mapping) or not sample.get("absent"):
            continue
        absences.append(
            {
                "actor_id": str(actor_id),
                "nurec_track_id": sample.get("nurec_track_id"),
                "reason": sample.get("absent_reason"),
            }
        )
    return {
        "schema_version": "nurec_dynamic_pose_request.v1",
        "scene_id": frame["scene_id"],
        "frame_id": frame["frame_id"],
        "tick_index": context.get("tick_index"),
        "simulation_time_sec": frame["simulation_time_sec"],
        "scenario_time_sec": context.get("scenario_time_sec"),
        "pose_interval_sec": deepcopy(dict(frame["pose_interval_sec"])),
        "coordinate_contract": {
            "carla_input": "scene_local_ego_start",
            "nurec_request": "nuscenes_global",
            "carla_to_nurec_global_transform": transform,
        },
        "dynamic_object_sha256": frame["shared_dynamic_object_sha256"],
        "dynamic_objects": deepcopy(frame["shared_dynamic_objects"]),
        "actor_pose_pairs": physical_records,
        "actor_absences": sorted(absences, key=lambda item: item["actor_id"]),
    }
