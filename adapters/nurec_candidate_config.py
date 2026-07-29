"""Derive a bounded NuRec candidate config from audited M8 selections."""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from typing import Any


class NuRecCandidateConfigError(ValueError):
    """Raised when a candidate config cannot be derived safely."""


_CAMERAS = (
    "camera_front",
    "camera_front_left",
    "camera_front_right",
    "camera_back",
    "camera_back_left",
    "camera_back_right",
)


def derive_candidate_config(
    source_config: Mapping[str, Any],
    registry: Mapping[str, Any],
    render_selection: Mapping[str, Any],
    quality_windows: Mapping[str, Any],
    *,
    max_samples_per_epoch: int = 1000,
    max_epochs: int = 1,
) -> dict[str, Any]:
    """Return a fresh low-budget config for a selected candidate scene.

    The complete CARLA registry is intentionally not copied into the NuRec
    layer list.  The selection is only a render candidate; CARLA physics keeps
    its complete registry and is audited separately.
    """

    if not isinstance(source_config, Mapping):
        raise NuRecCandidateConfigError("source config must be an object")
    _validate_registry(registry)
    if render_selection.get("schema_version") != "nurec_render_selection.v1":
        raise NuRecCandidateConfigError("render selection must use nurec_render_selection.v1")
    if render_selection.get("status") != "passed":
        raise NuRecCandidateConfigError("render selection is not passed")
    if quality_windows.get("schema_version") != "lidar_quality_window_manifest.v1":
        raise NuRecCandidateConfigError("quality window manifest must use lidar_quality_window_manifest.v1")
    if quality_windows.get("status") != "passed":
        raise NuRecCandidateConfigError("quality window manifest is not passed")
    if not isinstance(max_samples_per_epoch, int) or isinstance(max_samples_per_epoch, bool) or max_samples_per_epoch < 1:
        raise NuRecCandidateConfigError("max_samples_per_epoch must be positive")
    if not isinstance(max_epochs, int) or isinstance(max_epochs, bool) or max_epochs < 1:
        raise NuRecCandidateConfigError("max_epochs must be positive")

    records = {str(row["object_id"]): row for row in registry["records"]}
    selected = {str(value) for value in render_selection.get("selected_object_ids") or [] if str(value)}
    if not selected:
        raise NuRecCandidateConfigError("render selection has no selected_object_ids")
    unknown = sorted(selected - set(records))
    if unknown:
        raise NuRecCandidateConfigError("render selection contains unknown objects: " + ", ".join(unknown))
    candidate_ids = {str(value) for value in quality_windows.get("candidate_object_ids") or [] if str(value)}
    required_ids = {str(value) for value in quality_windows.get("required_object_ids") or [] if str(value)}
    if not candidate_ids or not required_ids.issubset(candidate_ids):
        raise NuRecCandidateConfigError("quality window manifest has invalid candidate/required IDs")
    if not candidate_ids.issubset(selected):
        raise NuRecCandidateConfigError("quality-window candidates are absent from render selection")

    dynamic_rigids: list[str] = []
    dynamic_deformables: list[str] = []
    static_selected: list[str] = []
    for object_id in sorted(selected):
        record = records[object_id]
        track_id = str((record.get("nurec") or {}).get("track_id") or "")
        role = str(record.get("role") or "")
        if role == "road_boundary":
            continue
        if role == "static_obstacle":
            static_selected.append(object_id)
            continue
        if role not in {"background_replay", "controlled_lead_vehicle", "controlled_pedestrian"}:
            raise NuRecCandidateConfigError(f"object {object_id} has unsupported candidate role: {role}")
        if not track_id:
            raise NuRecCandidateConfigError(f"dynamic candidate {object_id} has no NuRec track_id")
        semantic = str(record.get("semantic_class") or "")
        if semantic == "vehicle":
            dynamic_rigids.append(track_id)
        elif semantic in {"pedestrian", "two_wheeler"}:
            dynamic_deformables.append(track_id)
        else:
            raise NuRecCandidateConfigError(f"dynamic candidate {object_id} has unsupported semantic_class: {semantic}")
    if static_selected:
        raise NuRecCandidateConfigError(
            "selected static objects require an explicit static NuRec geometry layer; refusing a fake generation path: "
            + ", ".join(static_selected)
        )
    if not required_ids.issubset(selected):
        raise NuRecCandidateConfigError("required quality-window objects are absent from selection")
    required_tracks = {
        str((records[object_id].get("nurec") or {}).get("track_id") or "")
        for object_id in required_ids
    }
    if not required_tracks.issubset(set(dynamic_rigids + dynamic_deformables)):
        raise NuRecCandidateConfigError("required quality-window tracks are not configured in a dynamic layer")

    result = deepcopy(dict(source_config))
    dataset = _mapping(result, "dataset")
    trainer = _mapping(result, "trainer")
    checkpoint = _mapping(result, "checkpoint")
    artifact = _mapping(checkpoint, "artifact")
    sequence = _mapping(artifact, "sequence_tracks")
    dataset["n_samples_per_epoch"] = max_samples_per_epoch
    trainer["max_epochs"] = max_epochs
    sequence["enabled"] = True
    dataset["camera_ids"] = list(_CAMERAS)
    dataset["train_camera_ids"] = list(_CAMERAS)
    dataset["lidar_ids"] = ["lidar_top"]
    dataset["train_lidar_ids"] = ["lidar_top"]
    cuboids = _mapping(dataset, "cuboid_tracks_params")
    cuboids["track_label_sources"] = ["EXTERNAL"]
    dataset["generate_static_rigid_cuboid_tracks"] = {"enabled": False}
    layers = _mapping(_mapping(result, "model"), "layers")
    _set_layer_track_ids(layers, "dynamic_rigids", dynamic_rigids)
    _set_layer_track_ids(layers, "dynamic_deformables", dynamic_deformables)
    return result


def _set_layer_track_ids(layers: dict[str, Any], name: str, ids: list[str]) -> None:
    layer = _mapping(layers, name)
    tracks = _mapping(layer, "tracks")
    tracks["ids"] = list(ids)
    tracks["is_dynamic"] = True


def _mapping(parent: dict[str, Any], key: str) -> dict[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        value = {}
        parent[key] = value
    return value


def _validate_registry(registry: Mapping[str, Any]) -> None:
    if registry.get("schema_version") != "scene_object_registry.v1":
        raise NuRecCandidateConfigError("registry must use scene_object_registry.v1")
    if not isinstance(registry.get("records"), list) or not registry["records"]:
        raise NuRecCandidateConfigError("registry.records must be non-empty")
    if any(not isinstance(row, Mapping) or not str(row.get("object_id") or "") for row in registry["records"]):
        raise NuRecCandidateConfigError("registry records require object_id")


__all__ = ["NuRecCandidateConfigError", "derive_candidate_config"]
