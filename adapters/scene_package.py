from __future__ import annotations

from copy import deepcopy
from typing import Any


def build_scene_package(
    scene_ir: dict[str, Any],
    *,
    scene_ir_path: str,
    openscenario_path: str,
    opendrive_path: str | None,
    map_source: str,
    actor_bindings_path: str | None = None,
    nurec_usdz: str | None = None,
    nurec_checkpoint: str | None = None,
    reconstruction_package_path: str | None = None,
) -> dict[str, Any]:
    """Build the immutable handoff manifest for one complete source scene."""

    source = deepcopy(scene_ir.get("source") or {})
    if str(source.get("dataset", "")).lower() == "nuscenes":
        source["dataset"] = "nuscenes"
        if not source.get("scene_token"):
            raise ValueError("nuScenes source.scene_token is required")
    scene_id = str(source.get("scene_token") or scene_ir["scenario_id"])
    if source.get("dataset") == "nuscenes" and str(scene_ir["scenario_id"]) != scene_id:
        raise ValueError("nuScenes scenario_id must equal source.scene_token")
    source["scene_id"] = scene_id
    alignment = _sim_from_log_alignment(scene_ir)
    package = {
        "schema_version": "closed_loop_scene_package.v1",
        "scene_id": scene_id,
        "source": source,
        "coordinate_frame": deepcopy(scene_ir.get("coordinate_frame") or {}),
        "motion": {
            "scene_ir": str(scene_ir_path),
            "actor_bindings": str(actor_bindings_path) if actor_bindings_path else None,
        },
        "map": {
            "location": (scene_ir.get("map_context") or {}).get("location"),
            "opendrive": str(opendrive_path) if opendrive_path else None,
            "source": str(map_source),
        },
        "scenario": {"openscenario": str(openscenario_path)},
        "visual": {
            "nurec_usdz": str(nurec_usdz) if nurec_usdz else None,
            "nurec_checkpoint": str(nurec_checkpoint) if nurec_checkpoint else None,
            "reconstruction_package": (
                str(reconstruction_package_path) if reconstruction_package_path else None
            ),
        },
        "alignment": alignment,
    }
    from adapters.shared_protocol_validation import validate_document

    validate_document(package)
    return package


def _sim_from_log_alignment(scene_ir: dict[str, Any]) -> dict[str, Any]:
    frame = scene_ir.get("coordinate_frame") or {}
    origin = frame.get("origin_global_translation")
    yaw_deg = frame.get("origin_global_yaw_deg")
    if not isinstance(origin, list) or len(origin) != 3 or yaw_deg is None:
        return {"sim_from_log_transform": None, "status": "pending_runtime_alignment"}
    import math

    yaw = math.radians(float(yaw_deg))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    ox, oy, oz = (float(value) for value in origin)
    matrix = [
        cosine, sine, 0.0, -cosine * ox - sine * oy,
        -sine, cosine, 0.0, sine * ox - cosine * oy,
        0.0, 0.0, 1.0, -oz,
        0.0, 0.0, 0.0, 1.0,
    ]
    return {
        "sim_from_log_transform": matrix,
        "matrix_layout": "row_major_4x4",
        "source_frame": "nuscenes_global",
        "target_frame": "scene_local_ego_start",
        "status": "log_to_sim_defined",
    }
