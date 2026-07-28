from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping


CAMERA_IDS = {
    "camera_front",
    "camera_front_left",
    "camera_front_right",
    "camera_back",
    "camera_back_left",
    "camera_back_right",
}


class SceneObjectVisibilityError(ValueError):
    pass


def build_visibility_manifest(
    registry: Mapping[str, Any],
    run_config: Mapping[str, Any],
    frames: list[Mapping[str, Any]],
    nurec_trace: list[Mapping[str, Any]],
    calibration_capture: Mapping[str, Any],
    *,
    source_files: Mapping[str, Path] | None = None,
    max_range_m: float = 120.0,
) -> dict[str, Any]:
    """Project registered source objects into payload-proven six-camera frames.

    This is an M6 coverage check. It proves that each *geometrically visible*
    registered object has a CARLA counterpart and an exact NRE RGB payload. It
    intentionally does not infer semantic pixels or LiDAR occupancy; those are
    independent M8 checks.
    """

    scene_id = str(registry.get("scene_id") or "")
    if registry.get("schema_version") != "scene_object_registry.v1" or not scene_id:
        raise SceneObjectVisibilityError("a valid scene_object_registry.v1 is required")
    if calibration_capture.get("schema_version") != "nurec_camera_calibration_capture.v1":
        raise SceneObjectVisibilityError("a nurec_camera_calibration_capture.v1 is required")
    if calibration_capture.get("scene_id") != scene_id:
        raise SceneObjectVisibilityError("camera calibration scene_id does not match registry")
    if calibration_capture.get("intrinsics_status") != "passed":
        raise SceneObjectVisibilityError("camera calibration does not have source-bound intrinsics")
    if not math.isfinite(max_range_m) or max_range_m <= 0.0:
        raise SceneObjectVisibilityError("max_range_m must be finite and positive")

    cameras = _camera_models(run_config, calibration_capture)
    payloads = _complete_rgb_payloads(nurec_trace)
    frames_by_id = _frames_by_world_tick(frames)
    common_ids = sorted(set(cameras) and set(payloads) & set(frames_by_id))
    if not common_ids:
        raise SceneObjectVisibilityError("no frame has both CARLA state and complete six-camera NRE payloads")

    observations = []
    frame_rows = []
    for frame_id in common_ids:
        frame = frames_by_id[frame_id]
        ego = _scene_to_carla_pose(frame.get("ego_pose"), "frame ego_pose")
        frame_payloads = payloads[frame_id]
        frame_rows.append(
            {
                "world_tick_frame": frame_id,
                "simulation_time_sec": _number(frame.get("simulation_time_sec"), "simulation_time_sec"),
                "cameras": [frame_payloads[camera] for camera in sorted(cameras)],
            }
        )
        for record in registry.get("records") or []:
            if not isinstance(record, Mapping) or record.get("role") == "road_boundary":
                continue
            pose_and_extent = _record_pose_and_extent(record, frame.get("actor_states"))
            if pose_and_extent is None:
                # A registry is intentionally broader than one sensor tick.
                # A source track that has not started (or was already despawned)
                # cannot be claimed as visible in this frame.
                continue
            pose, extent = pose_and_extent
            projected = _project_box(pose, extent, ego, cameras, max_range_m)
            for camera_id, projection in projected.items():
                observations.append(
                    {
                        "object_id": str(record["object_id"]),
                        "source_track_id": (record.get("nurec") or {}).get("track_id"),
                        "camera": camera_id,
                        "frame_id": frame_id,
                        "t_sec": _number(frame.get("simulation_time_sec"), "simulation_time_sec"),
                        "safety_relevant": bool(record.get("safety_relevant", True)),
                        "observation_kind": "calibrated_3d_box_projection",
                        "projection": projection,
                        "evidence": {
                            "nre_payload_sha256": frame_payloads[camera_id]["payload_sha256"],
                            "nre_payload_relative_path": frame_payloads[camera_id].get("relative_path"),
                            "calibrated_sensor_token": cameras[camera_id]["calibrated_sensor_token"],
                            "intrinsics_table_sha256": cameras[camera_id]["intrinsics_table_sha256"],
                        },
                    }
                )
        # Road topology is represented through the camera payload set, never a
        # synthetic collision proxy. One row per actual camera makes that link explicit.
        boundary = next(
            (record for record in registry.get("records") or [] if isinstance(record, Mapping) and record.get("role") == "road_boundary"),
            None,
        )
        if boundary is not None:
            for camera_id in sorted(cameras):
                observations.append(
                    {
                        "object_id": str(boundary["object_id"]),
                        "camera": camera_id,
                        "frame_id": frame_id,
                        "t_sec": _number(frame.get("simulation_time_sec"), "simulation_time_sec"),
                        "safety_relevant": True,
                        "observation_kind": "road_topology_projection_target",
                        "evidence": {
                            "nre_payload_sha256": frame_payloads[camera_id]["payload_sha256"],
                            "nre_payload_relative_path": frame_payloads[camera_id].get("relative_path"),
                            "calibrated_sensor_token": cameras[camera_id]["calibrated_sensor_token"],
                            "intrinsics_table_sha256": cameras[camera_id]["intrinsics_table_sha256"],
                        },
                    }
                )

    return {
        "schema_version": "scene_object_visibility_manifest.v1",
        "scene_id": scene_id,
        "producer": {
            "name": "build_scene_object_visibility_manifest",
            "version": "v1",
            "scope": "calibrated_geometric_coverage",
            "limitations": [
                "No semantic detector or LiDAR occupancy claim is made here; both are M8 evidence.",
                "The manifest records geometrically visible source objects, not an occlusion-complete pixel segmentation.",
            ],
        },
        "source_files": _source_file_hashes(source_files or {}),
        "frames": frame_rows,
        "observations": observations,
        "summary": {
            "complete_six_camera_frame_count": len(common_ids),
            "observation_count": len(observations),
            "geometrically_visible_object_count": len({row["object_id"] for row in observations if row["observation_kind"] == "calibrated_3d_box_projection"}),
        },
    }


def _camera_models(run_config: Mapping[str, Any], capture: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    nurec = run_config.get("nurec_runtime")
    specs = nurec.get("camera_specs") if isinstance(nurec, Mapping) else None
    if not isinstance(specs, list):
        raise SceneObjectVisibilityError("run config requires nurec_runtime.camera_specs")
    requested = {str(item.get("sensor_id") or ""): item for item in specs if isinstance(item, Mapping)}
    captures = {
        str(item.get("sensor_id") or ""): item
        for item in capture.get("camera_records") or []
        if isinstance(item, Mapping)
    }
    if set(requested) != CAMERA_IDS or set(captures) != CAMERA_IDS:
        raise SceneObjectVisibilityError("exactly the six formal cameras are required")
    result = {}
    for sensor_id in sorted(CAMERA_IDS):
        item = captures[sensor_id]
        intrinsic = item.get("intrinsic_matrix_3x3")
        if not _intrinsic(intrinsic):
            raise SceneObjectVisibilityError(f"camera {sensor_id} has invalid intrinsic_matrix_3x3")
        transform = requested[sensor_id].get("sensor_to_ego")
        if not _rigid(transform):
            raise SceneObjectVisibilityError(f"camera {sensor_id} has invalid sensor_to_ego")
        if item.get("calibrated_sensor_token") != requested[sensor_id].get("calibrated_sensor_token"):
            raise SceneObjectVisibilityError(f"camera {sensor_id} calibration token differs from run config")
        source = item.get("intrinsics_source") or {}
        table_sha = str(source.get("table_sha256") or "") if isinstance(source, Mapping) else ""
        if len(table_sha) != 64:
            raise SceneObjectVisibilityError(f"camera {sensor_id} has no intrinsic source hash")
        result[sensor_id] = {
            "matrix": [[float(value) for value in row] for row in intrinsic],
            "sensor_to_ego": [float(value) for value in transform],
            "width": int(item.get("requested_resolution", {}).get("width") or 0),
            "height": int(item.get("requested_resolution", {}).get("height") or 0),
            "calibrated_sensor_token": str(item["calibrated_sensor_token"]),
            "intrinsics_table_sha256": table_sha,
        }
        if result[sensor_id]["width"] < 1 or result[sensor_id]["height"] < 1:
            raise SceneObjectVisibilityError(f"camera {sensor_id} has invalid requested resolution")
    return result


def _complete_rgb_payloads(trace: list[Mapping[str, Any]]) -> dict[int, dict[str, dict[str, Any]]]:
    result: dict[int, dict[str, dict[str, Any]]] = {}
    for row in trace:
        if str(row.get("status") or "") != "passed":
            continue
        frame_id = row.get("frame_id")
        if not isinstance(frame_id, int):
            continue
        records = row.get("records")
        if not isinstance(records, list):
            continue
        per_camera = {}
        for record in records:
            if not isinstance(record, Mapping) or record.get("modality") != "rgb" or record.get("status") != "passed":
                continue
            sensor_id = str(record.get("sensor_id") or "")
            metadata = record.get("response_metadata") or {}
            materialized = metadata.get("materialized_payload") if isinstance(metadata, Mapping) else None
            payload_hash = str((materialized or {}).get("sha256") or "") if isinstance(materialized, Mapping) else ""
            if sensor_id in CAMERA_IDS and len(payload_hash) == 64:
                per_camera[sensor_id] = {
                    "sensor_id": sensor_id,
                    "payload_sha256": payload_hash,
                    "relative_path": materialized.get("relative_path"),
                }
        if set(per_camera) == CAMERA_IDS:
            result[frame_id] = per_camera
    return result


def _frames_by_world_tick(frames: list[Mapping[str, Any]]) -> dict[int, Mapping[str, Any]]:
    result = {}
    for frame in frames:
        frame_id = frame.get("world_tick_frame")
        if not isinstance(frame_id, int):
            continue
        if frame_id in result:
            raise SceneObjectVisibilityError(f"duplicate CARLA world tick frame: {frame_id}")
        result[frame_id] = frame
    return result


def _record_pose_and_extent(
    record: Mapping[str, Any], actor_states: Any
) -> tuple[list[float], list[float]] | None:
    role = str(record.get("role") or "")
    object_id = str(record.get("object_id") or "")
    if role in {"background_replay", "controlled_lead_vehicle", "controlled_pedestrian"}:
        state = actor_states.get(object_id) if isinstance(actor_states, Mapping) else None
        # Background replay tracks are present physically in CARLA during M6,
        # but only selected actors are yet injected into NuRec. Their Scenario
        # IR reference pose is the honest source-scene pose for this coverage
        # projection; M7 later replaces this with a CARLA/NRE pose comparison.
        pose = (
            state.get("render_pose") or state.get("reference_pose")
            if isinstance(state, Mapping)
            else None
        )
        extent = state.get("extent_m") if isinstance(state, Mapping) else None
        if state is None:
            return None
        # M8 supplies independently measured CARLA bounding-box centre poses
        # rather than M6's source-render/reference pose.  This fallback is
        # intentionally only used when the explicit M6 fields are absent.
        if pose is None:
            pose = state.get("pose")
        if pose is None:
            return None
        if not isinstance(pose, Mapping) or not isinstance(extent, Mapping):
            raise SceneObjectVisibilityError(f"dynamic record {object_id} has incomplete frame actor state")
        return _scene_to_carla_pose(pose, f"actor {object_id} render_pose"), _extent(extent, object_id)
    state = actor_states.get(object_id) if isinstance(actor_states, Mapping) else None
    if isinstance(state, Mapping) and state.get("pose") is not None and state.get("extent_m") is not None:
        return (
            _scene_to_carla_pose(state["pose"], f"static {object_id} physical pose"),
            _extent(state["extent_m"], object_id),
        )
    placement = (record.get("carla") or {}).get("placement")
    if not isinstance(placement, Mapping):
        raise SceneObjectVisibilityError(f"static record {object_id} has no CARLA placement")
    return _scene_to_carla_pose(placement, f"static {object_id} placement"), _default_static_extent(record)


def _scene_to_carla_pose(pose: Any, label: str) -> list[float]:
    if not isinstance(pose, Mapping):
        raise SceneObjectVisibilityError(f"{label} must be an object")
    try:
        values = [float(pose[key]) for key in ("x", "y", "z")]
        yaw = float(pose.get("yaw") or 0.0)
    except (KeyError, TypeError, ValueError) as exc:
        raise SceneObjectVisibilityError(f"{label} requires finite x/y/z/yaw") from exc
    if not all(math.isfinite(value) for value in values + [yaw]):
        raise SceneObjectVisibilityError(f"{label} must be finite")
    return [values[0], -values[1], values[2], -math.radians(yaw)]


def _extent(value: Mapping[str, Any], label: str) -> list[float]:
    try:
        result = [float(value[axis]) for axis in ("x", "y", "z")]
    except (KeyError, TypeError, ValueError) as exc:
        raise SceneObjectVisibilityError(f"{label} requires x/y/z extents") from exc
    if not all(math.isfinite(item) and item > 0.0 for item in result):
        raise SceneObjectVisibilityError(f"{label} extents must be finite and positive")
    return result


def _default_static_extent(record: Mapping[str, Any]) -> list[float]:
    category = str(record.get("category") or "").lower()
    if "trafficcone" in category or "cone" in category:
        return [0.25, 0.25, 0.45]
    if "barrier" in category:
        return [1.0, 0.3, 0.6]
    if "vehicle" in category or str(record.get("semantic_class")) == "vehicle":
        return [2.5, 1.1, 1.2]
    return [0.75, 0.75, 0.75]


def _project_box(
    pose: list[float], extent: list[float], ego: list[float], cameras: Mapping[str, Mapping[str, Any]], max_range_m: float
) -> dict[str, dict[str, Any]]:
    distance = math.dist(pose[:3], ego[:3])
    if distance > max_range_m:
        return {}
    corners = _box_corners(pose, extent)
    result = {}
    for camera_id, camera in cameras.items():
        ego_from_camera = camera["sensor_to_ego"]
        camera_from_world = _inverse_rigid(_matmul(_pose_matrix(ego), ego_from_camera))
        points = [_transform_point(camera_from_world, corner) for corner in corners]
        front = [point for point in points if point[2] > 0.05]
        if not front:
            continue
        intrinsic = camera["matrix"]
        pixels = [
            [intrinsic[0][0] * point[0] / point[2] + intrinsic[0][2], intrinsic[1][1] * point[1] / point[2] + intrinsic[1][2]]
            for point in front
        ]
        min_x, max_x = min(point[0] for point in pixels), max(point[0] for point in pixels)
        min_y, max_y = min(point[1] for point in pixels), max(point[1] for point in pixels)
        width, height = float(camera["width"]), float(camera["height"])
        if max_x < 0.0 or min_x > width or max_y < 0.0 or min_y > height:
            continue
        result[camera_id] = {
            "bbox_xyxy_px": [max(0.0, min_x), max(0.0, min_y), min(width, max_x), min(height, max_y)],
            "depth_range_m": [min(point[2] for point in front), max(point[2] for point in front)],
            "distance_to_ego_m": distance,
        }
    return result


def _box_corners(pose: list[float], extent: list[float]) -> list[list[float]]:
    cos_yaw, sin_yaw = math.cos(pose[3]), math.sin(pose[3])
    result = []
    for dx in (-extent[0], extent[0]):
        for dy in (-extent[1], extent[1]):
            for dz in (-extent[2], extent[2]):
                result.append([pose[0] + cos_yaw * dx - sin_yaw * dy, pose[1] + sin_yaw * dx + cos_yaw * dy, pose[2] + dz])
    return result


def _pose_matrix(pose: list[float]) -> list[float]:
    cos_yaw, sin_yaw = math.cos(pose[3]), math.sin(pose[3])
    return [cos_yaw, -sin_yaw, 0.0, pose[0], sin_yaw, cos_yaw, 0.0, pose[1], 0.0, 0.0, 1.0, pose[2], 0.0, 0.0, 0.0, 1.0]


def _matmul(left: list[float], right: list[float]) -> list[float]:
    return [sum(left[row * 4 + index] * right[index * 4 + col] for index in range(4)) for row in range(4) for col in range(4)]


def _inverse_rigid(matrix: list[float]) -> list[float]:
    rotation = [[matrix[row * 4 + col] for col in range(3)] for row in range(3)]
    transpose = [[rotation[col][row] for col in range(3)] for row in range(3)]
    translation = [matrix[3], matrix[7], matrix[11]]
    inverse_translation = [-sum(transpose[row][col] * translation[col] for col in range(3)) for row in range(3)]
    return [transpose[0][0], transpose[0][1], transpose[0][2], inverse_translation[0], transpose[1][0], transpose[1][1], transpose[1][2], inverse_translation[1], transpose[2][0], transpose[2][1], transpose[2][2], inverse_translation[2], 0.0, 0.0, 0.0, 1.0]


def _transform_point(matrix: list[float], point: list[float]) -> list[float]:
    return [sum(matrix[row * 4 + col] * [point[0], point[1], point[2], 1.0][col] for col in range(4)) for row in range(3)]


def _intrinsic(value: Any) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(isinstance(row, list) and len(row) == 3 for row in value)


def _rigid(value: Any) -> bool:
    return isinstance(value, list) and len(value) == 16 and all(isinstance(item, (int, float)) and math.isfinite(float(item)) for item in value)


def _number(value: Any, label: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
        raise SceneObjectVisibilityError(f"{label} must be finite")
    return float(value)


def _source_file_hashes(files: Mapping[str, Path]) -> dict[str, dict[str, str]]:
    result = {}
    for name, path in sorted(files.items()):
        result[str(name)] = {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    return result
