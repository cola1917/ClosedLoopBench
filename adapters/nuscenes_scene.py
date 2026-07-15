from __future__ import annotations

import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable


DEFAULT_METRICS = ["collision", "min_ttc", "route_progress", "comfort_jerk", "rule_violation"]


class NuScenesDataError(ValueError):
    """Raised when the nuScenes metadata cannot describe the requested scene."""


def build_scene_ir(
    dataroot: str | Path,
    scene: str,
    *,
    version: str = "v1.0-mini",
) -> dict[str, Any]:
    """Build normalized Scenario IR directly from nuScenes metadata tables.

    ``scene`` may be either the human-readable scene name or its token. The
    first keyframe ego pose defines the local origin and heading.
    """

    root = Path(dataroot)
    table_root = root / version
    if not table_root.is_dir():
        raise FileNotFoundError(f"nuScenes metadata directory not found: {table_root}")

    tables = _load_tables(
        table_root,
        (
            "scene",
            "sample",
            "sample_data",
            "ego_pose",
            "calibrated_sensor",
            "sensor",
            "sample_annotation",
            "instance",
            "category",
            "log",
        ),
    )
    scene_record = next(
        (
            record
            for record in tables["scene"]
            if record.get("name") == scene or record.get("token") == scene
        ),
        None,
    )
    if scene_record is None:
        raise NuScenesDataError(f"nuScenes scene not found by name or token: {scene}")

    samples_by_token = _index(tables["sample"])
    samples = _walk_sample_chain(scene_record, samples_by_token)
    if not samples:
        raise NuScenesDataError(f"scene {scene_record['name']} has no samples")

    sample_data_by_sample: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in tables["sample_data"]:
        if record.get("sample_token") in samples_by_token:
            sample_data_by_sample[str(record["sample_token"])].append(record)

    ego_poses = _index(tables["ego_pose"])
    calibrated = _index(tables["calibrated_sensor"])
    sensors = _index(tables["sensor"])
    ego_records = [
        _ego_pose_for_sample(sample, sample_data_by_sample, ego_poses, calibrated, sensors)
        for sample in samples
    ]

    origin_translation = tuple(float(value) for value in ego_records[0]["translation"])
    origin_yaw = _quaternion_yaw(ego_records[0]["rotation"])
    start_timestamp = int(samples[0]["timestamp"])
    end_timestamp = int(samples[-1]["timestamp"])

    ego_trajectory = [
        _state_from_pose(
            int(sample["timestamp"]),
            pose["translation"],
            pose["rotation"],
            start_timestamp,
            origin_translation,
            origin_yaw,
        )
        for sample, pose in zip(samples, ego_records)
    ]
    _add_speeds(ego_trajectory)

    sample_tokens = {str(sample["token"]) for sample in samples}
    instances = _index(tables["instance"])
    categories = _index(tables["category"])
    annotations_by_instance: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for annotation in tables["sample_annotation"]:
        if annotation.get("sample_token") in sample_tokens:
            annotations_by_instance[str(annotation["instance_token"])].append(annotation)

    actors = []
    for instance_token, annotations in sorted(annotations_by_instance.items()):
        annotations.sort(key=lambda record: int(samples_by_token[record["sample_token"]]["timestamp"]))
        trajectory = [
            _state_from_pose(
                int(samples_by_token[annotation["sample_token"]]["timestamp"]),
                annotation["translation"],
                annotation["rotation"],
                start_timestamp,
                origin_translation,
                origin_yaw,
            )
            for annotation in annotations
        ]
        _add_speeds(trajectory)
        instance = instances.get(instance_token, {})
        category = categories.get(str(instance.get("category_token", "")), {})
        category_name = str(category.get("name", "unknown"))
        actors.append(
            {
                "actor_id": instance_token,
                "source_track_id": instance_token,
                "role": "context",
                "type": _actor_type(category_name),
                "category": category_name,
                "dimensions": _dimensions(annotations[0].get("size", [])),
                "initial_state": trajectory[0],
                "reference_trajectory": trajectory,
                "policy_hints": {
                    "mvp": "replay",
                    "final_closed_loop": "reference_conditioned_reactive_rule_based",
                },
                "source_annotation_tokens": [str(record["token"]) for record in annotations],
            }
        )

    logs = _index(tables["log"])
    log_record = logs.get(str(scene_record.get("log_token", "")), {})
    duration_sec = (end_timestamp - start_timestamp) / 1_000_000.0
    scene_token = str(scene_record["token"])
    scenario_ir = {
        "schema_version": "scenario_ir.v1",
        "scenario_id": scene_token,
        "scenario_type": "nuscenes_reconstructed_scene",
        "windows": {
            "event": {"start_sec": 0.0, "end_sec": duration_sec},
            "warmup": {"start_sec": 0.0, "end_sec": 0.0},
            "reconstruction": {"start_sec": 0.0, "end_sec": duration_sec},
        },
        "ego": {
            "track_id": "ego",
            "initial_state": ego_trajectory[0],
            "reference_trajectory": ego_trajectory,
            "route": {
                "source": "reference_trajectory",
                "note": "Route is inferred from nuScenes ego keyframe poses.",
            },
        },
        "actors": actors,
        "evaluation": {"metrics": list(DEFAULT_METRICS)},
        "source": {
            "dataset": "nuscenes",
            "scene_id": scene_token,
            "root": str(root),
            "version": version,
            "scene_name": str(scene_record["name"]),
            "scene_token": scene_token,
            "description": str(scene_record.get("description", "")),
            "first_sample_token": str(scene_record["first_sample_token"]),
            "last_sample_token": str(scene_record["last_sample_token"]),
            "start_timestamp_us": start_timestamp,
            "end_timestamp_us": end_timestamp,
            "sample_count": len(samples),
        },
        "map_context": {
            "location": log_record.get("location"),
            "log_token": scene_record.get("log_token"),
            "feature_counts": {},
            "features": [],
        },
        "coordinate_frame": {
            "name": "scene_local_ego_start",
            "units": {"position": "meter", "time": "second", "yaw": "degree"},
            "handedness": "right",
            "x_axis": "initial_ego_forward",
            "y_axis": "initial_ego_left",
            "origin_global_translation": list(origin_translation),
            "origin_global_rotation_wxyz": [float(value) for value in ego_records[0]["rotation"]],
            "origin_global_yaw_deg": math.degrees(origin_yaw),
            "transform": "local_xy = R(-origin_yaw) * (global_xy - origin_xy)",
        },
        "sensors": {
            "available_capabilities": ["camera", "lidar", "radar"],
            "camera_calibration": "deferred_to_dataset_adapter",
        },
        "events": {"trigger": None, "mined_events": []},
        "data_requirements": {
            "reconstruction": {
                "required": ["camera_images", "camera_calibration", "ego_pose", "actor_tracks"]
            },
            "closed_loop": {
                "required": ["ego_initial_state", "actor_initial_states", "map_context"]
            },
        },
        "risk_metrics": {
            "trigger_time_sec": 0.0,
            "trigger_tag": None,
            "actor_count": len(actors),
            "ego_reference_state_count": len(ego_trajectory),
        },
        "dataset_refs": {
            "source": {
                "dataset": "nuscenes",
                "root": str(root),
                "scene_id": scene_token,
                "version": version,
                "scene_name": str(scene_record["name"]),
                "scene_token": scene_token,
            },
            "sample_refs": {"status": "deferred", "refs": []},
            "index_refs": {"status": "deferred", "refs": []},
        },
        "variants": {
            "mvp": {
                "ego_speed_delta_mps": [-2.0, 0.0, 2.0],
                "actor_start_time_delta_sec": [-1.0, 0.0, 1.0],
                "weather": ["clear"],
            },
            "final_closed_loop": {
                "ego_policy": ["baseline", "ros2_stack"],
                "actor_policy": ["replay", "reactive_rule_based"],
                "sensor_domain": ["carla", "nurec", "cosmos_transfer1"],
            },
        },
        "diagnostics": {
            "actor_count": len(actors),
            "ego_pose_source": "sample keyframe LIDAR_TOP preferred, otherwise first available sensor",
        },
    }
    from adapters.shared_protocol_validation import validate_document

    validate_document(scenario_ir)
    return scenario_ir


def _load_tables(root: Path, names: Iterable[str]) -> dict[str, list[dict[str, Any]]]:
    tables = {}
    for name in names:
        path = root / f"{name}.json"
        if not path.is_file():
            raise FileNotFoundError(f"required nuScenes table not found: {path}")
        value = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(value, list):
            raise NuScenesDataError(f"nuScenes table must contain a JSON array: {path}")
        tables[name] = value
    return tables


def _index(records: Iterable[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    return {str(record["token"]): record for record in records}


def _walk_sample_chain(
    scene_record: dict[str, Any], samples_by_token: dict[str, dict[str, Any]]
) -> list[dict[str, Any]]:
    result = []
    token = str(scene_record["first_sample_token"])
    seen = set()
    while token:
        if token in seen:
            raise NuScenesDataError(f"cycle in sample chain at token {token}")
        seen.add(token)
        sample = samples_by_token.get(token)
        if sample is None:
            raise NuScenesDataError(f"sample token referenced by scene is missing: {token}")
        if sample.get("scene_token") != scene_record.get("token"):
            raise NuScenesDataError(f"sample {token} belongs to a different scene")
        result.append(sample)
        if token == scene_record.get("last_sample_token"):
            break
        token = str(sample.get("next", ""))
    if not result or result[-1].get("token") != scene_record.get("last_sample_token"):
        raise NuScenesDataError("sample chain ended before last_sample_token")
    return result


def _ego_pose_for_sample(
    sample: dict[str, Any],
    by_sample: dict[str, list[dict[str, Any]]],
    ego_poses: dict[str, dict[str, Any]],
    calibrated: dict[str, dict[str, Any]],
    sensors: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    candidates = by_sample.get(str(sample["token"]), [])
    if not candidates:
        raise NuScenesDataError(f"sample has no keyframe sample_data: {sample['token']}")

    def priority(record: dict[str, Any]) -> tuple[int, int, int, str]:
        calibration = calibrated.get(str(record.get("calibrated_sensor_token", "")), {})
        sensor = sensors.get(str(calibration.get("sensor_token", "")), {})
        return (
            0 if sensor.get("channel") == "LIDAR_TOP" else 1,
            0 if record.get("is_key_frame", False) else 1,
            abs(int(record.get("timestamp", sample["timestamp"])) - int(sample["timestamp"])),
            str(record.get("token", "")),
        )

    selected = min(candidates, key=priority)
    pose = ego_poses.get(str(selected.get("ego_pose_token", "")))
    if pose is None:
        raise NuScenesDataError(f"sample_data references missing ego pose: {selected.get('token')}")
    return pose


def _state_from_pose(
    timestamp_us: int,
    translation: Iterable[float],
    rotation: Iterable[float],
    start_timestamp_us: int,
    origin: tuple[float, float, float],
    origin_yaw: float,
) -> dict[str, float]:
    px, py, pz = (float(value) for value in translation)
    dx, dy = px - origin[0], py - origin[1]
    cos_yaw, sin_yaw = math.cos(origin_yaw), math.sin(origin_yaw)
    x = cos_yaw * dx + sin_yaw * dy
    y = -sin_yaw * dx + cos_yaw * dy
    yaw = _normalize_angle(_quaternion_yaw(rotation) - origin_yaw)
    return {
        "t_sec": (timestamp_us - start_timestamp_us) / 1_000_000.0,
        "x": x,
        "y": y,
        "z": pz - origin[2],
        "yaw": math.degrees(yaw),
        "speed_mps": 0.0,
    }


def _quaternion_yaw(rotation: Iterable[float]) -> float:
    values = [float(value) for value in rotation]
    if len(values) != 4:
        raise NuScenesDataError(f"expected wxyz quaternion, got {values}")
    w, x, y, z = values
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _add_speeds(trajectory: list[dict[str, float]]) -> None:
    if len(trajectory) < 2:
        return
    for index, state in enumerate(trajectory):
        before = trajectory[max(0, index - 1)]
        after = trajectory[min(len(trajectory) - 1, index + 1)]
        delta_time = after["t_sec"] - before["t_sec"]
        if delta_time > 0:
            state["speed_mps"] = math.hypot(after["x"] - before["x"], after["y"] - before["y"]) / delta_time


def _actor_type(category: str) -> str:
    if category.startswith("human.pedestrian"):
        return "pedestrian"
    if category.startswith("vehicle.bicycle") or category.startswith("vehicle.motorcycle"):
        return "two_wheeler"
    if category.startswith("vehicle"):
        return "vehicle"
    return "object"


def _dimensions(size: Iterable[float]) -> dict[str, float]:
    values = [float(value) for value in size]
    if len(values) != 3:
        return {"width": 0.0, "length": 0.0, "height": 0.0}
    return {"width": values[0], "length": values[1], "height": values[2]}
