from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_multimodal import validate_nurec_multimodal_frame


CAMERA_CHANNELS = (
    "CAM_FRONT",
    "CAM_FRONT_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
)
LOGICAL_IDS = {
    "CAM_FRONT": "camera_front",
    "CAM_FRONT_LEFT": "camera_front_left",
    "CAM_FRONT_RIGHT": "camera_front_right",
    "CAM_BACK": "camera_back",
    "CAM_BACK_LEFT": "camera_back_left",
    "CAM_BACK_RIGHT": "camera_back_right",
    "LIDAR_TOP": "lidar_top",
}


def prepare_probe_frames(
    dataroot: str | Path,
    *,
    version: str,
    scene_name: str,
    track_id: str,
    actor_type: str,
    camera_channels: Iterable[str] = CAMERA_CHANNELS,
    lidar_channel: str = "LIDAR_TOP",
    render_width: int = 400,
    render_height: int = 225,
    delta_m: float = 1.0,
    runtime_scene_start_us: int | None = None,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    if actor_type not in {"vehicle", "pedestrian", "two_wheeler", "object"}:
        raise ValueError(f"unsupported actor_type: {actor_type}")
    if not math.isfinite(delta_m) or delta_m < 0.05:
        raise ValueError("delta_m must be finite and at least 0.05")
    if render_width < 1 or render_height < 1:
        raise ValueError("render dimensions must be positive")

    root = Path(dataroot) / version
    tables = {
        name: _load_array(root / f"{name}.json")
        for name in (
            "scene",
            "sample",
            "sample_data",
            "sample_annotation",
            "ego_pose",
            "calibrated_sensor",
            "sensor",
        )
    }
    scenes = [row for row in tables["scene"] if row.get("name") == scene_name]
    if len(scenes) != 1:
        raise ValueError(f"expected one scene named {scene_name}, found {len(scenes)}")
    scene = scenes[0]
    samples = _scene_samples(scene, tables["sample"])
    sample_by_token = _index(samples)
    sample_data = _index(tables["sample_data"])
    ego_poses = _index(tables["ego_pose"])
    calibrations = _index(tables["calibrated_sensor"])
    sensors = _index(tables["sensor"])
    sample_channels = _sample_channel_index(
        tables["sample_data"], calibrations, sensors
    )

    annotations = [
        row
        for row in tables["sample_annotation"]
        if row.get("instance_token") == track_id and row.get("sample_token") in sample_by_token
    ]
    if not annotations:
        raise ValueError(f"track has no annotations in {scene_name}: {track_id}")
    selected = min(
        annotations,
        key=lambda annotation: _actor_ego_distance(
            annotation,
            sample_by_token[str(annotation["sample_token"])],
            lidar_channel,
            sample_data,
            ego_poses,
            sample_channels,
        ),
    )
    sample = sample_by_token[str(selected["sample_token"])]
    sample_index = next(index for index, row in enumerate(samples) if row["token"] == sample["token"])
    first_timestamp_us = int(samples[0]["timestamp"])
    sample_timestamp_us = int(sample["timestamp"])
    simulation_time_sec = (sample_timestamp_us - first_timestamp_us) / 1_000_000.0
    runtime_start_us = (
        first_timestamp_us
        if runtime_scene_start_us is None
        else int(runtime_scene_start_us)
    )
    if runtime_start_us < 0 or runtime_start_us > first_timestamp_us:
        raise ValueError(
            "runtime_scene_start_us must be non-negative and no later than the first sample"
        )

    actor_pose = _pose(selected["translation"], selected["rotation"])
    dynamic_object = {
        "actor_id": track_id,
        "track_id": track_id,
        "actor_type": actor_type,
        "pose_source": "scenario_ir_reference_trajectory",
        "pose_reference": "source_track_frame",
        "pose_pair": {"start": deepcopy(actor_pose), "end": deepcopy(actor_pose)},
    }
    dynamic_objects = [dynamic_object]
    dynamic_digest = _digest(dynamic_objects)

    camera_requests = []
    calibration_evidence = []
    for channel in camera_channels:
        request, evidence = _sensor_request(
            sample,
            channel,
            "rgb",
            sample_data,
            ego_poses,
            calibrations,
            sensors,
            sample_channels,
            dynamic_digest,
            scene_id=str(scene["token"]),
            frame_id=sample_index,
            parameters={"width": render_width, "height": render_height},
        )
        camera_requests.append(request)
        calibration_evidence.append(evidence)

    lidar_request, lidar_evidence = _sensor_request(
        sample,
        lidar_channel,
        "lidar",
        sample_data,
        ego_poses,
        calibrations,
        sensors,
        sample_channels,
        dynamic_digest,
        scene_id=str(scene["token"]),
        frame_id=sample_index,
        parameters={"device_type": "PANDAR128"},
    )
    calibration_evidence.append(lidar_evidence)

    baseline = {
        "schema_version": "nurec_multimodal_frame.v1",
        "scene_id": str(scene["token"]),
        "frame_id": sample_index,
        "simulation_time_sec": simulation_time_sec,
        "pose_interval_sec": {"start": simulation_time_sec, "end": simulation_time_sec},
        "coordinate_frame": {
            "input": "scene_local_ego_start",
            "render": "nuscenes_global",
            "transform_source": "closed_loop_scene_package.v1.alignment",
            "alignment_status": "runtime_validated",
        },
        "shared_dynamic_objects": dynamic_objects,
        "shared_dynamic_object_sha256": dynamic_digest,
        "modalities": {
            "rgb": {"requests": camera_requests},
            "lidar": {"requests": [lidar_request]},
        },
        "synchronization": {
            "clock": "carla_snapshot",
            "policy": "same_frame_same_pose_interval_same_dynamic_objects",
            "fail_closed": True,
        },
    }
    validate_nurec_multimodal_frame(baseline)

    moved = deepcopy(baseline)
    for endpoint in ("start", "end"):
        moved["shared_dynamic_objects"][0]["pose_pair"][endpoint]["position_m"]["x"] += delta_m
    moved_digest = _digest(moved["shared_dynamic_objects"])
    moved["shared_dynamic_object_sha256"] = moved_digest
    for modality in ("rgb", "lidar"):
        for request in moved["modalities"][modality]["requests"]:
            request["dynamic_object_sha256"] = moved_digest
    validate_nurec_multimodal_frame(moved)

    context = {
        "schema_version": "nurec_pose_probe_context.v1",
        "scene_name": scene_name,
        "scene_id": str(scene["token"]),
        "track_id": track_id,
        "actor_type": actor_type,
        "selection": "minimum_annotation_to_lidar_ego_xy_distance",
        "sample_token": str(sample["token"]),
        "sample_index": sample_index,
        "scene_start_us": runtime_start_us,
        "sample_time_origin_us": first_timestamp_us,
        "sample_timestamp_us": sample_timestamp_us,
        "simulation_time_sec": simulation_time_sec,
        "actor_ego_distance_m": _actor_ego_distance(
            selected, sample, lidar_channel, sample_data, ego_poses, sample_channels
        ),
        "pose_delta_global_x_m": delta_m,
        "calibrations": calibration_evidence,
    }
    return baseline, moved, context


def _sensor_request(
    sample: dict[str, Any],
    channel: str,
    modality: str,
    sample_data: dict[str, dict[str, Any]],
    ego_poses: dict[str, dict[str, Any]],
    calibrations: dict[str, dict[str, Any]],
    sensors: dict[str, dict[str, Any]],
    sample_channels: dict[tuple[str, str], str],
    dynamic_digest: str,
    *,
    scene_id: str,
    frame_id: int,
    parameters: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    data_token = sample_channels.get((str(sample.get("token")), channel))
    if data_token not in sample_data:
        raise ValueError(f"sample has no data for channel {channel}")
    data = sample_data[data_token]
    calibration = calibrations.get(str(data.get("calibrated_sensor_token")))
    ego = ego_poses.get(str(data.get("ego_pose_token")))
    if calibration is None or ego is None:
        raise ValueError(f"channel {channel} lacks calibration or ego pose")
    sensor = sensors.get(str(calibration.get("sensor_token")))
    if sensor is None or sensor.get("channel") != channel:
        raise ValueError(f"calibration channel mismatch for {channel}")
    global_pose = _compose_pose(ego, calibration)
    logical_id = LOGICAL_IDS.get(channel, channel.lower())
    model = "nurec_recorded_camera" if modality == "rgb" else "PANDAR128"
    request = {
        "request_id": f"{scene_id}:{frame_id}:{modality}:{logical_id}",
        "modality": modality,
        "sensor": {
            "sensor_id": logical_id,
            "model": model,
            "parameters": parameters,
            "pose_pair": {"start": deepcopy(global_pose), "end": deepcopy(global_pose)},
        },
        "dynamic_object_sha256": dynamic_digest,
    }
    evidence = {
        "channel": channel,
        "logical_id": logical_id,
        "sample_data_token": str(data["token"]),
        "sample_data_timestamp_us": int(data["timestamp"]),
        "ego_pose_token": str(data["ego_pose_token"]),
        "calibrated_sensor_token": str(data["calibrated_sensor_token"]),
        "sensor_to_ego_translation": calibration["translation"],
        "sensor_to_ego_rotation_wxyz": calibration["rotation"],
        "global_sensor_pose": global_pose,
    }
    return request, evidence


def _compose_pose(ego: dict[str, Any], calibration: dict[str, Any]) -> dict[str, Any]:
    ego_q = tuple(float(value) for value in ego["rotation"])
    sensor_q = tuple(float(value) for value in calibration["rotation"])
    rotated = _rotate(ego_q, tuple(float(value) for value in calibration["translation"]))
    translation = [float(ego["translation"][i]) + rotated[i] for i in range(3)]
    return _pose(translation, _multiply_quaternion(ego_q, sensor_q))


def _pose(translation: Iterable[Any], rotation_wxyz: Iterable[Any]) -> dict[str, Any]:
    x, y, z = (float(value) for value in translation)
    w, qx, qy, qz = _normalized_quaternion(rotation_wxyz)
    return {
        "position_m": {"x": x, "y": y, "z": z},
        "orientation_xyzw": {"x": qx, "y": qy, "z": qz, "w": w},
    }


def _actor_ego_distance(
    annotation: dict[str, Any],
    sample: dict[str, Any],
    lidar_channel: str,
    sample_data: dict[str, dict[str, Any]],
    ego_poses: dict[str, dict[str, Any]],
    sample_channels: dict[tuple[str, str], str],
) -> float:
    token = sample_channels.get((str(sample.get("token")), lidar_channel))
    if token not in sample_data:
        raise ValueError(f"sample has no {lidar_channel}")
    ego = ego_poses.get(str(sample_data[token].get("ego_pose_token")))
    if ego is None:
        raise ValueError(f"sample {sample.get('token')} has no LiDAR ego pose")
    return math.hypot(
        float(annotation["translation"][0]) - float(ego["translation"][0]),
        float(annotation["translation"][1]) - float(ego["translation"][1]),
    )


def _scene_samples(scene: dict[str, Any], rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_token = _index(rows)
    result = []
    token = str(scene.get("first_sample_token") or "")
    while token:
        if token not in by_token:
            raise ValueError(f"scene sample chain references missing token: {token}")
        row = by_token[token]
        result.append(row)
        token = str(row.get("next") or "")
    if len(result) != int(scene.get("nbr_samples") or len(result)):
        raise ValueError("scene sample chain length does not match nbr_samples")
    return result


def _index(rows: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    return {str(row["token"]): row for row in rows}


def _sample_channel_index(
    sample_data_rows: list[dict[str, Any]],
    calibrations: dict[str, dict[str, Any]],
    sensors: dict[str, dict[str, Any]],
) -> dict[tuple[str, str], str]:
    candidates: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in sample_data_rows:
        calibration = calibrations.get(str(row.get("calibrated_sensor_token")))
        sensor = (
            sensors.get(str(calibration.get("sensor_token")))
            if calibration is not None
            else None
        )
        channel = str((sensor or {}).get("channel") or "")
        sample_token = str(row.get("sample_token") or "")
        if channel and sample_token:
            candidates.setdefault((sample_token, channel), []).append(row)

    result = {}
    for key, rows in candidates.items():
        keyframes = [row for row in rows if row.get("is_key_frame") is True]
        selected = keyframes[0] if len(keyframes) == 1 else None
        if selected is None:
            if len(rows) != 1:
                raise ValueError(
                    f"expected one keyframe sample_data row for sample/channel {key}, found {len(keyframes)}"
                )
            selected = rows[0]
        result[key] = str(selected["token"])
    return result


def _load_array(path: Path) -> list[dict[str, Any]]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, list) or any(not isinstance(row, dict) for row in value):
        raise ValueError(f"expected an array of objects: {path}")
    return value


def _normalized_quaternion(values: Iterable[Any]) -> tuple[float, float, float, float]:
    w, x, y, z = (float(value) for value in values)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError("quaternion must be finite and non-zero")
    return w / norm, x / norm, y / norm, z / norm


def _multiply_quaternion(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = _normalized_quaternion(left)
    rw, rx, ry, rz = _normalized_quaternion(right)
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _rotate(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    w, x, y, z = _normalized_quaternion(quaternion)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _digest(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build fixed-time NuRec RGB/LiDAR A/A/B frames from nuScenes calibration."
    )
    parser.add_argument("--dataroot", required=True, type=Path)
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument("--scene", required=True)
    parser.add_argument("--track-id", required=True)
    parser.add_argument("--actor-type", required=True)
    parser.add_argument("--camera-channel", action="append", default=[])
    parser.add_argument("--lidar-channel", default="LIDAR_TOP")
    parser.add_argument("--render-width", type=int, default=400)
    parser.add_argument("--render-height", type=int, default=225)
    parser.add_argument("--delta-m", type=float, default=1.0)
    parser.add_argument(
        "--runtime-scene-start-us",
        type=int,
        help="Measured NRE frame-start timestamp; defaults to the first nuScenes sample timestamp.",
    )
    parser.add_argument("--baseline-output", required=True, type=Path)
    parser.add_argument("--moved-output", required=True, type=Path)
    parser.add_argument("--context-output", required=True, type=Path)
    args = parser.parse_args(argv)
    outputs = (args.baseline_output, args.moved_output, args.context_output)
    existing = [str(path) for path in outputs if path.exists()]
    if existing:
        parser.error("outputs already exist: " + ", ".join(existing))
    try:
        baseline, moved, context = prepare_probe_frames(
            args.dataroot,
            version=args.version,
            scene_name=args.scene,
            track_id=args.track_id,
            actor_type=args.actor_type,
            camera_channels=args.camera_channel or CAMERA_CHANNELS,
            lidar_channel=args.lidar_channel,
            render_width=args.render_width,
            render_height=args.render_height,
            delta_m=args.delta_m,
            runtime_scene_start_us=args.runtime_scene_start_us,
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    for path, value in zip(outputs, (baseline, moved, context)):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(context, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
