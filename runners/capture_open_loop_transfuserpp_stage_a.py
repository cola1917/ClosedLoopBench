"""Capture native CARLA RGB/LiDAR observations for TransFuser++ Stage A.

The ego pose is owned by the Scenario IR.  CARLA is used only to render the
native sensors at each GT pose; no control is applied and physics is disabled
for the replay vehicle.  The resulting trace is suitable for
``run_open_loop_transfuserpp_stage_a`` and fails closed on missing or
frame-mismatched sensor data.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import queue
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agents.plugin_contract import canonical_sha256
from agents.ros2_observation_control_driver import _world_to_ego
from agents.transfuserpp_contract import (
    camera_adaptation_contract,
    validate_observation,
)
from adapters.open_loop_bbox_binding import (
    frame_binding,
    frame_dynamic_objects,
    load_actor_manifest,
)
from runners.run_open_loop_gt_replay import (
    EXPECTED_OPENDRIVE_SHA256,
    EXPECTED_SCENARIO_IR_SHA256,
    load_pinned_inputs,
)


TRACE_SCHEMA = "transfuserpp_stage_a_observation_trace.v1"
SOURCE = "carla_stage_a_native_rgb_lidar"
CAMERA_WIDTH = 1600
CAMERA_HEIGHT = 900
CAMERA_FOV_DEG = 100.0
FIXED_DELTA_SECONDS = 0.05
CAMERA_MOUNT = (1.5, 0.0, 2.4)
LIDAR_MOUNT = (0.0, 0.0, 2.5)


class StageACaptureError(RuntimeError):
    """Raised when native Stage A evidence cannot be made complete."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _file_ref(
    path: Path,
    *,
    container_path: str,
    encoding: str,
    coordinate_frame: str,
    axis_convention: str | None = None,
    sensor_to_ego: list[float] | None = None,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "path": container_path,
        "sha256": _sha256(path),
        "byte_count": path.stat().st_size,
        "encoding": encoding,
        "coordinate_frame": coordinate_frame,
    }
    if axis_convention is not None:
        result["axis_convention"] = axis_convention
    if sensor_to_ego is not None:
        result["sensor_to_ego"] = list(sensor_to_ego)
    return result


def _translation_matrix(x: float, y: float, z: float) -> list[float]:
    return [
        1.0,
        0.0,
        0.0,
        float(x),
        0.0,
        1.0,
        0.0,
        float(y),
        0.0,
        0.0,
        1.0,
        float(z),
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def _calibration() -> dict[str, Any]:
    camera_matrix = _translation_matrix(*CAMERA_MOUNT)
    lidar_matrix = _translation_matrix(*LIDAR_MOUNT)
    focal = CAMERA_WIDTH / (2.0 * math.tan(math.radians(CAMERA_FOV_DEG) / 2.0))
    return {
        "camera_sensor_id": "camera_front",
        "camera_sensor_to_ego": camera_matrix,
        "camera_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
        "camera_intrinsic": [
            [focal, 0.0, CAMERA_WIDTH / 2.0],
            [0.0, focal, CAMERA_HEIGHT / 2.0],
            [0.0, 0.0, 1.0],
        ],
        "lidar_sensor_id": "lidar_top",
        "lidar_sensor_to_ego": lidar_matrix,
        "lidar_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
        "camera_adaptation": camera_adaptation_contract(),
    }


def _route_for_frame(
    track: list[dict[str, float]], frame_index: int, *, lookahead_m: float = 7.5
) -> dict[str, Any]:
    state = track[frame_index]
    distances = [0.0]
    for left, right in zip(track, track[1:]):
        distances.append(
            distances[-1]
            + math.hypot(right["x"] - left["x"], right["y"] - left["y"])
        )
    target_distance = min(distances[-1], distances[frame_index] + lookahead_m)
    target_index = min(
        range(frame_index, len(track)),
        key=lambda index: abs(distances[index] - target_distance),
    )
    next_index = min(len(track) - 1, target_index + 1)
    target = _world_to_ego(track[target_index], state)
    target_next = _world_to_ego(track[next_index], state)
    return {
        "route_waypoints": [[row["x"], row["y"]] for row in track],
        "route_command": "LANE_FOLLOW",
        "target_point_ego_m": target,
        "target_point_next_ego_m": target_next,
        "target_point_coordinate_frame": "carla_ego",
        "progress_index": frame_index,
        "target_distance_along_route_m": distances[target_index],
        "next_target_distance_along_route_m": distances[next_index],
        "lookahead_m": lookahead_m,
        "source": "scenario_ir_reference_trajectory",
    }


def _actor_frame_provenance(
    manifest: Mapping[str, Any],
    frame: Mapping[str, Any],
    manifest_path: Path,
) -> dict[str, Any]:
    return {
        "schema_version": "open_loop_bbox_dynamic_provenance.v1",
        "actor_manifest_path": str(manifest_path.resolve()),
        "actor_manifest_sha256": str(manifest["manifest_sha256"]),
        "actor_manifest_file_sha256": str(
            manifest.get("manifest_file_sha256") or _sha256(manifest_path)
        ),
        "frame_id": int(frame["frame_id"]),
        "active_actor_ids": list(frame["active_actor_ids"]),
        "active_actor_set_sha256": str(frame["active_actor_set_sha256"]),
        "pose_digest": str(frame["pose_digest"]),
        "manifest_dynamic_object_sha256": str(frame["dynamic_object_sha256"]),
        "pose_source": "scenario_ir_actor_reference_trajectory",
        "track_binding_source": "formal_usdz_sequence_tracks.json",
        "coordinate_frame": "scene_local_ego_start_x_forward_y_left_z_up",
        "usdz_track_bindings": deepcopy(frame["usdz_track_bindings"]),
    }


def _carla_transform_for_bbox_center(
    carla: Any,
    actor: Any,
    state: Mapping[str, Any],
) -> Any:
    """Return a CARLA transform whose actor bbox center follows IR pose."""

    yaw_deg = -float(state.get("yaw", 0.0))
    yaw_rad = math.radians(yaw_deg)
    offset = getattr(getattr(actor, "bounding_box", None), "location", None)
    offset_x = float(getattr(offset, "x", 0.0))
    offset_y = float(getattr(offset, "y", 0.0))
    offset_z = float(getattr(offset, "z", 0.0))
    world_offset_x = math.cos(yaw_rad) * offset_x - math.sin(yaw_rad) * offset_y
    world_offset_y = math.sin(yaw_rad) * offset_x + math.cos(yaw_rad) * offset_y
    return carla.Transform(
        carla.Location(
            x=float(state["x"]) - world_offset_x,
            y=-float(state["y"]) - world_offset_y,
            z=float(state.get("z", 0.0)) - offset_z,
        ),
        carla.Rotation(yaw=yaw_deg),
    )


def _spawn_dynamic_actor(
    world: Any,
    blueprint_library: Any,
    carla: Any,
    dynamic_object: Mapping[str, Any],
    *,
    spawn_slot: int,
) -> Any:
    actor_type = str(dynamic_object.get("actor_type") or "")
    if actor_type == "vehicle":
        candidates = blueprint_library.filter("vehicle.*")
    elif actor_type == "pedestrian":
        candidates = blueprint_library.filter("walker.pedestrian.*")
    else:
        raise StageACaptureError(f"unsupported CARLA dynamic actor type: {actor_type}")
    if not candidates:
        raise StageACaptureError(f"CARLA has no blueprint for dynamic actor type: {actor_type}")
    blueprint = candidates[0]
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute(
            "role_name", f"m8_bbox_{str(dynamic_object['track_id'])[:24]}"
        )
    pose = dynamic_object["pose_pair"]["end"]
    # CARLA rejects a spawn when an IR actor overlaps the OpenDRIVE static
    # geometry or another actor.  Spawn each actor at a unique off-map holding
    # point first, disable physics, then teleport its bbox center to the exact
    # IR pose.  The resulting sensor frame still contains the actor at the
    # audited pose, while creation no longer depends on CARLA spawn collision
    # heuristics.
    holding_transform = carla.Transform(
        carla.Location(
            x=-1000.0 - 20.0 * float(spawn_slot),
            y=-1000.0,
            z=100.0,
        ),
        carla.Rotation(yaw=0.0),
    )
    actor = world.try_spawn_actor(
        blueprint,
        holding_transform,
    )
    if actor is None:
        raise StageACaptureError(
            f"failed to spawn CARLA dynamic actor for track {dynamic_object['track_id']}"
        )
    if hasattr(actor, "set_simulate_physics"):
        actor.set_simulate_physics(False)
    actor.set_transform(_carla_transform_for_bbox_center(carla, actor, pose))
    return actor


def _update_dynamic_actors(
    world: Any,
    blueprint_library: Any,
    carla: Any,
    dynamic_objects: list[Mapping[str, Any]],
    runtime_actors: dict[str, Any],
) -> dict[str, Any]:
    expected = {str(item["track_id"]): item for item in dynamic_objects}
    for track_id in sorted(set(runtime_actors) - set(expected)):
        actor = runtime_actors.pop(track_id)
        actor.destroy()
    for spawn_slot, (track_id, dynamic_object) in enumerate(expected.items()):
        actor = runtime_actors.get(track_id)
        if actor is None or not bool(getattr(actor, "is_alive", True)):
            actor = _spawn_dynamic_actor(
                world,
                blueprint_library,
                carla,
                dynamic_object,
                spawn_slot=spawn_slot,
            )
            runtime_actors[track_id] = actor
        actor.set_transform(
            _carla_transform_for_bbox_center(carla, actor, dynamic_object["pose_pair"]["end"])
        )
    return {
        "active_actor_ids": sorted(expected),
        "runtime_actor_ids": {
            track_id: int(getattr(runtime_actors[track_id], "id", -1))
            for track_id in sorted(expected)
        },
    }


def _runtime_dynamic_actor_evidence(
    carla: Any,
    dynamic_objects: list[Mapping[str, Any]],
    runtime_actors: Mapping[str, Any],
) -> dict[str, Any]:
    rows = []
    pose_errors = []
    for item in dynamic_objects:
        track_id = str(item["track_id"])
        actor = runtime_actors.get(track_id)
        if actor is None:
            raise StageACaptureError(f"active CARLA actor is missing: {track_id}")
        transform = actor.get_transform()
        bbox = getattr(actor, "bounding_box", None)
        offset = getattr(bbox, "location", None)
        yaw_rad = math.radians(float(transform.rotation.yaw))
        offset_x = float(getattr(offset, "x", 0.0))
        offset_y = float(getattr(offset, "y", 0.0))
        center_x = float(transform.location.x) + math.cos(yaw_rad) * offset_x - math.sin(yaw_rad) * offset_y
        center_y_carla = float(transform.location.y) + math.sin(yaw_rad) * offset_x + math.cos(yaw_rad) * offset_y
        center_y = -center_y_carla
        center_z = float(transform.location.z) + float(getattr(offset, "z", 0.0))
        actual_yaw = -float(transform.rotation.yaw)
        expected = item["pose_pair"]["end"]
        position_error = math.sqrt(
            (center_x - float(expected["x"])) ** 2
            + (center_y - float(expected["y"])) ** 2
            + (center_z - float(expected.get("z", 0.0))) ** 2
        )
        yaw_error = abs((actual_yaw - float(expected.get("yaw", 0.0)) + 180.0) % 360.0 - 180.0)
        pose_errors.append(
            {
                "track_id": track_id,
                "position_error_m": position_error,
                "yaw_error_deg": yaw_error,
            }
        )
        rows.append(
            {
                "track_id": track_id,
                "runtime_actor_id": int(getattr(actor, "id", -1)),
                "actor_type": str(item["actor_type"]),
                "x": center_x,
                "y": center_y,
                "z": center_z,
                "yaw": actual_yaw,
            }
        )
    if any(item["position_error_m"] > 0.10 or item["yaw_error_deg"] > 2.0 for item in pose_errors):
        raise StageACaptureError("CARLA dynamic actor pose is not aligned to the actor manifest")
    return {
        "runtime_dynamic_object_sha256": canonical_sha256(rows),
        "runtime_actor_ids": {item["track_id"]: item["runtime_actor_id"] for item in rows},
        "runtime_actor_count": len(rows),
        "pose_errors": pose_errors,
    }


def _wait_sensor(
    samples: queue.Queue[Any], expected_frame: int, *, timeout_sec: float
) -> Any:
    deadline = __import__("time").monotonic() + timeout_sec
    while __import__("time").monotonic() < deadline:
        try:
            sample = samples.get(timeout=max(0.01, deadline - __import__("time").monotonic()))
        except queue.Empty as exc:
            raise StageACaptureError(
                f"sensor did not produce CARLA frame {expected_frame}"
            ) from exc
        sample_frame = int(getattr(sample, "frame", -1))
        if sample_frame == expected_frame:
            return sample
        if sample_frame > expected_frame:
            raise StageACaptureError(
                f"sensor skipped CARLA frame {expected_frame}, received {sample_frame}"
            )
    raise StageACaptureError(f"sensor wait timed out for CARLA frame {expected_frame}")


def _save_camera(image: Any, path: Path) -> None:
    try:
        import numpy as np
        from PIL import Image
    except ImportError as exc:
        raise StageACaptureError("native capture needs numpy and Pillow") from exc
    pixels = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
        (int(image.height), int(image.width), 4)
    )
    rgb = pixels[:, :, :3][:, :, ::-1]
    Image.fromarray(rgb, mode="RGB").save(
        path, format="JPEG", quality=95, subsampling=0
    )


def _normalise_track(value: Any) -> list[dict[str, float]]:
    if not isinstance(value, list) or len(value) < 2:
        raise StageACaptureError("Scenario IR ego reference trajectory has fewer than two frames")
    result = []
    for index, row in enumerate(value):
        if not isinstance(row, Mapping):
            raise StageACaptureError(f"ego reference frame {index} is not an object")
        result.append(
            {
                "t_sec": float(row["t_sec"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row.get("z", 0.0)),
                "yaw": float(row.get("yaw", 0.0)),
                "speed_mps": float(row.get("speed_mps", 0.0)),
            }
        )
    return result


def capture_stage_a(
    *,
    scenario_ir_path: Path,
    opendrive_path: Path,
    output_dir: Path,
    runtime_config_path: Path,
    actor_manifest_path: Path | None = None,
    expected_scenario_ir_sha256: str = EXPECTED_SCENARIO_IR_SHA256,
    expected_opendrive_sha256: str = EXPECTED_OPENDRIVE_SHA256,
    host: str = "127.0.0.1",
    port: int = 2000,
    max_frames: int | None = None,
    sensor_timeout_sec: float = 5.0,
    container_root: str = "/sim-data",
) -> dict[str, Any]:
    if output_dir.exists():
        raise StageACaptureError(f"refusing to overwrite capture directory: {output_dir}")
    if not runtime_config_path.is_file():
        raise StageACaptureError(f"runtime config is unavailable: {runtime_config_path}")
    try:
        runtime_config = json.loads(runtime_config_path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise StageACaptureError(f"runtime config is unreadable: {exc}") from exc
    if not isinstance(runtime_config, dict):
        raise StageACaptureError("runtime config must be an object")
    inputs = load_pinned_inputs(
        scenario_ir_path,
        opendrive_path,
        expected_scenario_ir_sha256=expected_scenario_ir_sha256,
        expected_opendrive_sha256=expected_opendrive_sha256,
    )
    if actor_manifest_path is None:
        raise StageACaptureError(
            "actor_manifest_path is required for actor-aware open-loop capture"
        )
    actor_manifest = load_actor_manifest(
        actor_manifest_path,
        expected_scenario_ir_sha256=inputs.scenario_ir_sha256,
        expected_scene_id=str(inputs.scenario_ir["scenario_id"]),
    )
    track = _normalise_track((inputs.scenario_ir.get("ego") or {}).get("reference_trajectory"))
    if max_frames is not None:
        if isinstance(max_frames, bool) or max_frames < 2:
            raise StageACaptureError("max_frames must be at least two")
        track = track[:max_frames]
    if len(track) < 2:
        raise StageACaptureError("Stage A needs at least two GT frames")
    if len(actor_manifest["frames"]) < len(track):
        raise StageACaptureError(
            "actor manifest has fewer frames than the Scenario IR ego replay"
        )

    experiment = runtime_config.get("experiment")
    if not isinstance(experiment, Mapping):
        raise StageACaptureError("runtime config has no experiment identity")
    run_context = {
        "run_id": str(
            runtime_config.get("run_id")
            or experiment.get("run_id")
            or "scene0061-m5-stage-a"
        ),
        "scene_id": experiment.get("scene_id"),
        "case_id": experiment.get("case_id"),
        "seed": experiment.get("seed"),
        "identity": {
            name: experiment.get(name)
            for name in (
                "artifact_sha256",
                "scene_package_sha256",
                "scenario_ir_sha256",
                "immutable_matrix_sha256",
                "source_run_config_sha256",
                "variant_config_sha256",
                "run_config_sha256",
            )
        },
    }
    calibration = _calibration()
    output_dir.mkdir(parents=True)
    payload_root = output_dir / "payloads"
    payload_root.mkdir()
    container_payload_root = f"{container_root.rstrip('/')}/{output_dir.name}"
    trace_frames: list[dict[str, Any]] = []

    try:
        import carla
    except ImportError as exc:
        raise StageACaptureError("host CARLA 0.9.16 Python client is unavailable") from exc

    client = carla.Client(host, int(port))
    client.set_timeout(float(sensor_timeout_sec))
    if client.get_server_version() != "0.9.16":
        raise StageACaptureError(
            f"Stage A requires CARLA server 0.9.16, got {client.get_server_version()}"
        )
    original_world = client.get_world()
    original_settings = original_world.get_settings()
    world = client.generate_opendrive_world(
        opendrive_path.read_text(encoding="utf-8"),
        carla.OpendriveGenerationParameters(),
    )
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)
    ego = None
    camera = None
    lidar = None
    runtime_actors: dict[str, Any] = {}
    camera_samples: queue.Queue[Any] = queue.Queue()
    lidar_samples: queue.Queue[Any] = queue.Queue()
    try:
        blueprint_library = world.get_blueprint_library()
        candidates = blueprint_library.filter("vehicle.tesla.model3") or blueprint_library.filter("vehicle.*")
        if not candidates:
            raise StageACaptureError("CARLA has no vehicle blueprint for Stage A")
        ego_blueprint = candidates[0]
        ego_blueprint.set_attribute("role_name", "m5_stage_a_ego")
        first = track[0]
        ego_transform = carla.Transform(
            carla.Location(x=first["x"], y=-first["y"], z=first["z"] + 0.5),
            carla.Rotation(yaw=-first["yaw"]),
        )
        ego = world.try_spawn_actor(ego_blueprint, ego_transform)
        if ego is None:
            raise StageACaptureError("failed to spawn Stage A ego at the IR start pose")
        ego.set_simulate_physics(False)

        camera_blueprint = blueprint_library.find("sensor.camera.rgb")
        camera_blueprint.set_attribute("image_size_x", str(CAMERA_WIDTH))
        camera_blueprint.set_attribute("image_size_y", str(CAMERA_HEIGHT))
        camera_blueprint.set_attribute("fov", str(CAMERA_FOV_DEG))
        camera_blueprint.set_attribute("sensor_tick", str(FIXED_DELTA_SECONDS))
        camera = world.spawn_actor(
            camera_blueprint,
            carla.Transform(carla.Location(*CAMERA_MOUNT)),
            attach_to=ego,
        )
        camera.listen(camera_samples.put)

        lidar_blueprint = blueprint_library.find("sensor.lidar.ray_cast")
        for name, value in {
            "channels": "64",
            "range": "85.0",
            "points_per_second": "1300000",
            "rotation_frequency": str(1.0 / FIXED_DELTA_SECONDS),
            "upper_fov": "10.0",
            "lower_fov": "-30.0",
            "sensor_tick": str(FIXED_DELTA_SECONDS),
        }.items():
            lidar_blueprint.set_attribute(name, value)
        lidar = world.spawn_actor(
            lidar_blueprint,
            carla.Transform(carla.Location(*LIDAR_MOUNT)),
            attach_to=ego,
        )
        lidar.listen(lidar_samples.put)

        # Drop the first two sensor ticks after attachment.  CARLA can publish
        # the attachment/spawn frame and then skip the immediately following
        # frame while a high-resolution camera and dense LiDAR finish
        # initialization; scoring starts only after this warm-up.
        world.tick()
        world.tick()
        for frame_id, state in enumerate(track):
            actor_frame = frame_binding(actor_manifest, frame_id)
            dynamic_objects = frame_dynamic_objects(actor_manifest, frame_id)
            actor_provenance = _actor_frame_provenance(
                actor_manifest,
                actor_frame,
                actor_manifest_path,
            )
            runtime_binding = _update_dynamic_actors(
                world,
                blueprint_library,
                carla,
                dynamic_objects,
                runtime_actors,
            )
            ego.set_transform(
                carla.Transform(
                    carla.Location(x=state["x"], y=-state["y"], z=state["z"] + 0.5),
                    carla.Rotation(yaw=-state["yaw"]),
                )
            )
            world.tick()
            snapshot = world.get_snapshot()
            snapshot_frame = int(snapshot.frame)
            image = _wait_sensor(
                camera_samples, snapshot_frame, timeout_sec=sensor_timeout_sec
            )
            scan = _wait_sensor(
                lidar_samples, snapshot_frame, timeout_sec=sensor_timeout_sec
            )
            frame_dir = payload_root / f"frame_{frame_id:08d}"
            frame_dir.mkdir()
            camera_path = frame_dir / "camera_front.jpg"
            lidar_path = frame_dir / "lidar_top.bin"
            _save_camera(image, camera_path)
            lidar_path.write_bytes(bytes(scan.raw_data))
            if lidar_path.stat().st_size < 16 or lidar_path.stat().st_size % 16:
                raise StageACaptureError(
                    f"native LiDAR frame {frame_id} is not a float32 XYZI stream"
                )
            runtime_evidence = _runtime_dynamic_actor_evidence(
                carla,
                dynamic_objects,
                runtime_actors,
            )
            relative_camera = camera_path.relative_to(output_dir).as_posix()
            relative_lidar = lidar_path.relative_to(output_dir).as_posix()
            camera_ref = _file_ref(
                camera_path,
                container_path=f"{container_payload_root}/{relative_camera}",
                encoding="jpeg",
                coordinate_frame="camera_optical",
            )
            lidar_ref = _file_ref(
                lidar_path,
                container_path=f"{container_payload_root}/{relative_lidar}",
                encoding="float32_xyzi_little_endian",
                coordinate_frame="sensor_local",
                axis_convention="carla_sensor",
                sensor_to_ego=calibration["lidar_sensor_to_ego"],
            )
            observation = {
                "schema_version": "transfuserpp_observation.v1",
                "observation_id": f"{run_context['run_id']}-frame-{frame_id:08d}",
                "source": SOURCE,
                "frame_id": frame_id,
                "timestamp": state["t_sec"],
                "rgb": {"camera_front": camera_ref},
                "lidar": lidar_ref,
                "sensor_validity": {"camera_front": True, "lidar": True},
                "calibration": deepcopy(calibration),
                "ego_state": {
                    "pose": {
                        "x": state["x"],
                        "y": state["y"],
                        "z": state["z"],
                        "yaw": state["yaw"],
                    },
                    "speed_mps": state["speed_mps"],
                    "speed_source": "scenario_ir_reference_trajectory",
                    "compass_source": "scenario_ir_reference_trajectory",
                },
                "route": _route_for_frame(track, frame_id),
                "synchronization": {
                    "frame_id": frame_id,
                    "carla_snapshot_frame": snapshot_frame,
                    "clock": "scenario_ir_reference_trajectory",
                    "error_ms": 0.0,
                    "dynamic_object_sha256": actor_provenance[
                        "manifest_dynamic_object_sha256"
                    ],
                    "sensor_age_ticks": 0,
                },
                "run_context": deepcopy(run_context),
                "provenance": {
                    "gt_pose_replay": True,
                    "pose_source": "scenario_ir_reference_trajectory",
                    "input_source": SOURCE,
                    "input_variant": "raw_original_rgb_lidar",
                    "control_applied": False,
                    "physics_enabled": False,
                    "carla_server_version": client.get_server_version(),
                    "carla_map": world.get_map().name,
                    "carla_snapshot_elapsed_seconds": float(
                        snapshot.timestamp.elapsed_seconds
                    ),
                    "ir_pose_sha256": canonical_sha256(state),
                    "actor_manifest": deepcopy(actor_provenance),
                    "runtime_actor_creation": True,
                    "runtime_actor_ids": deepcopy(runtime_binding["runtime_actor_ids"]),
                    "runtime_actor_count": runtime_evidence["runtime_actor_count"],
                    "runtime_dynamic_object_sha256": runtime_evidence[
                        "runtime_dynamic_object_sha256"
                    ],
                    "runtime_actor_pose_errors": runtime_evidence["pose_errors"],
                },
            }
            validate_observation(observation)
            trace_frames.append(observation)
    finally:
        for actor in list(runtime_actors.values()) + [lidar, camera, ego]:
            if actor is not None:
                actor.destroy()
        try:
            world.apply_settings(original_settings)
        except Exception:
            pass

    trace = {
        "schema_version": TRACE_SCHEMA,
        "source": SOURCE,
        "scenario_ir": {
            "path": str(inputs.scenario_ir_path),
            "sha256": inputs.scenario_ir_sha256,
        },
        "opendrive": {
            "path": str(inputs.opendrive_path),
            "sha256": inputs.opendrive_sha256,
        },
        "runtime_config": {
            "path": str(runtime_config_path.resolve()),
            "sha256": _sha256(runtime_config_path),
        },
        "capture": {
            "carla_server_version": "0.9.16",
            "map": "Carla/Maps/OpenDriveMap",
            "fixed_delta_seconds": FIXED_DELTA_SECONDS,
            "camera": {"width": CAMERA_WIDTH, "height": CAMERA_HEIGHT, "fov_deg": CAMERA_FOV_DEG},
            "camera_mount": list(CAMERA_MOUNT),
            "lidar_mount": list(LIDAR_MOUNT),
            "ego_pose_owner": "scenario_ir_reference_trajectory",
            "control_affects_next_ego_pose": False,
            "dynamic_actor_creation": True,
            "dynamic_actor_count": actor_manifest["summary"]["actor_count"],
            "actor_manifest": {
                "path": str(actor_manifest_path.resolve()),
                "sha256": actor_manifest["manifest_sha256"],
                "file_sha256": actor_manifest["manifest_file_sha256"],
                "summary": deepcopy(actor_manifest["summary"]),
            },
        },
        "frames": trace_frames,
        "actor_manifest": {
            "path": str(actor_manifest_path.resolve()),
            "sha256": actor_manifest["manifest_sha256"],
            "file_sha256": actor_manifest["manifest_file_sha256"],
            "summary": deepcopy(actor_manifest["summary"]),
        },
    }
    trace_path = output_dir / "native_stage_a_observations.json"
    trace_path.write_text(
        json.dumps(trace, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return {
        "status": "captured",
        "trace": str(trace_path),
        "frame_count": len(trace_frames),
        "payload_root": str(payload_root),
        "trace_sha256": _sha256(trace_path),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-ir", type=Path, required=True)
    parser.add_argument("--opendrive", type=Path, required=True)
    parser.add_argument("--runtime-config", type=Path, required=True)
    parser.add_argument("--actor-manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-scenario-ir-sha256", default=EXPECTED_SCENARIO_IR_SHA256)
    parser.add_argument("--expected-opendrive-sha256", default=EXPECTED_OPENDRIVE_SHA256)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--max-frames", type=int, default=3)
    parser.add_argument("--sensor-timeout-sec", type=float, default=5.0)
    parser.add_argument("--container-root", default="/sim-data")
    args = parser.parse_args(argv)
    try:
        result = capture_stage_a(
            scenario_ir_path=args.scenario_ir,
            opendrive_path=args.opendrive,
            runtime_config_path=args.runtime_config,
            actor_manifest_path=args.actor_manifest,
            output_dir=args.output_dir,
            expected_scenario_ir_sha256=args.expected_scenario_ir_sha256,
            expected_opendrive_sha256=args.expected_opendrive_sha256,
            host=args.host,
            port=args.port,
            max_frames=args.max_frames,
            sensor_timeout_sec=args.sensor_timeout_sec,
            container_root=args.container_root,
        )
    except (OSError, ValueError, RuntimeError, StageACaptureError) as exc:
        print(json.dumps({"status": "failed", "error": f"{type(exc).__name__}: {exc}"}))
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
