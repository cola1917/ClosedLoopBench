from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runners.prepare_nurec_pose_probe_frames import (
    _index,
    _load_array,
    _sample_channel_index,
    _scene_samples,
)


def load_nuscenes_ego_landmarks(
    dataroot: str | Path,
    *,
    version: str,
    scene_name: str,
    lidar_channel: str = "LIDAR_TOP",
) -> tuple[str, list[dict[str, Any]]]:
    root = Path(dataroot) / version
    scenes = _load_array(root / "scene.json")
    samples = _load_array(root / "sample.json")
    sample_data_rows = _load_array(root / "sample_data.json")
    ego_poses = _index(_load_array(root / "ego_pose.json"))
    calibrations = _index(_load_array(root / "calibrated_sensor.json"))
    sensors = _index(_load_array(root / "sensor.json"))
    selected = [row for row in scenes if row.get("name") == scene_name]
    if len(selected) != 1:
        raise ValueError(f"expected one scene named {scene_name}, found {len(selected)}")
    scene = selected[0]
    scene_samples = _scene_samples(scene, samples)
    sample_data = _index(sample_data_rows)
    channels = _sample_channel_index(sample_data_rows, calibrations, sensors)

    result = []
    for index, sample in enumerate(scene_samples):
        data_token = channels.get((str(sample["token"]), lidar_channel))
        data = sample_data.get(str(data_token))
        ego = ego_poses.get(str((data or {}).get("ego_pose_token")))
        if data is None or ego is None:
            raise ValueError(f"sample {sample['token']} lacks {lidar_channel} ego pose")
        result.append(
            {
                "landmark_id": f"lidar_ego_keyframe_{index:03d}",
                "sample_token": str(sample["token"]),
                "sample_data_token": str(data["token"]),
                "timestamp_us": int(data["timestamp"]),
                "log_global": {
                    "x": float(ego["translation"][0]),
                    "y": float(ego["translation"][1]),
                    "z": float(ego["translation"][2]),
                    "yaw_deg": _yaw_degrees(ego["rotation"]),
                },
            }
        )
    return str(scene["token"]), result


def attach_nurec_runtime_references(
    landmarks: Iterable[dict[str, Any]],
    *,
    artifact_path: str | Path,
    xodr_data: str,
    nurec_example_root: str | Path,
) -> list[dict[str, Any]]:
    """Attach the exact NuRec rig pose used by NVIDIA's CARLA adapter.

    The nuScenes mini ego-pose table used for scene-0061 has a zeroed vertical
    component.  NuRec's USDZ rig trajectory contains the reconstructed road
    elevation, so a formal 3D runtime check must interpolate that trajectory at
    each LiDAR keyframe timestamp instead of pretending the raw log z is an
    observable height.
    """

    example_root = str(Path(nurec_example_root).resolve())
    if example_root not in sys.path:
        sys.path.insert(0, example_root)
    try:
        import numpy as np
        from projection_functions import get_t_rig_enu_from_ecef
        from scenario import Scenario
    except ImportError as exc:
        raise ValueError(f"cannot import NVIDIA NuRec example runtime: {exc}") from exc

    artifact = Path(artifact_path)
    scenario = Scenario(str(artifact))
    scenario_to_carla = np.asarray(
        get_t_rig_enu_from_ecef(scenario.t_world_base, xodr_data), dtype=float
    )
    if scenario_to_carla.shape != (4, 4):
        raise ValueError("NVIDIA scenario-to-CARLA transform is not 4x4")

    attached = []
    for landmark in landmarks:
        timestamp_us = int(landmark["timestamp_us"])
        pose = scenario.ego_poses.interpolate_pose_matrix(timestamp_us)
        if pose is None:
            raise ValueError(
                f"NuRec ego trajectory has no pose for timestamp {timestamp_us}"
            )
        matrix = scenario_to_carla @ np.asarray(pose, dtype=float)
        item = deepcopy(landmark)
        item["nurec_runtime"] = {
            "x": float(matrix[0, 3]),
            "y": float(matrix[1, 3]),
            "z": float(matrix[2, 3]),
            "yaw_deg": _normalize_degrees(
                math.degrees(math.atan2(float(matrix[1, 0]), float(matrix[0, 0])))
            ),
            "timestamp_us": timestamp_us,
            "pose_source": "nurec_usdz_rig_trajectory_interpolated",
        }
        attached.append(item)
    return attached


def capture_observations(
    scene_package: dict[str, Any],
    landmarks: Iterable[dict[str, Any]],
    *,
    carla_map: Any,
    carla_module: Any,
    simulator_version: str,
    renderer_version: str,
    artifact_path: str | Path,
    carla_frame: int,
) -> dict[str, Any]:
    scene_id = str(scene_package.get("scene_id") or "")
    matrix = (scene_package.get("alignment") or {}).get("sim_from_log_transform")
    if not scene_id or not isinstance(matrix, list) or len(matrix) != 16:
        raise ValueError("Scene Package lacks scene_id or sim_from_log_transform")
    artifact = Path(artifact_path)
    if not artifact.is_file():
        raise ValueError(f"NuRec artifact does not exist: {artifact}")

    captured = []
    reference_modes = set()
    for landmark in landmarks:
        log_global = landmark.get("log_global")
        if not isinstance(log_global, dict):
            raise ValueError("landmark lacks log_global")
        log_expected = _apply(matrix, log_global)
        nurec_runtime = landmark.get("nurec_runtime")
        if nurec_runtime is None:
            expected = log_expected
            reference_modes.add("scene_package_rigid_transform")
        elif isinstance(nurec_runtime, dict):
            expected = {
                name: float(nurec_runtime[name]) for name in ("x", "y", "z")
            }
            reference_modes.add("nurec_usdz_ego_trajectory")
        else:
            raise ValueError("nurec_runtime must be an object when provided")
        # Canonical scene coordinates are right-handed x-forward/y-left;
        # CARLA's API is x-forward/y-right. Convert only at the API boundary.
        query = carla_module.Location(
            x=expected["x"], y=-expected["y"], z=expected["z"]
        )
        waypoint = carla_map.get_waypoint(
            query,
            project_to_road=True,
            lane_type=carla_module.LaneType.Driving,
        )
        if waypoint is None:
            raise ValueError(f"no CARLA driving waypoint for {landmark['landmark_id']}")
        transform = waypoint.transform
        measured = {
            "x": float(transform.location.x),
            "y": -float(transform.location.y),
            "z": float(transform.location.z),
            "yaw_deg": _normalize_degrees(-float(transform.rotation.yaw)),
        }
        captured.append(
            {
                "landmark_id": str(landmark["landmark_id"]),
                "log_global": dict(log_global),
                **(
                    {"nurec_runtime": deepcopy(nurec_runtime)}
                    if nurec_runtime is not None
                    else {}
                ),
                "sim_measured": measured,
                "measurement": {
                    "sample_token": landmark.get("sample_token"),
                    "sample_data_token": landmark.get("sample_data_token"),
                    "timestamp_us": landmark.get("timestamp_us"),
                    "carla_road_id": int(waypoint.road_id),
                    "carla_lane_id": int(waypoint.lane_id),
                    "carla_is_junction": bool(waypoint.is_junction),
                    "carla_lane_width_m": float(waypoint.lane_width),
                },
            }
        )

    if len(captured) < 3:
        raise ValueError("at least three runtime landmarks are required")
    if len(reference_modes) != 1:
        raise ValueError("runtime landmarks must use one consistent pose reference")
    reference_mode = reference_modes.pop()
    return {
        "schema_version": "runtime_alignment_observations.v1",
        "scene_id": scene_id,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "runtime": {
            "simulator": f"CARLA {simulator_version}",
            "renderer": f"NRE {renderer_version}",
            "capture_method": (
                "nurec_usdz_rig_poses_at_nuscenes_lidar_keyframes_projected_"
                "to_loaded_carla_opendrive_driving_waypoints"
                if reference_mode == "nurec_usdz_ego_trajectory"
                else "all_nuscenes_lidar_keyframe_ego_poses_projected_to_"
                "loaded_carla_opendrive_driving_waypoints"
            ),
            "nurec_artifact_sha256": _sha256(artifact),
        },
        "capture": {
            "map_name": str(carla_map.name),
            "carla_frame": int(carla_frame),
            "coordinate_boundary": "canonical_y_left_to_carla_y_right",
            "landmark_source": (
                "nuscenes_lidar_keyframe_timestamps_plus_nurec_usdz_ego_trajectory"
                if reference_mode == "nurec_usdz_ego_trajectory"
                else "nuscenes_raw_ego_pose_and_lidar_keyframes"
            ),
            "runtime_reference": reference_mode,
            "vertical_reference": (
                "nurec_usdz_ego_trajectory"
                if reference_mode == "nurec_usdz_ego_trajectory"
                else "nuscenes_global_z"
            ),
            "landmark_count": len(captured),
        },
        "landmarks": captured,
    }


def _apply(matrix: list[Any], point: dict[str, Any]) -> dict[str, float]:
    values = [float(value) for value in matrix]
    x, y, z = (float(point[name]) for name in ("x", "y", "z"))
    return {
        "x": values[0] * x + values[1] * y + values[2] * z + values[3],
        "y": values[4] * x + values[5] * y + values[6] * z + values[7],
        "z": values[8] * x + values[9] * y + values[10] * z + values[11],
    }


def _yaw_degrees(rotation_wxyz: Iterable[Any]) -> float:
    w, x, y, z = (float(value) for value in rotation_wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError("ego quaternion must be finite and non-zero")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return _normalize_degrees(
        math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    )


def _normalize_degrees(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Capture nuScenes-to-live-CARLA runtime alignment landmarks."
    )
    parser.add_argument("--dataroot", required=True, type=Path)
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument("--scene", required=True)
    parser.add_argument("--scene-package", required=True, type=Path)
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--opendrive", required=True, type=Path)
    parser.add_argument(
        "--nurec-example-root",
        type=Path,
        required=True,
        help="NVIDIA CARLA Python example directory containing scenario.py",
    )
    parser.add_argument("--renderer-version", required=True)
    parser.add_argument("--carla-python-api", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    if args.carla_python_api:
        sys.path.insert(0, str(args.carla_python_api.resolve()))
    try:
        import carla

        package = json.loads(args.scene_package.read_text(encoding="utf-8"))
        scene_id, landmarks = load_nuscenes_ego_landmarks(
            args.dataroot, version=args.version, scene_name=args.scene
        )
        if scene_id != package.get("scene_id"):
            raise ValueError("nuScenes scene token does not match Scene Package")
        xodr = args.opendrive.read_text(encoding="utf-8")
        if not xodr.strip():
            raise ValueError("OpenDRIVE file is empty")
        landmarks = attach_nurec_runtime_references(
            landmarks,
            artifact_path=args.artifact,
            xodr_data=xodr,
            nurec_example_root=args.nurec_example_root,
        )
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout_sec)
        generation_type = getattr(carla, "OpendriveGenerationParameters", None)
        world = (
            client.generate_opendrive_world(xodr, generation_type())
            if generation_type is not None
            else client.generate_opendrive_world(xodr)
        )
        snapshot = world.get_snapshot()
        observations = capture_observations(
            package,
            landmarks,
            carla_map=world.get_map(),
            carla_module=carla,
            simulator_version=client.get_server_version(),
            renderer_version=args.renderer_version,
            artifact_path=args.artifact,
            carla_frame=snapshot.frame,
        )
    except (ImportError, OSError, ValueError, json.JSONDecodeError, RuntimeError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(observations, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "status": "captured",
                "landmark_count": len(observations["landmarks"]),
                "map_name": observations["capture"]["map_name"],
                "output": str(args.output),
            }
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
