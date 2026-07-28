#!/usr/bin/env python3
"""Diagnose dynamic-track LiDAR support in an NCore sequence store.

Run this inside the NCore converter image.  It reconstructs each LiDAR return
in the NCore world frame using the stored lidar-to-rig and rig-to-world poses,
then counts points inside the matching cuboids.  This is diagnostic evidence:
it does not relax any M8 gate.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

import numpy as np


def _yaw_matrix(yaw: float) -> np.ndarray:
    cosine, sine = np.cos(yaw), np.sin(yaw)
    return np.asarray(((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)))


def _count(points_world: np.ndarray, centroid: Iterable[float], size: Iterable[float], yaw: float, padding: np.ndarray) -> int:
    # For row vectors, delta @ R(yaw) is the global-to-box local transform.
    local = (points_world - np.asarray(tuple(centroid), dtype=np.float32)) @ _yaw_matrix(yaw)
    half_size = np.asarray(tuple(size), dtype=np.float32) / 2.0 + padding
    return int(np.all(np.abs(local) <= half_size, axis=1).sum())


def _pose_at(matrices: np.ndarray, timestamps: np.ndarray, timestamp_us: int) -> np.ndarray:
    index = int(np.searchsorted(timestamps, timestamp_us))
    choices = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(timestamps)]
    if not choices:
        raise ValueError("NCore rig trajectory has no poses")
    chosen = min(choices, key=lambda candidate: abs(int(timestamps[candidate]) - timestamp_us))
    return matrices[chosen]


def audit(manifest: Path, track_ids: set[str], padding_m: tuple[float, float, float]) -> dict[str, Any]:
    from ncore.impl.data.v4.components import CuboidsComponent, LidarSensorComponent, PosesComponent, SequenceComponentGroupsReader

    reader = SequenceComponentGroupsReader([manifest])
    lidar_readers = reader.open_component_readers(LidarSensorComponent.Reader)
    if len(lidar_readers) != 1:
        raise ValueError(f"expected exactly one lidar reader, got {sorted(lidar_readers)}")
    lidar_id, lidar = next(iter(lidar_readers.items()))
    poses = reader.open_component_readers(PosesComponent.Reader)
    if len(poses) != 1:
        raise ValueError(f"expected exactly one pose reader, got {sorted(poses)}")
    pose_reader = next(iter(poses.values()))
    sensor_to_rig = np.asarray(pose_reader.get_static_pose(lidar_id, "rig"), dtype=np.float64)
    rig_to_world, rig_pose_times = pose_reader.get_dynamic_pose("rig", "world")
    rig_to_world = np.asarray(rig_to_world, dtype=np.float64)
    rig_pose_times = np.asarray(rig_pose_times, dtype=np.uint64)

    cuboid_readers = reader.open_component_readers(CuboidsComponent.Reader)
    observations_by_timestamp: dict[int, list[Any]] = defaultdict(list)
    for cuboid_reader in cuboid_readers.values():
        for observation in cuboid_reader.get_observations():
            if str(observation.track_id) in track_ids:
                observations_by_timestamp[int(observation.timestamp_us)].append(observation)

    records: dict[str, dict[str, Any]] = {
        track_id: {
            "track_id": track_id,
            "cuboid_observation_count": 0,
            "matched_lidar_frame_count": 0,
            "nonzero_exact_box_frame_count": 0,
            "nonzero_padded_box_frame_count": 0,
            "sum_exact_box_hit_points": 0,
            "sum_padded_box_hit_points": 0,
            "max_exact_box_hit_points": 0,
            "max_padded_box_hit_points": 0,
            "issues": [],
        }
        for track_id in sorted(track_ids)
    }
    for observations in observations_by_timestamp.values():
        for observation in observations:
            records[str(observation.track_id)]["cuboid_observation_count"] += 1

    frame_end_times = [int(interval[1]) for interval in np.asarray(lidar.frames_timestamps_us)]
    available_times = set(frame_end_times)
    for timestamp, observations in observations_by_timestamp.items():
        if timestamp not in available_times:
            for observation in observations:
                records[str(observation.track_id)]["issues"].append(f"no_lidar_frame_at_{timestamp}")

    zero_padding = np.zeros(3, dtype=np.float32)
    requested_padding = np.asarray(padding_m, dtype=np.float32)
    for frame_index, timestamp in enumerate(frame_end_times):
        observations = observations_by_timestamp.get(timestamp)
        if not observations:
            continue
        direction = lidar.get_frame_ray_bundle_data(timestamp, "direction")
        distance = lidar.get_frame_ray_bundle_return_data(timestamp, "distance_m", 0)[0]
        valid = np.asarray(lidar.get_frame_ray_bundle_return_valid_mask(timestamp)).reshape(-1)
        points_sensor = direction[valid] * distance[valid, None]
        points_h = np.concatenate((points_sensor, np.ones((len(points_sensor), 1), dtype=np.float32)), axis=1)
        world_from_sensor = _pose_at(rig_to_world, rig_pose_times, timestamp) @ sensor_to_rig
        points_world = (points_h @ world_from_sensor.T)[:, :3]
        for observation in observations:
            track_id = str(observation.track_id)
            bbox = observation.bbox3
            yaw = float(bbox.rot[2])
            exact = _count(points_world, bbox.centroid, bbox.dim, yaw, zero_padding)
            padded = _count(points_world, bbox.centroid, bbox.dim, yaw, requested_padding)
            record = records[track_id]
            record["matched_lidar_frame_count"] += 1
            record["sum_exact_box_hit_points"] += exact
            record["sum_padded_box_hit_points"] += padded
            record["max_exact_box_hit_points"] = max(record["max_exact_box_hit_points"], exact)
            record["max_padded_box_hit_points"] = max(record["max_padded_box_hit_points"], padded)
            record["nonzero_exact_box_frame_count"] += int(exact > 0)
            record["nonzero_padded_box_frame_count"] += int(padded > 0)

    for record in records.values():
        if record["cuboid_observation_count"] == 0:
            record["status"] = "track_missing_from_ncore_cuboids"
        elif record["matched_lidar_frame_count"] == 0:
            record["status"] = "cuboid_lidar_time_unmatched"
        elif record["nonzero_padded_box_frame_count"] == 0:
            record["status"] = "ncore_dynamic_lidar_absent"
        elif record["nonzero_exact_box_frame_count"] == record["matched_lidar_frame_count"]:
            record["status"] = "ncore_dynamic_lidar_supported"
        else:
            record["status"] = "ncore_dynamic_lidar_sparse"

    counts: dict[str, int] = {}
    for record in records.values():
        counts[record["status"]] = counts.get(record["status"], 0) + 1
    return {
        "schema_version": "ncore_dynamic_lidar_support_audit.v1",
        "status": "diagnostic_only",
        "manifest": str(manifest),
        "lidar_id": lidar_id,
        "lidar_frame_count": int(lidar.frames_count),
        "box_padding_m": list(padding_m),
        "coordinate_chain": "lidar_top -> rig -> world, matched to CuboidsComponent BBox3 world coordinates",
        "summary": {"track_count": len(records), "status_counts": counts},
        "tracks": list(records.values()),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--track-ids", required=True, help="comma-separated NCore track identifiers")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--padding-m", default="0.5,0.5,0.25")
    args = parser.parse_args()
    track_ids = {item.strip() for item in args.track_ids.split(",") if item.strip()}
    if not track_ids:
        raise ValueError("--track-ids must contain at least one identifier")
    padding = tuple(float(item.strip()) for item in args.padding_m.split(","))
    if len(padding) != 3:
        raise ValueError("--padding-m must provide x,y,z")
    report = audit(args.manifest, track_ids, padding)  # type: ignore[arg-type]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["summary"], sort_keys=True))


if __name__ == "__main__":
    main()
