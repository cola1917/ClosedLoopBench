#!/usr/bin/env python3
"""Diagnose whether the NuRec LiDAR renderer emitted dynamic vehicle meshes.

Reproduces the M8 root-cause investigation (docs/open_loop_m8_debug_log.md):
for a chosen frame of the scene-0061 NuRec multimodal 20 FPS dataset, computes
the expected sensor-frame position of every controllable vehicle from the
USDZ sequence tracks and counts LiDAR returns near each position. The finding
was that only some vehicles render (e.g. 085fb7c4 with ~45 points) while the
lead vehicle c1958768 - visible in RGB - has 0 points in LiDAR.

Usage:
    python3 -m tools.diagnose_nurec_dynamic_vehicle_lidar --frame 0 \
        --usdz /path/to/last.usdz \
        --dataset /path/to/multimodal_20fps

The USDZ is the scene-0061 renderable artifact; the dataset must contain
frames.jsonl and lidar/*.xyzi.bin.
"""

from __future__ import annotations

import argparse
import json
import zipfile
from pathlib import Path
from typing import Any

import numpy as np

VEHICLE_LABELS = {"automobile", "bus", "heavy_truck", "Other Vehicle - Construction Vehicle"}
CONTROLLABLE_FLAG = "DYNAMIC|CONTROLLABLE"
MATCHED_TRACKS = {
    "c1958768d48640948f6053d04cffd35b": "lead vehicle (target)",
    "bc38961ca0ac": "matched track",
    "42641eb6adcb": "matched track",
    "085fb7c41191": "matched track",
    "a60047adc78a": "matched track",
    "85246a44cc63": "static vehicle",
}


def load_artifact(usdz: Path) -> tuple[list[int], list[np.ndarray], dict[str, Any], np.ndarray]:
    with zipfile.ZipFile(usdz) as archive:
        rig = json.loads(archive.read("rig_trajectories.json"))
        sequence = json.loads(archive.read("sequence_tracks.json"))
    trajectory = rig["rig_trajectories"][0]
    rig_timestamps = [int(value) for value in trajectory["T_rig_world_timestamps_us"]]
    rig_matrices = [np.asarray(matrix, dtype=float) for matrix in trajectory["T_rig_worlds"]]
    chunks = sequence.get("sequence_tracks", sequence)
    if isinstance(chunks, dict):
        chunks = list(chunks.values())
    tracks: dict[str, dict[str, Any]] = {}
    for chunk in chunks:
        data = chunk["tracks_data"]
        for index, raw_id in enumerate(data["tracks_id"]):
            track_id = str(raw_id)
            tracks[track_id] = {
                "timestamps_us": [int(v) for v in data["tracks_timestamps_us"][index]],
                "poses": [np.asarray(p, dtype=float) for p in data["tracks_poses"][index]],
                "flag": str(data["tracks_flags"][index]),
                "label": str(data["tracks_label_class"][index]),
            }
    calibrations = rig.get("lidar_calibrations") or {}
    lidar_key = next((key for key in calibrations if "lidar_top" in key), None)
    if lidar_key is None:
        raise SystemExit("USDZ has no lidar_top calibration")
    T_sensor_rig = np.asarray(calibrations[lidar_key]["T_sensor_rig"], dtype=float)
    return rig_timestamps, rig_matrices, tracks, T_sensor_rig


def interpolate(
    timestamps: list[int], poses: list[np.ndarray], timestamp_us: int
) -> np.ndarray:
    index = int(np.searchsorted(timestamps, timestamp_us))
    if index == 0:
        return poses[0]
    if index >= len(timestamps):
        return poses[-1]
    t0, t1 = timestamps[index - 1], timestamps[index]
    weight = (timestamp_us - t0) / (t1 - t0)
    return poses[index - 1] + weight * (poses[index] - poses[index - 1])


def rig_pose(
    timestamps: list[int], matrices: list[np.ndarray], timestamp_us: int
) -> np.ndarray:
    index = int(np.searchsorted(timestamps, timestamp_us))
    if index == 0:
        return matrices[0]
    if index >= len(timestamps):
        return matrices[-1]
    t0, t1 = timestamps[index - 1], timestamps[index]
    weight = (timestamp_us - t0) / (t1 - t0)
    m0, m1 = matrices[index - 1], matrices[index]
    result = np.eye(4)
    result[:3, :3] = m0[:3, :3] + weight * (m1[:3, :3] - m0[:3, :3])
    result[:3, 3] = m0[:3, 3] + weight * (m1[:3, 3] - m0[:3, 3])
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame", type=int, default=0)
    parser.add_argument("--usdz", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    args = parser.parse_args(argv)

    rig_timestamps, rig_matrices, tracks, T_sensor_rig = load_artifact(args.usdz)
    rows = [
        json.loads(line)
        for line in args.dataset.joinpath("frames.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    if args.frame < 0 or args.frame >= len(rows):
        raise SystemExit(f"frame {args.frame} out of range (0..{len(rows) - 1})")
    row = rows[args.frame]
    timestamp_us = int(row["timestamp_us"])
    lidar_path = args.dataset / row["lidar"]["original_path"]
    points = np.fromfile(lidar_path, dtype="<f4").reshape(-1, 4)[:, :3]

    T_world_sensor = rig_pose(rig_timestamps, rig_matrices, timestamp_us) @ T_sensor_rig
    T_sensor_world = np.linalg.inv(T_world_sensor)

    vehicles = [
        (track_id, track)
        for track_id, track in sorted(tracks.items())
        if track["flag"] == CONTROLLABLE_FLAG and track["label"] in VEHICLE_LABELS
    ]
    print(f"frame {args.frame} timestamp_us={timestamp_us} points={len(points)}")
    print(f"controllable vehicles: {len(vehicles)}")
    print(f"{'track':24s} {'label':10s} {'sensor pos':>22s} {'pts<2m':>6s} {'pts<5m':>6s} note")
    for track_id, track in vehicles:
        world_pose = interpolate(track["timestamps_us"], track["poses"], timestamp_us)
        sensor_pos = (T_sensor_world @ np.r_[world_pose[:3], 1.0])[:3]
        distance = np.linalg.norm(points[:, :3] - sensor_pos, axis=1)
        within_2m = int((distance < 2.0).sum())
        within_5m = int((distance < 5.0).sum())
        note = MATCHED_TRACKS.get(track_id, "")
        if track_id == "c1958768d48640948f6053d04cffd35b":
            roi = row["lidar"].get("original_target_response_position_m")
            note += f" (server ROI {[round(v, 2) for v in roi] if roi else 'n/a'})"
        print(
            f"{track_id[:24]:24s} {track['label']:10s} {sensor_pos.round(1)!s:>22s} "
            f"{within_2m:6d} {within_5m:6d} {note}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
