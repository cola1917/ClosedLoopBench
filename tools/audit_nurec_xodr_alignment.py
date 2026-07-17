#!/usr/bin/env python3
"""Measure recorded NuRec ego poses against the active CARLA OpenDRIVE map."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


def percentile(values, q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), q))


def angle_difference_degrees(left: float, right: float) -> float:
    return abs((left - right + 180.0) % 360.0 - 180.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--carla-root", type=Path, required=True)
    parser.add_argument("--nurec-example-root", type=Path, required=True)
    parser.add_argument("--usdz", type=Path, required=True)
    parser.add_argument("--xodr", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    sys.path.insert(0, str(args.carla_root / "PythonAPI" / "carla"))
    sys.path.insert(0, str(args.nurec_example_root))
    import carla
    from projection_functions import get_t_rig_enu_from_ecef
    from scenario import Scenario
    from utils import mat_to_carla_transform

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()
    carla_map = world.get_map()
    scenario = Scenario(str(args.usdz))
    xodr = args.xodr.read_text(encoding="utf-8")
    scenario_to_carla = get_t_rig_enu_from_ecef(scenario.t_world_base, xodr)

    samples = []
    for index, pose in enumerate(scenario.ego_poses.poses):
        transform = mat_to_carla_transform(scenario_to_carla @ pose)
        waypoint = carla_map.get_waypoint(
            transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if waypoint is None:
            samples.append({"index": index, "waypoint": None})
            continue
        dx = transform.location.x - waypoint.transform.location.x
        dy = transform.location.y - waypoint.transform.location.y
        dz = transform.location.z - waypoint.transform.location.z
        horizontal = math.hypot(dx, dy)
        samples.append(
            {
                "index": index,
                "horizontal_centerline_distance_m": horizontal,
                "vertical_distance_m": abs(dz),
                "heading_error_deg": angle_difference_degrees(
                    transform.rotation.yaw, waypoint.transform.rotation.yaw
                ),
                "lane_width_m": waypoint.lane_width,
                "inside_lane_by_center_distance": horizontal <= waypoint.lane_width / 2.0,
                "road_id": waypoint.road_id,
                "lane_id": waypoint.lane_id,
                "is_junction": waypoint.is_junction,
            }
        )

    valid = [sample for sample in samples if sample.get("waypoint", True) is not None]
    horizontal = [sample["horizontal_centerline_distance_m"] for sample in valid]
    vertical = [sample["vertical_distance_m"] for sample in valid]
    heading = [sample["heading_error_deg"] for sample in valid]
    report = {
        "schema_version": "nurec_xodr_alignment.v1",
        "map_name": carla_map.name,
        "pose_count": len(samples),
        "waypoint_count": len(valid),
        "inside_lane_fraction": (
            sum(sample["inside_lane_by_center_distance"] for sample in valid) / len(valid)
            if valid
            else 0.0
        ),
        "horizontal_centerline_distance_m": {
            "p50": percentile(horizontal, 50) if horizontal else None,
            "p95": percentile(horizontal, 95) if horizontal else None,
            "max": max(horizontal) if horizontal else None,
        },
        "vertical_distance_m": {
            "p50": percentile(vertical, 50) if vertical else None,
            "p95": percentile(vertical, 95) if vertical else None,
            "max": max(vertical) if vertical else None,
        },
        "heading_error_deg": {
            "p50": percentile(heading, 50) if heading else None,
            "p95": percentile(heading, 95) if heading else None,
            "max": max(heading) if heading else None,
        },
        "unique_road_lane_pairs": sorted(
            {f"{sample['road_id']}:{sample['lane_id']}" for sample in valid}
        ),
        "junction_pose_count": sum(sample["is_junction"] for sample in valid),
        "samples": samples,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "samples"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
