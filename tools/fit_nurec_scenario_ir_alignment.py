#!/usr/bin/env python3
"""Fit the 2D rigid transform from Scenario IR coordinates to NuRec coordinates."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


def _percentiles(values: np.ndarray) -> dict[str, float]:
    return {
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def _fit_orthogonal_2d(
    source: np.ndarray, target: np.ndarray, *, allow_reflection: bool
) -> tuple[np.ndarray, np.ndarray]:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if not allow_reflection and np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def _fit_report(
    source: np.ndarray, target: np.ndarray, *, allow_reflection: bool
) -> dict[str, object]:
    matrix, translation = _fit_orthogonal_2d(
        source, target, allow_reflection=allow_reflection
    )
    aligned = source @ matrix.T + translation
    residual = np.linalg.norm(aligned - target, axis=1)
    axis_angle = math.degrees(math.atan2(matrix[1, 0], matrix[0, 0]))
    return {
        "x_axis_angle_deg": axis_angle,
        "determinant": float(np.linalg.det(matrix)),
        "translation_xy_m": translation.tolist(),
        "matrix_3x3": [
            [float(matrix[0, 0]), float(matrix[0, 1]), float(translation[0])],
            [float(matrix[1, 0]), float(matrix[1, 1]), float(translation[1])],
            [0.0, 0.0, 1.0],
        ],
        "residual_m": _percentiles(residual),
        "endpoint_residual_m": {
            "first": float(residual[0]),
            "last": float(residual[-1]),
        },
    }


def _trajectory_length(points: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nurec-example-root", type=Path, required=True)
    parser.add_argument("--usdz", type=Path, required=True)
    parser.add_argument("--scenario-ir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--nurec-time-scale", type=float)
    args = parser.parse_args()

    sys.path.insert(0, str(args.nurec_example_root))
    from scenario import Scenario

    ir = json.loads(args.scenario_ir.read_text(encoding="utf-8"))
    reference = ir["ego"]["reference_trajectory"]
    ir_times = np.asarray([row["t_sec"] for row in reference], dtype=float)
    ir_points = np.asarray([[row["x"], row["y"]] for row in reference], dtype=float)

    scenario = Scenario(str(args.usdz))
    nurec_times = np.asarray(scenario.ego_poses.timestamps, dtype=float)
    nurec_points = np.asarray(
        [np.asarray(pose, dtype=float)[:2, 3] for pose in scenario.ego_poses.poses],
        dtype=float,
    )
    if len(nurec_times) != len(nurec_points):
        raise ValueError("NuRec timestamp and pose counts differ")
    if len(ir_times) < 2 or len(nurec_times) < 2:
        raise ValueError("At least two poses are required")

    raw_ir_range = [float(ir_times[0]), float(ir_times[-1])]
    raw_nurec_range = [float(nurec_times[0]), float(nurec_times[-1])]
    ir_times = ir_times - ir_times[0]
    nurec_times = nurec_times - nurec_times[0]
    if args.nurec_time_scale is None:
        candidates = (1.0, 1e-3, 1e-6, 1e-9)
        time_scale = min(
            candidates,
            key=lambda value: abs(
                math.log(max(float(nurec_times[-1]) * value, 1e-15) / float(ir_times[-1]))
            ),
        )
    else:
        time_scale = args.nurec_time_scale
    nurec_times *= time_scale
    end_time = min(float(ir_times[-1]), float(nurec_times[-1]))
    keep = nurec_times <= end_time + 1e-9
    fit_times = nurec_times[keep]
    target = nurec_points[keep]
    source = np.column_stack(
        [
            np.interp(fit_times, ir_times, ir_points[:, 0]),
            np.interp(fit_times, ir_times, ir_points[:, 1]),
        ]
    )

    native_fit = _fit_report(source, target, allow_reflection=False)
    carla_target = target.copy()
    carla_target[:, 1] *= -1.0
    carla_fit = _fit_report(source, carla_target, allow_reflection=True)
    unaligned = np.linalg.norm(source - target, axis=1)
    source_length = _trajectory_length(source)
    target_length = _trajectory_length(target)

    report = {
        "schema_version": "nurec_scenario_ir_alignment.v1",
        "source_frame": ir["coordinate_frame"]["name"],
        "target_frame": "nurec_world_base",
        "ir_pose_count": len(reference),
        "nurec_pose_count": len(nurec_points),
        "fit_pose_count": len(source),
        "ir_duration_s": float(ir_times[-1]),
        "nurec_duration_s": float(nurec_times[-1]),
        "fit_duration_s": end_time,
        "nurec_time_scale_to_seconds": time_scale,
        "raw_timestamp_range": {
            "scenario_ir": raw_ir_range,
            "nurec": raw_nurec_range,
        },
        "rigid_transform_ir_to_nurec": native_fit,
        "orthogonal_transform_ir_to_carla": carla_fit,
        "unaligned_residual_m": _percentiles(unaligned),
        "trajectory_length_m": {
            "scenario_ir_interpolated": source_length,
            "nurec": target_length,
            "ratio_nurec_over_ir": (
                target_length / source_length if source_length > 1e-12 else None
            ),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
