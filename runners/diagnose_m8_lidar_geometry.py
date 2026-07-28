"""Diagnose M8 LiDAR/world geometry without changing the safety gate."""

from __future__ import annotations

import argparse
import itertools
import json
import math
import struct
from pathlib import Path
from typing import Any, Mapping


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"{path} contains a non-object JSONL row")
            rows.append(value)
    return rows


def _payload_path(ref: Mapping[str, Any], payload_root: Path | None) -> Path:
    path = Path(str(ref.get("path") or ""))
    if path.is_file():
        return path
    relative = ref.get("relative_path")
    if payload_root is not None and isinstance(relative, str):
        candidate = payload_root / relative
        if candidate.is_file():
            return candidate
    if payload_root is not None:
        # Occupancy rows intentionally retain only the absolute payload path.
        # On a local replay copy, recover the frame by its stable directory
        # suffix without trusting the remote filesystem prefix.
        parts = list(path.parts)
        try:
            suffix = Path(*parts[parts.index("algorithm_sensor_payloads") :])
        except ValueError:
            suffix = None
        if suffix is not None:
            matches = list(payload_root.rglob(suffix.name))
            matches = [item for item in matches if str(item).endswith(str(suffix))]
            if len(matches) == 1:
                return matches[0]
    raise FileNotFoundError(f"LiDAR payload is not available: {path}")


def _matrix(value: Any) -> list[float]:
    if not isinstance(value, list) or len(value) != 16:
        raise ValueError("sensor_to_ego must be a 4x4 matrix")
    return [float(item) for item in value]


def _point_to_world(
    point: tuple[float, float, float],
    matrix: list[float],
    ego: Mapping[str, Any],
    axis: tuple[int, int, int],
    signs: tuple[int, int, int],
) -> tuple[float, float, float]:
    source = [float(value) for value in point]
    x, y, z = (signs[index] * source[axis[index]] for index in range(3))
    ego_x = matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3]
    ego_y = matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7]
    ego_z = matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11]
    yaw = math.radians(float(ego.get("yaw", 0.0)))
    return (
        float(ego["x"]) + math.cos(yaw) * ego_x - math.sin(yaw) * ego_y,
        float(ego["y"]) + math.sin(yaw) * ego_x + math.cos(yaw) * ego_y,
        float(ego["z"]) + ego_z,
    )


def _inside(point: tuple[float, float, float], state: Mapping[str, Any]) -> bool:
    pose = state["pose"]
    extent = state["extent_m"]
    yaw = math.radians(float(pose.get("yaw", 0.0)))
    dx, dy = point[0] - float(pose["x"]), point[1] - float(pose["y"])
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return (
        abs(local_x) <= float(extent["x"]) + 0.10
        and abs(local_y) <= float(extent["y"]) + 0.10
        and abs(point[2] - float(pose["z"])) <= float(extent["z"]) + 0.10
    )


def diagnose(
    runtime_rows: list[Mapping[str, Any]],
    occupancy_rows: list[Mapping[str, Any]],
    expected_rows: list[Mapping[str, Any]],
    *,
    payload_root: Path | None = None,
) -> dict[str, Any]:
    occupancy_by_frame = {int(row["frame_id"]): row for row in occupancy_rows}
    expected_by_frame = {int(row["frame_id"]): row for row in expected_rows}
    ticks = []
    for runtime in runtime_rows:
        frame_id = int(runtime["frame_id"])
        occupancy = occupancy_by_frame[frame_id]
        expected = expected_by_frame[frame_id]
        payload_ref = occupancy["lidar_payload"]
        body = _payload_path(payload_ref, payload_root).read_bytes()
        points = [point[:3] for point in struct.iter_unpack("<4f", body)]
        matrix = _matrix(occupancy["sensor_to_ego"])
        states = {str(state["object_id"]): state for state in runtime["object_states"]}
        expected_objects = {
            str(item["object_id"]): item
            for item in expected["expected_world_objects"]
        }
        candidates = []
        for axis in itertools.permutations(range(3)):
            for signs in itertools.product((-1, 1), repeat=3):
                world = [
                    _point_to_world(point, matrix, runtime["ego_state"]["pose"], axis, signs)
                    for point in points
                ]
                supported = sum(
                    bool(item["expected_lidar_support"])
                    and any(_inside(point, states[object_id]) for point in world)
                    for object_id, item in expected_objects.items()
                    if object_id in states
                )
                candidates.append(
                    {
                        "axis_source_to_carla": list(axis),
                        "signs": list(signs),
                        "expected_supported_object_count": supported,
                    }
                )
        candidates.sort(
            key=lambda item: item["expected_supported_object_count"], reverse=True
        )
        identity = next(
            item
            for item in candidates
            if item["axis_source_to_carla"] == [0, 1, 2]
            and item["signs"] == [1, 1, 1]
        )
        sensor_bounds = {
            axis: [min(point[index] for point in points), max(point[index] for point in points)]
            for index, axis in enumerate(("x", "y", "z"))
        }
        ticks.append(
            {
                "frame_id": frame_id,
                "simulation_time_sec": runtime.get("simulation_time_sec"),
                "point_count": len(points),
                "sensor_bounds_m": sensor_bounds,
                "identity_expected_supported_object_count": identity[
                    "expected_supported_object_count"
                ],
                "best_axis_candidate": candidates[0],
                "axis_candidate_count": len(candidates),
            }
        )
    return {
        "schema_version": "m8_lidar_geometry_diagnostic.v1",
        "status": "diagnostic_only",
        "interpretation": (
            "Axis candidates are geometry diagnostics only; M8 status remains governed "
            "by lidar_world_audit.v1."
        ),
        "ticks": ticks,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-trace", required=True, type=Path)
    parser.add_argument("--occupancy", required=True, type=Path)
    parser.add_argument("--expected", required=True, type=Path)
    parser.add_argument("--payload-root", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"refusing to overwrite immutable diagnostic: {args.output}")
    result = diagnose(
        _read_jsonl(args.runtime_trace),
        _read_jsonl(args.occupancy),
        _read_jsonl(args.expected),
        payload_root=args.payload_root,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": result["status"], "tick_count": len(result["ticks"])}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
