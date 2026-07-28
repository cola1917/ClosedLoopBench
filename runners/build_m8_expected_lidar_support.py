from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.lidar_world_support import (
    LidarWorldSupportError,
    expected_lidar_support_from_physical_boxes,
)
from runners.build_m8_lidar_occupancy import _lidar_sensor_to_ego


DEFAULT_MAX_RANGE_M = 80.0


def build_expected_lidar_support(
    runtime_rows: list[Mapping[str, Any]],
    run_config: Mapping[str, Any],
    *,
    max_range_m: float = DEFAULT_MAX_RANGE_M,
) -> list[dict[str, Any]]:
    """Build explicit per-tick CARLA physical LiDAR expectation rows."""

    sensor_to_ego = _lidar_sensor_to_ego(run_config)
    rows = []
    seen = set()
    for runtime in runtime_rows:
        frame_id = runtime.get("frame_id")
        if not isinstance(frame_id, int) or isinstance(frame_id, bool) or frame_id in seen:
            raise LidarWorldSupportError("M8 runtime frames require unique integer frame_id")
        seen.add(frame_id)
        ego_state = runtime.get("ego_state")
        states = runtime.get("object_states")
        if not isinstance(ego_state, Mapping) or not isinstance(states, list):
            raise LidarWorldSupportError(f"M8 runtime frame {frame_id} lacks ego_state or object_states")
        rows.append(
            {
                "schema_version": "m8_expected_lidar_support.v1",
                "frame_id": frame_id,
                "simulation_time_sec": runtime.get("simulation_time_sec"),
                "sensor_to_ego": sensor_to_ego,
                "expected_world_objects": expected_lidar_support_from_physical_boxes(
                    ego_pose=ego_state.get("pose"),
                    sensor_to_ego=sensor_to_ego,
                    object_states=states,
                    max_range_m=max_range_m,
                ),
            }
        )
    if not rows:
        raise LidarWorldSupportError("M8 expected LiDAR support requires at least one runtime frame")
    return rows


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a JSON object")
    return value


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build occlusion-aware CARLA LiDAR support expectations for M8.")
    parser.add_argument("--m8-runtime-trace", required=True, type=Path)
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--max-range-m", type=float, default=DEFAULT_MAX_RANGE_M)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite M8 expected LiDAR support: {args.output}")
        rows = build_expected_lidar_support(
            _read_jsonl(args.m8_runtime_trace), _read_json(args.run_config), max_range_m=args.max_range_m
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text("".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in rows), encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, LidarWorldSupportError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output": str(args.output), "frame_count": len(rows)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
