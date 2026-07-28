from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.lidar_world_support import (
    LidarWorldSupportError,
    lidar_occupancy_from_xyzi,
    summarize_xyzi_payload,
)


def build_m8_lidar_occupancy(
    runtime_rows: list[Mapping[str, Any]],
    nurec_rows: list[Mapping[str, Any]],
    run_config: Mapping[str, Any],
) -> list[dict[str, Any]]:
    """Join same-frame CARLA truth to materialized NRE LiDAR payload bytes."""

    sensor_to_ego = _lidar_sensor_to_ego(run_config)
    payload_by_frame = _lidar_payloads(nurec_rows)
    result = []
    for runtime in runtime_rows:
        frame_id = runtime.get("frame_id")
        if not isinstance(frame_id, int) or isinstance(frame_id, bool):
            raise LidarWorldSupportError("M8 runtime row requires integer frame_id")
        payload = payload_by_frame.get(frame_id)
        if payload is None:
            raise LidarWorldSupportError(f"M8 frame {frame_id} has no passed materialized LiDAR payload")
        path = Path(str(payload["path"]))
        if not path.is_file():
            raise LidarWorldSupportError(f"M8 LiDAR payload does not exist: {path}")
        body = path.read_bytes()
        actual_sha = hashlib.sha256(body).hexdigest()
        if actual_sha != payload["sha256"]:
            raise LidarWorldSupportError(f"M8 LiDAR payload SHA-256 mismatch: {path}")
        ego = runtime.get("ego_state")
        if isinstance(ego, Mapping):
            ego_pose = ego.get("pose")
        else:
            ego_pose = None
        states = runtime.get("object_states")
        if not isinstance(states, list):
            raise LidarWorldSupportError(f"M8 runtime frame {frame_id} has no object_states")
        result.append(
            {
                "schema_version": "m8_lidar_occupancy.v1",
                "frame_id": frame_id,
                "simulation_time_sec": runtime.get("simulation_time_sec"),
                "lidar_payload": payload,
                "sensor_to_ego": sensor_to_ego,
                "sensor_payload_summary": summarize_xyzi_payload(body),
                "occupancy": lidar_occupancy_from_xyzi(
                    body,
                    ego_pose=ego_pose,
                    sensor_to_ego=sensor_to_ego,
                    object_states=states,
                ),
            }
        )
    return result


def _lidar_sensor_to_ego(config: Mapping[str, Any]) -> list[Any]:
    runtime = config.get("nurec_runtime")
    specs = runtime.get("lidar_specs") if isinstance(runtime, Mapping) else None
    if not isinstance(specs, list):
        raise LidarWorldSupportError("run config requires nurec_runtime.lidar_specs")
    matches = [item for item in specs if isinstance(item, Mapping) and item.get("sensor_id") == "lidar_top"]
    if len(matches) != 1:
        raise LidarWorldSupportError("run config requires exactly one lidar_top spec")
    value = matches[0].get("sensor_to_ego")
    if not isinstance(value, list) or len(value) != 16:
        raise LidarWorldSupportError("lidar_top has no 4x4 sensor_to_ego")
    return value


def _lidar_payloads(rows: list[Mapping[str, Any]]) -> dict[int, dict[str, Any]]:
    result = {}
    for row in rows:
        if row.get("status") != "passed" or not isinstance(row.get("frame_id"), int):
            continue
        records = row.get("records")
        if not isinstance(records, list):
            continue
        matches = [item for item in records if isinstance(item, Mapping) and item.get("modality") == "lidar" and item.get("sensor_id") == "lidar_top" and item.get("status") == "passed"]
        if len(matches) != 1:
            continue
        metadata = matches[0].get("response_metadata")
        materialized = metadata.get("materialized_payload") if isinstance(metadata, Mapping) else None
        if not isinstance(materialized, Mapping):
            continue
        path, sha = materialized.get("path"), materialized.get("sha256")
        if materialized.get("coordinate_frame") != "sensor_local" or materialized.get("axis_convention") != "carla_sensor":
            raise LidarWorldSupportError(f"frame {row['frame_id']} LiDAR is not normalized sensor_local/carla_sensor")
        if not isinstance(path, str) or not isinstance(sha, str) or len(sha) != 64:
            raise LidarWorldSupportError(f"frame {row['frame_id']} LiDAR materialization has no identity")
        frame_id = row["frame_id"]
        if frame_id in result:
            raise LidarWorldSupportError(f"duplicate NRE LiDAR payload frame: {frame_id}")
        result[frame_id] = {"path": path, "sha256": sha, "sensor_id": "lidar_top"}
    return result


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a JSON object")
    return value


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compute independent NRE LiDAR occupancy for M8 physical boxes.")
    parser.add_argument("--m8-runtime-trace", required=True, type=Path)
    parser.add_argument("--nurec-multimodal-trace", required=True, type=Path)
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite M8 LiDAR occupancy: {args.output}")
        rows = build_m8_lidar_occupancy(
            _read_jsonl(args.m8_runtime_trace),
            _read_jsonl(args.nurec_multimodal_trace),
            _read_json(args.run_config),
        )
        if not rows:
            raise LidarWorldSupportError("M8 LiDAR occupancy requires at least one frame")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text("".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in rows), encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, LidarWorldSupportError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output": str(args.output), "frame_count": len(rows)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
