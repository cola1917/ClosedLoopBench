from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_safety_audit import SceneSafetyAuditError


def build_m8_evidence_from_artifacts(
    runtime_rows: list[Mapping[str, Any]],
    expected_visibility: Mapping[str, Any],
    expected_lidar_rows: list[Mapping[str, Any]],
    occupancy_rows: list[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """Join immutable same-frame M8 artifacts into four-stream audit input."""

    visibility_by_frame: dict[int, list[dict[str, Any]]] = {}
    for observation in expected_visibility.get("observations") or []:
        if not isinstance(observation, Mapping) or observation.get("observation_kind") != "calibrated_3d_box_projection":
            continue
        frame_id = observation.get("frame_id")
        if not isinstance(frame_id, int):
            raise SceneSafetyAuditError("M8 visibility observation has no integer frame_id")
        visibility_by_frame.setdefault(frame_id, []).append({**dict(observation), "expected_visible": True})
    expected_by_frame = _unique_by_frame(expected_lidar_rows, "expected LiDAR support")
    occupancy_by_frame = _unique_by_frame(occupancy_rows, "LiDAR occupancy")
    result = []
    seen = set()
    for runtime in runtime_rows:
        frame_id = runtime.get("frame_id")
        if not isinstance(frame_id, int) or isinstance(frame_id, bool) or frame_id in seen:
            raise SceneSafetyAuditError("M8 runtime frames require unique integer frame_id")
        seen.add(frame_id)
        expected = expected_by_frame.get(frame_id)
        occupancy = occupancy_by_frame.get(frame_id)
        if expected is None or occupancy is None:
            raise SceneSafetyAuditError(f"M8 frame {frame_id} lacks expected LiDAR or occupancy artifact")
        if frame_id not in visibility_by_frame:
            raise SceneSafetyAuditError(f"M8 frame {frame_id} lacks calibrated visibility artifact")
        _check_same_tick_time(runtime, expected, "expected LiDAR support")
        _check_same_tick_time(runtime, occupancy, "LiDAR occupancy")
        objects = expected.get("expected_world_objects")
        supported = occupancy.get("occupancy")
        if not isinstance(objects, list) or not isinstance(supported, list):
            raise SceneSafetyAuditError(f"M8 frame {frame_id} has invalid LiDAR artifacts")
        result.append(
            {
                "frame_id": frame_id,
                "simulation_time_sec": runtime.get("simulation_time_sec"),
                "collision": {
                    key: runtime.get(key)
                    for key in ("ego_state", "object_states", "collision_events", "collision_detected")
                },
                "lane": {"lane_state": runtime.get("lane_state")},
                "visibility": {"projections": visibility_by_frame.get(frame_id, [])},
                "lidar_world": {"expected_world_objects": objects, "lidar_occupancy": supported},
            }
        )
    if not result:
        raise SceneSafetyAuditError("M8 evidence requires at least one runtime frame")
    runtime_ids = set(seen)
    for label, rows in (("visibility", visibility_by_frame), ("expected LiDAR support", expected_by_frame), ("LiDAR occupancy", occupancy_by_frame)):
        extra = sorted(set(rows) - runtime_ids)
        if extra:
            raise SceneSafetyAuditError(f"M8 {label} contains frames absent from runtime: {extra}")
    return result


def _check_same_tick_time(runtime: Mapping[str, Any], evidence: Mapping[str, Any], label: str) -> None:
    runtime_time = runtime.get("simulation_time_sec")
    evidence_time = evidence.get("simulation_time_sec")
    if evidence_time is None or runtime_time is None:
        return
    try:
        delta = abs(float(runtime_time) - float(evidence_time))
    except (TypeError, ValueError):
        raise SceneSafetyAuditError(f"M8 {label} has invalid simulation time")
    if delta > 1e-6:
        raise SceneSafetyAuditError(f"M8 {label} is not same-tick with runtime frame {runtime.get('frame_id')}")


def _unique_by_frame(rows: list[Mapping[str, Any]], label: str) -> dict[int, Mapping[str, Any]]:
    result = {}
    for row in rows:
        frame_id = row.get("frame_id")
        if not isinstance(frame_id, int) or isinstance(frame_id, bool):
            raise SceneSafetyAuditError(f"M8 {label} row has no integer frame_id")
        if frame_id in result:
            raise SceneSafetyAuditError(f"duplicate M8 {label} frame: {frame_id}")
        result[frame_id] = row
    return result


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a JSON object")
    return value


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Join immutable M8 artifacts into safety-audit input.")
    parser.add_argument("--m8-runtime-trace", required=True, type=Path)
    parser.add_argument("--expected-visibility", required=True, type=Path)
    parser.add_argument("--expected-lidar-support", required=True, type=Path)
    parser.add_argument("--lidar-occupancy", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite M8 evidence input: {args.output}")
        rows = build_m8_evidence_from_artifacts(
            _read_jsonl(args.m8_runtime_trace),
            _read_json(args.expected_visibility),
            _read_jsonl(args.expected_lidar_support),
            _read_jsonl(args.lidar_occupancy),
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text("".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in rows), encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, SceneSafetyAuditError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output": str(args.output), "frame_count": len(rows)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
