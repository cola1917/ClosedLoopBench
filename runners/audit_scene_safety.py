from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Callable, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_safety_audit import (
    SceneSafetyAuditError,
    audit_collision_tick,
    audit_lane_tick,
    audit_lidar_world_tick,
    audit_visibility_tick,
)


AUDITS: tuple[tuple[str, str, Callable[..., dict[str, Any]]], ...] = (
    ("collision", "collision_audit.v1.jsonl", audit_collision_tick),
    ("lane", "lane_audit.v1.jsonl", audit_lane_tick),
    ("visibility", "visibility_audit.v1.jsonl", audit_visibility_tick),
    ("lidar_world", "lidar_world_audit.v1.jsonl", audit_lidar_world_tick),
)


def audit_m8_evidence(
    registry: Mapping[str, Any], evidence_rows: list[Mapping[str, Any]]
) -> dict[str, list[dict[str, Any]]]:
    """Turn one raw evidence row per CARLA tick into the four M8 streams."""

    output = {name: [] for name, _, _ in AUDITS}
    seen_frames: set[int] = set()
    for evidence in evidence_rows:
        frame_id = evidence.get("frame_id")
        if not isinstance(frame_id, int) or isinstance(frame_id, bool):
            raise SceneSafetyAuditError("M8 evidence row requires integer frame_id")
        if frame_id in seen_frames:
            raise SceneSafetyAuditError(f"duplicate M8 evidence frame: {frame_id}")
        seen_frames.add(frame_id)
        for name, _, function in AUDITS:
            payload = evidence.get(name)
            if not isinstance(payload, Mapping):
                raise SceneSafetyAuditError(f"M8 evidence frame {frame_id} lacks {name} payload")
            tick = {"frame_id": frame_id, "simulation_time_sec": evidence.get("simulation_time_sec"), **dict(payload)}
            output[name].append(
                function(registry, tick) if name == "collision" else function(tick)
            )
    return output


def write_m8_evidence(
    rows: Mapping[str, list[dict[str, Any]]], output_dir: Path
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    artifacts = {}
    failures = 0
    for name, filename, _ in AUDITS:
        records = list(rows.get(name) or [])
        path = output_dir / filename
        if path.exists():
            raise FileExistsError(f"refusing to overwrite immutable M8 audit: {path}")
        path.write_text(
            "".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in records),
            encoding="utf-8",
        )
        failed = sum(row.get("status") != "passed" for row in records)
        failures += failed
        artifacts[name] = {"path": str(path), "tick_count": len(records), "failed_tick_count": failed}
    summary = {
        "schema_version": "scene_safety_audit_summary.v1",
        "status": "passed" if failures == 0 and all(item["tick_count"] > 0 for item in artifacts.values()) else "failed",
        "artifacts": artifacts,
    }
    summary_path = output_dir / "scene_safety_audit_summary.v1.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def _read_jsonl(path: Path) -> list[Mapping[str, Any]]:
    rows = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise SceneSafetyAuditError(f"invalid JSONL at {path}:{number}") from exc
        if not isinstance(row, Mapping):
            raise SceneSafetyAuditError(f"M8 JSONL row must be an object at {path}:{number}")
        rows.append(row)
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate fail-closed Phase 2 M8 per-tick safety audits.")
    parser.add_argument("--scene-object-registry", required=True)
    parser.add_argument("--evidence-jsonl", required=True, help="One complete raw M8 evidence object per CARLA tick.")
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)
    try:
        registry = json.loads(Path(args.scene_object_registry).read_text(encoding="utf-8"))
        if not isinstance(registry, Mapping):
            raise SceneSafetyAuditError("scene object registry must be an object")
        rows = audit_m8_evidence(registry, _read_jsonl(Path(args.evidence_jsonl)))
        summary = write_m8_evidence(rows, Path(args.output_dir))
    except (OSError, SceneSafetyAuditError, ValueError) as exc:
        parser.error(str(exc))
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
