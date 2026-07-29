from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.lidar_quality_windows import (
    LidarQualityWindowError,
    build_lidar_quality_window_manifest,
)


def _load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build editable LiDAR quality windows for M8 smoke.")
    parser.add_argument("--scene-object-registry", required=True, type=Path)
    parser.add_argument(
        "--source-lidar-support",
        required=True,
        action="append",
        type=Path,
        help="ncore *_lidar_support_audit.v2 report; repeat for dynamic/static reports",
    )
    parser.add_argument("--frame-times", type=Path)
    parser.add_argument("--candidate-object-id", action="append", default=[])
    parser.add_argument("--required-object-id", action="append", default=[])
    parser.add_argument("--min-exact-points", type=int, default=1)
    parser.add_argument("--min-padded-points", type=int, default=1)
    parser.add_argument("--min-consecutive-frames", type=int, default=3)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite immutable quality-window evidence: {args.output}")
        frame_times = _load(args.frame_times) if args.frame_times else None
        reports = [_load(path) for path in args.source_lidar_support]
        report = build_lidar_quality_window_manifest(
            _load(args.scene_object_registry),
            reports,
            frame_times_sec=frame_times,
            candidate_object_ids=args.candidate_object_id or None,
            required_object_ids=args.required_object_id or None,
            min_exact_points=args.min_exact_points,
            min_padded_points=args.min_padded_points,
            min_consecutive_frames=args.min_consecutive_frames,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, LidarQualityWindowError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": report["status"], "output": str(args.output), "summary": report["summary"]}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
