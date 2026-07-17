from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_260_client import build_nurec_260_client
from adapters.nurec_multimodal import NuRecMultimodalError
from adapters.nurec_pose_probe import run_nurec_dynamic_pose_ab_probe


def run_probe(
    config_path: Path,
    baseline_frame_path: Path,
    moved_frame_path: Path,
    *,
    track_id: str,
) -> dict[str, Any]:
    config = _load_object(config_path)
    baseline = _load_object(baseline_frame_path)
    moved = _load_object(moved_frame_path)
    client = build_nurec_260_client(config)
    try:
        return run_nurec_dynamic_pose_ab_probe(
            track_id,
            baseline,
            moved,
            dispatch_frame=client.dispatch_frame,
        )
    finally:
        client.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run a fixed-time NuRec 26.04 A/B dynamic-pose probe. Both RGB and "
            "LiDAR render digests must change before the track can be promoted."
        )
    )
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--baseline-frame", required=True, type=Path)
    parser.add_argument("--moved-frame", required=True, type=Path)
    parser.add_argument("--track-id", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    try:
        result = run_probe(
            args.config,
            args.baseline_frame,
            args.moved_frame,
            track_id=args.track_id,
        )
    except (OSError, ValueError, NuRecMultimodalError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"status": result["status"], "output": str(args.output)}))
    return 0 if result["status"] == "passed" else 2


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


if __name__ == "__main__":
    raise SystemExit(main())
