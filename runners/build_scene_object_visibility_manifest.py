from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_object_visibility import SceneObjectVisibilityError, build_visibility_manifest


def _load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain an object")
    return value


def _load_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build payload-bound M6 six-camera object visibility evidence.")
    parser.add_argument("--scene-object-registry", required=True, type=Path)
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--frame-trace", required=True, type=Path)
    parser.add_argument("--nurec-multimodal-trace", required=True, type=Path)
    parser.add_argument("--camera-calibration-capture", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-range-m", type=float, default=120.0)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite visibility manifest: {args.output}")
        manifest = build_visibility_manifest(
            _load_json(args.scene_object_registry),
            _load_json(args.run_config),
            _load_jsonl(args.frame_trace),
            _load_jsonl(args.nurec_multimodal_trace),
            _load_json(args.camera_calibration_capture),
            source_files={
                "registry": args.scene_object_registry,
                "run_config": args.run_config,
                "frame_trace": args.frame_trace,
                "nurec_multimodal_trace": args.nurec_multimodal_trace,
                "camera_calibration_capture": args.camera_calibration_capture,
            },
            max_range_m=args.max_range_m,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, SceneObjectVisibilityError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output": str(args.output), "summary": manifest["summary"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
