from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_render_selection import (
    NuRecRenderSelectionError,
    build_lifecycle_quality_manifest,
)


def _load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Compatibility builder for the legacy lifecycle LiDAR report; "
            "new M8 work must use the editable-quality-window manifest."
        )
    )
    parser.add_argument("--scene-object-registry", required=True, type=Path)
    parser.add_argument("--dynamic-source-support", required=True, type=Path)
    parser.add_argument("--min-exact-points", type=int, default=1)
    parser.add_argument("--min-padded-points", type=int, default=1)
    parser.add_argument("--min-supported-ticks", type=int, default=2)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite immutable lifecycle evidence: {args.output}")
        report = build_lifecycle_quality_manifest(
            _load(args.scene_object_registry),
            _load(args.dynamic_source_support),
            min_exact_points=args.min_exact_points,
            min_padded_points=args.min_padded_points,
            min_supported_ticks=args.min_supported_ticks,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, NuRecRenderSelectionError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": report["status"], "output": str(args.output), "summary": report["summary"]}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
