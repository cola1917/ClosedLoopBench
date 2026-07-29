from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_render_selection import NuRecRenderSelectionError, build_render_selection


def _load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build an auditable NuRec ego-corridor render selection.")
    parser.add_argument("--scene-object-registry", required=True, type=Path)
    parser.add_argument("--visibility-manifest", required=True, type=Path)
    parser.add_argument("--dynamic-source-support", required=True, type=Path)
    parser.add_argument("--static-source-support", required=True, type=Path)
    parser.add_argument("--mandatory-object-id", action="append", default=[])
    parser.add_argument("--max-corridor-range-m", type=float, default=80.0)
    parser.add_argument("--min-padded-coverage", type=float, default=0.75)
    parser.add_argument("--min-exact-coverage", type=float, default=0.50)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"refusing to overwrite immutable selection evidence: {args.output}")
    try:
        report = build_render_selection(
            _load(args.scene_object_registry),
            _load(args.visibility_manifest),
            _load(args.dynamic_source_support),
            _load(args.static_source_support),
            mandatory_object_ids=args.mandatory_object_id,
            max_corridor_range_m=args.max_corridor_range_m,
            min_padded_coverage=args.min_padded_coverage,
            min_exact_coverage=args.min_exact_coverage,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, NuRecRenderSelectionError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": report["status"], "output": str(args.output), "summary": report["summary"]}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
