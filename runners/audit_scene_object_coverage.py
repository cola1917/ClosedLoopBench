from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_object_registry import assert_scene_object_coverage_ready, audit_scene_object_coverage


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Audit an immutable M6 registry against payload-bound visibility evidence.")
    parser.add_argument("--scene-object-registry", required=True, type=Path)
    parser.add_argument("--visibility-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--require-ready", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite coverage audit: {args.output}")
        registry = json.loads(args.scene_object_registry.read_text(encoding="utf-8"))
        visibility = json.loads(args.visibility_manifest.read_text(encoding="utf-8"))
        audit = audit_scene_object_coverage(registry, visibility)
        if args.require_ready:
            assert_scene_object_coverage_ready(audit)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": audit["status"], "output": str(args.output), "issues": audit["issues"]}, ensure_ascii=False))
    return 0 if audit["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
