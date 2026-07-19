from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runtime.scene0061_video_manifest import (  # noqa: E402
    build_scene0061_video_manifest,
    validate_scene0061_video_manifest,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Build or validate the evidence-backed scene-0061 video shot manifest."
    )
    parser.add_argument("--evidence-root", required=True)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--output", help="Build a new manifest; existing files are not overwritten.")
    group.add_argument("--validate", help="Validate an existing manifest against the filesystem.")
    parser.add_argument("--created-at")
    args = parser.parse_args(argv)
    evidence_root = Path(args.evidence_root)
    if args.validate:
        manifest = json.loads(Path(args.validate).read_text(encoding="utf-8"))
        validate_scene0061_video_manifest(manifest, evidence_root=evidence_root)
        print(json.dumps({"status": "valid", "manifest": args.validate}, ensure_ascii=False))
        return 0

    output = Path(args.output)
    if output.exists():
        parser.error(f"refusing to overwrite existing output: {output}")
    manifest = build_scene0061_video_manifest(evidence_root, created_at=args.created_at)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "status": "written",
                "output": str(output),
                "availability": manifest["availability_summary"],
                "remote_capture_count": len(manifest["remote_capture_queue"]),
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
