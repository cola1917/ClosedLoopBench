from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_reconstruction_smoke import (
    NuRecReconstructionSmokeError,
    audit_reconstruction_smoke,
    load_config,
    load_track_ids,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a low-cost NuRec reconstruction source/config smoke gate."
    )
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--scene-object-registry", required=True, type=Path)
    parser.add_argument("--source-track-manifest", required=True, type=Path)
    parser.add_argument("--expected-camera-id", action="append", dest="expected_camera_ids")
    parser.add_argument("--max-samples-per-epoch", type=int, default=1000)
    parser.add_argument("--max-epochs", type=int, default=1)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"refusing to overwrite smoke evidence: {args.output}")
    try:
        config = load_config(args.config)
        registry = json.loads(args.scene_object_registry.read_text(encoding="utf-8-sig"))
        if not isinstance(registry, dict):
            raise NuRecReconstructionSmokeError("scene object registry must be an object")
        report = audit_reconstruction_smoke(
            config,
            registry,
            source_track_ids=load_track_ids(args.source_track_manifest),
            expected_camera_ids=args.expected_camera_ids,
            max_samples_per_epoch=args.max_samples_per_epoch,
            max_epochs=args.max_epochs,
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "output": str(args.output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
