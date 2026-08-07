"""Build the shared, fail-closed actor manifest for open-loop bbox scoring."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.open_loop_bbox_binding import (  # noqa: E402
    OpenLoopBBoxBindingError,
    build_actor_manifest,
    write_actor_manifest,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-ir", type=Path, required=True)
    parser.add_argument("--usdz", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frame-interval-sec", type=float, default=0.05)
    parser.add_argument("--usdz-time-tolerance-us", type=int, default=120_000)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise OpenLoopBBoxBindingError(
                f"refusing to overwrite actor manifest: {args.output}"
            )
        scenario_ir = json.loads(args.scenario_ir.read_text(encoding="utf-8"))
        manifest = build_actor_manifest(
            scenario_ir,
            scenario_ir_path=args.scenario_ir,
            usdz_path=args.usdz,
            frame_interval_sec=args.frame_interval_sec,
            usdz_time_tolerance_us=args.usdz_time_tolerance_us,
        )
        write_actor_manifest(args.output, manifest)
        print(
            json.dumps(
                {
                    "status": "built",
                    "output": str(args.output.resolve()),
                    "manifest_sha256": manifest["manifest_sha256"],
                    "summary": manifest["summary"],
                    "rejected_supported_ir_actors": len(
                        manifest["rejected_supported_ir_actors"]
                    ),
                },
                ensure_ascii=False,
                sort_keys=True,
            )
        )
        return 0
    except (OSError, ValueError, json.JSONDecodeError, OpenLoopBBoxBindingError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
