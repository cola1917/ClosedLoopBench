from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runtime.scene0061_variants import Scene0061VariantError, build_scene0061_variant


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build a corridor-constrained scene0061 TF++ S0/S2/S4 run config."
    )
    parser.add_argument("--base-run-config", type=Path, required=True)
    parser.add_argument(
        "--case-id",
        choices=("S0_original_replay", "S2_lead_hard_brake", "S4_pedestrian_early_crossing"),
        required=True,
    )
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--event-timestamp-sec", type=float)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--delta-report", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists() or args.delta_report.exists():
            parser.error("refusing to overwrite existing variant output")
        base = json.loads(args.base_run_config.read_text(encoding="utf-8"))
        variant, delta = build_scene0061_variant(
            base,
            case_id=args.case_id,
            seed=args.seed,
            event_timestamp_sec=args.event_timestamp_sec,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.delta_report.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(variant, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        args.delta_report.write_text(
            json.dumps(delta, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(delta, ensure_ascii=False, sort_keys=True))
        return 0
    except (OSError, ValueError, json.JSONDecodeError, Scene0061VariantError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
