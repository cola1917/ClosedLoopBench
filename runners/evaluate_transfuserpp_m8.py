"""Evaluate TransFuser++ intermediates against original scene GT for M8."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from agents.plugin_contract import strict_json_loads
from metrics.transfuserpp_m8 import (
    TransFuserPPM8Error,
    evaluate_m8_intermediate_trace,
)


def _load_records(path: Path) -> list[dict[str, Any]]:
    if path.is_dir():
        paths = sorted(path.glob("*.intermediate.json"))
        return [strict_json_loads(item.read_text(encoding="utf-8")) for item in paths]
    if path.suffix.lower() == ".jsonl":
        return [
            strict_json_loads(line)
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    value = strict_json_loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, list) else [value]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate TF++ intermediates against original Scenario IR GT. "
            "Non-raw variants require --allow-non-raw-input; formal bbox scoring requires --actor-manifest."
        )
    )
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--scenario-ir", type=Path, required=True)
    parser.add_argument(
        "--actor-manifest",
        type=Path,
        required=True,
        help="Shared dynamic-actor manifest used for formal same-frame bbox GT binding.",
    )
    parser.add_argument("--evidence-root", type=Path)
    parser.add_argument(
        "--expected-input-source",
        default="carla_stage_a_native_rgb_lidar",
        help="M8 raw input source; reconstructed/Harmonizer sources are rejected.",
    )
    parser.add_argument("--expected-scenario-ir-sha256")
    parser.add_argument(
        "--allow-non-raw-input",
        action="store_true",
        help="Allow reconstructed and Harmonizer RGB input variants for the formal comparison.",
    )
    parser.add_argument("--waypoint-spacing-sec", type=float, default=0.5)
    parser.add_argument(
        "--expected-frame-count",
        type=int,
        default=39,
        help="Formal scene-0061 frame count. Pass 0 to disable the count gate.",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            parser.error(f"refusing to overwrite existing output: {args.output}")
        scenario_ir = strict_json_loads(args.scenario_ir.read_text(encoding="utf-8"))
        expected_count = args.expected_frame_count or None
        report = evaluate_m8_intermediate_trace(
            _load_records(args.trace),
            scenario_ir=scenario_ir,
            scenario_ir_path=args.scenario_ir,
            expected_input_source=args.expected_input_source,
            evidence_root=args.evidence_root,
            expected_scenario_ir_sha256=args.expected_scenario_ir_sha256,
            waypoint_spacing_sec=args.waypoint_spacing_sec,
            expected_frame_count=expected_count,
            allow_non_raw_input=args.allow_non_raw_input,
            actor_manifest_path=args.actor_manifest,
            require_actor_manifest=True,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, ensure_ascii=False, sort_keys=True))
        return 0 if report["status"] == "evaluated" else 2
    except (OSError, ValueError, json.JSONDecodeError, TransFuserPPM8Error) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
