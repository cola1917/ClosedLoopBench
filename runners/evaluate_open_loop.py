"""Score open-loop predictions against a pinned Scenario IR trajectory."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from metrics.open_loop import score_open_loop_predictions
from runners.run_open_loop_gt_replay import (
    EXPECTED_OPENDRIVE_SHA256,
    EXPECTED_SCENARIO_IR_SHA256,
    load_pinned_inputs,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-ir", type=Path, default="outputs/scene0061_exchange_v2/scene_ir.json")
    parser.add_argument("--opendrive", type=Path, default="outputs/scene-0061/road.xodr")
    parser.add_argument("--expected-scenario-ir-sha256", default=EXPECTED_SCENARIO_IR_SHA256)
    parser.add_argument("--expected-opendrive-sha256", default=EXPECTED_OPENDRIVE_SHA256)
    parser.add_argument("--predictions", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.report.exists():
            raise ValueError(f"refusing to overwrite existing output: {args.report}")
        inputs = load_pinned_inputs(
            args.scenario_ir,
            args.opendrive,
            expected_scenario_ir_sha256=args.expected_scenario_ir_sha256,
            expected_opendrive_sha256=args.expected_opendrive_sha256,
        )
        predictions = json.loads(args.predictions.read_text(encoding="utf-8"))
        report = score_open_loop_predictions(
            inputs.scenario_ir,
            predictions,
            scenario_ir_path=str(inputs.scenario_ir_path),
            scenario_ir_sha256=inputs.scenario_ir_sha256,
            opendrive_path=str(inputs.opendrive_path),
            opendrive_sha256=inputs.opendrive_sha256,
        )
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"status": "written", "report": str(args.report)}, ensure_ascii=False))
        return 0 if report["execution_status"] == "completed" else 2
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
