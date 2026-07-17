from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.reconstruction_package import (
    load_reconstruction_package,
    load_reconstruction_result,
)
from adapters.reconstruction_planning import build_reconstruction_integration_plan


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate an existing NuRec result and plan its ClosedLoopBench integration."
    )
    parser.add_argument("--scenario-ir", required=True, type=Path)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--reconstruction-package", type=Path)
    source.add_argument("--reconstruction-result", type=Path)
    parser.add_argument("--exchange-root", type=Path)
    parser.add_argument("--expected-global-step", type=int, default=1000)
    parser.add_argument(
        "--expected-camera-id",
        action="append",
        dest="expected_camera_ids",
        help="Expected NuRec camera ID; repeat for every formal camera",
    )
    parser.add_argument("--expected-samples-per-epoch", type=int, default=1000)
    parser.add_argument("--expected-max-epochs", type=int, default=1)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)

    scenario_ir = json.loads(args.scenario_ir.read_text(encoding="utf-8"))
    scene_id = str(scenario_ir.get("scenario_id") or "")
    if args.reconstruction_result:
        if args.exchange_root is None:
            parser.error("--exchange-root is required with --reconstruction-result")
        package = load_reconstruction_result(
            args.reconstruction_result,
            exchange_root=args.exchange_root,
            expected_scene_id=scene_id,
        )
    else:
        package = load_reconstruction_package(
            args.reconstruction_package,
            expected_scene_id=scene_id,
        )
    plan_kwargs = {
        "expected_global_step": args.expected_global_step,
        "expected_samples_per_epoch": args.expected_samples_per_epoch,
        "expected_max_epochs": args.expected_max_epochs,
    }
    if args.expected_camera_ids:
        plan_kwargs["expected_camera_ids"] = tuple(args.expected_camera_ids)
    plan = build_reconstruction_integration_plan(scenario_ir, package, **plan_kwargs)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "passed", "plan": str(args.output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
