from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.ir_to_carla import build_carla_run_config


def write_carla_run_config(
    scenario_ir_path: Path,
    output: Path,
    *,
    carla_map: str,
    reconstruction_package: str | None = None,
) -> Path:
    scenario_ir = json.loads(scenario_ir_path.read_text(encoding="utf-8"))
    config = build_carla_run_config(
        scenario_ir,
        carla_map=carla_map,
        reconstruction_package_path=reconstruction_package,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Build ClosedLoopBench CARLA run config from Scenario IR.")
    parser.add_argument("--scenario-ir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--carla-map", default="Town04")
    parser.add_argument("--reconstruction-package", default=None)
    args = parser.parse_args(argv)

    output = write_carla_run_config(
        Path(args.scenario_ir),
        Path(args.output),
        carla_map=args.carla_map,
        reconstruction_package=args.reconstruction_package,
    )
    print(json.dumps({"carla_run_config": str(output)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

