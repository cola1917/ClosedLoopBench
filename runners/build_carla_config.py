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
    weather: str | None = None,
    odd_id: str | None = None,
    seed: int | None = None,
    algorithm_id: str = "basic_agent",
    algorithm_version: str | None = None,
    run_id: str | None = None,
    scene_version: str | None = None,
    carla_version: str = "0.9.16",
    fixed_delta_seconds: float = 0.05,
    actor_control_mode: str = "mixed",
    actor_style: str = "normal",
) -> Path:
    scenario_ir = json.loads(scenario_ir_path.read_text(encoding="utf-8"))
    config = build_carla_run_config(
        scenario_ir,
        carla_map=carla_map,
        reconstruction_package_path=reconstruction_package,
        weather=weather,
        odd_id=odd_id,
        seed=seed,
        algorithm_id=algorithm_id,
        algorithm_version=algorithm_version,
        run_id=run_id,
        scene_version=scene_version,
        carla_version=carla_version,
        fixed_delta_seconds=fixed_delta_seconds,
        actor_control_mode=actor_control_mode,
        actor_style=actor_style,
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
    parser.add_argument("--weather", default=None, help="CARLA WeatherParameters preset name.")
    parser.add_argument("--odd-id", default=None)
    parser.add_argument("--seed", default=None, type=int)
    parser.add_argument("--algorithm-id", default="basic_agent")
    parser.add_argument("--algorithm-version", default=None)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--scene-version", default=None)
    parser.add_argument("--carla-version", default="0.9.16")
    parser.add_argument("--fixed-delta-seconds", default=0.05, type=float)
    parser.add_argument(
        "--actor-control-mode",
        choices=("replay", "scripted", "traffic_manager", "mixed"),
        default="mixed",
    )
    parser.add_argument(
        "--actor-style",
        choices=("cautious", "normal", "aggressive"),
        default="normal",
    )
    args = parser.parse_args(argv)

    output = write_carla_run_config(
        Path(args.scenario_ir),
        Path(args.output),
        carla_map=args.carla_map,
        reconstruction_package=args.reconstruction_package,
        weather=args.weather,
        odd_id=args.odd_id,
        seed=args.seed,
        algorithm_id=args.algorithm_id,
        algorithm_version=args.algorithm_version,
        run_id=args.run_id,
        scene_version=args.scene_version,
        carla_version=args.carla_version,
        fixed_delta_seconds=args.fixed_delta_seconds,
        actor_control_mode=args.actor_control_mode,
        actor_style=args.actor_style,
    )
    print(json.dumps({"carla_run_config": str(output)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

