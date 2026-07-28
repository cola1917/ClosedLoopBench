from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_object_registry import (
    attach_dynamic_replay_to_carla_run,
    attach_static_obstacles_to_carla_run,
    registry_sha256,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Derive an M6 CARLA run config with immutable static collision proxies."
    )
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--scene-object-registry", required=True, type=Path)
    parser.add_argument(
        "--scenario-ir",
        type=Path,
        help="Required with --include-dynamic-replay to materialize all dynamic source tracks.",
    )
    parser.add_argument("--include-dynamic-replay", action="store_true")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        registry_bytes = args.scene_object_registry.read_bytes()
        registry = json.loads(registry_bytes)
        run_config = json.loads(args.run_config.read_text(encoding="utf-8"))
        derived = attach_static_obstacles_to_carla_run(
            run_config,
            registry,
            registry_path=str(args.scene_object_registry.resolve()),
            registry_sha256=registry_sha256(registry_bytes),
        )
        if args.include_dynamic_replay:
            if args.scenario_ir is None:
                raise ValueError("--include-dynamic-replay requires --scenario-ir")
            derived = attach_dynamic_replay_to_carla_run(
                derived,
                registry,
                json.loads(args.scenario_ir.read_text(encoding="utf-8")),
            )
        if args.output.exists():
            raise ValueError(f"refusing to overwrite derived M6 config: {args.output}")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(derived, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(
        json.dumps(
            {
                "output": str(args.output),
                "static_obstacle_count": len(derived["static_obstacles"]),
                "dynamic_replay_count": len(derived.get("actors") or []),
                "registry_sha256": derived["scene_object_registry"]["sha256"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
