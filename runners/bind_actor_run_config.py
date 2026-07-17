from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.actor_binding import ActorBindingError, bind_carla_run_config


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Attach a verified Actor Binding Set to a CARLA run config."
    )
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--actor-bindings", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--allow-not-ready",
        action="store_true",
        help="Allow diagnostic configs whose binding readiness is not ready.",
    )
    args = parser.parse_args(argv)

    run_config = json.loads(args.run_config.read_text(encoding="utf-8"))
    binding_set = json.loads(args.actor_bindings.read_text(encoding="utf-8"))
    try:
        bound = bind_carla_run_config(
            run_config,
            binding_set,
            require_ready=not args.allow_not_ready,
        )
    except ActorBindingError as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(bound, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({"status": "ready", "output": str(args.output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
