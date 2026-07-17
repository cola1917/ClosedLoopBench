from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

from agents.algorithm_backend import (
    AlgorithmBackendError,
    load_backend,
    run_backend,
    validate_runtime_paths,
)


def build_runtime_config(environment: dict[str, str] | None = None) -> dict[str, Any]:
    env = os.environ if environment is None else environment
    required = {
        "plugin": "ALGORITHM_PLUGIN",
        "repo_path": "ALGORITHM_REPO_PATH",
        "checkpoint_path": "ALGORITHM_CHECKPOINT_PATH",
        "shared_data_path": "SIM_DATA_PATH",
    }
    missing = [variable for variable in required.values() if not env.get(variable)]
    if missing:
        raise AlgorithmBackendError("missing required environment: " + ", ".join(sorted(missing)))
    config: dict[str, Any] = {key: env[variable] for key, variable in required.items()}
    config.update(
        {
            "algorithm_id": env.get("ALGORITHM_ID", "external"),
            "ros_domain_id": env.get("ROS_DOMAIN_ID", "0"),
            "control_topic": env.get(
                "CONTROL_TOPIC", "/carla/ego_vehicle/vehicle_control_cmd"
            ),
            "observation_topic": env.get(
                "OBSERVATION_TOPIC", "/closed_loop/ego/observation"
            ),
            "carla_host": env.get("CARLA_HOST", "127.0.0.1"),
            "carla_port": int(env.get("CARLA_PORT", "2000")),
        }
    )
    config.update(
        validate_runtime_paths(
            repo_path=config["repo_path"],
            checkpoint_path=config["checkpoint_path"],
            shared_data_path=config["shared_data_path"],
        )
    )
    return config


def preflight(environment: dict[str, str] | None = None) -> tuple[dict[str, Any], Any]:
    config = build_runtime_config(environment)
    backend = load_backend(config["plugin"], config)
    return config, backend


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Start or validate an external ego algorithm plugin.")
    parser.add_argument("command", choices=("preflight", "run", "health"))
    parser.add_argument("--ready-file", default=os.environ.get("ALGORITHM_READY_FILE", "/tmp/algorithm.ready"))
    args = parser.parse_args(argv)
    ready_file = Path(args.ready_file)

    try:
        if args.command == "health":
            if not ready_file.is_file():
                raise AlgorithmBackendError(f"algorithm ready file is absent: {ready_file}")
            try:
                ready = json.loads(ready_file.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise AlgorithmBackendError(f"invalid algorithm ready file: {exc}") from exc
            if ready.get("status") != "ready":
                raise AlgorithmBackendError("algorithm ready file does not report ready")
            ready["heartbeat_unix"] = time.time()
            ready_file.write_text(json.dumps(ready), encoding="utf-8")
            print(json.dumps({"status": "healthy", "ready_file": str(ready_file)}))
            return 0

        config, backend = preflight()
        summary = {
            "status": "ready",
            "algorithm_id": config["algorithm_id"],
            "plugin": config["plugin"],
            "repo_path": config["repo_path"],
            "checkpoint_path": config["checkpoint_path"],
            "shared_data_path": config["shared_data_path"],
            "carla_endpoint": f"{config['carla_host']}:{config['carla_port']}",
            "heartbeat_unix": time.time(),
        }
        if args.command == "preflight":
            print(json.dumps(summary, indent=2))
            return 0

        lifecycle = getattr(backend, "run", None) or getattr(backend, "run_forever", None)
        if not callable(lifecycle):
            raise AlgorithmBackendError(
                "backend must provide blocking run() or run_forever(); no fake inference loop is supplied"
            )
        ready_file.parent.mkdir(parents=True, exist_ok=True)
        ready_file.write_text(json.dumps(summary), encoding="utf-8")
        print(json.dumps(summary), flush=True)
        run_backend(backend)
        return 0
    except (AlgorithmBackendError, OSError, ValueError) as exc:
        if ready_file.exists():
            ready_file.unlink()
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2
    finally:
        if args.command == "run" and ready_file.exists():
            ready_file.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
