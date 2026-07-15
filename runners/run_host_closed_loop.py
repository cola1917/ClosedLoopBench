from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_exchange import consume_scene_version
from adapters.shared_message_store import validate_artifact_on_disk
from adapters.shared_protocol_validation import validate_shared_document
from runners.build_carla_config import write_carla_run_config
from runners.run_carla_basic_agent import run_basic_agent, write_basic_agent_plan
from runtime.carla_probe import probe_carla
from runtime.host_orchestration import HostRuntimeError, validate_host_runtime


def prepare_host_run(
    *,
    exchange_root: Path,
    scene_id: str,
    version: str | None,
    run_dir: Path,
    carla_map: str = "Town04",
    carla_host: str = "127.0.0.1",
    carla_port: int = 2000,
    expected_carla_version: str = "0.9.16",
    algorithm_id: str = "basic_agent",
    algorithm_version: str | None = None,
    ego_driver: str = "basic_agent",
    control_topic: str = "/carla/ego_vehicle/vehicle_control_cmd",
    control_timeout_sec: float = 0.5,
    max_ticks: int = 600,
    algorithm_ready_file: Path | None = None,
    run_id: str | None = None,
    weather: str | None = None,
    odd_id: str | None = None,
    seed: int | None = None,
    fixed_delta_seconds: float = 0.05,
    actor_control_mode: str = "mixed",
    actor_style: str = "normal",
) -> dict[str, Any]:
    if ego_driver not in {"basic_agent", "ros2_control"}:
        raise ValueError(f"unsupported ego driver: {ego_driver}")
    if ego_driver == "basic_agent" and algorithm_id != "basic_agent":
        raise ValueError("basic_agent driver requires algorithm_id='basic_agent'")

    resolved = consume_scene_version(exchange_root, scene_id, version)
    run_dir.mkdir(parents=True, exist_ok=True)
    config_path = run_dir / "carla_run_config.json"
    ego_plan_path = run_dir / "ego_run_plan.json"
    host_plan_path = run_dir / "host_run_plan.json"

    reconstruction = (
        resolved["scene_package"]
        if resolved.get("nurec_usdz") or resolved.get("nurec_checkpoint")
        else None
    )
    write_carla_run_config(
        Path(resolved["scene_ir"]),
        config_path,
        carla_map=carla_map,
        reconstruction_package=reconstruction,
        algorithm_id=algorithm_id,
        algorithm_version=algorithm_version,
        run_id=run_id,
        scene_version=resolved["version"],
        carla_version=expected_carla_version,
        fixed_delta_seconds=fixed_delta_seconds,
        weather=weather,
        odd_id=odd_id,
        seed=seed,
        actor_control_mode=actor_control_mode,
        actor_style=actor_style,
    )
    write_basic_agent_plan(
        config_path,
        ego_plan_path,
        host=carla_host,
        port=carla_port,
        max_ticks=max_ticks,
        ego_driver=ego_driver,
        control_topic=control_topic,
        control_timeout_sec=control_timeout_sec,
        snap_to_map=True,
    )

    ready_file = algorithm_ready_file or (
        exchange_root / "runtime" / "ego-algorithm.ready.json"
    )
    host_plan = {
        "schema_version": "host_closed_loop_plan.v0",
        "run_id": run_id,
        "scene": {
            "id": resolved["scene_id"],
            "version": resolved["version"],
            "package": resolved["scene_package"],
        },
        "carla": {
            "deployment": "native_host",
            "host": carla_host,
            "port": int(carla_port),
            "timeout_sec": 3.0,
            "expected_version": expected_carla_version,
            "map": carla_map,
            "weather": weather,
            "odd_id": odd_id,
            "seed": seed,
            "fixed_delta_seconds": fixed_delta_seconds,
        },
        "actor_control": {"mode": actor_control_mode, "style": actor_style},
        "algorithm": {
            "id": algorithm_id,
            "version": algorithm_version,
            "driver": ego_driver,
            "control_topic": control_topic,
            "ready_file": str(ready_file.resolve()) if ego_driver == "ros2_control" else None,
        },
        "artifacts": {
            "run_config": str(config_path.resolve()),
            "ego_plan": str(ego_plan_path.resolve()),
            "result": str((run_dir / "host_run_result.json").resolve()),
            "closed_loop_report": str((run_dir / "closed_loop_report.json").resolve()),
            "host_plan": str(host_plan_path.resolve()),
        },
    }
    host_plan_path.write_text(
        json.dumps(host_plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return host_plan


def prepare_host_run_from_evaluation_request(
    request: dict[str, Any],
    *,
    exchange_root: Path,
    carla_map: str = "Town04",
    carla_host: str = "127.0.0.1",
    carla_port: int = 2000,
    control_topic: str = "/carla/ego_vehicle/vehicle_control_cmd",
    control_timeout_sec: float = 0.5,
    algorithm_ready_file: Path | None = None,
) -> dict[str, Any]:
    validate_shared_document(request)
    if request.get("schema_version") != "evaluation_run_request.v1":
        raise ValueError("request must use evaluation_run_request.v1")
    payload = request["payload"]
    scene_ref = payload["scene_package"]
    validate_artifact_on_disk(exchange_root, scene_ref)
    expected_scene_path = (
        f"scenes/{payload['scene_id']}/{payload['scene_version']}/scene_package.json"
    )
    if scene_ref["path"] != expected_scene_path:
        raise ValueError(
            f"scene_package path must be canonical: expected {expected_scene_path!r}"
        )
    checkpoint = (payload.get("algorithm") or {}).get("checkpoint")
    if checkpoint is not None:
        validate_artifact_on_disk(exchange_root, checkpoint)

    output_prefix = Path(payload["output_prefix"])
    run_dir = (exchange_root.resolve() / output_prefix).resolve()
    try:
        run_dir.relative_to(exchange_root.resolve())
    except ValueError as exc:
        raise ValueError("evaluation output_prefix escapes exchange root") from exc
    driver = payload["algorithm"]["driver"]
    if driver == "external_plugin":
        raise ValueError("external_plugin is not bound to the host runner; use ros2")
    ego_driver = "basic_agent" if driver == "basic_agent" else "ros2_control"
    delta = float(payload["simulator"]["fixed_delta_seconds"])
    return prepare_host_run(
        exchange_root=exchange_root,
        scene_id=payload["scene_id"],
        version=payload["scene_version"],
        run_dir=run_dir,
        carla_map=carla_map,
        carla_host=carla_host,
        carla_port=carla_port,
        expected_carla_version=payload["simulator"]["version"],
        algorithm_id=payload["algorithm"]["algorithm_id"],
        algorithm_version=payload["algorithm"]["algorithm_version"],
        ego_driver=ego_driver,
        control_topic=control_topic,
        control_timeout_sec=control_timeout_sec,
        max_ticks=int(math.ceil(float(payload["timeout_sec"]) / delta)),
        algorithm_ready_file=algorithm_ready_file,
        run_id=payload["run_id"],
        weather=(payload.get("odd") or {}).get("weather"),
        odd_id=payload["odd"]["odd_id"],
        seed=payload["seed"],
        fixed_delta_seconds=delta,
        actor_control_mode=payload["actor_control"]["mode"],
        actor_style=payload["actor_control"].get("style", "normal"),
    )


def execute_host_run(host_plan: dict[str, Any]) -> dict[str, Any]:
    readiness = validate_host_runtime(host_plan, probe=probe_carla)
    ego_plan_path = Path(host_plan["artifacts"]["ego_plan"])
    ego_plan = json.loads(ego_plan_path.read_text(encoding="utf-8"))
    result = run_basic_agent(ego_plan)
    payload = {"readiness": readiness, "result": result}
    Path(host_plan["artifacts"]["result"]).write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return payload


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Prepare or execute ClosedLoopBench against native host CARLA."
    )
    parser.add_argument("--exchange-root", required=True)
    parser.add_argument("--evaluation-request", default=None)
    parser.add_argument("--scene-id", default=None)
    parser.add_argument("--version", default=None)
    parser.add_argument("--run-dir", default=None)
    parser.add_argument("--carla-map", default="Town04")
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument("--carla-version", default="0.9.16")
    parser.add_argument("--algorithm-id", default="basic_agent")
    parser.add_argument("--algorithm-version", default=None)
    parser.add_argument("--ego-driver", choices=("basic_agent", "ros2_control"), default="basic_agent")
    parser.add_argument("--control-topic", default="/carla/ego_vehicle/vehicle_control_cmd")
    parser.add_argument("--control-timeout-sec", type=float, default=0.5)
    parser.add_argument("--max-ticks", type=int, default=600)
    parser.add_argument("--algorithm-ready-file", default=None)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)

    try:
        ready_file = Path(args.algorithm_ready_file) if args.algorithm_ready_file else None
        if args.evaluation_request:
            plan = prepare_host_run_from_evaluation_request(
                json.loads(Path(args.evaluation_request).read_text(encoding="utf-8")),
                exchange_root=Path(args.exchange_root),
                carla_map=args.carla_map,
                carla_host=args.carla_host,
                carla_port=args.carla_port,
                control_topic=args.control_topic,
                control_timeout_sec=args.control_timeout_sec,
                algorithm_ready_file=ready_file,
            )
        else:
            if not args.scene_id or not args.run_dir:
                raise ValueError("--scene-id and --run-dir are required without --evaluation-request")
            plan = prepare_host_run(
                exchange_root=Path(args.exchange_root),
                scene_id=args.scene_id,
                version=args.version,
                run_dir=Path(args.run_dir),
                carla_map=args.carla_map,
                carla_host=args.carla_host,
                carla_port=args.carla_port,
                expected_carla_version=args.carla_version,
                algorithm_id=args.algorithm_id,
                algorithm_version=args.algorithm_version,
                ego_driver=args.ego_driver,
                control_topic=args.control_topic,
                control_timeout_sec=args.control_timeout_sec,
                max_ticks=args.max_ticks,
                algorithm_ready_file=ready_file,
            )
        payload = execute_host_run(plan) if args.execute else {"status": "planned", "plan": plan}
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        if not args.execute:
            return 0
        return 0 if payload["result"].get("status") in {
            "completed", "ego_closed_loop", "interactive_closed_loop"
        } else 2
    except (HostRuntimeError, OSError, ValueError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
