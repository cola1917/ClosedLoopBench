from __future__ import annotations

import argparse
import importlib
import json
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runners.run_carla_basic_agent import build_basic_agent_plan, run_basic_agent
from runners.validate_multimodal_closed_loop import (
    MultimodalClosedLoopError,
    validate_multimodal_closed_loop_result,
)


class CarlaAcceptanceError(RuntimeError):
    """Raised when any one of the three real CARLA runs lacks required evidence."""


def run_acceptance_triplicate(
    run_config: dict[str, Any],
    output_root: Path,
    *,
    host: str = "127.0.0.1",
    port: int = 2000,
    max_ticks: int = 600,
    require_multimodal: bool = False,
    opendrive_path: str | None = None,
    ego_driver: str = "basic_agent",
    sensor_frame_handler_factory: Callable[
        [dict[str, Any], Path], Callable[[dict[str, Any]], dict[str, Any]]
    ]
    | None = None,
    execute: Callable[[dict[str, Any]], dict[str, Any]] = run_basic_agent,
) -> dict[str, Any]:
    if require_multimodal and sensor_frame_handler_factory is None and execute is run_basic_agent:
        raise CarlaAcceptanceError(
            "--require-multimodal needs a real sensor frame handler factory"
        )
    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    base_run_id = str(run_config.get("run_id") or "basic-agent-acceptance")
    results = []
    for attempt in range(1, 4):
        run_id = f"{base_run_id}-attempt-{attempt:02d}"
        run_dir = output_root / run_id
        run_dir.mkdir(parents=True, exist_ok=False)
        config = deepcopy(run_config)
        config["run_id"] = run_id
        experiment = dict(config.get("experiment") or {})
        experiment["run_id"] = run_id
        config["experiment"] = experiment
        report_path = run_dir / "closed_loop_report.json"
        plan = build_basic_agent_plan(
            config,
            host=host,
            port=port,
            max_ticks=max_ticks,
            synchronous=True,
            output=str(report_path),
            acceptance_evidence=True,
            multimodal_sensor_required=require_multimodal,
            opendrive_path=opendrive_path,
            ego_driver=ego_driver,
        )
        (run_dir / "basic_agent_plan.json").write_text(
            json.dumps(plan, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        if sensor_frame_handler_factory is None:
            result = execute(plan)
        else:
            handler = sensor_frame_handler_factory(config, run_dir)
            if not callable(handler):
                raise CarlaAcceptanceError(
                    f"attempt {attempt} sensor frame handler factory returned a non-callable"
                )
            try:
                result = execute(plan, sensor_frame_handler=handler)
            finally:
                close_handler = getattr(handler, "close", None)
                if close_handler is None:
                    if require_multimodal:
                        raise CarlaAcceptanceError(
                            f"attempt {attempt} real sensor handler has no close()"
                        )
                elif not callable(close_handler):
                    raise CarlaAcceptanceError(
                        f"attempt {attempt} sensor handler close is not callable"
                    )
                else:
                    try:
                        close_handler()
                    except Exception as exc:
                        raise CarlaAcceptanceError(
                            f"attempt {attempt} sensor handler cleanup failed: {exc}"
                        ) from exc
            result["sensor_handler_cleanup_succeeded"] = callable(
                getattr(handler, "close", None)
            )
        (run_dir / "runtime_result.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        results.append(result)

    summary = validate_acceptance_runs(results, require_multimodal=require_multimodal)
    aggregate = {
        "schema_version": "carla_acceptance_triplicate.v1",
        "run_count": 3,
        "scene_id": run_config.get("scenario_id"),
        "status": "passed",
        "runs": summary,
    }
    (output_root / "acceptance_triplicate.json").write_text(
        json.dumps(aggregate, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return aggregate


def validate_acceptance_runs(
    results: list[dict[str, Any]],
    *,
    require_multimodal: bool = False,
) -> list[dict[str, Any]]:
    if len(results) != 3:
        raise CarlaAcceptanceError("exactly three consecutive results are required")
    validated = []
    for index, result in enumerate(results, 1):
        if result.get("status") not in {"ego_closed_loop", "interactive_closed_loop"}:
            raise CarlaAcceptanceError(
                f"attempt {index} failed: {result.get('reason') or result.get('detail') or result.get('status')}"
            )
        report = result.get("report") or {}
        runtime = report.get("runtime") or {}
        summary = report.get("summary") or {}
        if not runtime.get("collision_sensor_available"):
            raise CarlaAcceptanceError(f"attempt {index} lacks collision sensor evidence")
        if not result.get("cleanup_succeeded"):
            raise CarlaAcceptanceError(f"attempt {index} cleanup did not succeed")
        if float(summary.get("route_progress") or 0.0) < 0.95:
            raise CarlaAcceptanceError(f"attempt {index} route progress is below 0.95")
        if int(runtime.get("frame_trace_count") or 0) < 1:
            raise CarlaAcceptanceError(f"attempt {index} has no frame trace")
        if result["status"] == "interactive_closed_loop" and not runtime.get(
            "actor_physical_response"
        ):
            raise CarlaAcceptanceError(
                f"attempt {index} claims interactive closure without physical Actor evidence"
            )
        multimodal_evidence = None
        if require_multimodal:
            if not result.get("sensor_handler_cleanup_succeeded"):
                raise CarlaAcceptanceError(
                    f"attempt {index} NuRec sensor handler cleanup did not succeed"
                )
            try:
                multimodal_evidence = validate_multimodal_closed_loop_result(result)
            except MultimodalClosedLoopError as exc:
                raise CarlaAcceptanceError(
                    f"attempt {index} lacks multimodal closed-loop evidence: {exc}"
                ) from exc
        validated.append(
            {
                "attempt": index,
                "status": result["status"],
                "route_progress": summary["route_progress"],
                "collision_count": summary.get("collision_count"),
                "frame_trace_count": runtime["frame_trace_count"],
                "actor_physical_response": runtime.get("actor_physical_response") or {},
                "cleanup_succeeded": True,
                "sensor_handler_cleanup_succeeded": result.get(
                    "sensor_handler_cleanup_succeeded"
                ),
                "multimodal_closed_loop": multimodal_evidence,
            }
        )
    return validated


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the same strict CARLA BasicAgent acceptance case three consecutive times."
    )
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--max-ticks", default=600, type=int)
    parser.add_argument("--opendrive", type=Path)
    parser.add_argument("--ego-driver", default="basic_agent")
    parser.add_argument(
        "--sensor-handler-factory",
        help="Python module:callable returning handler(run_config, attempt_dir).",
    )
    parser.add_argument(
        "--require-multimodal",
        action="store_true",
        help="Require actor-bound NuRec RGB/LiDAR evidence on every CARLA frame.",
    )
    args = parser.parse_args(argv)
    config = json.loads(args.run_config.read_text(encoding="utf-8"))
    try:
        handler_factory = (
            _load_callable(args.sensor_handler_factory)
            if args.sensor_handler_factory
            else None
        )
        result = run_acceptance_triplicate(
            config,
            args.output_root,
            host=args.host,
            port=args.port,
            max_ticks=args.max_ticks,
            require_multimodal=args.require_multimodal,
            opendrive_path=str(args.opendrive) if args.opendrive else None,
            ego_driver=args.ego_driver,
            sensor_frame_handler_factory=handler_factory,
        )
    except (CarlaAcceptanceError, FileExistsError, ImportError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


def _load_callable(spec: str) -> Callable[..., Any]:
    if ":" not in spec:
        raise CarlaAcceptanceError("sensor handler factory must use module:callable")
    module_name, attribute = spec.split(":", 1)
    value = getattr(importlib.import_module(module_name), attribute, None)
    if not callable(value):
        raise CarlaAcceptanceError(f"sensor handler factory is not callable: {spec}")
    return value


if __name__ == "__main__":
    raise SystemExit(main())
