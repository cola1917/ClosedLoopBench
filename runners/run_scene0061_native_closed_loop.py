"""Native (no-NuRec) multi-tick CARLA closed loop for scene-0061.

This is the M1 vertical slice of the decoupled plan: prove that the closed loop
actually *closes* — ego drives the OpenDRIVE route for many physical ticks and
``route_progress`` grows toward 1.0 — using CARLA-native physics only, with no
NuRec multimodal sensor handler on the critical path.

It deliberately differs from ``scene0061_live_tick.py`` in exactly the three
ways that kept that runner pinned to a one-tick smoke:

- ``max_ticks`` is a real horizon (default 600), not 1;
- ``acceptance_evidence`` is off, so the loop runs the whole route and reports
  whatever progress it reaches instead of failing closed below 0.95;
- no ``sensor_frame_handler`` is attached and ``multimodal_sensor_required`` is
  False, so the optional NuRec RGB/LiDAR path never gates the drive.

The NuRec photorealism path (r22's G0 evidence) is intentionally preserved and
re-attached later as an optional variant; it is not deleted, only lifted off
the critical path. See docs and the decoupled convergence plan.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runners.run_carla_basic_agent import (  # noqa: E402
    build_basic_agent_plan,
    build_run_config_provenance,
    run_basic_agent,
)


def run_native_closed_loop(
    *,
    config_path: Path,
    opendrive_path: Path,
    output_dir: Path,
    run_id: str | None,
    host: str,
    port: int,
    timeout_sec: float,
    max_ticks: int,
    ego_driver: str,
    carla_python_api: Path | None,
    follow_ego: bool,
    despawn_exhausted_actors: bool = False,
) -> dict:
    config_path = config_path.expanduser().resolve()
    xodr = opendrive_path.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"run config does not exist: {config_path}")
    if not xodr.is_file():
        raise FileNotFoundError(f"OpenDRIVE file does not exist: {xodr}")
    if output_dir.exists():
        raise FileExistsError(f"output directory already exists: {output_dir}")
    output_dir.mkdir(parents=True)
    report_path = output_dir / "closed_loop_report.json"

    run_config = json.loads(config_path.read_text(encoding="utf-8"))
    if run_id:
        run_config["run_id"] = run_id

    plan = build_basic_agent_plan(
        run_config,
        host=host,
        port=port,
        max_ticks=max_ticks,
        synchronous=True,
        output=str(report_path),
        follow_ego=follow_ego,
        acceptance_evidence=False,
        multimodal_sensor_required=False,
        opendrive_path=str(xodr),
        ego_driver=ego_driver,
        actor_autopilot=False,
        timeout_sec=timeout_sec,
        run_config_provenance=build_run_config_provenance(config_path),
    )
    if ego_driver == "basic_agent" and carla_python_api is not None:
        plan.setdefault("runtime", {})["carla_python_api_path"] = str(
            carla_python_api.expanduser().resolve()
        )
    if despawn_exhausted_actors:
        # Faithful replay semantics: an actor whose nuScenes annotation ended
        # leaves the world instead of freezing into a phantom roadblock.
        plan["despawn_actors_on_reference_exhausted"] = True

    # No sensor_frame_handler => native CARLA sensors / physics only.
    result = run_basic_agent(plan)

    report = result.get("report") or {}
    runtime = report.get("runtime") or {}
    summary_block = result.get("summary") or {}
    summary = {
        "status": result.get("status"),
        "reason": result.get("reason"),
        "detail": result.get("detail"),
        "ticks": summary_block.get("ticks"),
        "route_progress": summary_block.get("route_progress"),
        "collision_count": summary_block.get("collision_count"),
        "termination_reason": runtime.get("termination_reason"),
        "frame_trace_count": len(result.get("frame_trace") or []),
        "cleanup_succeeded": result.get("cleanup_succeeded"),
        "report_path": str(report_path),
        "output_dir": str(output_dir),
    }
    (output_dir / "native_run_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run a NATIVE (no-NuRec) multi-tick CARLA closed loop on a scene-0061 "
            "run config. M1 existence proof: the loop closes and route_progress grows."
        )
    )
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--opendrive", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--max-ticks", type=int, default=600)
    parser.add_argument("--ego-driver", default="basic_agent")
    parser.add_argument("--carla-python-api", type=Path, default=None)
    parser.add_argument("--follow-ego", action="store_true")
    parser.add_argument(
        "--despawn-exhausted-actors",
        action="store_true",
        help=(
            "destroy actors once their recorded reference trajectory ends "
            "(nuScenes annotation range left) instead of freezing them in place"
        ),
    )
    args = parser.parse_args(argv)

    try:
        summary = run_native_closed_loop(
            config_path=args.config,
            opendrive_path=args.opendrive,
            output_dir=args.output_dir,
            run_id=args.run_id,
            host=args.host,
            port=args.port,
            timeout_sec=args.timeout_sec,
            max_ticks=args.max_ticks,
            ego_driver=args.ego_driver,
            carla_python_api=args.carla_python_api,
            follow_ego=args.follow_ego,
            despawn_exhausted_actors=args.despawn_exhausted_actors,
        )
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "failed", "detail": f"{type(exc).__name__}: {exc}"}, ensure_ascii=False))
        return 2

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["status"] in {"ego_closed_loop", "interactive_closed_loop"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
