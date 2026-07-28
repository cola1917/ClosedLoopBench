"""Multi-tick NuRec closed loop for scene-0061 (M2).

Attaches r22's proven NuRec ``sensor_frame_handler`` to the multi-tick loop, so
the 6 NuRec RGB cameras (+ LiDAR) render each physical tick as the ego drives
the reconstructed scene. The camera JPEGs materialize under
``<output_dir>/algorithm_sensor_payloads/frame_XXXXXXXX/camera_*.jpg`` (keyed by
the CARLA world frame), ready for the multimodal video renderer.

Difference from ``scene0061_live_tick.py`` (which pinned NuRec to one tick):
``max_ticks`` is a real horizon, ``acceptance_evidence`` is off, and
``multimodal_sensor_required`` defaults False so a single bad NuRec frame does
not abort the whole drive (materialization happens inside ``dispatch_frame`` and
still occurs on best-effort frames). This is the M2 vertical slice: prove NuRec
renders across many consecutive ticks of a live closed loop.
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


def run_nurec_closed_loop(
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
    require_multimodal: bool,
    despawn_exhausted_actors: bool = False,
    nurec_concurrency: int = 1,
    nurec_extra_targets: list[str] | None = None,
    nurec_max_attempts: int = 1,
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

    # Build the NuRec handler bound to THIS run dir so camera JPEGs materialize
    # under output_dir/algorithm_sensor_payloads/. Opens a live gRPC channel to
    # the NRE service and issues get_available_cameras at construction, so NRE
    # must be up. Imported lazily so the module stays importable off-host.
    from adapters.nurec_260_client import build_nurec_260_handler  # noqa: E402

    handler = build_nurec_260_handler(
        run_config,
        output_dir,
        concurrency=max(1, int(nurec_concurrency)),
        extra_targets=list(nurec_extra_targets or []),
        max_attempts=max(1, int(nurec_max_attempts)),
        # A closed-loop drive legitimately outlives the recorded scan
        # coverage; render those frames at their logical window and record
        # status=out_of_native_scan_range instead of aborting the run.
        native_scan_alignment_required=False,
    )

    plan = build_basic_agent_plan(
        run_config,
        host=host,
        port=port,
        max_ticks=max_ticks,
        synchronous=True,
        output=str(report_path),
        acceptance_evidence=False,
        multimodal_sensor_required=require_multimodal,
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

    # run_basic_agent closes the handler in its finally block.
    result = run_basic_agent(plan, sensor_frame_handler=handler)

    report = result.get("report") or {}
    runtime = report.get("runtime") or {}
    summary_block = result.get("summary") or {}
    multimodal = runtime.get("multimodal_sensor") or {}
    payload_root = output_dir / "algorithm_sensor_payloads"
    materialized_frames = (
        sorted(p.name for p in payload_root.glob("frame_*")) if payload_root.is_dir() else []
    )
    summary = {
        "status": result.get("status"),
        "reason": result.get("reason"),
        "detail": result.get("detail"),
        "ticks": summary_block.get("ticks"),
        "route_progress": summary_block.get("route_progress"),
        "collision_count": summary_block.get("collision_count"),
        "termination_reason": runtime.get("termination_reason"),
        "frame_trace_count": len(result.get("frame_trace") or []),
        "multimodal_sensor": multimodal,
        "nurec_frames_materialized": len(materialized_frames),
        "first_frame_dir": materialized_frames[0] if materialized_frames else None,
        "last_frame_dir": materialized_frames[-1] if materialized_frames else None,
        "cleanup_succeeded": result.get("cleanup_succeeded"),
        "report_path": str(report_path),
        "output_dir": str(output_dir),
    }
    (output_dir / "nurec_run_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run a multi-tick NuRec closed loop on a scene-0061 config. M2: prove "
            "6 NuRec cameras render across many consecutive ticks of a live drive."
        )
    )
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--opendrive", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--max-ticks", type=int, default=60)
    parser.add_argument("--ego-driver", default="basic_agent")
    parser.add_argument("--carla-python-api", type=Path, default=None)
    parser.add_argument(
        "--require-multimodal",
        action="store_true",
        help="fail-closed if any tick's NuRec evidence is not passed (default: best-effort)",
    )
    parser.add_argument(
        "--despawn-exhausted-actors",
        action="store_true",
        help=(
            "destroy actors once their recorded reference trajectory ends "
            "(nuScenes annotation range left) instead of freezing them in place"
        ),
    )
    parser.add_argument(
        "--nurec-concurrency",
        type=int,
        default=1,
        help=(
            "per-frame NuRec sensor request fan-out (7 overlaps all 6 RGB + "
            "LiDAR requests; 1 keeps the historical serial dispatch)"
        ),
    )
    parser.add_argument(
        "--nurec-extra-targets",
        default="",
        help=(
            "comma-separated extra NRE SensorsimService targets (identical "
            "instances of the same scene) to round-robin render RPCs across"
        ),
    )
    parser.add_argument(
        "--nurec-max-attempts",
        type=int,
        default=1,
        help="per-request retry budget for transient NRE RPC failures",
    )
    args = parser.parse_args(argv)

    try:
        summary = run_nurec_closed_loop(
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
            require_multimodal=args.require_multimodal,
            despawn_exhausted_actors=args.despawn_exhausted_actors,
            nurec_concurrency=args.nurec_concurrency,
            nurec_extra_targets=[
                item.strip()
                for item in str(args.nurec_extra_targets).split(",")
                if item.strip()
            ],
            nurec_max_attempts=args.nurec_max_attempts,
        )
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "failed", "detail": f"{type(exc).__name__}: {exc}"}, ensure_ascii=False))
        return 2

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["status"] in {"ego_closed_loop", "interactive_closed_loop"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
