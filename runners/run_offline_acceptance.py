from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]

CORE_MVP_GATES = {
    "shared_protocol": (
        "tests.test_shared_protocol_schemas",
        "tests.test_shared_protocol_validation",
        "tests.test_shared_protocol_flow",
        "tests.test_shared_message_store",
        "tests.test_scene_exchange",
    ),
    "scene_compilation": (
        "tests.test_nuscenes_scene",
        "tests.test_nuscenes_map_to_opendrive",
        "tests.test_ir_to_openscenario",
        "tests.test_build_nuscenes_exchange",
        "tests.test_nuscenes_exchange_e2e",
    ),
    "closed_loop_contract": (
        "tests.test_basic_agent_runtime_loop",
        "tests.test_host_orchestration",
        "tests.test_ros2_control_driver",
        "tests.test_ego_observation",
        "tests.test_ros2_tcp_bridge",
        "tests.test_runtime_metric_sources",
    ),
    "experiment_and_report": (
        "tests.test_experiment_matrix",
        "tests.test_evaluation_protocol",
        "tests.test_report_comparison",
        "tests.test_closed_loop_report",
        "tests.test_evaluation_criteria",
        "tests.test_metric_collector",
    ),
}

EXTENDED_OFFLINE_GATES = {
    "scene0061_counterfactual_evaluation": (
        "tests.test_scene0061_counterfactual_matrix",
        "tests.test_extended_metrics",
        "tests.test_counterfactual_suite",
    ),
    "algorithm_plugin_conformance": (
        "tests.test_algorithm_plugin_contract",
        "tests.test_reference_pure_pursuit_plugin",
        "tests.test_algorithm_plugin_replay",
        "tests.test_algorithm_plugin_conformance",
        "tests.test_model_plugin_wrappers",
    ),
    "render_quality_and_video": (
        "tests.test_render_quality",
        "tests.test_scene0061_video_manifest",
    ),
}

# Keep the public name used by existing callers, but make the default scope the
# smallest independently useful product surface.
DEFAULT_GATES = CORE_MVP_GATES


def run_offline_acceptance(
    *,
    python_executable: str = sys.executable,
    runner: Callable[..., Any] = subprocess.run,
    full_suite: bool = False,
    extended: bool = False,
) -> dict[str, Any]:
    if full_suite:
        gates = {"full_suite": ("discover", "-s", "tests")}
        scope = "full_suite"
    elif extended:
        gates = {**CORE_MVP_GATES, **EXTENDED_OFFLINE_GATES}
        scope = "extended_offline"
    else:
        gates = CORE_MVP_GATES
        scope = "core_mvp"
    results = []
    for name, targets in gates.items():
        command = [python_executable, "-m", "unittest", *targets, "-q"]
        started = time.monotonic()
        completed = runner(
            command,
            cwd=str(PROJECT_ROOT),
            text=True,
            capture_output=True,
            check=False,
        )
        results.append(
            {
                "gate": name,
                "status": "passed" if completed.returncode == 0 else "failed",
                "returncode": int(completed.returncode),
                "duration_sec": round(time.monotonic() - started, 3),
                "command": command,
                "stdout_tail": _tail(completed.stdout),
                "stderr_tail": _tail(completed.stderr),
            }
        )
    return {
        "schema_version": "closed_loop_offline_acceptance.v0",
        "status": "passed" if all(item["status"] == "passed" for item in results) else "failed",
        "scope": scope,
        "environment_required": False,
        "claims": [
            "Scenario IR and exchange contracts are locally validated.",
            "Dry-run closed-loop reporting and metric contracts are locally validated.",
            "This command does not execute or validate the live CARLA, ROS 2, GPU, model, or NuRec gates.",
        ],
        "gate_count": len(results),
        "passed_gate_count": sum(item["status"] == "passed" for item in results),
        "gates": results,
        "remaining_environment_gate": "docs/environment_dependency_backlog.md",
    }


def _tail(value: str | None, lines: int = 20) -> list[str]:
    return (value or "").splitlines()[-lines:]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run ClosedLoopBench gates that need no runtime environment.")
    parser.add_argument("--output", required=True)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument(
        "--extended",
        action="store_true",
        help="Include counterfactual, algorithm-plugin, and render-quality offline gates.",
    )
    parser.add_argument("--full-suite", action="store_true")
    args = parser.parse_args(argv)
    result = run_offline_acceptance(
        python_executable=args.python,
        full_suite=args.full_suite,
        extended=args.extended,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": result["status"], "output": str(output)}, ensure_ascii=False))
    return 0 if result["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
