from __future__ import annotations

import argparse
import json
import sys
import time
from copy import deepcopy
from pathlib import Path
from typing import Any, Iterable

from agents.algorithm_backend import AlgorithmBackendError, load_backend
from agents.plugin_contract import AlgorithmPluginExecutor, PluginContractError, file_sha256


def load_observations(path: str | Path) -> list[dict[str, Any]]:
    source = Path(path)
    text = source.read_text(encoding="utf-8")
    if source.suffix.lower() == ".jsonl":
        observations = [json.loads(line) for line in text.splitlines() if line.strip()]
    else:
        payload = json.loads(text)
        observations = payload.get("observations") if isinstance(payload, dict) else payload
    if not isinstance(observations, list) or not observations:
        raise ValueError("observation input must contain a non-empty list")
    if any(not isinstance(item, dict) for item in observations):
        raise ValueError("every observation must be an object")
    return observations


def replay_observations(
    *,
    plugin_spec: str,
    plugin_config: dict[str, Any],
    observations: list[dict[str, Any]],
    scene_context: dict[str, Any] | None = None,
    timeout_frames: Iterable[int] = (),
    exception_frames: Iterable[int] = (),
    wall_clock: bool = False,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    backend = load_backend(plugin_spec, plugin_config)
    executor = AlgorithmPluginExecutor(
        backend,
        plugin_config,
        already_initialized=True,
        evidence_classification="offline_conformance",
    )
    executor.initialize()
    executor.reset(scene_context or {"mode": "offline_replay"})
    timeout_set = set(timeout_frames)
    exception_set = set(exception_frames)
    trace = []
    previous_timestamp: float | None = None
    started = time.perf_counter()
    try:
        for observation in observations:
            frame_id = observation.get("frame_id")
            timestamp = observation.get("timestamp", observation.get("t_sec", 0.0))
            if wall_clock and previous_timestamp is not None:
                time.sleep(max(0.0, float(timestamp) - previous_timestamp))
            previous_timestamp = float(timestamp)
            forced_failure = (
                "timeout"
                if frame_id in timeout_set
                else "backend_exception"
                if frame_id in exception_set
                else None
            )
            result = executor.predict(observation, forced_failure=forced_failure)
            trace.append(
                {
                    "frame_id": frame_id,
                    "timestamp": timestamp,
                    "execution_status": result["execution_status"],
                    "evidence_classification": "offline_conformance",
                    "control": result["control"],
                    "detail": result.get("detail", {}),
                }
            )
    finally:
        executor.close()
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    fallback_count = sum(item["execution_status"] == "fallback" for item in trace)
    report = {
        "schema_version": "algorithm_offline_replay_report.v1",
        "execution_status": "completed" if fallback_count == 0 else "completed_with_fallback",
        "evidence_classification": "offline_conformance",
        "real_carla_nurec_closed_loop": False,
        "plugin": plugin_spec,
        "plugin_identity": deepcopy(executor.identity),
        "frame_count": len(trace),
        "control_count": len(trace) - fallback_count,
        "fallback_count": fallback_count,
        "fallback_rate": fallback_count / len(trace),
        "wall_clock_replay": wall_clock,
        "elapsed_ms": elapsed_ms,
        "remote_validation_required": True,
    }
    return trace, report


def verify_determinism(
    *,
    plugin_spec: str,
    plugin_config: dict[str, Any],
    observations: list[dict[str, Any]],
    scene_context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    first, _ = replay_observations(
        plugin_spec=plugin_spec,
        plugin_config=plugin_config,
        observations=deepcopy(observations),
        scene_context=scene_context,
    )
    second, _ = replay_observations(
        plugin_spec=plugin_spec,
        plugin_config=plugin_config,
        observations=deepcopy(observations),
        scene_context=scene_context,
    )
    def stable(trace: list[dict[str, Any]]) -> list[dict[str, Any]]:
        result = deepcopy(trace)
        for item in result:
            item["control"].pop("inference_ms", None)
        return result
    passed = stable(first) == stable(second)
    return {
        "passed": passed,
        "comparison": "exact_control_trace_excluding_measured_inference_ms",
    }


def _write_jsonl(path: Path, records: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(record, sort_keys=True) + "\n" for record in records),
        encoding="utf-8",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Replay JSON/JSONL observations through an ego plugin without CARLA/NuRec."
    )
    parser.add_argument("--plugin", required=True, help="module:factory")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--observations", required=True, type=Path)
    parser.add_argument("--control-trace", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--scene-context", type=Path)
    parser.add_argument("--simulate-timeout-frame", type=int, action="append", default=[])
    parser.add_argument("--simulate-exception-frame", type=int, action="append", default=[])
    parser.add_argument("--wall-clock", action="store_true")
    parser.add_argument("--verify-determinism", action="store_true")
    args = parser.parse_args(argv)
    try:
        for output in (args.control_trace, args.report):
            if output.exists():
                parser.error(f"refusing to overwrite existing output: {output}")
        config = json.loads(args.config.read_text(encoding="utf-8"))
        observations = load_observations(args.observations)
        scene_context = (
            json.loads(args.scene_context.read_text(encoding="utf-8"))
            if args.scene_context
            else {"mode": "offline_replay"}
        )
        trace, report = replay_observations(
            plugin_spec=args.plugin,
            plugin_config=config,
            observations=observations,
            scene_context=scene_context,
            timeout_frames=args.simulate_timeout_frame,
            exception_frames=args.simulate_exception_frame,
            wall_clock=args.wall_clock,
        )
        report["input_sha256"] = file_sha256(args.observations)
        if args.verify_determinism:
            report["determinism"] = verify_determinism(
                plugin_spec=args.plugin,
                plugin_config=config,
                observations=observations,
                scene_context=scene_context,
            )
            if not report["determinism"]["passed"]:
                report["execution_status"] = "failed"
        _write_jsonl(args.control_trace, trace)
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
        print(json.dumps(report, sort_keys=True))
        return 0 if report["execution_status"] != "failed" else 2
    except (OSError, ValueError, json.JSONDecodeError, AlgorithmBackendError, PluginContractError) as exc:
        print(json.dumps({"execution_status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
