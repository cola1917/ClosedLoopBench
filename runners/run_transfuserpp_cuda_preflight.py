from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from copy import deepcopy
from pathlib import Path

from agents.plugin_contract import canonical_sha256, strict_json_loads
from agents.transfuserpp_contract import cuda_runtime_identity
from agents.transfuserpp_runtime import TransFuserPPModelRuntime


def _percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = int(position)
    upper = min(len(ordered) - 1, lower + 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def run_cuda_preflight(config: dict, observation: dict) -> dict:
    gate = config.get("cuda_gate") or {}
    warmup_count = int(gate.get("warmup_iterations") or 0)
    measured_count = int(gate.get("measured_iterations") or 0)
    if warmup_count < 1 or measured_count < 3:
        raise ValueError("cuda gate requires >=1 warmup and >=3 measured iterations")
    for name in ("max_peak_memory_bytes", "max_p95_latency_ms", "max_p99_latency_ms"):
        value = gate.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) <= 0:
            raise ValueError(f"cuda gate {name} must be finite and positive")
    runtime = TransFuserPPModelRuntime(config)
    latencies: list[float] = []
    try:
        torch = runtime.torch
        torch.cuda.synchronize(runtime.device)
        base_frame = int(observation.get("frame_id") or 0)
        base_time = float(observation.get("timestamp", observation.get("t_sec", 0.0)))
        run_context = deepcopy(dict(observation.get("run_context") or {}))
        run_context["run_id"] = f"cuda-preflight-{canonical_sha256(cuda_runtime_identity(config))[:16]}"
        for index in range(warmup_count + measured_count):
            sample = deepcopy(observation)
            sample["frame_id"] = base_frame + index
            sample["timestamp"] = base_time + index * 0.05
            sample["synchronization"]["frame_id"] = sample["frame_id"]
            sample["run_context"] = deepcopy(run_context)
            if index == warmup_count:
                torch.cuda.synchronize(runtime.device)
                torch.cuda.reset_peak_memory_stats(runtime.device)
            started = time.perf_counter()
            runtime.predict(sample)
            torch.cuda.synchronize(runtime.device)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            if index >= warmup_count:
                latencies.append(elapsed_ms)
        health = runtime.health_check()
        peak = int(torch.cuda.max_memory_allocated(runtime.device))
        p50 = _percentile(latencies, 0.50)
        p95 = _percentile(latencies, 0.95)
        p99 = _percentile(latencies, 0.99)
        passed = (
            health.get("real_checkpoint_loaded") is True
            and health.get("tensor_warmup_completed") is True
            and peak > 0
            and peak <= int(gate["max_peak_memory_bytes"])
            and p95 <= float(gate["max_p95_latency_ms"])
            and p99 <= float(gate["max_p99_latency_ms"])
        )
        return {
            "schema_version": "transfuserpp_cuda_preflight.v1",
            "status": "passed" if passed else "failed",
            "real_checkpoint_loaded": health.get("real_checkpoint_loaded") is True,
            "tensor_warmup_completed": health.get("tensor_warmup_completed") is True,
            "warmup_iterations": warmup_count,
            "measured_iterations": measured_count,
            "latency_ms": {
                "samples": latencies,
                "mean": statistics.fmean(latencies),
                "p50": p50,
                "p95": p95,
                "p99": p99,
            },
            "cuda_peak_memory_allocated_bytes": peak,
            "cuda_device_name": health.get("cuda_device_name"),
            "torch_version": health.get("torch_version"),
            "torch_cuda_version": health.get("torch_cuda_version"),
            "gate": deepcopy(gate),
            "runtime_identity": cuda_runtime_identity(config),
            "experiment": deepcopy(dict(config.get("experiment") or {})),
            "observation_sha256": canonical_sha256(observation),
        }
    finally:
        runtime.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run real TF++ CUDA warmup/VRAM/latency gate.")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--observation", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise FileExistsError(f"refusing to overwrite: {args.output}")
        config = strict_json_loads(args.config.read_text(encoding="utf-8"))
        observation = strict_json_loads(args.observation.read_text(encoding="utf-8"))
        report = run_cuda_preflight(config, observation)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(report, sort_keys=True))
        return 0 if report["status"] == "passed" else 2
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
