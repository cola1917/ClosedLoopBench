"""Validate the M4 hardening evidence for three TF++ short-horizon attempts.

M4 intentionally hardens the completed M3 vertical slice.  It does not turn a
short-horizon smoke into a route-completion claim: every attempt instead proves
the same pinned model identity, fail-closed NuRec delivery, replayable LiDAR
axis normalization, and one-to-one model-control intermediate evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agents.plugin_contract import strict_json_loads  # noqa: E402
from metrics.transfuserpp_intermediate import evaluate_intermediate_trace  # noqa: E402
from runners.validate_nurec_lidar_axis_regression import validate_run  # noqa: E402


IDENTITY_KEYS = (
    "repo_revision",
    "repo_sha256",
    "checkpoint_sha256",
    "model_config_sha256",
    "carla_agents_sha256",
    "container_image_digest",
)


def _load_json(path: Path) -> dict[str, Any]:
    value = strict_json_loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _identity_problems(
    actual: Mapping[str, Any], expected: Mapping[str, Any], *, label: str
) -> list[str]:
    return [
        f"{label}_{key}_mismatch"
        for key in IDENTITY_KEYS
        if actual.get(key) != expected.get(key)
    ]


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        raise FileNotFoundError(f"missing trace: {path}")
    rows = []
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = strict_json_loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"trace row {line_no} must be an object: {path}")
        rows.append(value)
    return rows


def _validate_attempt(
    *,
    run_dir: Path,
    evidence_root: Path,
    run_config: Mapping[str, Any],
    runtime_config: Mapping[str, Any],
    cuda_preflight: Mapping[str, Any],
    min_frame_count: int,
) -> dict[str, Any]:
    problems: list[str] = []
    report_path = run_dir / "closed_loop_report.json"
    report = _load_json(report_path)
    runtime = report.get("runtime") or {}
    summary = report.get("summary") or {}
    run_id = str(report.get("run_id") or "")
    if not run_id:
        problems.append("run_id_missing")
    if report.get("status") != "ego_closed_loop":
        problems.append("closed_loop_report_status_not_ego_closed_loop")
    if runtime.get("termination_reason") != "max_ticks":
        problems.append("closed_loop_termination_not_max_ticks")
    if runtime.get("cleanup_succeeded") is not True:
        problems.append("closed_loop_cleanup_not_succeeded")
    if int(summary.get("collision_count") or 0) != 0:
        problems.append("collision_count_nonzero")

    frame_trace = _read_jsonl(run_dir / "frame_trace.jsonl")
    sensor_trace = _read_jsonl(run_dir / "nurec_multimodal_trace.jsonl")
    frame_count = int(runtime.get("frame_trace_count") or 0)
    if frame_count < min_frame_count:
        problems.append("frame_count_below_m4_minimum")
    if len(frame_trace) != frame_count or len(sensor_trace) != frame_count:
        problems.append("frame_trace_or_sensor_trace_count_mismatch")

    sensor = runtime.get("multimodal_sensor") or {}
    if not sensor.get("required") or sensor.get("status") != "passed":
        problems.append("multimodal_gate_not_fail_closed_or_not_passed")
    if int(sensor.get("frame_count") or 0) != frame_count:
        problems.append("multimodal_frame_count_mismatch")
    if int(sensor.get("passed_frame_count") or 0) != frame_count:
        problems.append("multimodal_passed_frame_count_mismatch")
    if set(sensor.get("modalities") or []) != {"rgb", "lidar"}:
        problems.append("multimodal_modalities_mismatch")

    for row in sensor_trace:
        modalities = row.get("modalities") or {}
        if row.get("status") != "passed":
            problems.append("nurec_frame_not_passed")
            break
        for modality in ("rgb", "lidar"):
            values = modalities.get(modality) or {}
            if int(values.get("requested_count") or 0) < 1 or int(
                values.get("passed_count") or 0
            ) != int(values.get("requested_count") or 0):
                problems.append(f"nurec_{modality}_coverage_incomplete")
                break

    diagnostics = runtime.get("ego_driver_diagnostics") or {}
    binding = diagnostics.get("algorithm_sensor_binding") or {}
    fallback_count = int(diagnostics.get("fallback_count") or 0)
    initialization_fallback_count = int(
        binding.get("initialization_safe_stop_count") or 0
    )
    control_count = int(diagnostics.get("control_count") or 0)
    sensor_frame_count = int(binding.get("received_frame_count") or 0)
    if runtime.get("ego_driver") != "ros2_observation_control":
        problems.append("ego_driver_not_ros2_observation_control")
    if fallback_count != initialization_fallback_count:
        problems.append("non_initialization_fallback_count_nonzero")
    if int(diagnostics.get("mismatched_control_count") or 0) != 0:
        problems.append("mismatched_control_count_nonzero")
    if control_count <= 0 or sensor_frame_count not in {
        control_count,
        control_count + 1,
    }:
        problems.append("sensor_frame_to_control_count_mismatch")
    if diagnostics.get("matched_frame_ratio") != 1.0:
        problems.append("matched_frame_ratio_not_one")

    expected_identity = {
        key: runtime_config.get(key) for key in IDENTITY_KEYS
    }
    run_identity = run_config.get("algorithm_runtime_identity") or {}
    problems.extend(
        _identity_problems(run_identity, expected_identity, label="run_config_identity")
    )
    cuda_identity = cuda_preflight.get("runtime_identity") or {}
    if cuda_preflight.get("status") != "passed" or not cuda_preflight.get(
        "real_checkpoint_loaded"
    ):
        problems.append("cuda_preflight_not_passed_or_checkpoint_not_loaded")
    problems.extend(
        _identity_problems(cuda_identity, expected_identity, label="cuda_identity")
    )

    contract = run_config.get("algorithm_evidence_contract") or {}
    relative_root = Path(str(contract.get("intermediate_root_relative") or ""))
    if not run_id or relative_root.is_absolute() or ".." in relative_root.parts:
        problems.append("intermediate_root_invalid")
        records: list[dict[str, Any]] = []
    else:
        intermediate_dir = evidence_root / relative_root / run_id
        records = [
            _load_json(path)
            for path in sorted(intermediate_dir.glob("*.intermediate.json"))
        ]
        if len(records) != control_count:
            problems.append("intermediate_control_count_mismatch")
    if records:
        for record in records:
            provenance = record.get("provenance") or {}
            if not provenance.get("real_checkpoint_loaded") or provenance.get(
                "execution_mode"
            ) != "remote_model_inference":
                problems.append("intermediate_not_real_checkpoint_inference")
                break
            problems.extend(
                _identity_problems(
                    record.get("identity") or {}, expected_identity, label="intermediate_identity"
                )
            )
        try:
            intermediate_evaluation = evaluate_intermediate_trace(
                records, evidence_root=evidence_root
            )
            if intermediate_evaluation.get("status") != "evaluated":
                problems.append("intermediate_trace_validation_failed")
        except (OSError, ValueError) as exc:
            intermediate_evaluation = {"status": "failed", "error": str(exc)}
            problems.append("intermediate_trace_validation_failed")
    else:
        intermediate_evaluation = {"status": "failed", "error": "no records"}
        problems.append("intermediate_trace_missing")

    axis = validate_run(run_dir, expected_matrix_sha256=(
        "8277ba837a2779bf041c9a1ee8a8f78f8c912192d06b74082a01a1706d96d925"
    ))
    if axis.get("status") != "passed" or axis.get("lidar_frames_checked") != frame_count:
        problems.append("lidar_axis_regression_failed_or_incomplete")

    return {
        "schema_version": "scene0061_m4_attempt_validation.v1",
        "status": "passed" if not problems else "failed",
        "run_id": run_id,
        "run_dir": str(run_dir),
        "report_sha256": _sha256(report_path),
        "frame_count": frame_count,
        "route_progress": summary.get("route_progress"),
        "collision_count": summary.get("collision_count"),
        "termination_reason": runtime.get("termination_reason"),
        "cleanup_succeeded": runtime.get("cleanup_succeeded"),
        "control_count": control_count,
        "initialization_fallback_count": initialization_fallback_count,
        "non_initialization_fallback_count": max(
            0, fallback_count - initialization_fallback_count
        ),
        "mismatched_control_count": diagnostics.get("mismatched_control_count"),
        "intermediate_record_count": len(records),
        "intermediate_evaluation": intermediate_evaluation,
        "lidar_axis_regression": axis,
        "problems": sorted(set(problems)),
    }


def validate_triplicate(
    *,
    run_dirs: list[Path],
    evidence_root: Path,
    run_config_path: Path,
    runtime_config_path: Path,
    cuda_preflight_path: Path,
    min_frame_count: int,
) -> dict[str, Any]:
    if len(run_dirs) != 3:
        raise ValueError("M4 requires exactly three independent run directories")
    run_config = _load_json(run_config_path)
    runtime_config = _load_json(runtime_config_path)
    cuda_preflight = _load_json(cuda_preflight_path)
    attempts = [
        _validate_attempt(
            run_dir=run_dir.resolve(),
            evidence_root=evidence_root.resolve(),
            run_config=run_config,
            runtime_config=runtime_config,
            cuda_preflight=cuda_preflight,
            min_frame_count=min_frame_count,
        )
        for run_dir in run_dirs
    ]
    run_ids = [attempt["run_id"] for attempt in attempts]
    problems = []
    if len(set(run_ids)) != len(run_ids):
        problems.append("attempt_run_ids_not_unique")
    if any(attempt["status"] != "passed" for attempt in attempts):
        problems.append("one_or_more_attempts_failed")
    return {
        "schema_version": "scene0061_m4_triplicate.v1",
        "status": "passed" if not problems else "failed",
        "scope": "M4_short_horizon_hardening_not_route_completion",
        "attempt_count": len(attempts),
        "min_frame_count_per_attempt": min_frame_count,
        "run_config": {
            "path": str(run_config_path.resolve()),
            "sha256": _sha256(run_config_path),
        },
        "runtime_config": {
            "path": str(runtime_config_path.resolve()),
            "sha256": _sha256(runtime_config_path),
        },
        "cuda_preflight": {
            "path": str(cuda_preflight_path.resolve()),
            "sha256": _sha256(cuda_preflight_path),
        },
        "attempts": attempts,
        "problems": problems,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Fail-closed M4 validation for three TF++ short-horizon runs."
    )
    parser.add_argument("--run-dir", action="append", type=Path, required=True)
    parser.add_argument("--evidence-root", required=True, type=Path)
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--runtime-config", required=True, type=Path)
    parser.add_argument("--cuda-preflight", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--min-frame-count", type=int, default=60)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    try:
        result = validate_triplicate(
            run_dirs=args.run_dir,
            evidence_root=args.evidence_root,
            run_config_path=args.run_config,
            runtime_config_path=args.runtime_config,
            cuda_preflight_path=args.cuda_preflight,
            min_frame_count=args.min_frame_count,
        )
    except (OSError, ValueError) as exc:
        result = {"status": "failed", "detail": str(exc)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if result.get("status") == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
