from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Callable

from agents.plugin_contract import canonical_sha256, strict_json_loads


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runners.run_carla_basic_agent import (
    _import_basic_agent_cls,
    build_basic_agent_plan,
    run_basic_agent,
)
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
    carla_python_api_path: str | Path | None = None,
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
    if ego_driver == "basic_agent" and execute is run_basic_agent:
        try:
            _import_basic_agent_cls(carla_python_api_path)
        except Exception as exc:
            raise CarlaAcceptanceError(
                f"CARLA BasicAgent preflight failed before attempt creation: {exc}"
            ) from exc
    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    transfuserpp_required = _is_transfuserpp_run(run_config)
    if transfuserpp_required:
        _validate_transfuserpp_run_config_identity(run_config)
        _validate_transfuserpp_external_evidence(run_config)
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
            snap_to_map=bool(opendrive_path),
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
        if transfuserpp_required:
            algorithm_validation = _validate_transfuserpp_attempt_evidence(
                result,
                config=config,
                evidence_root=output_root,
                run_id=run_id,
            )
            result["algorithm_evidence_validation"] = algorithm_validation
            (run_dir / "transfuserpp_attempt_validation.json").write_text(
                json.dumps(algorithm_validation, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        (run_dir / "runtime_result.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        if transfuserpp_required and result["algorithm_evidence_validation"]["status"] != "passed":
            raise CarlaAcceptanceError(
                f"attempt {attempt} TransFuser++ evidence failed: "
                + ", ".join(result["algorithm_evidence_validation"]["problems"])
            )
        results.append(result)

    summary = validate_acceptance_runs(
        results,
        require_multimodal=require_multimodal,
        require_algorithm_clean=transfuserpp_required,
    )
    aggregate = {
        "schema_version": "carla_acceptance_triplicate.v1",
        "run_count": 3,
        "scene_id": run_config.get("scenario_id"),
        "status": "passed",
        "runs": summary,
    }
    if transfuserpp_required:
        # This runner proves control/multimodal transport and closed-loop KPI
        # gates.  Render-quality evidence is evaluated separately after the
        # raw frames exist, so a successful triplicate must not silently claim
        # perception-ranking eligibility.
        aggregate.update(
            {
                "evidence_classification": "control_only",
                "perception_ranking_eligible": False,
                "perception_ranking_gate": "separate_bound_render_quality_report_required",
            }
        )
    (output_root / "acceptance_triplicate.json").write_text(
        json.dumps(aggregate, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return aggregate


def validate_acceptance_runs(
    results: list[dict[str, Any]],
    *,
    require_multimodal: bool = False,
    require_algorithm_clean: bool = False,
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
        if require_algorithm_clean and (
            (result.get("algorithm_evidence_validation") or {}).get("status")
            != "passed"
        ):
            raise CarlaAcceptanceError(
                f"attempt {index} lacks clean TransFuser++ algorithm evidence"
            )
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
                "algorithm_evidence_validation": result.get(
                    "algorithm_evidence_validation"
                ),
            }
        )
    return validated


def _is_transfuserpp_run(config: dict[str, Any]) -> bool:
    experiment = config.get("experiment") or {}
    ego = config.get("ego") or {}
    return (
        experiment.get("algorithm_id") == "transfuserpp_v5"
        or ego.get("algorithm_id") == "transfuserpp_v5"
    )


def _validate_transfuserpp_external_evidence(config: dict[str, Any]) -> None:
    nurec = config.get("nurec_runtime") or {}
    runtime_scene_id = str(nurec.get("runtime_scene_id") or "")
    reference = nurec.get("lidar_coordinate_validation") or {}
    path = Path(str(reference.get("evidence_path") or ""))
    expected = str(reference.get("evidence_sha256") or "")
    try:
        file_valid = path.is_file() and _file_sha256(path) == expected
        evidence = strict_json_loads(path.read_text(encoding="utf-8")) if file_valid else {}
    except (OSError, ValueError):
        file_valid = False
        evidence = {}
    lidar_specs = [
        row
        for row in nurec.get("lidar_specs") or []
        if isinstance(row, dict) and row.get("sensor_id") == "lidar_top"
    ]
    lidar_spec = lidar_specs[0] if len(lidar_specs) == 1 else {}
    experiment = config.get("experiment") or {}
    matrix = lidar_spec.get("sensor_to_ego")
    live = evidence.get("live_render_lidar") or {}
    content_valid = (
        evidence.get("schema_version")
        == "scene0061_lidar_coordinate_validation.v1"
        and evidence.get("status") == "passed"
        and evidence.get("scene_id") == experiment.get("scene_id")
        and bool(runtime_scene_id)
        and evidence.get("runtime_scene_id") == runtime_scene_id
        and evidence.get("artifact_sha256") == experiment.get("artifact_sha256")
        and evidence.get("sensor_id") == "lidar_top"
        and evidence.get("device_type") == lidar_spec.get("model")
        and evidence.get("response_coordinate_frame") == "sensor_local"
        and evidence.get("axis_convention") == "carla_sensor"
        and evidence.get("sensor_to_ego_coordinate_frame")
        == "carla_x_forward_y_right_z_up"
        and isinstance(matrix, list)
        and evidence.get("sensor_to_ego") == matrix
        and evidence.get("sensor_to_ego_sha256") == canonical_sha256(matrix)
        and live.get("status") == "passed"
        and live.get("rpc_status") == "ok"
        and live.get("payload_sha256_valid") is True
        and isinstance(live.get("point_count"), int)
        and int(live["point_count"]) > 0
        and live.get("timestamp_inside_artifact_range") is True
        and live.get("scene_start_matches_artifact") is True
    )
    if not file_valid or not content_valid:
        raise CarlaAcceptanceError(
            "TransFuser++ LiDAR coordinate evidence file/hash/schema cannot be verified"
        )
    _validate_transfuserpp_cuda_evidence(config)


def _validate_transfuserpp_cuda_evidence(config: dict[str, Any]) -> None:
    reference = config.get("algorithm_gpu_validation") or {}
    path = Path(str(reference.get("evidence_path") or ""))
    expected_sha = str(reference.get("evidence_sha256") or "")
    try:
        file_valid = path.is_file() and _file_sha256(path) == expected_sha
        evidence = strict_json_loads(path.read_text(encoding="utf-8")) if file_valid else {}
    except (OSError, ValueError):
        file_valid, evidence = False, {}
    latency = evidence.get("latency_ms") or {}
    gate = ((config.get("algorithm_runtime_identity") or {}).get("cuda_gate") or {})
    samples = latency.get("samples") or []
    experiment_names = (
        "scene_id",
        "scene_version",
        "case_id",
        "seed",
        "artifact_sha256",
        "scene_package_sha256",
        "scenario_ir_sha256",
        "immutable_matrix_sha256",
        "source_run_config_sha256",
        "variant_config_sha256",
        "run_config_sha256",
    )
    run_experiment = config.get("experiment") or {}
    expected_experiment = {name: run_experiment.get(name) for name in experiment_names}
    content_valid = (
        reference.get("status") == "bound"
        and evidence.get("schema_version") == "transfuserpp_cuda_preflight.v1"
        and evidence.get("status") == "passed"
        and evidence.get("real_checkpoint_loaded") is True
        and evidence.get("tensor_warmup_completed") is True
        and evidence.get("runtime_identity") == config.get("algorithm_runtime_identity")
        and evidence.get("gate") == gate
        and all(value is not None for value in expected_experiment.values())
        and evidence.get("experiment") == expected_experiment
        and isinstance(evidence.get("warmup_iterations"), int)
        and evidence.get("warmup_iterations") == gate.get("warmup_iterations")
        and isinstance(evidence.get("measured_iterations"), int)
        and evidence.get("measured_iterations") == gate.get("measured_iterations")
        and isinstance(samples, list)
        and len(samples) == evidence.get("measured_iterations")
        and all(isinstance(value, (int, float)) and math.isfinite(float(value)) and value >= 0 for value in samples)
        and isinstance(evidence.get("cuda_peak_memory_allocated_bytes"), int)
        and 0 < evidence.get("cuda_peak_memory_allocated_bytes") <= gate.get("max_peak_memory_bytes", -1)
        and isinstance(latency.get("p95"), (int, float))
        and latency.get("p95") <= gate.get("max_p95_latency_ms", -1)
        and isinstance(latency.get("p99"), (int, float))
        and latency.get("p99") <= gate.get("max_p99_latency_ms", -1)
    )
    if not file_valid or not content_valid:
        raise CarlaAcceptanceError(
            "TransFuser++ CUDA warmup/VRAM/latency evidence cannot be verified"
        )


def _validate_transfuserpp_run_config_identity(config: dict[str, Any]) -> None:
    identity = config.get("config_identity") or {}
    payload = {
        key: value
        for key, value in config.items()
        if key not in {"config_identity", "algorithm_gpu_validation"}
    }
    if (
        identity.get("schema_version")
        != "closedloopbench_run_config_identity.v1"
        or identity.get("hash_scope")
        != "whole_run_config_excluding_config_identity_and_algorithm_gpu_validation"
        or identity.get("canonical_sha256") != canonical_sha256(payload)
    ):
        raise CarlaAcceptanceError(
            "TransFuser++ run config canonical identity cannot be verified"
        )


def _validate_transfuserpp_attempt_evidence(
    result: dict[str, Any],
    *,
    config: dict[str, Any],
    evidence_root: Path,
    run_id: str,
) -> dict[str, Any]:
    from metrics.transfuserpp_intermediate import evaluate_intermediate_trace

    problems: list[str] = []
    diagnostics = (
        ((result.get("report") or {}).get("runtime") or {}).get(
            "ego_driver_diagnostics"
        )
        or {}
    )
    fallback_count = int(diagnostics.get("fallback_count") or 0)
    initialization_fallback_count = int(
        diagnostics.get("algorithm_sensor_binding", {}).get(
            "initialization_safe_stop_count",
            diagnostics.get("initialization_safe_stop_count") or 0,
        )
        or 0
    )
    non_initialization_fallback_count = max(
        0, fallback_count - initialization_fallback_count
    )
    mismatched_count = int(diagnostics.get("mismatched_control_count") or 0)
    control_count = int(diagnostics.get("control_count") or 0)
    binding = diagnostics.get("algorithm_sensor_binding") or {}
    sensor_frame_count = int(binding.get("received_frame_count") or 0)
    rejected_frame_count = int(binding.get("rejected_frame_count") or 0)
    if non_initialization_fallback_count:
        problems.append("non_initialization_fallback_count_nonzero")
    if mismatched_count:
        problems.append("mismatched_control_count_nonzero")
    if rejected_frame_count:
        problems.append("sensor_frame_rejection_count_nonzero")
    if control_count <= 0 or sensor_frame_count not in {
        control_count,
        control_count + 1,
    }:
        problems.append("sensor_frame_to_control_count_mismatch")

    contract = config.get("algorithm_evidence_contract") or {}
    relative = str(contract.get("intermediate_root_relative") or "")
    relative_path = Path(relative)
    if not relative or relative_path.is_absolute() or ".." in relative_path.parts:
        problems.append("intermediate_root_relative_invalid")
        intermediate_root = evidence_root / "invalid"
    else:
        intermediate_root = evidence_root / relative_path
    run_root = intermediate_root / run_id
    record_paths = sorted(run_root.glob("*.intermediate.json")) if run_root.is_dir() else []
    records: list[dict[str, Any]] = []
    for path in record_paths:
        try:
            records.append(strict_json_loads(path.read_text(encoding="utf-8")))
        except (OSError, ValueError):
            problems.append(f"intermediate_record_unreadable:{path.name}")
    evaluation = None
    if len(records) != control_count:
        problems.append("intermediate_record_count_mismatch")
    if records:
        try:
            evaluation = evaluate_intermediate_trace(
                records, evidence_root=evidence_root
            )
            if evaluation.get("status") != "evaluated":
                problems.append("intermediate_trace_validation_failed")
        except (OSError, ValueError) as exc:
            evaluation = {"status": "failed", "error": str(exc)}
            problems.append("intermediate_trace_validation_failed")
    else:
        problems.append("intermediate_trace_missing")
    failure_trace = intermediate_root / "backend_failures" / f"{run_id}.jsonl"
    if failure_trace.is_file() and failure_trace.stat().st_size > 0:
        problems.append("backend_failure_trace_nonempty")
    return {
        "schema_version": "transfuserpp_acceptance_attempt_validation.v1",
        "status": "passed" if not problems else "failed",
        "run_id": run_id,
        "control_count": control_count,
        "sensor_frame_count": sensor_frame_count,
        "initialization_fallback_count": initialization_fallback_count,
        "non_initialization_fallback_count": non_initialization_fallback_count,
        "mismatched_control_count": mismatched_count,
        "sensor_frame_rejection_count": rejected_frame_count,
        "intermediate_record_count": len(records),
        "backend_failure_trace": str(failure_trace),
        "intermediate_evaluation": evaluation,
        "evidence_classification": (
            (evaluation or {}).get("evidence_classification") or "control_only"
        ),
        "perception_ranking_eligible": False,
        "problems": sorted(set(problems)),
    }


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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
        "--carla-python-api",
        type=Path,
        help="CARLA PythonAPI/carla directory containing agents/navigation/basic_agent.py.",
    )
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
    try:
        config = strict_json_loads(args.run_config.read_text(encoding="utf-8"))
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
            carla_python_api_path=args.carla_python_api,
            sensor_frame_handler_factory=handler_factory,
        )
    except (CarlaAcceptanceError, FileExistsError, ImportError, OSError, ValueError) as exc:
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
