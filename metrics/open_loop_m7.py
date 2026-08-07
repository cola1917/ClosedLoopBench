"""Frozen triplicate acceptance contract for the open-loop M7 evaluation."""

from __future__ import annotations

import hashlib
import json
import math
import re
from copy import deepcopy
from pathlib import Path, PurePosixPath
from typing import Any, Iterable, Mapping, Sequence

from metrics.open_loop import validate_open_loop_report


M7_REPORT_SCHEMA = "open_loop_multimodal_m7_triplicate_report.v1"
INTERMEDIATE_EVALUATION_SCHEMA = "transfuserpp_intermediate_evaluation.v1"
EXPECTED_SEEDS = (41, 43, 47)
EXPECTED_CASE_ID = "S0_original_replay"
EXPECTED_FRAME_COUNT = 39
EXPECTED_SENSOR_SOURCE = "nurec_stage_b_6cam_rgb_lidar"
EXPECTED_IMAGE_DIGEST = "sha256:d5f814b8aab88bbef08e70a4f915771658221190d2501e2894815977d3db6394"

ALGORITHM_IDENTITY_FIELDS = (
    "repo_sha256",
    "checkpoint_sha256",
    "model_config_sha256",
    "repo_revision",
    "carla_agents_sha256",
    "adapter_source_sha256",
    "container_image_digest",
)

FIXED_EXPERIMENT_FIELDS = (
    "scene_id",
    "scene_version",
    "case_id",
    "artifact_sha256",
    "scene_package_sha256",
    "scenario_ir_sha256",
    "immutable_matrix_sha256",
    "source_run_config_sha256",
)

NUMERIC_METRICS = (
    ("metrics", "ade_m"),
    ("metrics", "fde_m"),
    ("metrics", "lateral_error_p95_m"),
    ("metrics", "heading_error_p95_deg"),
    ("metrics", "prediction_point_count"),
    ("metrics", "collision_proxy_count"),
    ("metrics", "latency_ms", "count"),
    ("metrics", "latency_ms", "mean_ms"),
    ("metrics", "latency_ms", "p95_ms"),
    ("metrics", "latency_ms", "max_ms"),
)


class OpenLoopM7Error(ValueError):
    """Raised when formal triplicate evidence cannot be frozen safely."""


def aggregate_open_loop_m7(
    reports: Iterable[Mapping[str, Any]],
    *,
    report_paths: Sequence[Path | str] | None = None,
    intermediate_evaluations: Iterable[Mapping[str, Any]] = (),
    intermediate_evaluation_paths: Sequence[Path | str] | None = None,
    evidence_root: Path | None = None,
) -> dict[str, Any]:
    """Validate and aggregate the three real M7 seed reports.

    The function intentionally raises on any incomplete or inconsistent input;
    callers must not write a partial aggregate as a formal result.
    """

    report_rows = list(reports)
    if len(report_rows) != len(EXPECTED_SEEDS):
        raise OpenLoopM7Error(
            f"M7 requires exactly {len(EXPECTED_SEEDS)} reports, found {len(report_rows)}"
        )
    if report_paths is not None and len(report_paths) != len(report_rows):
        raise OpenLoopM7Error("report_paths must have one entry per report")

    evaluation_rows = list(intermediate_evaluations)
    if len(evaluation_rows) != len(EXPECTED_SEEDS):
        raise OpenLoopM7Error(
            "M7 requires one evaluated intermediate report per seed"
        )
    if intermediate_evaluation_paths is not None and len(intermediate_evaluation_paths) != len(
        evaluation_rows
    ):
        raise OpenLoopM7Error(
            "intermediate_evaluation_paths must have one entry per evaluation"
        )

    prepared = [
        _prepare_report(
            report,
            report_path=(report_paths[index] if report_paths is not None else None),
            evidence_root=evidence_root,
        )
        for index, report in enumerate(report_rows)
    ]
    prepared.sort(key=lambda item: item["seed"])
    seeds = tuple(item["seed"] for item in prepared)
    if seeds != EXPECTED_SEEDS:
        raise OpenLoopM7Error(
            f"M7 seeds must be exactly {list(EXPECTED_SEEDS)}, found {list(seeds)}"
        )

    fixed_identity = prepared[0]["identity"]
    fixed_experiment = prepared[0]["experiment"]
    for item in prepared[1:]:
        if item["identity"] != fixed_identity:
            raise OpenLoopM7Error("algorithm identity differs across formal seeds")
        for field in FIXED_EXPERIMENT_FIELDS:
            if item["experiment"].get(field) != fixed_experiment.get(field):
                raise OpenLoopM7Error(
                    f"formal experiment identity differs across seeds: {field}"
                )

    evaluations = [
        _prepare_intermediate_evaluation(
            evaluation,
            evaluation_path=(
                intermediate_evaluation_paths[index]
                if intermediate_evaluation_paths is not None
                else None
            ),
        )
        for index, evaluation in enumerate(evaluation_rows)
    ]
    evaluations.sort(key=lambda item: item["seed"])
    if tuple(item["seed"] for item in evaluations) != EXPECTED_SEEDS:
        raise OpenLoopM7Error("intermediate evaluation seeds do not match M7 seeds")

    for report, evaluation in zip(prepared, evaluations):
        if report["seed"] != evaluation["seed"]:
            raise OpenLoopM7Error("report and intermediate evaluation seed mismatch")
        if evaluation["experiment"].get("case_id") != EXPECTED_CASE_ID:
            raise OpenLoopM7Error("intermediate evaluation case_id is not S0_original_replay")
        for field in ("scene_id", "scene_version", "scenario_ir_sha256"):
            if evaluation["experiment"].get(field) != report["experiment"].get(field):
                raise OpenLoopM7Error(
                    f"report/intermediate identity mismatch: {field}"
                )

    run_config_hashes = [item["experiment"].get("run_config_sha256") for item in prepared]
    if len(set(run_config_hashes)) != len(run_config_hashes):
        raise OpenLoopM7Error("formal seeds unexpectedly share run_config_sha256")

    metric_summaries = {
        _metric_name(path): _summarize_metric(
            {
                str(item["seed"]): _lookup(item["report"], path)
                for item in prepared
            }
        )
        for path in NUMERIC_METRICS
    }

    report_bindings = []
    for report, evaluation in zip(prepared, evaluations):
        report_bindings.append(
            {
                "seed": report["seed"],
                "run_id": report["report"].get("run_id"),
                "report": report["file"],
                "runtime_config": report["runtime_config"],
                "observation_trace": report["observation_trace"],
                "intermediate_evaluation": evaluation["file"],
                "intermediate_frame_count": evaluation["evaluation"].get("frame_count"),
            }
        )

    return {
        "schema_version": M7_REPORT_SCHEMA,
        "stage": "M7_formal_triplicate",
        "execution_status": "completed",
        "acceptance_status": "passed",
        "evidence_classification": "open_loop_multimodal",
        "scene_id": fixed_experiment.get("scene_id"),
        "scene_version": fixed_experiment.get("scene_version"),
        "scenario_id": prepared[0]["report"].get("scenario_id"),
        "case_id": EXPECTED_CASE_ID,
        "seeds": list(EXPECTED_SEEDS),
        "sample_count": len(EXPECTED_SEEDS),
        "algorithm_identity": deepcopy(fixed_identity),
        "experiment_identity": deepcopy(fixed_experiment),
        "config_comparability": {
            "status": "same_pinned_configuration_across_seeds",
            "fixed_fields": list(FIXED_EXPERIMENT_FIELDS),
            "varying_fields": [
                "seed",
                "run_id",
                "run_config_sha256",
                "variant_config_sha256",
                "runtime_config_sha256",
                "intermediate_output_dir",
            ],
            "source_sensor": EXPECTED_SENSOR_SOURCE,
        },
        "metrics": {
            "variance_definition": "population",
            "summary": metric_summaries,
        },
        "per_seed": {
            str(item["seed"]): {
                "run_id": item["report"].get("run_id"),
                "metrics": deepcopy(item["report"].get("metrics") or {}),
                "frame_sync": deepcopy(item["report"].get("frame_sync") or {}),
                "tfpp": deepcopy(item["report"].get("tfpp") or {}),
                "nurec": deepcopy(item["report"].get("nurec") or {}),
                "intermediate_evaluation": {
                    "status": evaluations[index]["evaluation"].get("status"),
                    "evidence_classification": evaluations[index]["evaluation"].get(
                        "evidence_classification"
                    ),
                    "frame_count": evaluations[index]["evaluation"].get("frame_count"),
                },
            }
            for index, item in enumerate(prepared)
        },
        "formal_gates": {
            "all_execution_status_completed": True,
            "all_real_checkpoint_loaded": True,
            "all_frame_counts_39": True,
            "all_intermediate_counts_39": True,
            "all_fallback_counts_zero": True,
            "all_frame_sync_zero_drop_mismatch": True,
            "all_rgb6_and_lidar1_passed": True,
            "all_dynamic_actor_creation_false": True,
            "intermediate_evaluation_status": "evaluated_for_all_seeds",
        },
        "sensor_source": EXPECTED_SENSOR_SOURCE,
        "ego_pose_source": "scenario_ir_reference_trajectory",
        "control_affects_next_ego_pose": False,
        "claims_m8": False,
        "claims_m9": False,
        "remote_validation_required": True,
        "full_3d_occupancy": {
            "status": "unavailable",
            "reason": "M7 retains the existing control/BEV proxy boundary; no full-scene dense occupancy ground truth is bound",
        },
        "artifact_bindings": report_bindings,
    }


def _prepare_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | str | None,
    evidence_root: Path | None,
) -> dict[str, Any]:
    try:
        validate_open_loop_report(report)
    except (TypeError, ValueError) as exc:
        raise OpenLoopM7Error(f"invalid open-loop report: {exc}") from exc

    _require(report.get("execution_status") == "completed", "report execution is not completed")
    _require(
        report.get("real_tfpp_checkpoint_loaded") is True,
        "report does not prove real TF++ checkpoint load",
    )
    _require(
        report.get("real_carla_stage_b_open_loop") is True,
        "report is not a real NuRec Stage B open-loop run",
    )
    _require(report.get("sensor_source") == EXPECTED_SENSOR_SOURCE, "sensor source is not formal NuRec Stage B")
    _require(report.get("claims_m8") is False and report.get("claims_m9") is False, "M8/M9 claim boundary is invalid")

    sync = report.get("frame_sync") or {}
    for field, expected in (
        ("source_frame_count", EXPECTED_FRAME_COUNT),
        ("prediction_frame_count", EXPECTED_FRAME_COUNT),
        ("matched_frame_count", EXPECTED_FRAME_COUNT),
        ("dropped_frame_count", 0),
        ("frame_mismatch_count", 0),
        ("scored_frame_mismatch_count", 0),
    ):
        _require(sync.get(field) == expected, f"frame_sync.{field} is not {expected}")

    tfpp = report.get("tfpp") or {}
    _require(tfpp.get("intermediate_count") == EXPECTED_FRAME_COUNT, "TF++ intermediate count is not 39")
    _require(tfpp.get("fallback_count") == 0, "TF++ fallback_count is not zero")

    nurec = report.get("nurec") or {}
    for field, expected in (
        ("frame_count", EXPECTED_FRAME_COUNT),
        ("camera_count", 6),
        ("lidar_count", 1),
        ("dynamic_actor_creation", False),
        ("dynamic_object_count", 0),
        ("all_frames_rgb6_passed", True),
        ("all_frames_lidar_passed", True),
        ("all_frames_raw_normalized_lidar_verified", True),
    ):
        _require(nurec.get(field) == expected, f"nurec.{field} is not {expected!r}")

    manifest = report.get("runtime_manifest") or {}
    _require(manifest.get("execution_status") == "prepared", "runtime manifest is not prepared")
    _require(manifest.get("real_checkpoint_loaded") is True, "runtime manifest lacks real checkpoint proof")
    identity = _algorithm_identity(report, manifest)
    _require(
        identity.get("container_image_digest") == EXPECTED_IMAGE_DIGEST,
        "formal image digest is not the pinned M6/M7 image",
    )

    config, config_path = _load_runtime_config(report, evidence_root)
    experiment = report.get("experiment")
    if not isinstance(experiment, Mapping):
        experiment = config.get("experiment") if config else None
    if not isinstance(experiment, Mapping):
        raise OpenLoopM7Error("formal report does not contain a bound experiment identity")
    experiment = dict(experiment)
    _require(experiment.get("case_id") == EXPECTED_CASE_ID, "formal case_id is not S0_original_replay")
    seed = _seed_from_experiment_or_refs(experiment, report)
    _require(seed in EXPECTED_SEEDS, f"unexpected formal seed: {seed}")
    _require(experiment.get("scenario_ir_sha256"), "scenario IR hash is missing from experiment identity")

    if config is not None:
        _require(config.get("seed") == seed, "runtime config seed does not match report")
        _require(config.get("case_id") == EXPECTED_CASE_ID, "runtime config case_id is not formal S0")
        open_loop = config.get("open_loop") or {}
        for field, expected in (
            ("evidence_classification", "open_loop_multimodal"),
            ("sensor_source", EXPECTED_SENSOR_SOURCE),
            ("control_affects_next_ego_pose", False),
            ("claims_m8", False),
            ("claims_m9", False),
            ("dynamic_actor_creation", False),
            ("dynamic_object_count", 0),
        ):
            _require(open_loop.get(field) == expected, f"runtime config open_loop.{field} is invalid")

    return {
        "seed": seed,
        "report": dict(report),
        "identity": identity,
        "experiment": experiment,
        "file": _file_ref(report_path),
        "runtime_config": _bound_artifact_ref(
            report.get("runtime_config_path"),
            report.get("runtime_config_sha256"),
            config_path,
        ),
        "observation_trace": _bound_artifact_ref(
            report.get("observation_trace_path"),
            report.get("observation_trace_sha256"),
            _resolve_bound_path(report.get("observation_trace_path"), evidence_root),
        ),
    }


def _prepare_intermediate_evaluation(
    evaluation: Mapping[str, Any], *, evaluation_path: Path | str | None
) -> dict[str, Any]:
    if evaluation.get("schema_version") != INTERMEDIATE_EVALUATION_SCHEMA:
        raise OpenLoopM7Error("intermediate evaluation schema is not frozen v1")
    if evaluation.get("status") != "evaluated":
        raise OpenLoopM7Error("intermediate evaluation is not evaluated")
    if evaluation.get("frame_count") != EXPECTED_FRAME_COUNT:
        raise OpenLoopM7Error("intermediate evaluation does not cover 39 frames")
    experiment = evaluation.get("experiment")
    if not isinstance(experiment, Mapping):
        raise OpenLoopM7Error("intermediate evaluation lacks experiment identity")
    seed = experiment.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise OpenLoopM7Error("intermediate evaluation seed is invalid")
    return {
        "seed": seed,
        "experiment": dict(experiment),
        "evaluation": dict(evaluation),
        "file": _file_ref(evaluation_path),
    }


def _algorithm_identity(report: Mapping[str, Any], manifest: Mapping[str, Any]) -> dict[str, Any]:
    manifest_identity = manifest.get("identity") or {}
    plugin_identity = report.get("plugin_identity") or {}
    result = {}
    for field in ALGORITHM_IDENTITY_FIELDS:
        value = manifest_identity.get(field)
        if value is None:
            value = plugin_identity.get(field)
        if value is None:
            value = (report.get("runtime_manifest") or {}).get(field)
        if value is None:
            raise OpenLoopM7Error(f"algorithm identity field is missing: {field}")
        result[field] = value
    return result


def _load_runtime_config(
    report: Mapping[str, Any], evidence_root: Path | None
) -> tuple[dict[str, Any] | None, Path | None]:
    raw_path = report.get("runtime_config_path")
    if not raw_path:
        return None, None
    path = _resolve_bound_path(raw_path, evidence_root)
    if path is None or not path.is_file():
        if report.get("experiment") is not None:
            return None, None
        raise OpenLoopM7Error(f"runtime config is unavailable: {raw_path}")
    digest = _sha256(path)
    expected = str(report.get("runtime_config_sha256") or "")
    if expected and digest != expected:
        raise OpenLoopM7Error("runtime config SHA-256 does not match report binding")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise OpenLoopM7Error(f"runtime config is not valid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise OpenLoopM7Error("runtime config must be a JSON object")
    return value, path


def _seed_from_experiment_or_refs(
    experiment: Mapping[str, Any], report: Mapping[str, Any]
) -> int:
    value = experiment.get("seed")
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    candidates = []
    for frame in report.get("frames") or []:
        reference = frame.get("intermediate_record_ref") if isinstance(frame, Mapping) else None
        if isinstance(reference, Mapping):
            candidates.append(str(reference.get("path") or ""))
    matches = {int(value) for path in candidates for value in re.findall(r"/seed_(\d+)(?:/|$)", path)}
    if len(matches) != 1:
        raise OpenLoopM7Error("formal seed is not unambiguously bound")
    return next(iter(matches))


def _resolve_bound_path(value: Any, evidence_root: Path | None) -> Path | None:
    raw = str(value or "")
    if not raw:
        return None
    direct = Path(raw)
    if direct.is_file():
        return direct.resolve()
    if evidence_root is None:
        return None
    root = evidence_root.resolve()
    posix = PurePosixPath(raw.replace("\\", "/"))
    if posix.is_absolute():
        try:
            relative = posix.relative_to(PurePosixPath("/sim-data"))
        except ValueError:
            return None
    else:
        relative = posix
    if ".." in relative.parts:
        return None
    candidate = (root / Path(*relative.parts)).resolve()
    try:
        candidate.relative_to(root)
    except ValueError:
        return None
    return candidate if candidate.is_file() else None


def _bound_artifact_ref(
    declared_path: Any, expected_sha256: Any, resolved_path: Path | None
) -> dict[str, Any] | None:
    if not declared_path and resolved_path is None:
        return None
    result = {
        "declared_path": str(declared_path) if declared_path else None,
        "sha256": str(expected_sha256) if expected_sha256 else None,
    }
    if resolved_path is not None:
        actual = _sha256(resolved_path)
        if result["sha256"] and result["sha256"] != actual:
            raise OpenLoopM7Error(f"bound artifact SHA-256 mismatch: {resolved_path}")
        result.update({"host_path": str(resolved_path), "size_bytes": resolved_path.stat().st_size})
    return result


def _file_ref(path: Path | str | None) -> dict[str, Any] | None:
    if path is None:
        return None
    target = Path(path).expanduser().resolve()
    if not target.is_file():
        raise OpenLoopM7Error(f"artifact file is unavailable: {target}")
    return {
        "path": str(path),
        "host_path": str(target),
        "sha256": _sha256(target),
        "size_bytes": target.stat().st_size,
    }


def _lookup(value: Mapping[str, Any], path: Sequence[str]) -> Any:
    current: Any = value
    for part in path:
        if not isinstance(current, Mapping):
            return None
        current = current.get(part)
    return current


def _metric_name(path: Sequence[str]) -> str:
    return ".".join(path)


def _summarize_metric(values: Mapping[str, Any]) -> dict[str, Any]:
    numeric: dict[str, float | int] = {}
    for seed, value in values.items():
        if value is None:
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise OpenLoopM7Error(f"metric value for seed {seed} is not finite numeric data")
        numeric[seed] = value
    ordered = [float(numeric[seed]) for seed in sorted(numeric, key=int)]
    mean = sum(ordered) / len(ordered) if ordered else None
    variance = (
        sum((value - mean) ** 2 for value in ordered) / len(ordered)
        if ordered and mean is not None
        else None
    )
    return {
        "values_by_seed": dict(values),
        "available_count": len(ordered),
        "mean": mean,
        "variance": variance,
        "standard_deviation": math.sqrt(variance) if variance is not None else None,
    }


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise OpenLoopM7Error(message)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
