"""Offline, fail-closed render-quality evaluation for NuRec evidence.

This module intentionally does not mutate source imagery.  Metrics that require
an actor mask or multiple frames are reported as unavailable when the request
does not provide trustworthy evidence.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping, Sequence

import numpy as np
from PIL import Image


EVIDENCE_CLASSIFICATIONS = {
    "perception_eligible",
    "control_only",
    "quality_stress",
    "rejected",
}
_CLASSIFICATION_PRIORITY = {
    "perception_eligible": 0,
    "control_only": 1,
    "quality_stress": 2,
    "rejected": 3,
}
DEFAULT_THRESHOLDS = {
    "dark_luma_max": 8.0,
    "max_invalid_pixel_ratio": 0.001,
    "max_dark_pixel_ratio": 0.25,
    "max_actor_hole_ratio": 0.05,
    "reject_actor_hole_ratio": 0.25,
    "max_boundary_discontinuity": 0.45,
    "max_temporal_flicker": 0.20,
    "min_edited_region_change": 0.005,
    "max_unchanged_background_change": 0.03,
}
FORMAL_CAMERA_NAMES = {
    "camera_front",
    "camera_front_left",
    "camera_front_right",
    "camera_back",
    "camera_back_left",
    "camera_back_right",
}
FORMAL_CAMERA_RESOLUTION = (800, 450)
FORMAL_EXPERIMENT_IDENTITY_FIELDS = (
    "scene_id",
    "scene_version",
    "case_id",
    "seed",
    "run_id",
    "artifact_sha256",
    "scene_package_sha256",
    "scenario_ir_sha256",
    "immutable_matrix_sha256",
    "source_run_config_sha256",
    "variant_config_sha256",
)
FORMAL_REQUIRED_CAMERA_METRICS = (
    "baseline_dark_pixel_ratio",
    "edited_dark_pixel_ratio",
    "baseline_invalid_pixel_ratio",
    "edited_invalid_pixel_ratio",
    "baseline_sharpness_laplacian_variance",
    "edited_sharpness_laplacian_variance",
    "global_ssim",
    "temporal_flicker",
    "actor_roi_hole_ratio",
    "actor_boundary_discontinuity",
    "edited_region_change",
    "unchanged_background_stability",
)
RGB_LIDAR_CHANGE_SOURCE_SCHEMA = "rgb_lidar_actor_change_source_report.v1"


class RenderQualityError(ValueError):
    """Raised when an evaluation request is structurally unsafe or ambiguous."""


@dataclass(frozen=True)
class _Frame:
    path: Path
    rgb: np.ndarray
    invalid: np.ndarray


def evaluate_render_quality(
    request: Mapping[str, Any], *, base_dir: Path | None = None
) -> dict[str, Any]:
    """Evaluate paired baseline/edited RGB sequences without changing inputs."""

    _require_schema(request, "render_quality_evaluation_request.v1")
    scene_id = _nonempty(request.get("scene_id"), "scene_id")
    case_id = _nonempty(request.get("case_id"), "case_id")
    edit_kind = _nonempty(request.get("edit_kind"), "edit_kind")
    if edit_kind not in {
        "original_replay",
        "light_vehicle_edit",
        "pedestrian_edit",
        "vehicle_removal",
        "harmonizer_ab",
    }:
        raise RenderQualityError(f"unsupported edit_kind: {edit_kind}")

    artifact = request.get("artifact")
    if not isinstance(artifact, Mapping):
        raise RenderQualityError("artifact must be an object")
    artifact_sha = _sha256_text(artifact.get("sha256"), "artifact.sha256")

    thresholds = dict(DEFAULT_THRESHOLDS)
    overrides = request.get("thresholds", {})
    if not isinstance(overrides, Mapping):
        raise RenderQualityError("thresholds must be an object")
    unknown = sorted(set(overrides) - set(DEFAULT_THRESHOLDS))
    if unknown:
        raise RenderQualityError(f"unknown thresholds: {unknown}")
    for name, value in overrides.items():
        thresholds[name] = _finite_number(value, f"thresholds.{name}")
    _validate_thresholds(thresholds)

    cameras = request.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise RenderQualityError("cameras must be a non-empty list")
    names: set[str] = set()
    root = Path(base_dir) if base_dir is not None else Path.cwd()
    camera_reports: list[dict[str, Any]] = []
    for camera in cameras:
        if not isinstance(camera, Mapping):
            raise RenderQualityError("each camera must be an object")
        name = _nonempty(camera.get("camera_name"), "camera_name")
        if name in names:
            raise RenderQualityError(f"duplicate camera_name: {name}")
        names.add(name)
        camera_reports.append(
            _evaluate_camera(camera, root=root, thresholds=thresholds, edit_kind=edit_kind)
        )

    consistency = _evaluate_multimodal_consistency(
        request.get("rgb_lidar_actor_change"),
        root=root,
        expected_actor_change=edit_kind != "original_replay",
        scene_id=scene_id,
        case_id=case_id,
        artifact_sha256=artifact_sha,
        experiment=(
            request.get("experiment")
            if isinstance(request.get("experiment"), Mapping)
            else None
        ),
        target_track_id=request.get("target_track_id"),
    )
    reasons: list[str] = []
    classification = _worst(
        [report["evidence_classification"] for report in camera_reports]
    )
    for report in camera_reports:
        reasons.extend(
            f"{report['camera_name']}: {reason}"
            for reason in report["classification_reasons"]
        )

    if edit_kind == "vehicle_removal" and classification != "rejected":
        classification = "quality_stress"
        reasons.append("vehicle removal is quality_stress unless stricter gates reject it")

    if edit_kind not in {"original_replay", "harmonizer_ab"}:
        if consistency["status"] == "failed":
            classification = "rejected"
            reasons.append("RGB/LiDAR actor-change consistency failed")
        elif consistency["status"] == "unavailable" and classification == "perception_eligible":
            classification = "control_only"
            reasons.append("RGB/LiDAR actor-change consistency is unavailable")

    harmonizer_applied = bool(request.get("harmonizer_applied", False))
    source_classification = request.get("source_evidence_classification")
    if source_classification is not None and source_classification not in EVIDENCE_CLASSIFICATIONS:
        raise RenderQualityError("source_evidence_classification is invalid")
    if harmonizer_applied:
        if source_classification is None:
            if classification == "perception_eligible":
                classification = "control_only"
            reasons.append(
                "Harmonizer has no source classification and cannot establish perception eligibility"
            )
        elif _CLASSIFICATION_PRIORITY[classification] < _CLASSIFICATION_PRIORITY[source_classification]:
            classification = source_classification
            reasons.append(
                "Harmonizer cannot upgrade the source evidence classification"
            )

    remote_required = request.get("remote_validation_required", True)
    if not isinstance(remote_required, bool):
        raise RenderQualityError("remote_validation_required must be boolean")

    aggregate = _aggregate_camera_metrics(camera_reports)
    report = {
        "schema_version": "render_quality_report.v1",
        "status": "offline_quality_evaluation",
        "scene_id": scene_id,
        "case_id": case_id,
        "target_track_id": request.get("target_track_id"),
        "edit_kind": edit_kind,
        "artifact": {
            "path": artifact.get("path"),
            "sha256": artifact_sha,
            "immutable": bool(artifact.get("immutable", True)),
        },
        "experiment": (
            dict(request["experiment"])
            if isinstance(request.get("experiment"), Mapping)
            else None
        ),
        "evidence_classification": classification,
        "remote_validation_required": remote_required,
        "classification_reasons": _dedupe(reasons),
        "harmonizer": {
            "applied": harmonizer_applied,
            "source_evidence_classification": source_classification,
            "policy": "never_upgrade_source_evidence",
        },
        "thresholds": thresholds,
        "rgb_lidar_actor_change_consistency": consistency,
        "cameras": camera_reports,
        "aggregate": aggregate,
        "limitations": _dedupe(
            [
                limitation
                for camera in camera_reports
                for limitation in camera["limitations"]
            ]
            + [
                "This is an offline image-quality evaluation, not a CARLA/NuRec closed-loop acceptance result."
            ]
        ),
    }
    validate_render_quality_report(report)
    return report


def validate_render_quality_report(report: Mapping[str, Any]) -> None:
    _require_schema(report, "render_quality_report.v1")
    if report.get("status") != "offline_quality_evaluation":
        raise RenderQualityError("render report status must be offline_quality_evaluation")
    if report.get("evidence_classification") not in EVIDENCE_CLASSIFICATIONS:
        raise RenderQualityError("render report evidence_classification is invalid")
    if not isinstance(report.get("remote_validation_required"), bool):
        raise RenderQualityError("render report remote_validation_required must be boolean")
    cameras = report.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise RenderQualityError("render report requires per-camera results")
    for camera in cameras:
        if camera.get("evidence_classification") not in EVIDENCE_CLASSIFICATIONS:
            raise RenderQualityError("camera evidence_classification is invalid")
        metrics = camera.get("metrics")
        if not isinstance(metrics, Mapping) or not metrics:
            raise RenderQualityError("camera metrics must be a non-empty object")
        for name, metric in metrics.items():
            if not isinstance(metric, Mapping) or not isinstance(metric.get("available"), bool):
                raise RenderQualityError(f"metric {name} requires an availability contract")
            if not metric["available"] and not metric.get("reason"):
                raise RenderQualityError(f"unavailable metric {name} requires a reason")


def formal_perception_quality_problems(
    report: Mapping[str, Any],
    *,
    experiment: Mapping[str, Any],
    source_report_ref: Mapping[str, Any] | None,
) -> list[str]:
    """Return every reason a report cannot grant formal perception eligibility.

    This deliberately validates the evidence files again at the ranking boundary.
    A copied classification string or a structurally valid but detached report is
    never sufficient to upgrade an algorithm trace.
    """

    problems: list[str] = []
    try:
        validate_render_quality_report(report)
    except (RenderQualityError, TypeError, ValueError) as exc:
        problems.append(f"render_quality_report_invalid:{exc}")

    if report.get("evidence_classification") != "perception_eligible":
        problems.append("render_quality_report_not_perception_eligible")
    if report.get("remote_validation_required") is not True:
        problems.append("render_quality_remote_validation_contract_missing")

    bound_experiment = report.get("experiment")
    if not isinstance(bound_experiment, Mapping):
        problems.append("render_quality_experiment_identity_missing")
    else:
        for name in FORMAL_EXPERIMENT_IDENTITY_FIELDS:
            if bound_experiment.get(name) != experiment.get(name):
                problems.append(f"render_quality_experiment_identity_mismatch:{name}")

    source_problem = _formal_source_report_problem(report, source_report_ref)
    if source_problem:
        problems.append(source_problem)

    cameras = report.get("cameras")
    if not isinstance(cameras, list):
        problems.append("render_quality_formal_cameras_missing")
        cameras = []
    names = [camera.get("camera_name") for camera in cameras if isinstance(camera, Mapping)]
    if len(cameras) != len(FORMAL_CAMERA_NAMES) or set(names) != FORMAL_CAMERA_NAMES:
        problems.append("render_quality_formal_six_camera_set_mismatch")
    if len(names) != len(set(names)):
        problems.append("render_quality_duplicate_camera")
    for camera in cameras:
        if isinstance(camera, Mapping):
            problems.extend(_formal_camera_problems(camera))
        else:
            problems.append("render_quality_camera_invalid")
    for input_name in ("baseline_frames", "edited_frames", "actor_masks"):
        evidence_paths = [
            str(reference.get("path") or "")
            for camera in cameras
            if isinstance(camera, Mapping)
            for reference in ((camera.get("inputs") or {}).get(input_name) or [])
            if isinstance(reference, Mapping)
        ]
        if len(evidence_paths) != len(set(evidence_paths)):
            problems.append(f"render_quality_input_ref_reuse:{input_name}")

    aggregate = report.get("aggregate")
    if not isinstance(aggregate, Mapping):
        problems.append("render_quality_aggregate_missing")
    else:
        if aggregate.get("camera_count") != len(FORMAL_CAMERA_NAMES):
            problems.append("render_quality_aggregate_camera_count_mismatch")
        if aggregate.get("evidence_classification") != "perception_eligible":
            problems.append("render_quality_aggregate_not_perception_eligible")
        aggregate_metrics = aggregate.get("metrics")
        if not isinstance(aggregate_metrics, Mapping) or not aggregate_metrics:
            problems.append("render_quality_aggregate_metrics_empty")
        else:
            for name in FORMAL_REQUIRED_CAMERA_METRICS:
                if not _finite_available_metric(aggregate_metrics.get(name)):
                    problems.append(f"render_quality_aggregate_metric_unavailable:{name}")

    consistency = report.get("rgb_lidar_actor_change_consistency")
    if not isinstance(consistency, Mapping):
        problems.append("render_quality_rgb_lidar_consistency_missing")
    else:
        expected_change = report.get("edit_kind") != "original_replay"
        if consistency.get("status") != "passed":
            problems.append("render_quality_rgb_lidar_consistency_not_passed")
        if consistency.get("rgb_actor_changed") is not expected_change:
            problems.append("render_quality_rgb_change_flag_mismatch")
        if consistency.get("lidar_actor_changed") is not expected_change:
            problems.append("render_quality_lidar_change_flag_mismatch")
        try:
            verified_source = _load_multimodal_change_source_report(
                consistency.get("source_report_ref"),
                root=Path.cwd(),
                scene_id=str(report.get("scene_id") or ""),
                case_id=str(report.get("case_id") or ""),
                artifact_sha256=str((report.get("artifact") or {}).get("sha256") or ""),
                experiment=(
                    report.get("experiment")
                    if isinstance(report.get("experiment"), Mapping)
                    else None
                ),
                target_track_id=report.get("target_track_id"),
            )
            verified_ref = verified_source.pop("_source_report_ref")
            if consistency.get("source_report_ref") != verified_ref:
                problems.append("render_quality_rgb_lidar_source_report_ref_mismatch")
            if consistency.get("source_report") != verified_source:
                problems.append("render_quality_rgb_lidar_source_report_content_mismatch")
        except (OSError, RenderQualityError, TypeError, ValueError) as exc:
            problems.append(f"render_quality_rgb_lidar_source_report_invalid:{exc}")

    harmonizer = report.get("harmonizer")
    if not isinstance(harmonizer, Mapping):
        problems.append("render_quality_harmonizer_contract_missing")
    else:
        applied = harmonizer.get("applied")
        if not isinstance(applied, bool):
            problems.append("render_quality_harmonizer_applied_invalid")
        if harmonizer.get("policy") != "never_upgrade_source_evidence":
            problems.append("render_quality_harmonizer_policy_invalid")
        source_classification = harmonizer.get("source_evidence_classification")
        if (
            source_classification is not None
            and source_classification not in EVIDENCE_CLASSIFICATIONS
        ):
            problems.append("render_quality_harmonizer_source_classification_invalid")
        if (
            applied is True and source_classification != "perception_eligible"
        ):
            problems.append("render_quality_harmonizer_illegal_upgrade")
        if report.get("edit_kind") == "harmonizer_ab" and applied is not True:
            problems.append("render_quality_harmonizer_ab_not_applied")

    return list(dict.fromkeys(problems))


def _formal_source_report_problem(
    report: Mapping[str, Any], source_report_ref: Mapping[str, Any] | None
) -> str | None:
    problem = _formal_file_ref_problem(source_report_ref, expected_kind="json")
    if problem:
        return f"render_quality_source_report_{problem}"
    path = Path(str((source_report_ref or {}).get("path") or ""))
    try:
        stored = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return "render_quality_source_report_json_unreadable"
    expected = dict(report)
    expected.pop("_source_report_ref", None)
    if stored != expected:
        return "render_quality_source_report_content_mismatch"
    return None


def _formal_camera_problems(camera: Mapping[str, Any]) -> list[str]:
    name = str(camera.get("camera_name") or "unknown")
    problems: list[str] = []
    if camera.get("evidence_classification") != "perception_eligible":
        problems.append(f"render_quality_camera_not_perception_eligible:{name}")
    resolution = camera.get("resolution")
    if not isinstance(resolution, Mapping) or (
        resolution.get("width"),
        resolution.get("height"),
    ) != FORMAL_CAMERA_RESOLUTION:
        problems.append(f"render_quality_camera_resolution_mismatch:{name}")
    frame_count = camera.get("frame_count")
    if isinstance(frame_count, bool) or not isinstance(frame_count, int) or frame_count < 2:
        problems.append(f"render_quality_camera_frame_count_invalid:{name}")
        frame_count = 0

    provenance = camera.get("mask_provenance")
    if not isinstance(provenance, Mapping):
        problems.append(f"render_quality_mask_provenance_missing:{name}")
    else:
        if provenance.get("reliable") is not True:
            problems.append(f"render_quality_actor_mask_unreliable:{name}")
        if str(provenance.get("kind") or "").strip().lower() in {"", "none"}:
            problems.append(f"render_quality_actor_mask_kind_invalid:{name}")
        if not str(provenance.get("source") or "").strip():
            problems.append(f"render_quality_actor_mask_source_missing:{name}")

    inputs = camera.get("inputs")
    if not isinstance(inputs, Mapping):
        problems.append(f"render_quality_camera_input_refs_missing:{name}")
    else:
        for input_name, kind in (
            ("baseline_frames", "rgb"),
            ("edited_frames", "rgb"),
            ("actor_masks", "mask"),
        ):
            refs = inputs.get(input_name)
            if not isinstance(refs, list) or len(refs) != frame_count or not refs:
                problems.append(f"render_quality_input_ref_count_mismatch:{name}:{input_name}")
                continue
            for index, reference in enumerate(refs):
                problem = _formal_file_ref_problem(reference, expected_kind=kind)
                if problem:
                    problems.append(
                        f"render_quality_input_ref_invalid:{name}:{input_name}:{index}:{problem}"
                    )

    metrics = camera.get("metrics")
    if not isinstance(metrics, Mapping) or not metrics:
        problems.append(f"render_quality_camera_metrics_empty:{name}")
    else:
        for metric_name in FORMAL_REQUIRED_CAMERA_METRICS:
            if not _finite_available_metric(metrics.get(metric_name)):
                problems.append(
                    f"render_quality_camera_metric_unavailable:{name}:{metric_name}"
                )
    return problems


def _formal_file_ref_problem(value: Any, *, expected_kind: str) -> str | None:
    if not isinstance(value, Mapping):
        return "reference_missing"
    path = Path(str(value.get("path") or ""))
    digest = str(value.get("sha256") or "").lower()
    size = value.get("size_bytes")
    if not re.fullmatch(r"[0-9a-f]{64}", digest):
        return "sha256_invalid"
    if isinstance(size, bool) or not isinstance(size, int) or size <= 0:
        return "size_invalid"
    if not path.is_file():
        return "file_missing"
    try:
        if path.stat().st_size != size:
            return "size_mismatch"
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            return "sha256_mismatch"
        if expected_kind in {"rgb", "mask"}:
            with Image.open(path) as image:
                image.load()
                if image.size != FORMAL_CAMERA_RESOLUTION:
                    return "image_resolution_mismatch"
                if expected_kind == "mask":
                    mask = np.asarray(image.convert("L"), dtype=np.uint8) > 127
                    if not mask.any():
                        return "actor_mask_empty"
        elif expected_kind == "json":
            json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError):
        return "file_unreadable"
    return None


def _finite_available_metric(value: Any) -> bool:
    if not isinstance(value, Mapping) or value.get("available") is not True:
        return False
    metric_value = value.get("value")
    return (
        not isinstance(metric_value, bool)
        and isinstance(metric_value, (int, float, np.number))
        and math.isfinite(float(metric_value))
    )


def _evaluate_camera(
    camera: Mapping[str, Any], *, root: Path, thresholds: Mapping[str, float], edit_kind: str
) -> dict[str, Any]:
    name = _nonempty(camera.get("camera_name"), "camera_name")
    baseline_paths = _path_list(camera.get("baseline_frames"), f"{name}.baseline_frames")
    edited_paths = _path_list(camera.get("edited_frames"), f"{name}.edited_frames")
    if len(baseline_paths) != len(edited_paths):
        raise RenderQualityError(f"{name}: baseline/edited frame counts differ")

    baseline = [_load_frame(_resolve(root, path)) for path in baseline_paths]
    edited = [_load_frame(_resolve(root, path)) for path in edited_paths]
    for index, (before, after) in enumerate(zip(baseline, edited)):
        if before.rgb.shape != after.rgb.shape:
            raise RenderQualityError(f"{name}: frame {index} dimensions differ")
    shape = baseline[0].rgb.shape
    if any(frame.rgb.shape != shape for frame in baseline + edited):
        raise RenderQualityError(f"{name}: all sequence frames must share dimensions")

    provenance = camera.get("mask_provenance")
    if not isinstance(provenance, Mapping):
        raise RenderQualityError(f"{name}: mask_provenance is required")
    reliable = provenance.get("reliable")
    if not isinstance(reliable, bool):
        raise RenderQualityError(f"{name}: mask_provenance.reliable must be boolean")
    mask_kind = _nonempty(provenance.get("kind"), f"{name}.mask_provenance.kind")
    limitations = provenance.get("limitations", [])
    if not isinstance(limitations, list) or not all(isinstance(item, str) for item in limitations):
        raise RenderQualityError(f"{name}: mask limitations must be strings")
    masks_raw = camera.get("actor_masks")
    masks: list[np.ndarray] | None = None
    mask_paths: list[str] = []
    if masks_raw is not None:
        mask_paths = _path_list(masks_raw, f"{name}.actor_masks")
        if len(mask_paths) != len(edited):
            raise RenderQualityError(f"{name}: actor mask count must match frame count")
        masks = [_load_mask(_resolve(root, path), shape[:2]) for path in mask_paths]
    if reliable and masks is None:
        raise RenderQualityError(f"{name}: reliable mask provenance requires actor_masks")
    if masks is not None and not reliable:
        limitations = list(limitations) + [
            "Actor masks were supplied but marked unreliable; ROI metrics are not eligibility gates."
        ]

    dark_threshold = float(thresholds["dark_luma_max"])
    baseline_dark = [_dark_ratio(frame, dark_threshold) for frame in baseline]
    edited_dark = [_dark_ratio(frame, dark_threshold) for frame in edited]
    baseline_invalid = [float(frame.invalid.mean()) for frame in baseline]
    edited_invalid = [float(frame.invalid.mean()) for frame in edited]
    metrics: dict[str, dict[str, Any]] = {
        "baseline_dark_pixel_ratio": _metric(_mean(baseline_dark), "ratio"),
        "edited_dark_pixel_ratio": _metric(_mean(edited_dark), "ratio"),
        "baseline_invalid_pixel_ratio": _metric(_mean(baseline_invalid), "ratio"),
        "edited_invalid_pixel_ratio": _metric(_mean(edited_invalid), "ratio"),
        "baseline_sharpness_laplacian_variance": _metric(
            _mean([_sharpness(frame.rgb) for frame in baseline]), "variance"
        ),
        "edited_sharpness_laplacian_variance": _metric(
            _mean([_sharpness(frame.rgb) for frame in edited]), "variance"
        ),
        "psnr": _paired_metric(baseline, edited, _psnr, "dB"),
        "global_ssim": _paired_metric(baseline, edited, _global_ssim, "score"),
        "temporal_flicker": _temporal_metric(edited),
    }

    roi_usable = masks is not None and reliable
    if roi_usable:
        metrics["actor_roi_hole_ratio"] = _metric(
            _mean(
                [
                    _actor_hole_ratio(frame, mask, dark_threshold)
                    for frame, mask in zip(edited, masks or [])
                ]
            ),
            "ratio",
        )
        boundary_values = [
            value
            for frame, mask in zip(edited, masks or [])
            if (value := _boundary_discontinuity(frame.rgb, mask)) is not None
        ]
        metrics["actor_boundary_discontinuity"] = (
            _metric(_mean(boundary_values), "normalized_abs_difference")
            if boundary_values
            else _unavailable("actor mask has no measurable interior/exterior boundary")
        )
        metrics["edited_region_change"] = _metric(
            _mean(
                [
                    _masked_change(before.rgb, after.rgb, mask)
                    for before, after, mask in zip(baseline, edited, masks or [])
                ]
            ),
            "normalized_abs_difference",
        )
        metrics["unchanged_background_stability"] = _metric(
            _mean(
                [
                    _masked_change(before.rgb, after.rgb, ~_dilate(mask))
                    for before, after, mask in zip(baseline, edited, masks or [])
                ]
            ),
            "normalized_abs_difference",
        )
    else:
        reason = "no reliable actor mask; precise actor ROI measurement is unavailable"
        metrics["actor_roi_hole_ratio"] = _unavailable(reason)
        metrics["actor_boundary_discontinuity"] = _unavailable(reason)
        metrics["edited_region_change"] = _unavailable(reason)
        metrics["unchanged_background_stability"] = _unavailable(reason)
        limitations = list(limitations) + [reason]

    reasons: list[str] = []
    classification = "perception_eligible"
    if _value(metrics["edited_invalid_pixel_ratio"]) > thresholds["max_invalid_pixel_ratio"]:
        classification = "rejected"
        reasons.append("invalid pixel ratio exceeds reject threshold")
    if _value(metrics["edited_dark_pixel_ratio"]) > thresholds["max_dark_pixel_ratio"]:
        classification = "rejected"
        reasons.append("dark pixel ratio exceeds reject threshold")
    if roi_usable:
        hole = _value(metrics["actor_roi_hole_ratio"])
        if hole > thresholds["reject_actor_hole_ratio"]:
            classification = "rejected"
            reasons.append("actor ROI hole ratio exceeds reject threshold")
        elif hole > thresholds["max_actor_hole_ratio"] and classification != "rejected":
            classification = "quality_stress"
            reasons.append("actor ROI hole ratio exceeds perception threshold")
        boundary = metrics["actor_boundary_discontinuity"]
        if boundary["available"] and _value(boundary) > thresholds["max_boundary_discontinuity"]:
            if classification != "rejected":
                classification = "quality_stress"
            reasons.append("actor boundary discontinuity exceeds perception threshold")
        if edit_kind not in {"original_replay", "harmonizer_ab"}:
            change = _value(metrics["edited_region_change"])
            if change < thresholds["min_edited_region_change"]:
                classification = "rejected"
                reasons.append("requested actor edit is not visible in the reliable ROI")
        background = _value(metrics["unchanged_background_stability"])
        if background > thresholds["max_unchanged_background_change"] and classification != "rejected":
            classification = "quality_stress"
            reasons.append("unchanged background changed beyond threshold")
    elif classification == "perception_eligible":
        classification = "control_only"
        reasons.append("no reliable actor mask; perception eligibility cannot be established")

    if metrics["temporal_flicker"]["available"]:
        if _value(metrics["temporal_flicker"]) > thresholds["max_temporal_flicker"]:
            if classification != "rejected":
                classification = "quality_stress"
            reasons.append("temporal flicker exceeds threshold")
    else:
        limitations = list(limitations) + [metrics["temporal_flicker"]["reason"]]

    limitations = list(limitations) + [
        "Global SSIM is a whole-image scalar, not a localized perceptual similarity metric.",
        "Temporal flicker is an unregistered whole-frame difference and includes legitimate scene motion.",
    ]

    if edit_kind == "vehicle_removal" and classification != "rejected":
        classification = "quality_stress"
        reasons.append("vehicle-removal imagery is restricted to quality_stress")

    return {
        "camera_name": name,
        "frame_count": len(edited),
        "resolution": {"width": int(shape[1]), "height": int(shape[0])},
        "inputs": {
            "baseline_frames": [_file_ref(frame.path) for frame in baseline],
            "edited_frames": [_file_ref(frame.path) for frame in edited],
            "actor_masks": [_file_ref(_resolve(root, path)) for path in mask_paths],
        },
        "mask_provenance": {
            "kind": mask_kind,
            "reliable": reliable,
            "source": provenance.get("source"),
            "limitations": list(limitations),
        },
        "metrics": metrics,
        "evidence_classification": classification,
        "classification_reasons": _dedupe(reasons),
        "limitations": _dedupe(list(limitations)),
    }


def _evaluate_multimodal_consistency(
    value: Any,
    *,
    root: Path,
    expected_actor_change: bool,
    scene_id: str,
    case_id: str,
    artifact_sha256: str,
    experiment: Mapping[str, Any] | None,
    target_track_id: Any,
) -> dict[str, Any]:
    if value is None:
        return {
            "status": "unavailable",
            "rgb_actor_changed": None,
            "lidar_actor_changed": None,
            "expected_actor_change": expected_actor_change,
            "source_report_ref": None,
            "source_report": None,
            "reason": "no paired RGB/LiDAR actor-change evidence was supplied",
        }
    if not isinstance(value, Mapping):
        raise RenderQualityError("rgb_lidar_actor_change must be an object")
    source = _load_multimodal_change_source_report(
        value.get("source_report_ref"),
        root=root,
        scene_id=scene_id,
        case_id=case_id,
        artifact_sha256=artifact_sha256,
        experiment=experiment,
        target_track_id=target_track_id,
    )
    flags = source["change_flags"]
    rgb = flags["rgb_actor_changed"]
    lidar = flags["lidar_actor_changed"]
    passed = rgb is expected_actor_change and lidar is expected_actor_change
    return {
        "status": "passed" if passed else "failed",
        "rgb_actor_changed": rgb,
        "lidar_actor_changed": lidar,
        "expected_actor_change": expected_actor_change,
        "source_report_ref": source.pop("_source_report_ref"),
        "source_report": source,
        "reason": (
            None
            if passed
            else "RGB and LiDAR actor-change flags do not match the requested edit"
        ),
    }


def _load_multimodal_change_source_report(
    reference: Any,
    *,
    root: Path,
    scene_id: str,
    case_id: str,
    artifact_sha256: str,
    experiment: Mapping[str, Any] | None,
    target_track_id: Any,
) -> dict[str, Any]:
    """Read and verify the immutable source report behind the change claim."""

    if not isinstance(reference, Mapping):
        raise RenderQualityError("RGB/LiDAR source_report_ref is required")
    raw_path = str(reference.get("path") or "")
    path = _resolve(root, raw_path).resolve()
    normalized_ref = dict(reference)
    normalized_ref["path"] = str(path)
    problem = _formal_file_ref_problem(normalized_ref, expected_kind="json")
    if problem:
        raise RenderQualityError(f"RGB/LiDAR source report {problem}")
    try:
        source = _strict_json_mapping(path)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise RenderQualityError(f"RGB/LiDAR source report is unreadable: {exc}") from exc
    if source.get("schema_version") != RGB_LIDAR_CHANGE_SOURCE_SCHEMA:
        raise RenderQualityError("RGB/LiDAR source report schema_version mismatch")
    if source.get("status") != "passed":
        raise RenderQualityError("RGB/LiDAR source report status must be passed")
    if reference.get("schema_version") != source["schema_version"]:
        raise RenderQualityError("RGB/LiDAR source report reference schema mismatch")
    if reference.get("status") != source["status"]:
        raise RenderQualityError("RGB/LiDAR source report reference status mismatch")

    source_experiment = source.get("experiment")
    if not isinstance(source_experiment, Mapping):
        raise RenderQualityError("RGB/LiDAR source report experiment is required")
    required_identity = {
        "scene_id": scene_id,
        "case_id": case_id,
        "artifact_sha256": artifact_sha256,
    }
    if experiment is not None:
        for name in FORMAL_EXPERIMENT_IDENTITY_FIELDS:
            if name in experiment:
                required_identity[name] = experiment[name]
    for name, expected in required_identity.items():
        if source_experiment.get(name) != expected:
            raise RenderQualityError(
                f"RGB/LiDAR source report experiment mismatch: {name}"
            )

    expected_track = _nonempty(target_track_id, "target_track_id")
    if source.get("target_track_id") != expected_track:
        raise RenderQualityError("RGB/LiDAR source report target_track_id mismatch")
    frame_range = _validate_change_frame_range(source.get("frame_range"))
    payloads = _validate_change_payloads(
        source.get("payloads"), root=path.parent, frame_range=frame_range
    )
    flags = source.get("change_flags")
    if not isinstance(flags, Mapping):
        raise RenderQualityError("RGB/LiDAR source report change_flags are required")
    normalized_flags: dict[str, bool] = {}
    for modality in ("rgb", "lidar"):
        name = f"{modality}_actor_changed"
        declared = flags.get(name)
        if not isinstance(declared, bool):
            raise RenderQualityError(f"RGB/LiDAR source report {name} must be boolean")
        before = [item["sha256"] for item in payloads[modality]["baseline"]]
        after = [item["sha256"] for item in payloads[modality]["edited"]]
        measured = before != after
        if declared is not measured:
            raise RenderQualityError(
                f"RGB/LiDAR source report {name} conflicts with payload hashes"
            )
        normalized_flags[name] = declared
    return {
        "schema_version": source["schema_version"],
        "status": source["status"],
        "experiment": dict(source_experiment),
        "target_track_id": expected_track,
        "frame_range": frame_range,
        "payloads": payloads,
        "change_flags": normalized_flags,
        "_source_report_ref": {
            **_file_ref(path),
            "schema_version": source["schema_version"],
            "status": source["status"],
        },
    }


def _validate_change_frame_range(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise RenderQualityError("RGB/LiDAR source report frame_range is required")
    result: dict[str, Any] = {}
    for phase in ("baseline", "edited"):
        row = value.get(phase)
        if not isinstance(row, Mapping):
            raise RenderQualityError(f"RGB/LiDAR frame_range.{phase} is required")
        start_frame = row.get("start_frame_id")
        end_frame = row.get("end_frame_id")
        frame_count = row.get("frame_count")
        if any(
            isinstance(item, bool) or not isinstance(item, int)
            for item in (start_frame, end_frame, frame_count)
        ) or int(start_frame) < 0 or int(end_frame) < int(start_frame) or int(frame_count) <= 0:
            raise RenderQualityError(f"RGB/LiDAR frame_range.{phase} frame bounds are invalid")
        start_time = _finite_number(
            row.get("start_timestamp_sec"), f"frame_range.{phase}.start_timestamp_sec"
        )
        end_time = _finite_number(
            row.get("end_timestamp_sec"), f"frame_range.{phase}.end_timestamp_sec"
        )
        if start_time < 0.0 or end_time < start_time:
            raise RenderQualityError(f"RGB/LiDAR frame_range.{phase} timestamps are invalid")
        result[phase] = {
            "start_frame_id": int(start_frame),
            "end_frame_id": int(end_frame),
            "frame_count": int(frame_count),
            "start_timestamp_sec": start_time,
            "end_timestamp_sec": end_time,
        }
    if result["baseline"]["frame_count"] != result["edited"]["frame_count"]:
        raise RenderQualityError("RGB/LiDAR baseline/edited frame counts differ")
    return result


def _validate_change_payloads(
    value: Any, *, root: Path, frame_range: Mapping[str, Any]
) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise RenderQualityError("RGB/LiDAR source report payloads are required")
    result: dict[str, Any] = {}
    used_paths: set[str] = set()
    for modality in ("rgb", "lidar"):
        group = value.get(modality)
        if not isinstance(group, Mapping):
            raise RenderQualityError(f"RGB/LiDAR payloads.{modality} is required")
        result[modality] = {}
        for phase in ("baseline", "edited"):
            rows = group.get(phase)
            frame_count = int(frame_range[phase]["frame_count"])
            if not isinstance(rows, list) or not rows or len(rows) % frame_count:
                raise RenderQualityError(
                    f"RGB/LiDAR payloads.{modality}.{phase} must cover every declared frame"
                )
            normalized = []
            for index, reference in enumerate(rows):
                normalized_ref = _validated_change_payload_ref(
                    reference,
                    root=root,
                    modality=modality,
                    name=f"payloads.{modality}.{phase}[{index}]",
                )
                if normalized_ref["path"] in used_paths:
                    raise RenderQualityError("RGB/LiDAR payload file reference is reused")
                used_paths.add(normalized_ref["path"])
                normalized.append(normalized_ref)
            result[modality][phase] = normalized
        if len(result[modality]["baseline"]) != len(result[modality]["edited"]):
            raise RenderQualityError(
                f"RGB/LiDAR payloads.{modality} baseline/edited counts differ"
            )
    return result


def _validated_change_payload_ref(
    value: Any, *, root: Path, modality: str, name: str
) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise RenderQualityError(f"{name} must be a structured file reference")
    path = _resolve(root, str(value.get("path") or "")).resolve()
    normalized = dict(value)
    normalized["path"] = str(path)
    problem = _formal_file_ref_problem(normalized, expected_kind="binary")
    if problem:
        raise RenderQualityError(f"{name} {problem}")
    if value.get("kind") != modality:
        raise RenderQualityError(f"{name} kind must be {modality}")
    encoding = _nonempty(value.get("encoding"), f"{name}.encoding")
    if modality == "rgb":
        try:
            with Image.open(path) as image:
                image.load()
                if image.width <= 0 or image.height <= 0:
                    raise RenderQualityError(f"{name} RGB payload is empty")
        except (OSError, ValueError) as exc:
            raise RenderQualityError(f"{name} RGB payload is unreadable: {exc}") from exc
    elif encoding != "float32_xyzi_little_endian" or path.stat().st_size % 16:
        raise RenderQualityError(
            f"{name} LiDAR payload must be a float32 XYZI stream"
        )
    return {
        **_file_ref(path),
        "kind": modality,
        "encoding": encoding,
    }


def _strict_json_mapping(path: Path) -> dict[str, Any]:
    def pairs_hook(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    value = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=pairs_hook)
    if not isinstance(value, dict):
        raise ValueError("JSON root must be an object")
    return value


def _aggregate_camera_metrics(cameras: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    names = sorted({name for camera in cameras for name in camera["metrics"]})
    metrics: dict[str, Any] = {}
    for name in names:
        available = [
            camera["metrics"][name]["value"]
            for camera in cameras
            if camera["metrics"].get(name, {}).get("available")
            and camera["metrics"][name].get("value") is not None
        ]
        metrics[name] = (
            _metric(_mean(available), cameras[0]["metrics"][name].get("unit"))
            if available
            else _unavailable("metric unavailable for every camera")
        )
    return {
        "camera_count": len(cameras),
        "evidence_classification": _worst(
            [camera["evidence_classification"] for camera in cameras]
        ),
        "metrics": metrics,
    }


def _load_frame(path: Path) -> _Frame:
    if not path.is_file():
        raise RenderQualityError(f"image does not exist: {path}")
    try:
        with Image.open(path) as image:
            image.load()
            alpha = None
            if "A" in image.getbands():
                alpha = np.asarray(image.getchannel("A"), dtype=np.uint8)
            rgb = np.asarray(image.convert("RGB"), dtype=np.float64)
    except Exception as exc:  # Pillow raises several format-specific exceptions.
        raise RenderQualityError(f"cannot decode image {path}: {exc}") from exc
    invalid = ~np.isfinite(rgb).all(axis=2)
    if alpha is not None:
        invalid |= alpha == 0
    return _Frame(path=path.resolve(), rgb=rgb, invalid=invalid)


def _load_mask(path: Path, shape: tuple[int, int]) -> np.ndarray:
    if not path.is_file():
        raise RenderQualityError(f"actor mask does not exist: {path}")
    try:
        with Image.open(path) as image:
            mask = np.asarray(image.convert("L"), dtype=np.uint8) > 127
    except Exception as exc:
        raise RenderQualityError(f"cannot decode actor mask {path}: {exc}") from exc
    if mask.shape != shape:
        raise RenderQualityError(f"actor mask dimensions differ: {path}")
    if not mask.any():
        raise RenderQualityError(f"actor mask is empty: {path}")
    return mask


def _dark_ratio(frame: _Frame, threshold: float) -> float:
    luma = _gray(frame.rgb)
    return float(np.logical_or(luma <= threshold, frame.invalid).mean())


def _actor_hole_ratio(frame: _Frame, mask: np.ndarray, threshold: float) -> float:
    holes = np.logical_or(_gray(frame.rgb) <= threshold, frame.invalid)
    return float(holes[mask].mean())


def _sharpness(rgb: np.ndarray) -> float:
    gray = _gray(rgb)
    padded = np.pad(gray, 1, mode="edge")
    laplacian = (
        -4.0 * padded[1:-1, 1:-1]
        + padded[:-2, 1:-1]
        + padded[2:, 1:-1]
        + padded[1:-1, :-2]
        + padded[1:-1, 2:]
    )
    return float(np.var(laplacian))


def _psnr(before: np.ndarray, after: np.ndarray) -> float | None:
    mse = float(np.mean((before - after) ** 2))
    if mse == 0.0:
        return None
    return float(20.0 * math.log10(255.0) - 10.0 * math.log10(mse))


def _global_ssim(before: np.ndarray, after: np.ndarray) -> float:
    left = _gray(before)
    right = _gray(after)
    mu_left = float(left.mean())
    mu_right = float(right.mean())
    var_left = float(left.var())
    var_right = float(right.var())
    covariance = float(np.mean((left - mu_left) * (right - mu_right)))
    c1 = (0.01 * 255.0) ** 2
    c2 = (0.03 * 255.0) ** 2
    return float(
        ((2 * mu_left * mu_right + c1) * (2 * covariance + c2))
        / ((mu_left**2 + mu_right**2 + c1) * (var_left + var_right + c2))
    )


def _paired_metric(
    baseline: Sequence[_Frame],
    edited: Sequence[_Frame],
    function: Any,
    unit: str,
) -> dict[str, Any]:
    values = [
        result
        for before, after in zip(baseline, edited)
        if (result := function(before.rgb, after.rgb)) is not None
    ]
    if not values:
        return {
            "available": True,
            "value": None,
            "unit": unit,
            "special_value": "infinite" if function is _psnr else "identical",
        }
    return _metric(_mean(values), unit)


def _temporal_metric(frames: Sequence[_Frame]) -> dict[str, Any]:
    if len(frames) < 2:
        return _unavailable("temporal flicker requires at least two edited frames")
    values = [
        float(np.mean(np.abs(left.rgb - right.rgb)) / 255.0)
        for left, right in zip(frames, frames[1:])
    ]
    return _metric(_mean(values), "normalized_abs_difference")


def _boundary_discontinuity(rgb: np.ndarray, mask: np.ndarray) -> float | None:
    values: list[np.ndarray] = []
    horizontal_crossing = mask[:, 1:] != mask[:, :-1]
    if horizontal_crossing.any():
        horizontal_difference = np.mean(np.abs(rgb[:, 1:] - rgb[:, :-1]), axis=2)
        values.append(horizontal_difference[horizontal_crossing] / 255.0)
    vertical_crossing = mask[1:, :] != mask[:-1, :]
    if vertical_crossing.any():
        vertical_difference = np.mean(np.abs(rgb[1:, :] - rgb[:-1, :]), axis=2)
        values.append(vertical_difference[vertical_crossing] / 255.0)
    if not values:
        return None
    return float(np.concatenate(values).mean())


def _masked_change(before: np.ndarray, after: np.ndarray, mask: np.ndarray) -> float:
    if not mask.any():
        return 0.0
    change = np.mean(np.abs(before - after), axis=2) / 255.0
    return float(change[mask].mean())


def _dilate(mask: np.ndarray) -> np.ndarray:
    padded = np.pad(mask, 1, mode="constant", constant_values=False)
    result = np.zeros_like(mask)
    for row in range(3):
        for column in range(3):
            result |= padded[row : row + mask.shape[0], column : column + mask.shape[1]]
    return result


def _gray(rgb: np.ndarray) -> np.ndarray:
    return rgb[..., 0] * 0.2126 + rgb[..., 1] * 0.7152 + rgb[..., 2] * 0.0722


def _metric(value: float, unit: str | None) -> dict[str, Any]:
    return {"available": True, "value": float(value), "unit": unit}


def _unavailable(reason: str) -> dict[str, Any]:
    return {"available": False, "value": None, "unit": None, "reason": reason}


def _value(metric: Mapping[str, Any]) -> float:
    value = metric.get("value")
    if value is None:
        raise RenderQualityError("metric has no finite value")
    return float(value)


def _resolve(root: Path, raw: str) -> Path:
    path = Path(raw)
    return path if path.is_absolute() else root / path


def _path_list(value: Any, name: str) -> list[str]:
    if not isinstance(value, list) or not value or not all(isinstance(item, str) and item for item in value):
        raise RenderQualityError(f"{name} must be a non-empty list of paths")
    return list(value)


def _file_ref(path: Path) -> dict[str, Any]:
    return {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "size_bytes": path.stat().st_size,
    }


def _mean(values: Sequence[float]) -> float:
    if not values:
        raise RenderQualityError("cannot aggregate an empty metric")
    return float(sum(float(value) for value in values) / len(values))


def _worst(classifications: Sequence[str]) -> str:
    return max(classifications, key=lambda item: _CLASSIFICATION_PRIORITY[item])


def _dedupe(values: Sequence[str]) -> list[str]:
    return list(dict.fromkeys(value for value in values if value))


def _nonempty(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RenderQualityError(f"{name} must be a non-empty string")
    return value


def _sha256_text(value: Any, name: str) -> str:
    text = _nonempty(value, name).lower()
    if len(text) != 64 or any(character not in "0123456789abcdef" for character in text):
        raise RenderQualityError(f"{name} must be a sha256 hex digest")
    return text


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise RenderQualityError(f"{name} must be a finite number")
    return float(value)


def _validate_thresholds(thresholds: Mapping[str, float]) -> None:
    if not 0.0 <= thresholds["dark_luma_max"] <= 255.0:
        raise RenderQualityError("dark_luma_max must be between 0 and 255")
    for name in (
        "max_invalid_pixel_ratio",
        "max_dark_pixel_ratio",
        "max_actor_hole_ratio",
        "reject_actor_hole_ratio",
        "max_boundary_discontinuity",
        "max_temporal_flicker",
        "min_edited_region_change",
        "max_unchanged_background_change",
    ):
        if not 0.0 <= thresholds[name] <= 1.0:
            raise RenderQualityError(f"{name} must be between 0 and 1")
    if thresholds["reject_actor_hole_ratio"] < thresholds["max_actor_hole_ratio"]:
        raise RenderQualityError(
            "reject_actor_hole_ratio must be >= max_actor_hole_ratio"
        )


def _require_schema(value: Mapping[str, Any], expected: str) -> None:
    if not isinstance(value, Mapping) or value.get("schema_version") != expected:
        raise RenderQualityError(f"schema_version must be {expected}")
