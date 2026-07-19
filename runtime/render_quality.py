"""Offline, fail-closed render-quality evaluation for NuRec evidence.

This module intentionally does not mutate source imagery.  Metrics that require
an actor mask or multiple frames are reported as unavailable when the request
does not provide trustworthy evidence.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
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

    consistency = _evaluate_multimodal_consistency(request.get("rgb_lidar_actor_change"))
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
        "edit_kind": edit_kind,
        "artifact": {
            "path": artifact.get("path"),
            "sha256": artifact_sha,
            "immutable": bool(artifact.get("immutable", True)),
        },
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
        if not isinstance(metrics, Mapping):
            raise RenderQualityError("camera metrics must be an object")
        for name, metric in metrics.items():
            if not isinstance(metric, Mapping) or not isinstance(metric.get("available"), bool):
                raise RenderQualityError(f"metric {name} requires an availability contract")
            if not metric["available"] and not metric.get("reason"):
                raise RenderQualityError(f"unavailable metric {name} requires a reason")


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


def _evaluate_multimodal_consistency(value: Any) -> dict[str, Any]:
    if value is None:
        return {
            "status": "unavailable",
            "rgb_actor_changed": None,
            "lidar_actor_changed": None,
            "evidence_paths": [],
            "reason": "no paired RGB/LiDAR actor-change evidence was supplied",
        }
    if not isinstance(value, Mapping):
        raise RenderQualityError("rgb_lidar_actor_change must be an object")
    rgb = value.get("rgb_actor_changed")
    lidar = value.get("lidar_actor_changed")
    if not isinstance(rgb, bool) or not isinstance(lidar, bool):
        raise RenderQualityError("RGB/LiDAR actor change flags must be boolean")
    evidence_paths = value.get("evidence_paths", [])
    if not isinstance(evidence_paths, list) or not all(isinstance(path, str) for path in evidence_paths):
        raise RenderQualityError("RGB/LiDAR evidence_paths must be strings")
    passed = rgb and lidar
    return {
        "status": "passed" if passed else "failed",
        "rgb_actor_changed": rgb,
        "lidar_actor_changed": lidar,
        "evidence_paths": evidence_paths,
        "reason": None if passed else "edited actor must change in both RGB and LiDAR",
    }


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
