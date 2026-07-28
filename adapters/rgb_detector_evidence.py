"""Bind an independently versioned RGB detector to M8 physical projections."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any


class RgbDetectorEvidenceError(ValueError):
    pass


def match_detector_evidence(
    projections: list[Mapping[str, Any]], detector_evidence: Mapping[str, Any]
) -> list[dict[str, Any]]:
    """Return M8 independent-observation rows for class/IoU matched boxes."""

    _validate_provenance(detector_evidence)
    detections = detector_evidence.get("detections")
    if not isinstance(detections, list):
        raise RgbDetectorEvidenceError("detector evidence requires detections list")
    threshold = float((detector_evidence.get("model") or {}).get("score_threshold"))
    normalized = [_detection(item, threshold) for item in detections]
    result = []
    for projection in projections:
        object_id = str(projection.get("object_id") or "")
        camera = str(projection.get("camera") or "")
        frame_id = projection.get("frame_id")
        box = ((projection.get("projection") or {}).get("bbox_xyxy_px"))
        semantic_class = str(projection.get("semantic_class") or "")
        if not object_id or not camera or not isinstance(frame_id, int) or not _box(box):
            raise RgbDetectorEvidenceError("projection requires object_id, camera, frame_id, semantic_class and box")
        matches = [
            detection for detection in normalized
            if detection["frame_id"] == frame_id
            and detection["camera"] == camera
            and detection["eligible"]
            and _class_compatible(semantic_class, detection["semantic_class"])
            and _iou(box, detection["bbox_xyxy_px"]) >= 0.25
        ]
        if not matches:
            continue
        best = max(matches, key=lambda item: (item["score"], _iou(box, item["bbox_xyxy_px"])))
        result.append(
            {
                "object_id": object_id,
                "sensor_id": camera,
                "modality": "rgb_detector",
                "status": "passed",
                "frame_id": frame_id,
                "match": {"iou": _iou(box, best["bbox_xyxy_px"]), "score": best["score"], "detected_class": best["semantic_class"]},
                "model": dict(detector_evidence["model"]),
            }
        )
    return result


def _validate_provenance(evidence: Mapping[str, Any]) -> None:
    if evidence.get("schema_version") != "rgb_detector_evidence.v1":
        raise RgbDetectorEvidenceError("unsupported RGB detector evidence schema")
    model = evidence.get("model")
    if not isinstance(model, Mapping):
        raise RgbDetectorEvidenceError("detector evidence requires model provenance")
    for key in ("name", "version", "weight_sha256", "class_mapping_sha256"):
        value = model.get(key)
        if not isinstance(value, str) or not value:
            raise RgbDetectorEvidenceError(f"detector model requires {key}")
    for key in ("weight_sha256", "class_mapping_sha256"):
        value = str(model[key])
        if len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
            raise RgbDetectorEvidenceError(f"detector model {key} must be lowercase SHA-256")
    score = model.get("score_threshold")
    if not isinstance(score, (int, float)) or isinstance(score, bool) or not 0.0 <= float(score) <= 1.0:
        raise RgbDetectorEvidenceError("detector model score_threshold must be in [0, 1]")


def _detection(value: Any, threshold: float) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise RgbDetectorEvidenceError("detector detection must be an object")
    frame_id, camera, category = value.get("frame_id"), str(value.get("camera") or ""), str(value.get("semantic_class") or "")
    score, box = value.get("score"), value.get("bbox_xyxy_px")
    if not isinstance(frame_id, int) or not camera or not category or not isinstance(score, (int, float)) or not _box(box):
        raise RgbDetectorEvidenceError("detector detection has invalid frame/camera/class/score/box")
    if not math.isfinite(float(score)):
        raise RgbDetectorEvidenceError("detector score must be finite")
    return {"frame_id": frame_id, "camera": camera, "semantic_class": category, "score": float(score), "bbox_xyxy_px": [float(item) for item in box], "eligible": float(score) >= threshold}


def _box(value: Any) -> bool:
    return isinstance(value, list) and len(value) == 4 and all(isinstance(item, (int, float)) and math.isfinite(float(item)) for item in value) and float(value[2]) > float(value[0]) and float(value[3]) > float(value[1])


def _iou(left: list[Any], right: list[Any]) -> float:
    x1, y1 = max(float(left[0]), float(right[0])), max(float(left[1]), float(right[1]))
    x2, y2 = min(float(left[2]), float(right[2])), min(float(left[3]), float(right[3]))
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = (float(left[2]) - float(left[0])) * (float(left[3]) - float(left[1])) + (float(right[2]) - float(right[0])) * (float(right[3]) - float(right[1])) - intersection
    return intersection / union if union > 0.0 else 0.0


def _class_compatible(expected: str, detected: str) -> bool:
    aliases = {"vehicle": {"car", "truck", "bus", "motorcycle", "bicycle"}, "pedestrian": {"person"}, "two_wheeler": {"bicycle", "motorcycle"}}
    return expected == detected or detected in aliases.get(expected, set())
