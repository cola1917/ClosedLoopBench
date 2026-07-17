from __future__ import annotations

import hashlib
import json
import math
from copy import deepcopy
from typing import Any, Callable, Mapping

from adapters.nurec_multimodal import (
    NuRecMultimodalError,
    validate_nurec_multimodal_evidence,
    validate_nurec_multimodal_frame,
)


FrameDispatcher = Callable[[Mapping[str, Any]], Mapping[str, Any]]


def run_nurec_dynamic_pose_ab_probe(
    track_id: str,
    baseline_frame: Mapping[str, Any],
    moved_frame: Mapping[str, Any],
    *,
    dispatch_frame: FrameDispatcher,
) -> dict[str, Any]:
    """Prove that changing one dynamic root pose changes both rendered modalities.

    The baseline is rendered twice and must be repeatable. All three requests
    keep scene time, sensor poses, and every other dynamic object fixed. A
    successful RPC alone is insufficient: the aggregate RGB and LiDAR response
    digests must be stable across A/A and each change for B.
    """

    baseline = deepcopy(dict(baseline_frame))
    moved = deepcopy(dict(moved_frame))
    validate_nurec_multimodal_frame(baseline)
    validate_nurec_multimodal_frame(moved)
    baseline_actor, moved_actor, pose_delta_m = _validate_ab_pair(
        str(track_id), baseline, moved
    )
    del baseline_actor, moved_actor
    if not callable(dispatch_frame):
        raise NuRecMultimodalError("pose probe dispatch_frame must be callable")

    baseline_evidence = dict(dispatch_frame(baseline))
    baseline_repeat_evidence = dict(dispatch_frame(baseline))
    moved_evidence = dict(dispatch_frame(moved))
    _validate_evidence_identity(baseline_evidence, baseline)
    _validate_evidence_identity(baseline_repeat_evidence, baseline)
    _validate_evidence_identity(moved_evidence, moved)

    modality_results = {}
    issues = []
    for modality in ("rgb", "lidar"):
        baseline_digest = _aggregate_modality_digest(baseline_evidence, modality)
        baseline_repeat_digest = _aggregate_modality_digest(
            baseline_repeat_evidence, modality
        )
        moved_digest = _aggregate_modality_digest(moved_evidence, modality)
        repeatable = (
            baseline_digest is not None
            and baseline_repeat_digest is not None
            and baseline_digest == baseline_repeat_digest
        )
        changed = (
            repeatable
            and moved_digest is not None
            and baseline_digest != moved_digest
        )
        status = "passed" if changed else "failed"
        if baseline_digest is None:
            issues.append(f"{modality}_baseline_failed")
        if baseline_repeat_digest is None:
            issues.append(f"{modality}_baseline_repeat_failed")
        if (
            baseline_digest is not None
            and baseline_repeat_digest is not None
            and baseline_digest != baseline_repeat_digest
        ):
            issues.append(f"{modality}_baseline_unrepeatable")
        if moved_digest is None:
            issues.append(f"{modality}_moved_failed")
        if baseline_digest is not None and moved_digest == baseline_digest:
            issues.append(f"{modality}_render_unchanged")
        modality_results[modality] = {
            "status": status,
            "dynamic_object_sha256": moved["shared_dynamic_object_sha256"],
            "baseline_payload_sha256": baseline_digest,
            "baseline_repeat_payload_sha256": baseline_repeat_digest,
            "moved_payload_sha256": moved_digest,
            "baseline_repeatable": repeatable,
            "content_changed": changed,
        }

    probe = {
        "frame_id": moved["frame_id"],
        "pose_delta_m": pose_delta_m,
        "baseline_dynamic_object_sha256": baseline[
            "shared_dynamic_object_sha256"
        ],
        "dynamic_object_sha256": moved["shared_dynamic_object_sha256"],
        "modalities": modality_results,
    }
    return {
        "schema_version": "nurec_dynamic_pose_ab_probe.v1",
        "scene_id": moved["scene_id"],
        "track_id": str(track_id),
        "status": "passed" if not issues else "failed",
        "issues": sorted(set(issues)),
        "probe": probe,
        "baseline_evidence": baseline_evidence,
        "baseline_repeat_evidence": baseline_repeat_evidence,
        "moved_evidence": moved_evidence,
    }


def _validate_ab_pair(
    track_id: str,
    baseline: Mapping[str, Any],
    moved: Mapping[str, Any],
) -> tuple[Mapping[str, Any], Mapping[str, Any], float]:
    for field in ("scene_id", "frame_id", "simulation_time_sec", "pose_interval_sec"):
        if baseline.get(field) != moved.get(field):
            raise NuRecMultimodalError(
                f"pose probe must keep {field} fixed between baseline and moved requests"
            )
    for modality in ("rgb", "lidar"):
        baseline_sensors = [
            item["sensor"] for item in baseline["modalities"][modality]["requests"]
        ]
        moved_sensors = [
            item["sensor"] for item in moved["modalities"][modality]["requests"]
        ]
        if baseline_sensors != moved_sensors:
            raise NuRecMultimodalError(
                f"pose probe must keep {modality} sensor requests fixed"
            )

    baseline_by_track = {
        str(item["track_id"]): item for item in baseline["shared_dynamic_objects"]
    }
    moved_by_track = {
        str(item["track_id"]): item for item in moved["shared_dynamic_objects"]
    }
    if set(baseline_by_track) != set(moved_by_track):
        raise NuRecMultimodalError("pose probe changed the dynamic track set")
    if track_id not in baseline_by_track:
        raise NuRecMultimodalError(f"pose probe target track is absent: {track_id}")
    for other_track in sorted(set(baseline_by_track) - {track_id}):
        if baseline_by_track[other_track] != moved_by_track[other_track]:
            raise NuRecMultimodalError(
                f"pose probe changed non-target track: {other_track}"
            )
    baseline_actor = baseline_by_track[track_id]
    moved_actor = moved_by_track[track_id]
    baseline_identity = dict(baseline_actor)
    moved_identity = dict(moved_actor)
    baseline_identity.pop("pose_pair", None)
    moved_identity.pop("pose_pair", None)
    if baseline_identity != moved_identity:
        raise NuRecMultimodalError("pose probe changed target actor identity")
    pose_delta = _position_delta(
        baseline_actor["pose_pair"]["end"]["position_m"],
        moved_actor["pose_pair"]["end"]["position_m"],
    )
    if pose_delta < 0.05:
        raise NuRecMultimodalError("pose probe target delta must be at least 0.05 m")
    if baseline["shared_dynamic_object_sha256"] == moved["shared_dynamic_object_sha256"]:
        raise NuRecMultimodalError("pose probe dynamic-object payload did not change")
    return baseline_actor, moved_actor, pose_delta


def _validate_evidence_identity(
    evidence: Mapping[str, Any], frame: Mapping[str, Any]
) -> None:
    validate_nurec_multimodal_evidence(evidence)
    for field in ("scene_id", "frame_id", "simulation_time_sec"):
        if evidence.get(field) != frame.get(field):
            raise NuRecMultimodalError(
                f"pose probe evidence {field} does not match its request"
            )
    if evidence.get("dynamic_object_sha256") != frame.get(
        "shared_dynamic_object_sha256"
    ):
        raise NuRecMultimodalError(
            "pose probe evidence dynamic-object digest does not match its request"
        )


def _aggregate_modality_digest(
    evidence: Mapping[str, Any], modality: str
) -> str | None:
    selected = [
        record for record in evidence["records"] if record["modality"] == modality
    ]
    if not selected or any(
        record.get("status") != "passed" or not record.get("payload_sha256")
        for record in selected
    ):
        return None
    payload = [
        {
            "sensor_id": str(record["sensor_id"]),
            "payload_sha256": str(record["payload_sha256"]),
        }
        for record in sorted(selected, key=lambda item: str(item["sensor_id"]))
    ]
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _position_delta(first: Mapping[str, Any], second: Mapping[str, Any]) -> float:
    return math.sqrt(
        sum((float(second[axis]) - float(first[axis])) ** 2 for axis in ("x", "y", "z"))
    )
