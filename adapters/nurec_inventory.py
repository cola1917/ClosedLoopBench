from __future__ import annotations

import hashlib
import re
from pathlib import Path
from typing import Any, Mapping

from adapters.scene_object_registry import validate_scene_object_registry
from adapters.shared_protocol_validation import validate_document


class NuRecInventoryError(ValueError):
    """Raised when runtime track discovery/probe evidence is malformed."""


_DYNAMIC_ROLES = {
    "background_replay",
    "controlled_lead_vehicle",
    "controlled_pedestrian",
}

_NCORE_CLASS_TO_REGISTRY_CLASS = {
    "automobile": "vehicle",
    "bus": "vehicle",
    "heavy_truck": "vehicle",
    "Other Vehicle - Construction Vehicle": "vehicle",
    "bicycle": "two_wheeler",
    "motorcycle": "two_wheeler",
    "pedestrian": "pedestrian",
}


def audit_registry_ncore_dynamic_closure(
    registry: Mapping[str, Any],
    ncore_track_audit: Mapping[str, Any],
) -> dict[str, Any]:
    """Fail closed unless CARLA and NCore name the identical dynamic tracks.

    The NCore sidecar is produced from the same cuboid-track selectors passed to
    NuRec.  A count-only comparison is insufficient: an unregistered NuRec
    track has no CARLA collision proxy, while a missing registered track makes
    a CARLA actor visually unverifiable.  This audit therefore requires a
    bidirectional track-ID match and compatible semantic classes.
    """

    try:
        validate_scene_object_registry(registry)
    except ValueError as exc:
        raise NuRecInventoryError(str(exc)) from exc
    if ncore_track_audit.get("schema_version") != 1:
        raise NuRecInventoryError("NCore track audit must use schema_version 1")
    selected_tracks = ncore_track_audit.get("selected_tracks")
    tracks = selected_tracks if isinstance(selected_tracks, list) else ncore_track_audit.get("eligible_tracks")
    if not isinstance(tracks, list):
        raise NuRecInventoryError("NCore track audit requires eligible_tracks list")
    selected_track_collection = "selected_tracks" if isinstance(selected_tracks, list) else "eligible_tracks"
    selection_missing = ncore_track_audit.get("selected_track_ids_missing_from_eligible") or []
    if not isinstance(selection_missing, list) or any(not str(track_id) for track_id in selection_missing):
        raise NuRecInventoryError("NCore selected_track_ids_missing_from_eligible must be a list of IDs")

    ncore_by_id: dict[str, Mapping[str, Any]] = {}
    malformed_tracks: list[str] = []
    for raw_track in tracks:
        if not isinstance(raw_track, Mapping):
            raise NuRecInventoryError("NCore eligible tracks must be objects")
        track_id = str(raw_track.get("track_id") or "")
        if not track_id or track_id in ncore_by_id:
            raise NuRecInventoryError("NCore eligible track IDs must be unique and non-empty")
        ncore_by_id[track_id] = raw_track
        if not str(raw_track.get("class_id") or "") or not str(raw_track.get("source") or ""):
            malformed_tracks.append(track_id)

    required_by_id: dict[str, Mapping[str, Any]] = {}
    malformed_records: list[str] = []
    for raw_record in registry["records"]:
        if raw_record.get("role") not in _DYNAMIC_ROLES:
            continue
        nurec = raw_record.get("nurec")
        track_id = str(nurec.get("track_id") or "") if isinstance(nurec, Mapping) else ""
        object_id = str(raw_record.get("object_id") or "")
        if not track_id or track_id in required_by_id:
            malformed_records.append(object_id or track_id or "<missing>")
            continue
        if nurec.get("representation") != "dynamic_track":
            malformed_records.append(object_id)
            continue
        required_by_id[track_id] = raw_record

    required_ids = set(required_by_id)
    ncore_ids = set(ncore_by_id)
    missing_ids = sorted(required_ids - ncore_ids)
    unexpected_ids = sorted(ncore_ids - required_ids)
    rows: list[dict[str, Any]] = []
    class_mismatch_ids: list[str] = []
    source_mismatch_ids: list[str] = []
    for track_id in sorted(required_ids):
        record = required_by_id[track_id]
        ncore_track = ncore_by_id.get(track_id)
        row = {
            "object_id": str(record["object_id"]),
            "track_id": track_id,
            "registry_semantic_class": str(record.get("semantic_class") or ""),
            "ncore_class_id": ncore_track.get("class_id") if ncore_track else None,
            "ncore_source": ncore_track.get("source") if ncore_track else None,
            "status": "passed",
            "issues": [],
        }
        if ncore_track is None:
            row["status"] = "missing_from_ncore"
            row["issues"].append("registered_dynamic_track_missing_from_ncore")
        else:
            mapped_class = _NCORE_CLASS_TO_REGISTRY_CLASS.get(str(ncore_track.get("class_id") or ""))
            if mapped_class != row["registry_semantic_class"]:
                class_mismatch_ids.append(track_id)
                row["issues"].append("ncore_registry_semantic_class_mismatch")
            if str(ncore_track.get("source") or "") != "EXTERNAL":
                source_mismatch_ids.append(track_id)
                row["issues"].append("ncore_track_source_is_not_external")
            if row["issues"]:
                row["status"] = "failed"
        rows.append(row)

    issues: list[str] = []
    if malformed_tracks:
        issues.append("ncore_track_metadata_malformed")
    if malformed_records:
        issues.append("registry_dynamic_track_metadata_malformed")
    if missing_ids:
        issues.append("registered_dynamic_tracks_missing_from_ncore")
    if unexpected_ids:
        issues.append("ncore_dynamic_tracks_missing_from_carla_registry")
    if class_mismatch_ids:
        issues.append("ncore_registry_semantic_class_mismatch")
    if source_mismatch_ids:
        issues.append("ncore_track_source_is_not_external")
    if selection_missing:
        issues.append("ncore_selected_track_ids_missing_from_eligible")
    if ncore_track_audit.get("pass") is not True:
        issues.append("ncore_dynamic_track_gate_not_passed")
    return {
        "schema_version": "scene_object_ncore_dynamic_closure_audit.v1",
        "scene_id": registry["scene_id"],
        "registry_schema_version": registry["schema_version"],
        "ncore_track_audit_schema_version": ncore_track_audit["schema_version"],
        "ncore_track_collection": selected_track_collection,
        "ncore_contract": dict(ncore_track_audit.get("contract") or {}),
        "records": rows,
        "missing_from_ncore": missing_ids,
        "unexpected_from_ncore": unexpected_ids,
        "class_mismatch_track_ids": class_mismatch_ids,
        "source_mismatch_track_ids": source_mismatch_ids,
        "selected_track_ids_missing_from_eligible": sorted(str(track_id) for track_id in selection_missing),
        "malformed_ncore_track_ids": sorted(malformed_tracks),
        "malformed_registry_object_ids": sorted(malformed_records),
        "summary": {
            "registry_dynamic_track_count": len(required_ids),
            "ncore_eligible_dynamic_track_count": len(ncore_ids),
            "matched_track_count": len(required_ids & ncore_ids),
            "missing_from_ncore_count": len(missing_ids),
            "unexpected_from_ncore_count": len(unexpected_ids),
            "class_mismatch_count": len(class_mismatch_ids),
            "source_mismatch_count": len(source_mismatch_ids),
            "selected_track_ids_missing_from_eligible_count": len(selection_missing),
        },
        "issues": issues,
        "status": "passed" if not issues else "failed",
    }


def audit_registry_source_content(
    registry: Mapping[str, Any],
    inventory: Mapping[str, Any],
) -> dict[str, Any]:
    """Join the full object registry to loaded NuRec RGB/LiDAR probe evidence.

    Missing or failed probes are retained as explicit failures.  In particular,
    an object absent from the loaded artifact is not treated as background
    geometry merely because CARLA has a physical actor for it.
    """

    if registry.get("schema_version") != "scene_object_registry.v1":
        raise NuRecInventoryError("registry must use scene_object_registry.v1")
    if inventory.get("schema_version") != "nurec_runtime_track_inventory.v1":
        raise NuRecInventoryError(
            "inventory must use nurec_runtime_track_inventory.v1"
        )
    scene_id = str(registry.get("scene_id") or "")
    if not scene_id:
        raise NuRecInventoryError("registry scene_id is required")
    records = registry.get("records")
    tracks = inventory.get("tracks")
    if not isinstance(records, list) or not isinstance(tracks, list):
        raise NuRecInventoryError("registry.records and inventory.tracks must be lists")
    by_track: dict[str, Mapping[str, Any]] = {}
    for track in tracks:
        if not isinstance(track, Mapping):
            raise NuRecInventoryError("inventory tracks must be objects")
        track_id = str(track.get("track_id") or "")
        if not track_id or track_id in by_track:
            raise NuRecInventoryError("inventory track IDs must be unique and non-empty")
        by_track[track_id] = track

    rows: list[dict[str, Any]] = []
    for record in records:
        if not isinstance(record, Mapping):
            raise NuRecInventoryError("registry records must be objects")
        object_id = str(record.get("object_id") or "")
        if not object_id:
            raise NuRecInventoryError("registry object_id is required")
        if record.get("role") == "road_boundary":
            rows.append(
                {
                    "object_id": object_id,
                    "role": "road_boundary",
                    "status": "not_applicable",
                    "issues": [],
                }
            )
            continue
        nurec = record.get("nurec")
        track_id = str(nurec.get("track_id") or "") if isinstance(nurec, Mapping) else ""
        if record.get("role") == "static_obstacle" or not track_id:
            rows.append(
                {
                    "object_id": object_id,
                    "role": str(record.get("role") or ""),
                    "status": "unverified",
                    "issues": ["static_source_content_evidence_missing"],
                }
            )
            continue
        track = by_track.get(track_id)
        if track is None:
            rows.append(
                {
                    "object_id": object_id,
                    "track_id": track_id,
                    "role": str(record.get("role") or ""),
                    "status": "missing_from_artifact",
                    "issues": ["track_missing_from_loaded_nurec_artifact"],
                }
            )
            continue
        issues = [str(issue) for issue in (track.get("issues") or [])]
        verified = track.get("dynamic_object_pose_verified") is True and not issues
        rows.append(
            {
                "object_id": object_id,
                "track_id": track_id,
                "role": str(record.get("role") or ""),
                "status": "verified" if verified else "unverified",
                "issues": issues or ([] if verified else ["dynamic_multimodal_probe_failed"]),
            }
        )

    required = [row for row in rows if row["status"] != "not_applicable"]
    failed = [row for row in required if row["status"] != "verified"]
    return {
        "schema_version": "nurec_source_content_audit.v1",
        "scene_id": scene_id,
        "registry_schema_version": registry.get("schema_version"),
        "inventory_schema_version": inventory.get("schema_version"),
        "records": rows,
        "summary": {
            "registry_required_count": len(required),
            "verified_count": sum(row["status"] == "verified" for row in required),
            "unverified_count": sum(row["status"] == "unverified" for row in required),
            "missing_from_artifact_count": sum(
                row["status"] == "missing_from_artifact" for row in required
            ),
            "not_applicable_count": sum(row["status"] == "not_applicable" for row in rows),
        },
        "issues": [
            {
                "object_id": row["object_id"],
                "track_id": row.get("track_id"),
                "status": row["status"],
                "issues": row["issues"],
            }
            for row in failed
        ],
        "status": "passed" if not failed else "failed",
    }


_TRACK_TOKEN = re.compile(r"^[0-9a-f]{32}$")


def build_nurec_runtime_track_inventory(
    actor_mapping: Mapping[Any, Any],
    *,
    artifact_path: str | Path,
    renderer_version: str,
    probe_results: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    """Record loaded tracks that pass one same-frame RGB and LiDAR pose probe."""

    artifact = Path(artifact_path)
    if not artifact.is_file():
        raise NuRecInventoryError(f"NuRec artifact does not exist: {artifact}")
    if not renderer_version:
        raise NuRecInventoryError("renderer_version is required")
    runtime_tracks = {
        str(track_id): value
        for track_id, value in actor_mapping.items()
        if _TRACK_TOKEN.fullmatch(str(track_id))
    }
    unknown_probes = sorted(set(probe_results) - set(runtime_tracks))
    if unknown_probes:
        raise NuRecInventoryError(
            "pose probes reference tracks absent from runtime actor_mapping: "
            + ", ".join(unknown_probes)
        )
    records = []
    for track_id, runtime_entry in sorted(runtime_tracks.items()):
        probe = probe_results.get(track_id)
        verified, issues = _probe_status(probe)
        actor_inst = getattr(runtime_entry, "actor_inst", runtime_entry)
        records.append(
            {
                "track_id": track_id,
                "runtime_actor_id": getattr(actor_inst, "id", None),
                "runtime_type_id": getattr(actor_inst, "type_id", None),
                "dynamic_object_pose_verified": verified,
                "probe": dict(probe) if isinstance(probe, Mapping) else None,
                "issues": issues,
            }
        )
    inventory = {
        "schema_version": "nurec_runtime_track_inventory.v1",
        "renderer": {"name": "nurec", "version": str(renderer_version)},
        "artifact": {
            "name": artifact.name,
            "sha256": _sha256(artifact),
            "size_bytes": artifact.stat().st_size,
        },
        "extraction_source": "loaded_nurec_scenario.actor_mapping_plus_dynamic_pose_probe",
        "tracks": records,
        "summary": {
            "runtime_track_count": len(records),
            "pose_verified_track_count": sum(
                record["dynamic_object_pose_verified"] for record in records
            ),
            "unverified_track_count": sum(
                not record["dynamic_object_pose_verified"] for record in records
            ),
        },
    }
    try:
        validate_document(inventory)
    except ValueError as exc:
        raise NuRecInventoryError(str(exc)) from exc
    return inventory


def _probe_status(probe: Mapping[str, Any] | None) -> tuple[bool, list[str]]:
    if not isinstance(probe, Mapping):
        return False, ["dynamic_pose_probe_missing"]
    issues = []
    frame_id = probe.get("frame_id")
    if not isinstance(frame_id, int) or isinstance(frame_id, bool):
        issues.append("probe_frame_id_missing")
    pose_delta = probe.get("pose_delta_m")
    if (
        not isinstance(pose_delta, (int, float))
        or isinstance(pose_delta, bool)
        or float(pose_delta) < 0.05
    ):
        issues.append("pose_delta_too_small")
    digest = str(probe.get("dynamic_object_sha256") or "")
    if not _is_sha256(digest):
        issues.append("dynamic_object_digest_invalid")
    baseline_digest = str(probe.get("baseline_dynamic_object_sha256") or "")
    if not _is_sha256(baseline_digest):
        issues.append("baseline_dynamic_object_digest_invalid")
    elif baseline_digest == digest:
        issues.append("dynamic_object_payload_unchanged")
    modalities = probe.get("modalities") or {}
    for modality in ("rgb", "lidar"):
        evidence = modalities.get(modality) or {}
        if evidence.get("status") != "passed":
            issues.append(f"{modality}_probe_failed")
        if evidence.get("dynamic_object_sha256") != digest:
            issues.append(f"{modality}_dynamic_object_digest_mismatch")
        baseline_payload = str(evidence.get("baseline_payload_sha256") or "")
        baseline_repeat_payload = str(
            evidence.get("baseline_repeat_payload_sha256") or ""
        )
        moved_payload = str(evidence.get("moved_payload_sha256") or "")
        if (
            not _is_sha256(baseline_payload)
            or not _is_sha256(baseline_repeat_payload)
            or not _is_sha256(moved_payload)
        ):
            issues.append(f"{modality}_render_digest_invalid")
        elif (
            baseline_payload != baseline_repeat_payload
            or evidence.get("baseline_repeatable") is not True
        ):
            issues.append(f"{modality}_baseline_unrepeatable")
        elif baseline_payload == moved_payload or evidence.get("content_changed") is not True:
            issues.append(f"{modality}_render_unchanged")
    return not issues, issues


def _is_sha256(value: str) -> bool:
    return len(value) == 64 and all(
        character in "0123456789abcdef" for character in value
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
