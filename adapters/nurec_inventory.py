from __future__ import annotations

import hashlib
import re
from pathlib import Path
from typing import Any, Mapping

from adapters.shared_protocol_validation import validate_document


class NuRecInventoryError(ValueError):
    """Raised when runtime track discovery/probe evidence is malformed."""


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
        moved_payload = str(evidence.get("moved_payload_sha256") or "")
        if not _is_sha256(baseline_payload) or not _is_sha256(moved_payload):
            issues.append(f"{modality}_render_digest_invalid")
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
