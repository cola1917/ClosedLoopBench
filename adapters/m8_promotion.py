"""Fail-closed promotion gate from a NuRec candidate to formal M8 use.

The candidate smoke gate proves that a low-cost source/config boundary is
eligible.  This module adds the separate runtime promotion boundary: a real
artifact may be promoted only when the candidate smoke, editable quality
window, and all four immutable same-tick M8 streams pass.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any


class M8PromotionError(ValueError):
    """Raised when promotion inputs are malformed."""


_STREAMS = ("collision", "lane", "visibility", "lidar_world")


def evaluate_m8_promotion(
    smoke_report: Mapping[str, Any],
    audit_summary: Mapping[str, Any],
    *,
    artifact_path: str | Path,
    scene_id: str | None = None,
) -> dict[str, Any]:
    """Return a promotion decision without mutating any input artifact."""

    issues: list[str] = []
    smoke_scene = str(smoke_report.get("scene_id") or "")
    if smoke_report.get("schema_version") != "nurec_reconstruction_smoke.v1":
        issues.append("invalid_smoke_report_schema")
    if smoke_report.get("status") != "passed":
        issues.append("candidate_smoke_failed")
    quality = smoke_report.get("editable_quality_windows")
    if not isinstance(quality, Mapping) or quality.get("required") is not True:
        issues.append("editable_quality_window_not_required")
    elif quality.get("status") != "passed":
        issues.append("editable_quality_window_failed")

    if scene_id is not None and smoke_scene != str(scene_id):
        issues.append("scene_id_mismatch")

    if audit_summary.get("schema_version") != "scene_safety_audit_summary.v1":
        issues.append("invalid_m8_audit_summary_schema")
    if audit_summary.get("status") != "passed":
        issues.append("m8_four_stream_summary_failed")

    stream_results: dict[str, Any] = {}
    artifacts = audit_summary.get("artifacts")
    if not isinstance(artifacts, Mapping):
        issues.append("m8_audit_summary_missing_artifacts")
        artifacts = {}
    for stream in _STREAMS:
        item = artifacts.get(stream)
        result = _inspect_stream(item, stream)
        stream_results[stream] = result
        if result["status"] != "passed":
            issues.extend(f"{stream}:{issue}" for issue in result["issues"])
    frame_sets = {
        stream: tuple(result.get("frame_ids") or [])
        for stream, result in stream_results.items()
    }
    nonempty_frame_sets = [set(value) for value in frame_sets.values() if value]
    if nonempty_frame_sets and any(value != nonempty_frame_sets[0] for value in nonempty_frame_sets[1:]):
        issues.append("m8_stream_frame_sets_mismatch")

    artifact = Path(artifact_path).expanduser().resolve()
    artifact_result: dict[str, Any]
    if not artifact.is_file():
        artifact_result = {"status": "failed", "path": str(artifact), "issues": ["artifact_missing"]}
        issues.append("artifact_missing")
    else:
        data = artifact.read_bytes()
        artifact_result = {
            "status": "passed",
            "path": str(artifact),
            "size_bytes": len(data),
            "sha256": hashlib.sha256(data).hexdigest(),
            "issues": [],
        }

    return {
        "schema_version": "m8_formal_promotion_gate.v1",
        "status": "passed" if not issues else "failed",
        "scene_id": smoke_scene or (str(scene_id) if scene_id is not None else None),
        "candidate_smoke": {
            "status": "passed" if smoke_report.get("status") == "passed" else "failed",
            "scene_id": smoke_scene,
            "editable_quality_windows": quality,
        },
        "m8_four_streams": stream_results,
        "m8_stream_frame_sets": {stream: list(ids) for stream, ids in frame_sets.items()},
        "artifact": artifact_result,
        "issues": sorted(set(issues)),
        "formal_reconstruction_allowed": not issues,
    }


def _inspect_stream(item: Any, stream: str) -> dict[str, Any]:
    issues: list[str] = []
    if not isinstance(item, Mapping):
        return {"status": "failed", "tick_count": 0, "failed_tick_count": None, "issues": ["stream_missing"]}
    path_value = item.get("path")
    if not path_value:
        issues.append("path_missing")
        path = None
    else:
        path = Path(str(path_value)).expanduser()
        if not path.is_file():
            issues.append("stream_file_missing")
    rows: list[Mapping[str, Any]] = []
    if path is not None and path.is_file():
        try:
            for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                if not line.strip():
                    continue
                value = json.loads(line)
                if not isinstance(value, Mapping):
                    raise ValueError(f"row {line_number} is not an object")
                rows.append(value)
        except (OSError, json.JSONDecodeError, UnicodeError, ValueError):
            issues.append("stream_file_invalid")
    if not rows:
        issues.append("stream_has_no_ticks")
    failed_rows = sum(row.get("status") != "passed" for row in rows)
    if failed_rows:
        issues.append("stream_contains_failed_tick")
    frame_ids = [row.get("frame_id") for row in rows]
    if any(not isinstance(frame_id, int) or isinstance(frame_id, bool) for frame_id in frame_ids):
        issues.append("stream_frame_id_invalid")
    elif len(frame_ids) != len(set(frame_ids)):
        issues.append("stream_frame_id_duplicate")
    return {
        "status": "passed" if not issues else "failed",
        "path": str(path) if path is not None else None,
        "tick_count": len(rows),
        "failed_tick_count": failed_rows,
        "frame_ids": frame_ids,
        "issues": sorted(set(issues)),
    }


__all__ = ["M8PromotionError", "evaluate_m8_promotion"]
