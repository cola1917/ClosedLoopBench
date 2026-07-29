"""Low-cost preflight checks for a NuRec reconstruction.

The smoke gate is deliberately source/config-only.  It does not claim that a
USDZ renders correctly; it prevents an expensive formal run when the config
cannot represent the safety-relevant object registry in the first place.
"""

from __future__ import annotations

import json
from collections.abc import Iterable, Mapping, Sequence
from pathlib import Path
from typing import Any


class NuRecReconstructionSmokeError(ValueError):
    """Raised when smoke-gate inputs are malformed."""


def load_config(path: str | Path) -> dict[str, Any]:
    """Load JSON or YAML without accepting a non-object document."""

    source = Path(path).read_text(encoding="utf-8-sig")
    try:
        import yaml
    except ImportError:
        try:
            value = json.loads(source)
        except json.JSONDecodeError as exc:
            raise NuRecReconstructionSmokeError(
                "PyYAML is required to read a non-JSON NuRec config"
            ) from exc
    else:
        value = yaml.safe_load(source)
    if not isinstance(value, dict):
        raise NuRecReconstructionSmokeError("NuRec config must be a mapping")
    return value


def load_track_ids(path: str | Path) -> set[str]:
    """Read track IDs from NuRec sequence-track or datasource-summary JSON."""

    value = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    ids: set[str] = set()

    def visit(node: Any) -> None:
        if isinstance(node, Mapping):
            track_ids = node.get("tracks_id")
            if isinstance(track_ids, list):
                ids.update(str(item) for item in track_ids if str(item))
            for child in node.values():
                visit(child)
        elif isinstance(node, list):
            for child in node:
                visit(child)

    visit(value)
    if not ids:
        raise NuRecReconstructionSmokeError(
            "source track manifest contains no tracks_data.tracks_id values"
        )
    return ids


def audit_reconstruction_smoke(
    config: Mapping[str, Any],
    registry: Mapping[str, Any],
    *,
    source_track_ids: Iterable[str],
    expected_camera_ids: Sequence[str] | None = None,
    max_samples_per_epoch: int = 1000,
    max_epochs: int = 1,
) -> dict[str, Any]:
    """Return a fail-closed preflight report for a prospective reconstruction."""

    _validate_registry(registry)
    if max_samples_per_epoch < 1 or max_epochs < 1:
        raise NuRecReconstructionSmokeError("smoke limits must be positive")

    source_ids = {str(value) for value in source_track_ids if str(value)}
    if not source_ids:
        raise NuRecReconstructionSmokeError("source_track_ids must not be empty")

    camera_ids = _find(config, "camera_ids")
    camera_value = next((value for value in camera_ids if isinstance(value, list)), None)
    expected = [str(value) for value in (expected_camera_ids or camera_value or [])]
    actual_cameras = [str(value) for value in (camera_value or [])]

    samples = _first_int(config, "n_samples_per_epoch")
    epochs = _first_int(config, "max_epochs")
    sequence_enabled = _nested_bool(
        config, ("checkpoint", "artifact", "sequence_tracks", "enabled")
    )
    configured_track_ids = _configured_track_ids(config)
    static_generation_enabled = next(
        (
            value
            for path in (
                ("generate_static_rigid_cuboid_tracks", "enabled"),
                ("dataset", "generate_static_rigid_cuboid_tracks", "enabled"),
                ("data", "generate_static_rigid_cuboid_tracks", "enabled"),
            )
            if (value := _nested_bool(config, path)) is not None
        ),
        None,
    )

    dynamic_records = [
        record
        for record in registry["records"]
        if record.get("role")
        in {"background_replay", "controlled_lead_vehicle", "controlled_pedestrian"}
    ]
    required_dynamic_ids = {
        str((record.get("nurec") or {}).get("track_id"))
        for record in dynamic_records
        if str((record.get("nurec") or {}).get("track_id") or "")
    }
    static_records = [
        record
        for record in registry["records"]
        if record.get("role") == "static_obstacle" and record.get("safety_relevant") is True
    ]
    static_track_ids = {
        str((record.get("nurec") or {}).get("track_id"))
        for record in static_records
        if str((record.get("nurec") or {}).get("track_id") or "")
    }
    missing_static_track_ids = sorted(static_track_ids - configured_track_ids)
    untracked_static_count = len(static_records) - len(static_track_ids)
    missing_dynamic = sorted(required_dynamic_ids - source_ids)
    issues: list[str] = []
    checks: dict[str, Any] = {}

    checks["camera_set"] = {
        "expected": expected,
        "actual": actual_cameras,
        "status": "passed" if expected and actual_cameras == expected else "failed",
    }
    if checks["camera_set"]["status"] != "passed":
        issues.append("camera_set_missing_or_mismatched")

    checks["training_budget"] = {
        "n_samples_per_epoch": samples,
        "max_epochs": epochs,
        "max_samples_per_epoch": max_samples_per_epoch,
        "max_epochs_allowed": max_epochs,
        "status": "passed"
        if samples is not None
        and epochs is not None
        and 0 < samples <= max_samples_per_epoch
        and 0 < epochs <= max_epochs
        else "failed",
    }
    if checks["training_budget"]["status"] != "passed":
        issues.append("formal_training_budget_would_run_before_smoke")

    checks["sequence_track_export"] = {
        "enabled": sequence_enabled,
        "status": "passed" if sequence_enabled is True else "failed",
    }
    if checks["sequence_track_export"]["status"] != "passed":
        issues.append("sequence_track_export_disabled")

    checks["dynamic_track_coverage"] = {
        "required_count": len(required_dynamic_ids),
        "source_count": len(source_ids),
        "missing_track_ids": missing_dynamic,
        "status": "passed" if not missing_dynamic else "failed",
    }
    if missing_dynamic:
        issues.append("registered_dynamic_track_missing_from_source")

    checks["static_object_representation"] = {
        "required_count": len(static_records),
        "generation_enabled": static_generation_enabled,
        "configured_track_count": len(configured_track_ids),
        "static_track_id_count": len(static_track_ids),
        "missing_static_track_ids": missing_static_track_ids,
        "untracked_static_count": untracked_static_count,
        "status": "passed"
        if not static_records
        or static_generation_enabled is True
        or (not missing_static_track_ids and untracked_static_count == 0)
        else "failed",
    }
    if static_records and checks["static_object_representation"]["status"] != "passed":
        if static_generation_enabled is not True and untracked_static_count:
            issues.append("static_object_generation_disabled")
        if missing_static_track_ids:
            issues.append("registered_static_track_missing_from_nurec_layers")
        if untracked_static_count:
            issues.append("static_objects_have_no_source_track_or_generation_path")

    return {
        "schema_version": "nurec_reconstruction_smoke.v1",
        "status": "passed" if not issues else "failed",
        "scene_id": str(registry["scene_id"]),
        "checks": checks,
        "summary": {
            "required_dynamic_track_count": len(required_dynamic_ids),
            "source_track_count": len(source_ids),
            "required_static_obstacle_count": len(static_records),
            "configured_track_count": len(configured_track_ids),
            "issue_count": len(issues),
        },
        "issues": issues,
    }


def _validate_registry(registry: Mapping[str, Any]) -> None:
    if registry.get("schema_version") != "scene_object_registry.v1":
        raise NuRecReconstructionSmokeError(
            "registry must use scene_object_registry.v1"
        )
    if not str(registry.get("scene_id") or ""):
        raise NuRecReconstructionSmokeError("registry scene_id is required")
    if not isinstance(registry.get("records"), list):
        raise NuRecReconstructionSmokeError("registry.records must be a list")


def _find(node: Any, key: str) -> list[Any]:
    found: list[Any] = []
    if isinstance(node, Mapping):
        for child_key, child in node.items():
            if child_key == key:
                found.append(child)
            found.extend(_find(child, key))
    elif isinstance(node, list):
        for child in node:
            found.extend(_find(child, key))
    return found


def _first_int(config: Mapping[str, Any], key: str) -> int | None:
    for value in _find(config, key):
        if isinstance(value, bool):
            continue
        try:
            return int(value)
        except (TypeError, ValueError):
            continue
    return None


def _nested_bool(node: Any, path: tuple[str, ...]) -> bool | None:
    current = node
    for key in path:
        if not isinstance(current, Mapping) or key not in current:
            return None
        current = current[key]
    return current if isinstance(current, bool) else None


def _configured_track_ids(config: Mapping[str, Any]) -> set[str]:
    layers = ((config.get("model") or {}).get("layers") if isinstance(config, Mapping) else None)
    if not isinstance(layers, Mapping):
        return set()
    result: set[str] = set()
    for layer_name in ("dynamic_rigids", "dynamic_deformables"):
        layer = layers.get(layer_name)
        tracks = layer.get("tracks") if isinstance(layer, Mapping) else None
        ids = tracks.get("ids") if isinstance(tracks, Mapping) else None
        if isinstance(ids, list):
            result.update(str(value) for value in ids if str(value))
    return result
