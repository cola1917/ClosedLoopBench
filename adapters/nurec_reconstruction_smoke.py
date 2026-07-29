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


def _validate_lidar_quality_windows(
    manifest: Mapping[str, Any],
    *,
    scene_id: str,
    registry_ids: set[str],
    selected_object_ids: set[str] | None,
) -> dict[str, Any]:
    """Validate editable-quality-window source evidence.

    This only validates candidate-input provenance.  It never removes a
    registry object or changes the CARLA collision/lane requirements.
    """

    if manifest.get("schema_version") != "lidar_quality_window_manifest.v1":
        raise NuRecReconstructionSmokeError(
            "LiDAR quality window manifest must use lidar_quality_window_manifest.v1"
        )
    if str(manifest.get("scene_id") or "") != str(scene_id):
        raise NuRecReconstructionSmokeError("LiDAR quality window manifest scene_id does not match registry")
    if manifest.get("status") != "passed":
        raise NuRecReconstructionSmokeError("LiDAR quality window manifest is not passed")
    semantics = manifest.get("window_semantics")
    if not isinstance(semantics, Mapping) or semantics.get("name") != "editable_quality_window":
        raise NuRecReconstructionSmokeError(
            "LiDAR quality window manifest must declare editable_quality_window semantics"
        )
    if semantics.get("lidar_world_closed_loop_claim_allowed_only_inside_window") is not True:
        raise NuRecReconstructionSmokeError(
            "LiDAR quality window manifest must scope closure claims to editable windows"
        )
    policy = manifest.get("policy")
    if not isinstance(policy, Mapping) or policy.get("quality_is_not_a_carla_physics_filter") is not True:
        raise NuRecReconstructionSmokeError(
            "LiDAR quality window manifest must declare quality_is_not_a_carla_physics_filter"
        )
    candidate = {str(value) for value in manifest.get("candidate_object_ids") or [] if str(value)}
    required = {str(value) for value in manifest.get("required_object_ids") or [] if str(value)}
    if not candidate or not required.issubset(candidate):
        raise NuRecReconstructionSmokeError(
            "LiDAR quality window manifest requires non-empty candidate IDs and subset required IDs"
        )
    if not candidate.issubset(registry_ids):
        raise NuRecReconstructionSmokeError("LiDAR quality window manifest contains unknown registry objects")
    if selected_object_ids is not None and not candidate.issubset(selected_object_ids):
        raise NuRecReconstructionSmokeError(
            "LiDAR quality window candidates must be included in render selection"
        )
    tracks = manifest.get("tracks")
    if not isinstance(tracks, list):
        raise NuRecReconstructionSmokeError("LiDAR quality window manifest tracks are required")
    by_object = {str(row.get("object_id")): row for row in tracks if isinstance(row, Mapping)}
    missing = sorted(
        object_id
        for object_id in required
        if not (by_object.get(object_id) or {}).get("editable_windows")
    )
    if missing:
        raise NuRecReconstructionSmokeError(
            "LiDAR quality window required objects have no editable window: " + ", ".join(missing)
        )
    return {
        "status": "passed",
        "candidate_object_count": len(candidate),
        "required_object_count": len(required),
        "editable_window_count": sum(
            len((by_object.get(object_id) or {}).get("editable_windows") or [])
            for object_id in candidate
        ),
    }


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
    render_selection: Mapping[str, Any] | None = None,
    lidar_quality_windows: Mapping[str, Any] | None = None,
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

    selected_object_ids: set[str] | None = None
    selection_status = "not_provided"
    if render_selection is not None:
        if render_selection.get("schema_version") != "nurec_render_selection.v1":
            raise NuRecReconstructionSmokeError("render selection must use nurec_render_selection.v1")
        if str(render_selection.get("scene_id") or "") != str(registry["scene_id"]):
            raise NuRecReconstructionSmokeError("render selection scene_id does not match registry")
        if render_selection.get("status") != "passed":
            raise NuRecReconstructionSmokeError("render selection is not passed")
        raw_selected = render_selection.get("selected_object_ids")
        if not isinstance(raw_selected, list) or not raw_selected:
            raise NuRecReconstructionSmokeError("render selection requires selected_object_ids")
        selected_object_ids = {str(value) for value in raw_selected if str(value)}
        registry_ids = {str(record["object_id"]) for record in registry["records"]}
        if not selected_object_ids.issubset(registry_ids):
            raise NuRecReconstructionSmokeError("render selection contains an unknown registry object")
        selection_status = "passed"

    lidar_quality_status: dict[str, Any] = {"status": "not_provided"}
    if lidar_quality_windows is not None:
        lidar_quality_status = _validate_lidar_quality_windows(
            lidar_quality_windows,
            scene_id=str(registry["scene_id"]),
            registry_ids={str(record["object_id"]) for record in registry["records"]},
            selected_object_ids=selected_object_ids,
        )

    def selected(record: Mapping[str, Any]) -> bool:
        return selected_object_ids is None or str(record["object_id"]) in selected_object_ids

    dynamic_records = [
        record
        for record in registry["records"]
        if record.get("role")
        in {"background_replay", "controlled_lead_vehicle", "controlled_pedestrian"}
        and selected(record)
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
        and selected(record)
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

    checks["editable_quality_windows"] = {
        "status": lidar_quality_status["status"],
        "required": lidar_quality_windows is not None,
        "candidate_object_count": lidar_quality_status.get("candidate_object_count"),
        "required_object_count": lidar_quality_status.get("required_object_count"),
        "editable_window_count": lidar_quality_status.get("editable_window_count"),
    }
    if lidar_quality_windows is not None and lidar_quality_status["status"] != "passed":
        issues.append("editable_quality_window_manifest_failed")

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
        "editable_quality_windows": lidar_quality_status,
        "render_selection": {
            "status": selection_status,
            "selected_object_count": len(selected_object_ids) if selected_object_ids is not None else None,
        },
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
