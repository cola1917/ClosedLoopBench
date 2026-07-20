"""Build and validate the honest, evidence-backed scene-0061 video shot list."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

from runtime.render_quality import EVIDENCE_CLASSIFICATIONS


VIDEO_CAPABILITIES = {
    "original_replay",
    "lead_slowdown_hard_brake",
    "pedestrian_early_crossing",
    "pedestrian_yield_abort",
    "carla_bbox_state_window",
    "six_camera_nurec_grid",
    "lidar_inset",
    "kpi_overlay",
    "frame_timestamp_sync",
    "black_hole_quality_stress",
    "algorithm_identity",
    "baseline_vs_edit_comparison",
}
AVAILABILITY_VALUES = {"available", "partial", "missing"}
SCENE_ID = "cc8c0bf57f984915a77078b10eb33198"
ARTIFACT_SHA256 = "69e2c36e31e113f9ad66968a0a0a4243c7989dae5582a91a43d150051eaf98b4"


class VideoManifestError(ValueError):
    """Raised when a video manifest is incomplete or overclaims evidence."""


def build_scene0061_video_manifest(
    evidence_root: Path, *, created_at: str | None = None
) -> dict[str, Any]:
    root = evidence_root.resolve()
    shots = [_materialize_shot(root, shot) for shot in _shot_plan()]
    manifest = {
        "schema_version": "scene0061_video_manifest.v1",
        "status": "capture_plan",
        "created_at": created_at or datetime.now(timezone.utc).isoformat(),
        "scene_id": SCENE_ID,
        "artifact_sha256": ARTIFACT_SHA256,
        "evidence_root": str(root),
        "purpose": (
            "Plan an evidence-backed project video; missing remote footage remains explicit "
            "and is never synthesized by this manifest."
        ),
        "required_capabilities": sorted(VIDEO_CAPABILITIES),
        "shots": shots,
        "remote_capture_queue": [
            shot["shot_id"] for shot in shots if shot["remote_capture_required"]
        ],
        "remote_validation_required": any(
            shot["remote_capture_required"] for shot in shots
        ),
        "availability_summary": {
            name: sum(shot["current_availability"] == name for shot in shots)
            for name in sorted(AVAILABILITY_VALUES)
        },
        "limitations": [
            "Existing screenshots and replay video do not prove a continuous interactive closed loop.",
            "Missing shots remain remote_capture_required and cannot be replaced by offline/fake outputs.",
            "Vehicle-removal black-hole material is quality_stress evidence, not a perception ranking input.",
        ],
    }
    validate_scene0061_video_manifest(manifest, evidence_root=root)
    return manifest


def validate_scene0061_video_manifest(
    manifest: Mapping[str, Any], *, evidence_root: Path | None = None
) -> None:
    if not isinstance(manifest, Mapping) or manifest.get("schema_version") != "scene0061_video_manifest.v1":
        raise VideoManifestError("schema_version must be scene0061_video_manifest.v1")
    if manifest.get("status") != "capture_plan":
        raise VideoManifestError("video manifest status must be capture_plan")
    if manifest.get("scene_id") != SCENE_ID:
        raise VideoManifestError("video manifest scene_id is not scene-0061")
    if manifest.get("artifact_sha256") != ARTIFACT_SHA256:
        raise VideoManifestError("video manifest artifact sha256 is not the formal 40k artifact")
    if manifest.get("required_capabilities") != sorted(VIDEO_CAPABILITIES):
        raise VideoManifestError("required_capabilities does not match the mandatory shot set")
    shots = manifest.get("shots")
    if not isinstance(shots, list) or not shots:
        raise VideoManifestError("video manifest requires shots")
    ids: set[str] = set()
    capabilities: set[str] = set()
    root = evidence_root.resolve() if evidence_root is not None else None
    for shot in shots:
        if not isinstance(shot, Mapping):
            raise VideoManifestError("each video shot must be an object")
        shot_id = _nonempty(shot.get("shot_id"), "shot_id")
        if shot_id in ids:
            raise VideoManifestError(f"duplicate shot_id: {shot_id}")
        ids.add(shot_id)
        capability = _nonempty(shot.get("capability"), f"{shot_id}.capability")
        if capability not in VIDEO_CAPABILITIES:
            raise VideoManifestError(f"{shot_id}: unsupported capability {capability}")
        if capability in capabilities:
            raise VideoManifestError(f"duplicate capability shot: {capability}")
        capabilities.add(capability)
        for field in ("scene_id", "case_id", "algorithm_id"):
            _nonempty(shot.get(field), f"{shot_id}.{field}")
        if shot["scene_id"] != SCENE_ID:
            raise VideoManifestError(f"{shot_id}: scene_id mismatch")
        frame_range = shot.get("frame_range")
        if not isinstance(frame_range, Mapping) or set(frame_range) != {"start", "end"}:
            raise VideoManifestError(f"{shot_id}: frame_range requires start/end")
        start, end = frame_range["start"], frame_range["end"]
        if (start is None) != (end is None):
            raise VideoManifestError(f"{shot_id}: frame range must be fully known or fully unknown")
        if start is not None and (
            isinstance(start, bool)
            or isinstance(end, bool)
            or not isinstance(start, int)
            or not isinstance(end, int)
            or start < 0
            or end < start
        ):
            raise VideoManifestError(f"{shot_id}: invalid frame range")
        _string_list(shot.get("required_overlays"), f"{shot_id}.required_overlays")
        _string_list(shot.get("expected_kpis"), f"{shot_id}.expected_kpis")
        if not isinstance(shot.get("remote_capture_required"), bool):
            raise VideoManifestError(f"{shot_id}: remote_capture_required must be boolean")
        if shot.get("current_availability") not in AVAILABILITY_VALUES:
            raise VideoManifestError(f"{shot_id}: current_availability is invalid")
        if shot.get("evidence_classification") not in EVIDENCE_CLASSIFICATIONS:
            raise VideoManifestError(f"{shot_id}: evidence_classification is invalid")
        paths = _shot_paths(shot)
        if shot["current_availability"] != "missing" and not paths:
            raise VideoManifestError(f"{shot_id}: available/partial shot requires evidence paths")
        if root is not None:
            actual = _availability(root, paths)
            if actual != shot["current_availability"]:
                raise VideoManifestError(
                    f"{shot_id}: claimed availability {shot['current_availability']} "
                    f"does not match filesystem {actual}"
                )
        if capability == "black_hole_quality_stress" and shot["evidence_classification"] not in {
            "quality_stress",
            "rejected",
        }:
            raise VideoManifestError("black-hole shot must remain quality_stress/rejected")
        if shot["current_availability"] != "available" and not shot["remote_capture_required"]:
            raise VideoManifestError(
                f"{shot_id}: incomplete evidence must remain in the remote capture queue"
            )

    missing = sorted(VIDEO_CAPABILITIES - capabilities)
    if missing:
        raise VideoManifestError(f"video manifest is missing capabilities: {missing}")
    queue = manifest.get("remote_capture_queue")
    expected_queue = [shot["shot_id"] for shot in shots if shot["remote_capture_required"]]
    if queue != expected_queue:
        raise VideoManifestError("remote_capture_queue does not match shot flags")
    if manifest.get("remote_validation_required") is not bool(expected_queue):
        raise VideoManifestError("remote_validation_required does not match the capture queue")
    expected_summary = {
        name: sum(shot["current_availability"] == name for shot in shots)
        for name in sorted(AVAILABILITY_VALUES)
    }
    if manifest.get("availability_summary") != expected_summary:
        raise VideoManifestError("availability_summary does not match shots")


def _materialize_shot(root: Path, planned: Mapping[str, Any]) -> dict[str, Any]:
    paths = list(planned["paths"])
    availability = _availability(root, paths)
    return {
        "shot_id": planned["shot_id"],
        "capability": planned["capability"],
        "scene_id": SCENE_ID,
        "case_id": planned["case_id"],
        "algorithm_id": planned["algorithm_id"],
        "frame_range": planned["frame_range"],
        "source_evidence_path": paths[0] if paths else None,
        "supporting_evidence_paths": paths[1:],
        "required_overlays": planned["required_overlays"],
        "expected_kpis": planned["expected_kpis"],
        "remote_capture_required": bool(planned["remote_capture_required"]),
        "current_availability": availability,
        "availability_evidence": {
            "existing_paths": [path for path in paths if (root / path).is_file()],
            "missing_paths": [path for path in paths if not (root / path).is_file()],
        },
        "evidence_classification": planned["evidence_classification"],
        "notes": planned["notes"],
    }


def _availability(root: Path, paths: Sequence[str]) -> str:
    if not paths:
        return "missing"
    count = sum((root / path).is_file() for path in paths)
    if count == len(paths):
        return "available"
    if count:
        return "partial"
    return "missing"


def _shot_paths(shot: Mapping[str, Any]) -> list[str]:
    primary = shot.get("source_evidence_path")
    supporting = shot.get("supporting_evidence_paths", [])
    if primary is not None and not isinstance(primary, str):
        raise VideoManifestError("source_evidence_path must be a string or null")
    if not isinstance(supporting, list) or not all(isinstance(path, str) for path in supporting):
        raise VideoManifestError("supporting_evidence_paths must be strings")
    return ([primary] if primary else []) + list(supporting)


def _shot_plan() -> list[dict[str, Any]]:
    remote = "local_development/remote_capture"
    dual = "formal_acceptance/dual_window.formal40k_v4"
    return [
        _plan(
            "V01_original_replay",
            "original_replay",
            "S0_original_replay",
            "replay_baseline",
            {"start": 0, "end": 576},
            ["handoff/scene0061_formal40k_v1_six_view_centered.mp4"],
            ["camera_names", "frame_id", "timestamp", "fps"],
            ["sensor_drop_count", "synchronization_error"],
            False,
            "control_only",
            "Existing six-camera replay baseline; it does not prove interactive control.",
        ),
        _plan(
            "V02_lead_slowdown_hard_brake",
            "lead_slowdown_hard_brake",
            "S1_lead_slowdown_or_S2_lead_hard_brake",
            "transfuserpp_v5",
            {"start": None, "end": None},
            [
                f"{remote}/transfuserpp/S2_lead_hard_brake/baseline_vs_edit.synced.mp4",
                f"{remote}/transfuserpp/S2_lead_hard_brake/counterfactual_comparison.json",
            ],
            ["baseline_vs_edit", "BEV_dynamic_proxy", "waypoints", "target_speed", "ego_control", "TTC"],
            ["route_completion", "min_ttc", "hard_brake_event_count", "collision_count"],
            True,
            "rejected",
            "No formal interactive footage exists yet.",
        ),
        _plan(
            "V03_pedestrian_early_crossing",
            "pedestrian_early_crossing",
            "S4_pedestrian_early_crossing",
            "transfuserpp_v5",
            {"start": None, "end": None},
            [
                f"{remote}/transfuserpp/S4_pedestrian_early_crossing/baseline_vs_edit.synced.mp4",
                f"{remote}/transfuserpp/S4_pedestrian_early_crossing/counterfactual_comparison.json",
            ],
            ["pedestrian_track_id", "CARLA_actor_id", "BEV_walker_proxy", "bbox", "target_speed", "ego_control"],
            ["min_ttc", "PET", "collision_count", "crossing_outcome"],
            True,
            "rejected",
            "Must show the mapped existing pedestrian, not a fabricated new actor.",
        ),
        _plan(
            "V04_pedestrian_yield_abort",
            "pedestrian_yield_abort",
            "S5_pedestrian_yield",
            "transfuserpp_v5",
            {"start": None, "end": None},
            [f"{remote}/pedestrian_yield_abort.synced.mp4"],
            ["actor_decision", "pedestrian_track_id", "TTC", "short_trajectory"],
            ["yield_outcome", "abort_outcome", "min_distance", "route_completion"],
            True,
            "rejected",
            "Requires a continuous interactive actor decision trace.",
        ),
        _plan(
            "V05_carla_bbox_state_window",
            "carla_bbox_state_window",
            "S0_original_replay",
            "state_explanation_only",
            {"start": 38, "end": 38},
            [f"{dual}/frame_00038.baseline.carla.png", f"{dual}/actor_mapping.json"],
            ["actor_bbox", "CARLA_actor_id", "NuRec_track_id", "actor_type", "speed"],
            ["actor_mapping_coverage"],
            False,
            "control_only",
            "Static state-explanation evidence; not a camera sensor output.",
        ),
        _plan(
            "V06_six_camera_nurec_grid",
            "six_camera_nurec_grid",
            "S0_original_replay",
            "replay_baseline",
            {"start": 0, "end": 576},
            ["handoff/scene0061_formal40k_v1_six_view_centered.mp4"],
            ["camera_name", "frame_id", "timestamp", "fps", "3x2_centered_layout"],
            ["camera_fps", "dropped_frames", "synchronization_error"],
            False,
            "control_only",
            "Raw NuRec RGB remains free of bbox overlays by default.",
        ),
        _plan(
            "V07_lidar_inset",
            "lidar_inset",
            "S1_lead_slowdown",
            "remote_algorithm_under_test",
            {"start": None, "end": None},
            [
                f"{remote}/transfuserpp/lidar_actor_change.synced.mp4",
                "diagnostics/lidar-probes/replay.formal40k_v1.vehicle_pose_probe.v2.json",
            ],
            ["lidar_top", "actor_track_id", "frame_id", "timestamp"],
            ["point_count", "RGB_LiDAR_actor_change_consistency", "synchronization_error"],
            True,
            "control_only",
            "The JSON proves a pose probe; a synchronized dynamic point-cloud inset is still missing.",
        ),
        _plan(
            "V08_kpi_overlay",
            "kpi_overlay",
            "S1_to_S6_summary",
            "transfuserpp_v5",
            {"start": None, "end": None},
            [
                f"{remote}/transfuserpp/formal_matrix_summary.json",
                f"{remote}/transfuserpp/S0_original_replay/final_triplicate/acceptance_triplicate.json",
            ],
            ["route_completion", "collision", "TTC", "PET", "latency", "sync"],
            ["triplicate_mean", "triplicate_std", "failure_rate"],
            True,
            "rejected",
            "No KPI is displayed until all required formal reports exist and are comparable.",
        ),
        _plan(
            "V09_frame_timestamp_sync",
            "frame_timestamp_sync",
            "S0_original_replay",
            "state_explanation_only",
            {"start": 38, "end": 38},
            [f"{dual}/dual_window_report.json"],
            ["frame_id", "simulation_timestamp", "synchronization_error"],
            ["dropped_frames", "max_sync_error_us"],
            False,
            "control_only",
            "Single-frame synchronization evidence only.",
        ),
        _plan(
            "V10_black_hole_quality_stress",
            "black_hole_quality_stress",
            "S7_lead_removed_quality_stress",
            "quality_stress_only",
            {"start": None, "end": None},
            [f"{remote}/lead_removed_black_hole_ab.mp4", f"{remote}/lead_removed_quality_report.json"],
            ["actor_ROI", "hole_ratio", "quality_classification", "limitation_label"],
            ["actor_roi_hole_ratio", "temporal_flicker", "unchanged_background_stability"],
            True,
            "quality_stress",
            "Vehicle removal is never admitted to perception ranking by default.",
        ),
        _plan(
            "V11_algorithm_identity",
            "algorithm_identity",
            "S1_to_S6_summary",
            "transfuserpp_v5",
            {"start": None, "end": None},
            [
                f"{remote}/transfuserpp/runtime_manifest.json",
                f"{remote}/transfuserpp/intermediate_trace_evaluation.json",
                f"{remote}/transfuserpp/HASHES.sha256.json",
            ],
            ["algorithm_id", "repo_hash", "checkpoint_hash", "config_hash", "inference_latency"],
            ["control_latency_p50", "control_latency_p95", "control_latency_p99"],
            True,
            "rejected",
            "Offline conformance identity cannot replace the formal remote runtime identity.",
        ),
        _plan(
            "V12_baseline_vs_edit_comparison",
            "baseline_vs_edit_comparison",
            "S0_original_replay_vs_S2_lead_hard_brake",
            "transfuserpp_v5",
            {"start": None, "end": None},
            [
                f"{remote}/transfuserpp/S2_lead_hard_brake/baseline_vs_edit.synced.mp4",
                f"{remote}/transfuserpp/S2_lead_hard_brake/counterfactual_comparison.json",
                f"{remote}/transfuserpp/S2_lead_hard_brake/render_quality_report.json",
            ],
            ["baseline", "edited", "same_frame_id", "actor_track_id"],
            ["RGB_actor_change", "LiDAR_actor_change", "synchronization_error"],
            True,
            "rejected",
            "Must bind the same artifact/checkpoint/seed and show RGB/LiDAR edit, BEV/box, planning, control, and KPI response.",
        ),
    ]


def _plan(
    shot_id: str,
    capability: str,
    case_id: str,
    algorithm_id: str,
    frame_range: dict[str, int | None],
    paths: list[str],
    overlays: list[str],
    kpis: list[str],
    remote: bool,
    classification: str,
    notes: str,
) -> dict[str, Any]:
    return {
        "shot_id": shot_id,
        "capability": capability,
        "case_id": case_id,
        "algorithm_id": algorithm_id,
        "frame_range": frame_range,
        "paths": paths,
        "required_overlays": overlays,
        "expected_kpis": kpis,
        "remote_capture_required": remote,
        "evidence_classification": classification,
        "notes": notes,
    }


def _nonempty(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise VideoManifestError(f"{name} must be a non-empty string")
    return value


def _string_list(value: Any, name: str) -> list[str]:
    if not isinstance(value, list) or not value or not all(isinstance(item, str) and item for item in value):
        raise VideoManifestError(f"{name} must be a non-empty string list")
    return value
