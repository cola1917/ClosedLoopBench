from __future__ import annotations

import hashlib
import math
from copy import deepcopy
from pathlib import Path, PurePosixPath
from statistics import fmean
from typing import Any, Iterable, Mapping

from agents.transfuserpp_contract import (
    REQUIRED_DENSE_KEYS,
    TransFuserPPContractError,
    validate_intermediate_record,
)
from runtime.render_quality import (
    RenderQualityError,
    formal_perception_quality_problems,
    validate_render_quality_report,
)


TARGET_TRACK_BY_CASE = {
    "S2_lead_hard_brake": "c1958768d48640948f6053d04cffd35b",
    "S4_pedestrian_early_crossing": "71603dd1a2ba4e9daf095535e38310ac",
}

class TransFuserPPIntermediateError(ValueError):
    """Raised when intermediate outputs cannot be compared without guessing."""


def evaluate_intermediate_trace(
    records: Iterable[Mapping[str, Any]],
    *,
    render_quality_classification: str | None = None,
    render_quality_report: Mapping[str, Any] | None = None,
    evidence_root: Path | None = None,
) -> dict[str, Any]:
    rows = _validated_rows(records, evidence_root=evidence_root)
    problems = _trace_integrity_problems(rows)
    if rows:
        for row in rows:
            dense_problem = _dense_ref_problem(row)
            if dense_problem:
                problems.append(f"dense_output_invalid:{row['frame_id']}:{dense_problem}")

    latencies = [float(row["latency_ms"]["inference"]) for row in rows]
    target_speeds = [float(row["outputs"]["target_speed_mps"]) for row in rows]
    brakes = [float(row["outputs"]["control"]["brake"]) for row in rows]
    speed_confidences = [
        max(float(value) for value in row["outputs"]["target_speed_probabilities"])
        for row in rows
    ]
    speed_entropies = [
        -sum(
            float(value) * math.log(max(float(value), 1e-12))
            for value in row["outputs"]["target_speed_probabilities"]
        )
        for row in rows
    ]
    proxy_samples = [
        sample
        for row in rows
        for sample in (row.get("dynamic_bev_proxy") or {}).get("actor_samples") or []
    ]
    available_matches = [
        bool(sample["center_class_match"])
        for sample in proxy_samples
        if sample.get("center_class_match") is not None
    ]
    detection = _box_proxy_metrics(rows)
    quality_classification, quality_problems, quality_identity = _quality_gate(
        rows,
        render_quality_report=render_quality_report,
        legacy_classification=render_quality_classification,
    )
    problems.extend(quality_problems)
    if quality_classification == "perception_eligible" and not problems:
        classification = "perception_eligible"
    elif quality_classification in {"quality_stress", "rejected"}:
        classification = quality_classification
    else:
        classification = "control_only"
    return {
        "schema_version": "transfuserpp_intermediate_evaluation.v1",
        "status": "evaluated" if rows and not problems else "failed",
        "evidence_classification": classification,
        "frame_count": len(rows),
        "identity": deepcopy(rows[0]["identity"]) if rows else None,
        "experiment": deepcopy(rows[0].get("experiment")) if rows else None,
        "latency_ms": _distribution(latencies),
        "target_speed_mps": _distribution(target_speeds),
        "target_speed_distribution": {
            "confidence": _distribution(speed_confidences),
            "entropy_nats": _distribution(speed_entropies),
            "bins_mps": deepcopy(rows[0]["outputs"]["target_speed_bins_mps"]) if rows else None,
        },
        "brake": {
            "active_frame_count": sum(value >= 0.5 for value in brakes),
            "active_frame_ratio": (
                sum(value >= 0.5 for value in brakes) / len(brakes) if brakes else None
            ),
            "first_active_timestamp": next(
                (
                    float(row["timestamp"])
                    for row in rows
                    if float(row["outputs"]["control"]["brake"]) >= 0.5
                ),
                None,
            ),
        },
        "dynamic_bev_proxy": {
            "scope": "actor_center_and_box_proxy_not_full_scene_occupancy",
            "center_sample_count": len(available_matches),
            "center_class_accuracy": (
                sum(available_matches) / len(available_matches)
                if available_matches
                else None
            ),
            "box_detection": detection,
        },
        "full_3d_occupancy": {
            "status": "unavailable",
            "reason": (
                "scene0061 has no matching dense voxel/free-space ground truth for the full NuRec scene"
            ),
            "voxel_miou": None,
            "ray_iou": None,
        },
        "render_quality_classification": quality_classification,
        "render_quality_identity": quality_identity,
        "fail_closed_reasons": sorted(set(problems)),
    }


def compare_counterfactual_traces(
    baseline_records: Iterable[Mapping[str, Any]],
    edited_records: Iterable[Mapping[str, Any]],
    *,
    event_timestamp: float | None = None,
    expected_case_id: str | None = None,
    evidence_root: Path | None = None,
    edited_evidence_root: Path | None = None,
) -> dict[str, Any]:
    baseline = _validated_rows(baseline_records, evidence_root=evidence_root)
    edited = _validated_rows(
        edited_records,
        evidence_root=edited_evidence_root or evidence_root,
    )
    problems = [
        f"baseline:{problem}" for problem in _trace_integrity_problems(baseline)
    ] + [f"edited:{problem}" for problem in _trace_integrity_problems(edited)]
    for label, rows in (("baseline", baseline), ("edited", edited)):
        for row in rows:
            dense_problem = _dense_ref_problem(row)
            if dense_problem:
                problems.append(
                    f"{label}:dense_output_invalid:{row['frame_id']}:{dense_problem}"
                )
    if event_timestamp is None:
        problems.append("event_timestamp_required")
    if not expected_case_id:
        problems.append("expected_case_id_required")
    if baseline and edited and _model_identity_key(baseline[0]) != _model_identity_key(edited[0]):
        problems.append("algorithm_checkpoint_or_config_identity_mismatch")
    if expected_case_id and edited:
        if (edited[0].get("experiment") or {}).get("case_id") != expected_case_id:
            problems.append("edited_case_id_mismatch")
    if baseline and (baseline[0].get("experiment") or {}).get("case_id") != "S0_original_replay":
        problems.append("baseline_case_must_be_s0_original_replay")
    if baseline and edited:
        _compare_experiment_identity(baseline[0], edited[0], problems)
    pairs = list(zip(baseline, edited))
    if len(baseline) != len(edited):
        problems.append("trace_length_mismatch")
    if pairs and any(
        abs(float(left["timestamp"]) - float(right["timestamp"])) > 0.001
        for left, right in pairs
    ):
        problems.append("trace_timestamp_alignment_exceeds_1ms")
    if event_timestamp is not None and pairs:
        timestamps = [float(left["timestamp"]) for left, _ in pairs]
        if not min(timestamps) < float(event_timestamp) < max(timestamps):
            problems.append("event_requires_pre_and_post_frames")

    target_speed_deltas = [
        float(right["outputs"]["target_speed_mps"])
        - float(left["outputs"]["target_speed_mps"])
        for left, right in pairs
    ]
    pre_event_speed_deltas = [
        delta
        for delta, (left, _) in zip(target_speed_deltas, pairs)
        if event_timestamp is not None and float(left["timestamp"]) < event_timestamp
    ]
    post_event_speed_deltas = [
        delta
        for delta, (left, _) in zip(target_speed_deltas, pairs)
        if event_timestamp is not None and float(left["timestamp"]) >= event_timestamp
    ]
    checkpoint_displacements = [
        _world_waypoint_displacement(
            left,
            right,
            left["outputs"]["route_checkpoints_ego_m"],
            right["outputs"]["route_checkpoints_ego_m"],
        )
        for left, right in pairs
    ]
    learned_waypoint_displacements = [
        _world_waypoint_displacement(
            left,
            right,
            left["outputs"]["waypoints_ego_m"],
            right["outputs"]["waypoints_ego_m"],
        )
        for left, right in pairs
    ]
    if any(value is None for value in checkpoint_displacements):
        problems.append("route_checkpoint_horizon_mismatch")
    if any(value is None for value in learned_waypoint_displacements):
        problems.append("learned_waypoint_horizon_mismatch")
    waypoint_displacements = [
        float(value) for value in checkpoint_displacements if value is not None
    ]
    learned_displacements = [
        float(value) for value in learned_waypoint_displacements if value is not None
    ]
    target_track = TARGET_TRACK_BY_CASE.get(expected_case_id or "")
    pre_event_equivalence = _pre_event_equivalence(
        pairs, event_timestamp=event_timestamp
    )
    problems.extend(pre_event_equivalence["problems"])
    dense = _dense_change_metrics(
        pairs,
        event_timestamp=event_timestamp,
        target_track=target_track,
    )
    if dense.get("status") != "evaluated":
        problems.append(f"bev_change_unavailable:{dense.get('reason', 'unknown')}")
    baseline_brake = _first_brake_timestamp(baseline, event_timestamp)
    edited_brake = _first_brake_timestamp(edited, event_timestamp)
    brake_response = (
        edited_brake - float(event_timestamp)
        if edited_brake is not None and event_timestamp is not None
        else None
    )
    response_timestamp = _first_response_timestamp(
        pairs, event_timestamp=event_timestamp
    )
    sensor_edit_observed = (
        dense.get("edited_region_change_ratio") is not None
        and float(dense["edited_region_change_ratio"]) > 0.0
    )
    route_planning_changed = (
        fmean(waypoint_displacements) > 0.05 if waypoint_displacements else False
    )
    learned_path_changed = (
        fmean(learned_displacements) > 0.05 if learned_displacements else False
    )
    target_speed_changed = any(
        abs(value) > 0.1 for value in post_event_speed_deltas
    )
    post_event_control_deltas = [
        {
            name: float(right["outputs"]["control"][name])
            - float(left["outputs"]["control"][name])
            for name in ("throttle", "steer", "brake")
        }
        for left, right in pairs
        if event_timestamp is not None and float(left["timestamp"]) >= event_timestamp
    ]
    differential_control_response = any(
        delta["brake"] >= 0.1 or delta["throttle"] <= -0.1
        for delta in post_event_control_deltas
    )
    if not sensor_edit_observed:
        problems.append("edited_target_not_observed_in_pose_aligned_bev")
    if not (route_planning_changed or learned_path_changed or target_speed_changed):
        problems.append("learned_planning_outputs_show_no_counterfactual_response")
    if not differential_control_response:
        problems.append("vehicle_control_shows_no_counterfactual_response")
    if expected_case_id == "S2_lead_hard_brake" and edited_brake is None:
        problems.append("s2_edited_trace_has_no_post_event_hard_brake")
    if expected_case_id == "S4_pedestrian_early_crossing" and response_timestamp is None:
        problems.append("s4_edited_trace_has_no_post_event_yield_response")
    if target_track:
        if not _target_proxy_available(edited, target_track):
            problems.append("edited_target_track_proxy_unavailable_or_out_of_bounds")
    return {
        "schema_version": "transfuserpp_counterfactual_intermediate_comparison.v1",
        "status": "evaluated" if pairs and not problems else "failed",
        "evidence_classification": "remote_validation_required",
        "baseline_experiment": deepcopy(baseline[0].get("experiment")) if baseline else None,
        "edited_experiment": deepcopy(edited[0].get("experiment")) if edited else None,
        "paired_frame_count": len(pairs),
        "identity_match": bool(
            baseline and edited and _model_identity_key(baseline[0]) == _model_identity_key(edited[0])
        ),
        "target_speed_delta_mps": _distribution(target_speed_deltas),
        "target_speed_event_analysis": {
            "pre_event_delta_mps": _distribution(pre_event_speed_deltas),
            "post_event_delta_mps": _distribution(post_event_speed_deltas),
            "pre_event_stable_within_0_1_mps": (
                all(abs(value) <= 0.1 for value in pre_event_speed_deltas)
                if pre_event_speed_deltas
                else None
            ),
            "post_event_nonincreasing": (
                any(value <= -0.1 for value in post_event_speed_deltas)
                if post_event_speed_deltas
                else None
            ),
        },
        "pre_event_equivalence": {
            key: value
            for key, value in pre_event_equivalence.items()
            if key != "problems"
        },
        "route_checkpoint_displacement_m": _distribution(waypoint_displacements),
        "learned_waypoint_displacement_m": _distribution(learned_displacements),
        "brake_response": {
            "event_timestamp": event_timestamp,
            "baseline_first_post_event_brake_timestamp": baseline_brake,
            "edited_first_post_event_brake_timestamp": edited_brake,
            "edited_response_latency_sec": brake_response,
            "first_counterfactual_response_timestamp": response_timestamp,
            "counterfactual_response_latency_sec": (
                response_timestamp - float(event_timestamp)
                if response_timestamp is not None and event_timestamp is not None
                else None
            ),
        },
        "bev_change": dense,
        "causal_chain": {
            "status": "passed"
            if sensor_edit_observed
            and (route_planning_changed or learned_path_changed or target_speed_changed)
            and differential_control_response
            else "failed",
            "sensor_edit_observed": sensor_edit_observed,
            "route_checkpoint_planning_changed": route_planning_changed,
            "learned_control_path_changed": learned_path_changed,
            "target_speed_changed": target_speed_changed,
            "differential_vehicle_control_response": differential_control_response,
            "control_braked_after_event": edited_brake is not None,
            "closed_loop_kpi_required_separately": True,
        },
        "full_3d_occupancy": {
            "status": "unavailable",
            "reason": "dynamic BEV comparison is not an Occ3D claim",
        },
        "fail_closed_reasons": sorted(set(problems)),
    }


def _validated_rows(
    records: Iterable[Mapping[str, Any]], *, evidence_root: Path | None = None
) -> list[dict[str, Any]]:
    result = []
    for index, record in enumerate(records):
        try:
            validated = validate_intermediate_record(record)
            _resolve_record_paths(validated, evidence_root=evidence_root)
            result.append(validated)
        except TransFuserPPContractError as exc:
            raise TransFuserPPIntermediateError(
                f"invalid intermediate frame at index {index}: {exc}"
            ) from exc
    return sorted(result, key=lambda row: (float(row["timestamp"]), int(row["frame_id"])))


def _resolve_record_paths(
    record: dict[str, Any], *, evidence_root: Path | None
) -> None:
    inputs = record.get("inputs") or {}
    for name in ("camera_front", "lidar_top"):
        reference = inputs.get(name)
        if isinstance(reference, dict):
            reference["path"] = str(
                _resolve_evidence_path(reference, evidence_root=evidence_root)
            )
    dense = record.get("dense_outputs")
    if isinstance(dense, dict):
        dense["path"] = str(
            _resolve_evidence_path(dense, evidence_root=evidence_root)
        )


def _resolve_evidence_path(
    reference: Mapping[str, Any], *, evidence_root: Path | None
) -> Path:
    declared_value = str(reference.get("path") or "")
    declared = Path(declared_value)
    if declared.is_file():
        return declared
    host_value = str(reference.get("host_path") or "")
    host_path = Path(host_value)
    if host_value and host_path.is_file():
        return host_path

    # Runtime records are written inside the container with /sim-data as the
    # mount root. The host evaluator receives the directory mounted there as
    # evidence_root, so sibling paths such as /sim-data/payloads and
    # /sim-data/transfuserpp_intermediates must be resolved from the same root.
    for value in (declared_value, host_value, str(reference.get("relative_path") or "")):
        candidate = _resolve_container_evidence_path(value, evidence_root)
        if candidate is not None:
            return candidate

    relative = str(reference.get("relative_path") or "").replace("\\", "/")
    relative_path = Path(relative)
    if (
        evidence_root is not None
        and relative
        and not relative_path.is_absolute()
        and ".." not in relative_path.parts
    ):
        root = evidence_root.resolve()
        candidate = (root / relative_path).resolve()
        try:
            candidate.relative_to(root)
        except ValueError:
            return declared
        if candidate.is_file():
            return candidate
    return declared


def _resolve_container_evidence_path(
    value: str, evidence_root: Path | None
) -> Path | None:
    if evidence_root is None or not value:
        return None
    declared = PurePosixPath(value.replace("\\", "/"))
    container_root = PurePosixPath("/sim-data")
    if not declared.is_absolute():
        return None
    try:
        relative = declared.relative_to(container_root)
    except ValueError:
        return None
    if ".." in relative.parts:
        return None
    root = evidence_root.resolve()
    candidate = (root / Path(*relative.parts)).resolve()
    try:
        candidate.relative_to(root)
    except ValueError:
        return None
    return candidate if candidate.is_file() else None


def _trace_integrity_problems(rows: list[Mapping[str, Any]]) -> list[str]:
    if not rows:
        return ["intermediate_trace_empty"]
    problems = []
    frame_ids = [int(row["frame_id"]) for row in rows]
    if any(current <= previous for previous, current in zip(frame_ids, frame_ids[1:])):
        problems.append("frame_ids_not_strictly_increasing")
    if any(current != previous + 1 for previous, current in zip(frame_ids, frame_ids[1:])):
        problems.append("sensor_frame_gap_detected")
    timestamps = [float(row["timestamp"]) for row in rows]
    if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
        problems.append("timestamps_not_strictly_increasing")
    if len({_identity_key(row) for row in rows}) != 1:
        problems.append("plugin_identity_changed_within_trace")
    if len({_experiment_key(row) for row in rows}) != 1:
        problems.append("experiment_identity_changed_within_trace")
    dynamic_hashes = {
        str((row.get("synchronization") or {}).get("dynamic_object_sha256") or "")
        for row in rows
    }
    if "" in dynamic_hashes:
        problems.append("dynamic_object_digest_missing")
    for row in rows:
        for name in ("camera_front", "lidar_top"):
            reference = (row.get("inputs") or {}).get(name) or {}
            path = Path(str(reference.get("path") or ""))
            if (
                not path.is_file()
                or _file_sha256(path) != str(reference.get("sha256") or "")
            ):
                problems.append(f"input_payload_invalid:{row['frame_id']}:{name}")
    return problems


def _identity_key(record: Mapping[str, Any]) -> tuple[str, ...]:
    identity = record["identity"]
    return (
        str(identity["repo_sha256"]),
        str(identity["checkpoint_sha256"]),
        str(identity["model_config_sha256"]),
        str(identity.get("repo_revision")),
        str(identity.get("runtime_config_sha256")),
        str(identity.get("carla_agents_sha256")),
        str(identity.get("adapter_source_sha256")),
        str(identity.get("container_image_digest")),
    )


def _model_identity_key(record: Mapping[str, Any]) -> tuple[str, ...]:
    identity = record["identity"]
    return (
        str(identity["repo_sha256"]),
        str(identity["checkpoint_sha256"]),
        str(identity["model_config_sha256"]),
        str(identity.get("repo_revision")),
        str(identity.get("carla_agents_sha256")),
        str(identity.get("adapter_source_sha256")),
        str(identity.get("container_image_digest")),
    )


def _experiment_key(record: Mapping[str, Any]) -> tuple[Any, ...]:
    experiment = record.get("experiment") or {}
    return tuple(
        experiment.get(name)
        for name in (
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
            "run_config_sha256",
        )
    )


def _compare_experiment_identity(
    baseline: Mapping[str, Any],
    edited: Mapping[str, Any],
    problems: list[str],
) -> None:
    left = baseline.get("experiment") or {}
    right = edited.get("experiment") or {}
    for name in (
        "scene_id",
        "scene_version",
        "seed",
        "artifact_sha256",
        "scene_package_sha256",
        "scenario_ir_sha256",
        "immutable_matrix_sha256",
        "source_run_config_sha256",
    ):
        if left.get(name) != right.get(name):
            problems.append(f"experiment_identity_mismatch:{name}")


def _quality_gate(
    rows: list[Mapping[str, Any]],
    *,
    render_quality_report: Mapping[str, Any] | None,
    legacy_classification: str | None,
) -> tuple[str, list[str], dict[str, Any] | None]:
    if render_quality_report is None:
        if legacy_classification in {"quality_stress", "rejected"}:
            return str(legacy_classification), [], None
        problems = (
            ["unbound_render_quality_classification_cannot_grant_perception_eligibility"]
            if legacy_classification == "perception_eligible"
            else []
        )
        return "unavailable", problems, None
    report = dict(render_quality_report)
    bound_ref = report.pop("_bound_report_ref", None)
    problems = []
    try:
        validate_render_quality_report(report)
    except RenderQualityError as exc:
        problems.append(f"render_quality_report_invalid:{exc}")
    if not isinstance(bound_ref, Mapping):
        problems.append("render_quality_report_file_binding_required")
    else:
        path = Path(str(bound_ref.get("path") or ""))
        digest = str(bound_ref.get("sha256") or "")
        if not path.is_file():
            problems.append("render_quality_report_file_missing")
        elif len(digest) != 64 or _file_sha256(path) != digest:
            problems.append("render_quality_report_sha256_mismatch")
    classification = str(report.get("evidence_classification") or "unavailable")
    if classification not in {
        "perception_eligible",
        "control_only",
        "quality_stress",
        "rejected",
    }:
        problems.append("render_quality_classification_invalid")
        classification = "unavailable"
    if rows:
        experiment = rows[0].get("experiment") or {}
        if report.get("scene_id") != experiment.get("scene_id"):
            problems.append("render_quality_scene_id_mismatch")
        if report.get("case_id") != experiment.get("case_id"):
            problems.append("render_quality_case_id_mismatch")
        if (report.get("artifact") or {}).get("sha256") != experiment.get(
            "artifact_sha256"
        ):
            problems.append("render_quality_artifact_mismatch")
    if classification == "perception_eligible":
        problems.extend(
            formal_perception_quality_problems(
                report,
                experiment=experiment if rows else {},
                source_report_ref=bound_ref,
            )
        )
    identity = {
        "scene_id": report.get("scene_id"),
        "case_id": report.get("case_id"),
        "artifact_sha256": (report.get("artifact") or {}).get("sha256"),
        "report_ref": deepcopy(dict(bound_ref)) if isinstance(bound_ref, Mapping) else None,
    }
    return classification, problems, identity


def _target_proxy_available(rows: list[Mapping[str, Any]], track_id: str) -> bool:
    for row in rows:
        for sample in (row.get("dynamic_bev_proxy") or {}).get("actor_samples") or []:
            if (
                sample.get("track_id") == track_id
                and sample.get("center_in_bev_bounds") is True
                and sample.get("predicted_center_bev_class") is not None
            ):
                return True
    return False


def _first_response_timestamp(
    pairs: list[tuple[Mapping[str, Any], Mapping[str, Any]]],
    *,
    event_timestamp: float | None,
) -> float | None:
    if event_timestamp is None:
        return None
    for left, right in pairs:
        timestamp = float(right["timestamp"])
        if timestamp < event_timestamp:
            continue
        speed_delta = float(right["outputs"]["target_speed_mps"]) - float(
            left["outputs"]["target_speed_mps"]
        )
        brake_delta = float(right["outputs"]["control"]["brake"]) - float(
            left["outputs"]["control"]["brake"]
        )
        if speed_delta <= -0.1 or brake_delta >= 0.2:
            return timestamp
    return None


def _dense_ref_problem(record: Mapping[str, Any]) -> str | None:
    value = record.get("dense_outputs") or {}
    path = Path(str(value.get("path") or ""))
    digest = str(value.get("sha256") or "")
    if not path.is_file():
        return "file_missing"
    if len(digest) != 64 or _file_sha256(path) != digest:
        return "sha256_mismatch"
    if value.get("encoding") != "numpy_npz":
        return "encoding_mismatch"
    try:
        import numpy as np

        with np.load(path) as dense:
            if any(key not in dense for key in REQUIRED_DENSE_KEYS):
                return "required_npz_key_missing"
            bev = dense["bev_semantic_labels"]
            perspective = dense["perspective_semantic_labels"]
            depth = dense["depth"]
            probabilities = dense["target_speed_probabilities"]
            grid = ((record.get("dynamic_bev_proxy") or {}).get("grid") or {})
            expected_shape = (grid.get("height"), grid.get("width"))
            if bev.ndim != 2 or tuple(bev.shape) != expected_shape:
                return "bev_shape_mismatch"
            if not np.issubdtype(bev.dtype, np.integer):
                return "bev_labels_not_integer"
            if perspective.ndim != 2 or not np.issubdtype(
                perspective.dtype, np.integer
            ):
                return "perspective_labels_invalid"
            if depth.ndim < 2 or depth.size == 0 or not np.isfinite(depth).all() or np.any(depth < 0):
                return "depth_invalid"
            expected_probabilities = np.asarray(
                (record.get("outputs") or {}).get("target_speed_probabilities") or [],
                dtype=np.float64,
            )
            if (
                probabilities.ndim != 1
                or probabilities.shape != expected_probabilities.shape
                or not np.isfinite(probabilities).all()
                or np.any(probabilities < 0)
                or abs(float(probabilities.sum()) - 1.0) > 1e-4
                or not np.allclose(
                    probabilities,
                    expected_probabilities,
                    rtol=1e-5,
                    atol=1e-6,
                )
            ):
                return "target_speed_probabilities_mismatch"
    except Exception:
        return "npz_unreadable"
    return None


def _box_proxy_metrics(rows: list[Mapping[str, Any]]) -> dict[str, Any]:
    distances = []
    matched = 0
    available = 0
    prediction_count = 0
    for row in rows:
        boxes = [
            box
            for box in (row["outputs"].get("bounding_boxes_ego") or [])
            if float(box[-1]) >= 0.1
        ]
        prediction_count += len(boxes)
        used: set[int] = set()
        for proxy in row.get("actor_proxies") or []:
            center = proxy.get("center_ego_m")
            actor_type = proxy.get("actor_type")
            if not isinstance(center, list) or len(center) < 2 or actor_type not in {"vehicle", "pedestrian"}:
                continue
            available += 1
            class_id = 0 if actor_type == "vehicle" else 1
            candidates = [
                (index, box)
                for index, box in enumerate(boxes)
                if index not in used
                if isinstance(box, list)
                and len(box) >= 9
                and int(round(float(box[7]))) == class_id
            ]
            if not candidates:
                continue
            scored = [
                (
                    index,
                    box,
                    math.hypot(
                        float(box[0]) - float(center[0]),
                        float(box[1]) - float(center[1]),
                    ),
                )
                for index, box in candidates
            ]
            index, _, distance = min(scored, key=lambda item: item[2])
            threshold = 4.0 if actor_type == "vehicle" else 2.0
            if distance <= threshold:
                matched += 1
                used.add(index)
                distances.append(distance)
    return {
        "proxy_count": available,
        "matched_count": matched,
        "recall": matched / available if available else None,
        "precision": matched / prediction_count if prediction_count else None,
        "prediction_count_after_confidence_gate": prediction_count,
        "confidence_gate": 0.1,
        "center_error_m": _distribution(distances),
        "matching": "greedy_unique_nearest_same_class_center_with_type_specific_gate",
    }


def _ego_pose_delta(
    left: Mapping[str, Any], right: Mapping[str, Any]
) -> tuple[float, float]:
    left_pose = (left.get("inputs") or {}).get("ego_pose") or {}
    right_pose = (right.get("inputs") or {}).get("ego_pose") or {}
    translation_m = math.hypot(
        float(right_pose.get("x", 0.0)) - float(left_pose.get("x", 0.0)),
        float(right_pose.get("y", 0.0)) - float(left_pose.get("y", 0.0)),
    )
    yaw_delta = (
        float(right_pose.get("yaw", 0.0))
        - float(left_pose.get("yaw", 0.0))
        + 180.0
    ) % 360.0 - 180.0
    return translation_m, abs(yaw_delta)


def _pre_event_equivalence(
    pairs: list[tuple[Mapping[str, Any], Mapping[str, Any]]],
    *,
    event_timestamp: float | None,
) -> dict[str, Any]:
    pre_pairs = [
        pair
        for pair in pairs
        if event_timestamp is not None and float(pair[0]["timestamp"]) < event_timestamp
    ]
    if not pre_pairs:
        return {
            "status": "failed",
            "paired_frame_count": 0,
            "problems": ["pre_event_equivalence_frames_required"],
        }
    problems: list[str] = []
    max_translation = 0.0
    max_yaw = 0.0
    max_speed_delta = 0.0
    max_control_delta = 0.0
    input_hash_mismatch_count = 0
    dynamic_state_mismatch_count = 0
    bev_change_pixels = 0
    bev_pixels = 0
    try:
        import numpy as np
    except Exception:
        return {
            "status": "failed",
            "paired_frame_count": len(pre_pairs),
            "problems": ["pre_event_equivalence_numpy_unavailable"],
        }
    for left, right in pre_pairs:
        translation_m, yaw_deg = _ego_pose_delta(left, right)
        max_translation = max(max_translation, translation_m)
        max_yaw = max(max_yaw, yaw_deg)
        max_speed_delta = max(
            max_speed_delta,
            abs(
                float(right["outputs"]["target_speed_mps"])
                - float(left["outputs"]["target_speed_mps"])
            ),
        )
        for name in ("throttle", "steer", "brake"):
            max_control_delta = max(
                max_control_delta,
                abs(
                    float(right["outputs"]["control"][name])
                    - float(left["outputs"]["control"][name])
                ),
            )
        for sensor_name in ("camera_front", "lidar_top"):
            left_sha = str((left.get("inputs") or {}).get(sensor_name, {}).get("sha256") or "")
            right_sha = str((right.get("inputs") or {}).get(sensor_name, {}).get("sha256") or "")
            if not left_sha or left_sha != right_sha:
                input_hash_mismatch_count += 1
        if str((left.get("synchronization") or {}).get("dynamic_object_sha256") or "") != str(
            (right.get("synchronization") or {}).get("dynamic_object_sha256") or ""
        ):
            dynamic_state_mismatch_count += 1
        try:
            with np.load(left["dense_outputs"]["path"]) as left_dense:
                left_bev = left_dense["bev_semantic_labels"]
            with np.load(right["dense_outputs"]["path"]) as right_dense:
                right_bev = right_dense["bev_semantic_labels"]
            if left_bev.shape != right_bev.shape:
                problems.append("pre_event_bev_shape_mismatch")
            else:
                bev_change_pixels += int(np.count_nonzero(left_bev != right_bev))
                bev_pixels += int(left_bev.size)
        except Exception:
            problems.append("pre_event_bev_unreadable")
    bev_change_ratio = bev_change_pixels / bev_pixels if bev_pixels else None
    if max_translation > 0.05:
        problems.append("pre_event_ego_translation_exceeds_0_05m")
    if max_yaw > 0.5:
        problems.append("pre_event_ego_yaw_exceeds_0_5deg")
    if max_speed_delta > 0.1:
        problems.append("pre_event_target_speed_delta_exceeds_0_1mps")
    if max_control_delta > 0.05:
        problems.append("pre_event_control_delta_exceeds_0_05")
    if input_hash_mismatch_count:
        problems.append("pre_event_sensor_payload_identity_mismatch")
    if dynamic_state_mismatch_count:
        problems.append("pre_event_dynamic_state_identity_mismatch")
    if bev_change_ratio is None or bev_change_ratio > 0.001:
        problems.append("pre_event_bev_change_exceeds_0_001")
    return {
        "status": "passed" if not problems else "failed",
        "paired_frame_count": len(pre_pairs),
        "max_ego_translation_delta_m": max_translation,
        "max_ego_yaw_delta_deg": max_yaw,
        "max_target_speed_delta_mps": max_speed_delta,
        "max_control_component_delta": max_control_delta,
        "sensor_payload_hash_mismatch_count": input_hash_mismatch_count,
        "dynamic_state_hash_mismatch_count": dynamic_state_mismatch_count,
        "bev_label_change_ratio": bev_change_ratio,
        "thresholds": {
            "ego_translation_m": 0.05,
            "ego_yaw_deg": 0.5,
            "target_speed_mps": 0.1,
            "control_component": 0.05,
            "bev_label_change_ratio": 0.001,
        },
        "problems": sorted(set(problems)),
    }


def _dense_change_metrics(
    pairs: list[tuple[Mapping[str, Any], Mapping[str, Any]]],
    *,
    event_timestamp: float | None,
    target_track: str | None,
) -> dict[str, Any]:
    try:
        import numpy as np
    except Exception:
        return {
            "status": "unavailable",
            "reason": "numpy_unavailable",
            "edited_region_change_ratio": None,
            "unchanged_background_stability": None,
        }
    region_changed = 0
    region_total = 0
    background_equal = 0
    background_total = 0
    aligned_frame_count = 0
    pose_divergent_frame_count = 0
    for left, right in pairs:
        if event_timestamp is None or float(right["timestamp"]) < event_timestamp:
            continue
        translation_m, yaw_deg = _ego_pose_delta(left, right)
        if translation_m > 0.1 or yaw_deg > 1.0:
            pose_divergent_frame_count += 1
            continue
        try:
            with np.load(left["dense_outputs"]["path"]) as left_dense:
                left_labels = left_dense["bev_semantic_labels"].copy()
            with np.load(right["dense_outputs"]["path"]) as right_dense:
                right_labels = right_dense["bev_semantic_labels"].copy()
        except Exception:
            return {
                "status": "unavailable",
                "reason": "dense_bev_labels_unreadable",
                "edited_region_change_ratio": None,
                "unchanged_background_stability": None,
            }
        if left_labels.shape != right_labels.shape:
            return {
                "status": "unavailable",
                "reason": "bev_shape_mismatch",
                "edited_region_change_ratio": None,
                "unchanged_background_stability": None,
            }
        aligned_frame_count += 1
        mask = _actor_roi_mask(
            np, right, right_labels.shape, target_track=target_track
        )
        changed = left_labels != right_labels
        region_changed += int(np.count_nonzero(changed & mask))
        region_total += int(np.count_nonzero(mask))
        background = ~mask
        background_equal += int(np.count_nonzero((~changed) & background))
        background_total += int(np.count_nonzero(background))
    status = "evaluated" if aligned_frame_count and region_total else "unavailable"
    reason = None
    if not aligned_frame_count:
        reason = "no_post_event_pose_aligned_pairs"
    elif not region_total:
        reason = "edited_target_proxy_roi_empty"
    return {
        "status": status,
        "reason": reason,
        "metric_scope": "target_actor_axis_aligned_proxy_roi_in_pose_aligned_post_event_predicted_bev_labels",
        "target_track_id": target_track,
        "post_event_pose_aligned_frame_count": aligned_frame_count,
        "post_event_pose_divergent_frame_count": pose_divergent_frame_count,
        "pose_alignment_threshold": {"translation_m": 0.1, "yaw_deg": 1.0},
        "target_roi_pixel_count": region_total,
        "edited_region_change_ratio": (
            region_changed / region_total if region_total else None
        ),
        "unchanged_background_stability": (
            background_equal / background_total if background_total else None
        ),
        "full_scene_occupancy_metric": False,
    }


def _actor_roi_mask(
    np: Any,
    record: Mapping[str, Any],
    shape: tuple[int, ...],
    *,
    target_track: str | None,
) -> Any:
    mask = np.zeros(shape, dtype=bool)
    proxy = record.get("dynamic_bev_proxy") or {}
    grid = proxy.get("grid") or {}
    min_x = float(grid.get("min_x_m", -32.0))
    max_x = float(grid.get("max_x_m", 32.0))
    min_y = float(grid.get("min_y_m", -32.0))
    max_y = float(grid.get("max_y_m", 32.0))
    height, width = shape[-2], shape[-1]
    for actor in record.get("actor_proxies") or []:
        if target_track is not None and actor.get("track_id") != target_track:
            continue
        center = actor.get("center_ego_m")
        extent = actor.get("extent_m") or {}
        if not isinstance(center, list) or len(center) < 2:
            continue
        half_x = max(0.75, float(extent.get("x", 1.0))) + 0.5
        half_y = max(0.5, float(extent.get("y", 0.5))) + 0.5
        row0 = int((float(center[0]) - half_x - min_x) / (max_x - min_x) * height)
        row1 = int((float(center[0]) + half_x - min_x) / (max_x - min_x) * height) + 1
        col0 = int((float(center[1]) - half_y - min_y) / (max_y - min_y) * width)
        col1 = int((float(center[1]) + half_y - min_y) / (max_y - min_y) * width) + 1
        mask[max(0, row0) : min(height, row1), max(0, col0) : min(width, col1)] = True
    return mask


def _world_waypoint_displacement(
    left_record: Mapping[str, Any],
    right_record: Mapping[str, Any],
    left: list[Any],
    right: list[Any],
) -> float | None:
    if not left or len(left) != len(right):
        return None
    left_world = _waypoints_to_world(left, (left_record.get("inputs") or {})["ego_pose"])
    right_world = _waypoints_to_world(right, (right_record.get("inputs") or {})["ego_pose"])
    return fmean(
        math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        for a, b in zip(left_world, right_world)
    )


def _waypoints_to_world(points: list[Any], pose: Mapping[str, Any]) -> list[list[float]]:
    yaw = math.radians(float(pose["yaw"]))
    result = []
    for point in points:
        x_forward = float(point[0])
        y_left = -float(point[1])  # model output uses CARLA ego y-right
        result.append(
            [
                float(pose["x"]) + math.cos(yaw) * x_forward - math.sin(yaw) * y_left,
                float(pose["y"]) + math.sin(yaw) * x_forward + math.cos(yaw) * y_left,
            ]
        )
    return result


def _first_brake_timestamp(
    rows: list[Mapping[str, Any]], event_timestamp: float | None
) -> float | None:
    return next(
        (
            float(row["timestamp"])
            for row in rows
            if (event_timestamp is None or float(row["timestamp"]) >= event_timestamp)
            and float(row["outputs"]["control"]["brake"]) >= 0.5
        ),
        None,
    )


def _distribution(values: list[float]) -> dict[str, float | int | None]:
    ordered = sorted(float(value) for value in values)
    return {
        "count": len(ordered),
        "mean": fmean(ordered) if ordered else None,
        "min": min(ordered) if ordered else None,
        "max": max(ordered) if ordered else None,
        "p50": _percentile(ordered, 50.0),
        "p95": _percentile(ordered, 95.0),
        "p99": _percentile(ordered, 99.0),
    }


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    position = (len(values) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
