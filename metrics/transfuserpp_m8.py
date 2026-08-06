"""M8 input-variant evaluation for TransFuser++ intermediates.

All variants use the original Scenario IR as ground truth.  The legacy
raw-only behavior remains the default; formal three-way evaluation opts into
the reconstructed and Harmonizer RGB variants explicitly.
"""

from __future__ import annotations

import hashlib
import math
from copy import deepcopy
from pathlib import Path
from statistics import fmean
from typing import Any, Iterable, Mapping

from adapters.open_loop_bbox_binding import (
    actor_ground_truth_rows,
    frame_binding,
    load_actor_manifest,
    validate_actor_manifest,
)
from metrics.transfuserpp_intermediate import (
    _validated_rows,
    evaluate_intermediate_trace,
)


M8_REPORT_SCHEMA = "transfuserpp_m8_raw_gt_evaluation.v2"
RAW_INPUT_SOURCES = frozenset(
    {
        "carla_stage_a_native_rgb_lidar",
        "raw_original_rgb_lidar",
        "nuscenes_original_rgb_lidar",
    }
)
RECONSTRUCTED_INPUT_SOURCES = frozenset(
    {
        "nurec_stage_b_6cam_rgb_lidar",
        "reconstructed_rgb_lidar",
        "harmonized_rgb_reconstructed_lidar",
    }
)
HARMONIZED_INPUT_SOURCES = frozenset({"harmonized_rgb_reconstructed_lidar"})
MODEL_CLASS_BY_ACTOR_TYPE = {"vehicle": 9, "pedestrian": 10}
PREDICTION_CLASS_BY_ACTOR_TYPE = {"vehicle": 0, "pedestrian": 1}
BBOX_CONFIDENCE_GATE = 0.1
BBOX_PRIMARY_IOU_THRESHOLD = 0.5
BBOX_SECONDARY_IOU_THRESHOLD = 0.25


class TransFuserPPM8Error(ValueError):
    """Raised when M8 evidence cannot be bound without guessing."""


def evaluate_m8_intermediate_trace(
    records: Iterable[Mapping[str, Any]],
    *,
    scenario_ir: Mapping[str, Any],
    scenario_ir_path: str | Path,
    expected_input_source: str = "carla_stage_a_native_rgb_lidar",
    evidence_root: Path | None = None,
    expected_scenario_ir_sha256: str | None = None,
    waypoint_spacing_sec: float = 0.5,
    expected_frame_count: int | None = None,
    allow_non_raw_input: bool = False,
    actor_manifest_path: str | Path | None = None,
    actor_manifest: Mapping[str, Any] | None = None,
    require_actor_manifest: bool = False,
) -> dict[str, Any]:
    """Evaluate a TF++ trace against original IR ego and actor truth.

    The base evaluator remains responsible for the intermediate contract and
    dense NPZ integrity.  This layer adds the M8 source/GT binding and metrics
    that require the original ego and actor trajectories.
    """

    if not isinstance(scenario_ir, Mapping):
        raise TransFuserPPM8Error("scenario_ir must be an object")
    if (
        isinstance(waypoint_spacing_sec, bool)
        or not isinstance(waypoint_spacing_sec, (int, float))
        or not math.isfinite(float(waypoint_spacing_sec))
        or float(waypoint_spacing_sec) <= 0.0
    ):
        raise TransFuserPPM8Error("waypoint_spacing_sec must be a positive finite number")
    if expected_frame_count is not None and (
        isinstance(expected_frame_count, bool) or expected_frame_count <= 0
    ):
        raise TransFuserPPM8Error("expected_frame_count must be positive when provided")
    if not isinstance(require_actor_manifest, bool):
        raise TransFuserPPM8Error("require_actor_manifest must be a boolean")
    if actor_manifest_path is not None and actor_manifest is not None:
        raise TransFuserPPM8Error(
            "provide actor_manifest_path or actor_manifest, not both"
        )

    ir_path = Path(scenario_ir_path)
    ir_sha256 = _sha256_file(ir_path)
    rows = _validated_rows(records, evidence_root=evidence_root)
    base = evaluate_intermediate_trace(rows)
    bound_actor_manifest: dict[str, Any] | None = None
    if actor_manifest_path is not None:
        bound_actor_manifest = load_actor_manifest(
            actor_manifest_path,
            expected_scenario_ir_sha256=ir_sha256,
            expected_scene_id=str(scenario_ir.get("scenario_id") or ""),
        )
    elif actor_manifest is not None:
        validate_actor_manifest(actor_manifest, require_usdz_file=False)
        bound_actor_manifest = deepcopy(dict(actor_manifest))
    ego_track = _track(
        (scenario_ir.get("ego") or {}).get("reference_trajectory"),
        "scenario_ir.ego.reference_trajectory",
        require_speed=True,
    )
    actors = (
        _actor_truth_from_manifest(bound_actor_manifest)
        if bound_actor_manifest is not None
        else _actor_truth(scenario_ir.get("actors"))
    )
    observed_input_source = _observed_input_source(rows)
    gt_binding = _ground_truth_binding(
        scenario_ir,
        ir_path=ir_path,
        ir_sha256=ir_sha256,
        ego_track=ego_track,
        actors=actors,
        actor_manifest=bound_actor_manifest,
    )

    problems: list[str] = list(base.get("fail_closed_reasons") or [])
    problems.extend(
        _ground_truth_problems(
            rows,
            scenario_ir=scenario_ir,
            ir_sha256=ir_sha256,
            expected_scenario_ir_sha256=expected_scenario_ir_sha256,
            ego_track=ego_track,
            expected_frame_count=expected_frame_count,
        )
    )
    actor_manifest_problems = _actor_manifest_problems(
        rows, actor_manifest=bound_actor_manifest
    )
    problems.extend(actor_manifest_problems)
    if require_actor_manifest and bound_actor_manifest is None:
        problems.append("m8_actor_manifest_required_for_formal_bbox")
    allowed_sources = RAW_INPUT_SOURCES | (
        RECONSTRUCTED_INPUT_SOURCES if allow_non_raw_input else frozenset()
    )
    if expected_input_source not in allowed_sources:
        problems.append("m8_expected_input_source_is_not_raw_original")
    if observed_input_source != expected_input_source:
        problems.append(
            "m8_input_source_mismatch:"
            f"expected={expected_input_source}:observed={observed_input_source}"
        )
    if observed_input_source in RECONSTRUCTED_INPUT_SOURCES and not allow_non_raw_input:
        problems.append("m8_reconstructed_or_harmonized_input_forbidden")
    if observed_input_source == "unknown":
        problems.append("m8_input_source_unbound")

    waypoint_metrics = _trajectory_metrics(
        rows,
        ego_track,
        output_name="waypoints_ego_m",
        spacing_sec=float(waypoint_spacing_sec),
    )
    checkpoint_metrics = _trajectory_metrics(
        rows,
        ego_track,
        output_name="route_checkpoints_ego_m",
        spacing_sec=float(waypoint_spacing_sec),
    )
    speed_metrics = _target_speed_metrics(
        rows,
        ego_track,
        spacing_sec=float(waypoint_spacing_sec),
    )
    if require_actor_manifest and bound_actor_manifest is None:
        bev_metrics = {
            "status": "unavailable",
            "reason": "actor_manifest_required_for_formal_bbox",
            "metric_scope": "no_bbox_score_without_same_frame_dynamic_actor_manifest",
        }
    else:
        bev_metrics = _dynamic_bev_metrics(rows, actors, ego_track)

    if not rows:
        problems.append("m8_intermediate_trace_empty")
    if any(item.get("status") == "unavailable" for item in (waypoint_metrics, checkpoint_metrics)):
        problems.append("m8_trajectory_metric_unavailable")
    if speed_metrics.get("status") != "evaluated":
        problems.append("m8_target_speed_metric_unavailable")
    if bev_metrics.get("status") != "evaluated":
        problems.append("m8_bev_metric_unavailable")

    # A raw LiDAR-to-camera projection must be bound before a sparse depth
    # comparison can be called a metric. Existing intermediate NPZs contain a
    # prediction only; they do not contain the corresponding raw depth target.
    depth_metrics = {
        "status": "unavailable",
        "reason": "raw_camera_lidar_projection_ground_truth_not_bound",
        "ground_truth_source": None,
        "metric_scope": "no_depth_score_without_same_frame_raw_projection",
        "evaluated_frame_count": 0,
        "mae": None,
        "rmse": None,
    }
    control_metrics = {
        "status": "prediction_only",
        "ground_truth_source": None,
        "reason": "Scenario IR has no human-driver control labels",
    }

    return {
        "schema_version": M8_REPORT_SCHEMA,
        "status": "evaluated" if rows and not problems else "failed",
        "evidence_classification": (
            "m8_actor_aware_bbox_gt_intermediate"
            if bound_actor_manifest is not None or require_actor_manifest
            else "m8_raw_gt_intermediate_legacy"
        ),
        "formal_bbox_evaluation": bound_actor_manifest is not None or require_actor_manifest,
        "input_variant_evaluation": allow_non_raw_input,
        "claims_m9": False,
        "frame_count": len(rows),
        "complete_trace": expected_frame_count is None or len(rows) == expected_frame_count,
        "input_binding": {
            "expected_source": expected_input_source,
            "observed_source": observed_input_source,
            "raw_input_source": observed_input_source in RAW_INPUT_SOURCES,
            "reconstruction_input_used": observed_input_source in RECONSTRUCTED_INPUT_SOURCES,
            "harmonizer_used": observed_input_source in HARMONIZED_INPUT_SOURCES,
            "harmonizer_rgb_only": observed_input_source in HARMONIZED_INPUT_SOURCES,
            "lidar_source": (
                "reconstructed_lidar"
                if observed_input_source in HARMONIZED_INPUT_SOURCES
                else observed_input_source
            ),
            "m9_only_inputs_forbidden_in_m8": not allow_non_raw_input,
            "sensor_provenance": _sensor_provenance(rows),
        },
        "ground_truth_binding": gt_binding,
        "actor_binding": {
            "required": require_actor_manifest,
            "status": "bound" if bound_actor_manifest is not None else "missing",
            "problems": actor_manifest_problems
            + (["m8_actor_manifest_required_for_formal_bbox"] if require_actor_manifest and bound_actor_manifest is None else []),
            "manifest": _manifest_reference(bound_actor_manifest),
        },
        "intermediate_validation": {
            "status": base.get("status"),
            "frame_count": base.get("frame_count"),
            "algorithm_identity": deepcopy(base.get("identity")),
            "fail_closed_reasons": list(base.get("fail_closed_reasons") or []),
        },
        "waypoints": waypoint_metrics,
        "route_checkpoints": checkpoint_metrics,
        "target_speed": speed_metrics,
        "dynamic_bev": bev_metrics,
        "depth": depth_metrics,
        "control": control_metrics,
        "full_3d_occupancy": {
            "status": "unavailable",
            "reason": "Scenario IR actor boxes are not dense voxel/free-space ground truth",
        },
        "fail_closed_reasons": sorted(set(problems)),
    }


def _ground_truth_binding(
    scenario_ir: Mapping[str, Any],
    *,
    ir_path: Path,
    ir_sha256: str,
    ego_track: list[dict[str, float]],
    actors: list[dict[str, Any]],
    actor_manifest: Mapping[str, Any] | None,
) -> dict[str, Any]:
    source = scenario_ir.get("source") or {}
    coordinate_frame = scenario_ir.get("coordinate_frame") or {}
    raw_actor_rows = scenario_ir.get("actors")
    raw_actor_count = len(raw_actor_rows) if isinstance(raw_actor_rows, list) else None
    return {
        "schema_version": str(scenario_ir.get("schema_version") or ""),
        "source_kind": "original_scenario_ir_from_raw_dataset",
        "dataset": source.get("dataset"),
        "scene_id": scenario_ir.get("scenario_id"),
        "scene_name": source.get("scene_name"),
        "scenario_ir_path": str(ir_path.resolve()),
        "scenario_ir_sha256": ir_sha256,
        "ego_frame_count": len(ego_track),
        "scenario_ir_actor_count": raw_actor_count,
        "evaluated_dynamic_actor_track_count": len(actors),
        "ground_truth_actor_source": (
            "open_loop_bbox_actor_manifest.v1"
            if actor_manifest is not None
            else "scenario_ir_actors_legacy"
        ),
        "actor_manifest": _manifest_reference(actor_manifest),
        "skipped_actor_types": ["object", "two_wheeler"],
        "coordinate_frame": deepcopy(coordinate_frame),
        "reconstruction_package_used": False,
        "harmonizer_used": False,
        "raw_annotation_fields": [
            "ego.reference_trajectory",
            "actors[].reference_trajectory",
            "actors[].dimensions",
            "actors[].type",
        ],
    }


def _manifest_reference(
    actor_manifest: Mapping[str, Any] | None,
) -> dict[str, Any]:
    if actor_manifest is None:
        return {
            "status": "missing",
            "schema_version": None,
            "path": None,
            "manifest_sha256": None,
            "manifest_file_sha256": None,
            "summary": None,
        }
    return {
        "status": "bound",
        "schema_version": actor_manifest.get("schema_version"),
        "path": actor_manifest.get("manifest_path"),
        "manifest_sha256": actor_manifest.get("manifest_sha256"),
        "manifest_file_sha256": actor_manifest.get("manifest_file_sha256"),
        "summary": deepcopy(actor_manifest.get("summary")),
    }


def _actor_truth_from_manifest(
    actor_manifest: Mapping[str, Any] | None,
) -> list[dict[str, Any]]:
    if actor_manifest is None:
        raise TransFuserPPM8Error("actor manifest is required")
    result = []
    for actor in actor_ground_truth_rows(actor_manifest):
        result.append(
            {
                "track_id": str(actor["track_id"]),
                "actor_type": str(actor["actor_type"]),
                "length_m": float(actor["length_m"]),
                "width_m": float(actor["width_m"]),
                "track": _track(
                    actor["track"],
                    f"actor_manifest.{actor['track_id']}.ir_trajectory",
                ),
            }
        )
    if not result:
        raise TransFuserPPM8Error("actor manifest contains no bbox actors")
    return result


def _actor_manifest_problems(
    rows: list[Mapping[str, Any]],
    *,
    actor_manifest: Mapping[str, Any] | None,
) -> list[str]:
    """Verify that every scored record carries the same dynamic frame proof."""

    if actor_manifest is None:
        return []
    manifest_sha = str(actor_manifest.get("manifest_sha256") or "")
    manifest_file_sha = str(actor_manifest.get("manifest_file_sha256") or "")
    problems: list[str] = []
    for row in rows:
        frame_id = int(row.get("frame_id", -1))
        try:
            expected = frame_binding(actor_manifest, frame_id)
        except (TypeError, ValueError) as exc:
            problems.append(f"m8_actor_manifest_frame_unavailable:{frame_id}:{exc}")
            continue
        provenance = row.get("provenance") or {}
        binding = provenance.get("actor_manifest")
        if not isinstance(binding, Mapping):
            problems.append(f"m8_actor_manifest_provenance_missing:{frame_id}")
            continue
        if binding.get("actor_manifest_sha256") != manifest_sha:
            problems.append(f"m8_actor_manifest_sha256_mismatch:{frame_id}")
        if manifest_file_sha and binding.get("actor_manifest_file_sha256") != manifest_file_sha:
            problems.append(f"m8_actor_manifest_file_sha256_mismatch:{frame_id}")
        if binding.get("frame_id") != frame_id:
            problems.append(f"m8_actor_manifest_frame_id_mismatch:{frame_id}")
        for field in (
            "active_actor_ids",
            "active_actor_set_sha256",
            "pose_digest",
            "manifest_dynamic_object_sha256",
        ):
            expected_value = (
                expected["dynamic_object_sha256"]
                if field == "manifest_dynamic_object_sha256"
                else expected[field]
            )
            if binding.get(field) != expected_value:
                problems.append(f"m8_actor_manifest_{field}_mismatch:{frame_id}")
        synchronization = row.get("synchronization") or {}
        if synchronization.get("dynamic_object_sha256") != expected["dynamic_object_sha256"]:
            problems.append(f"m8_dynamic_object_sha256_mismatch:{frame_id}")
    return sorted(set(problems))


def _ground_truth_problems(
    rows: list[Mapping[str, Any]],
    *,
    scenario_ir: Mapping[str, Any],
    ir_sha256: str,
    expected_scenario_ir_sha256: str | None,
    ego_track: list[dict[str, float]],
    expected_frame_count: int | None,
) -> list[str]:
    problems: list[str] = []
    if scenario_ir.get("schema_version") != "scenario_ir.v1":
        problems.append("m8_ground_truth_schema_invalid")
    source = scenario_ir.get("source") or {}
    if source.get("dataset") != "nuscenes":
        problems.append("m8_ground_truth_source_is_not_nuscenes")
    if expected_scenario_ir_sha256 and expected_scenario_ir_sha256 != ir_sha256:
        problems.append("m8_scenario_ir_sha256_mismatch")
    if expected_frame_count is not None and len(rows) != expected_frame_count:
        problems.append(
            f"m8_formal_frame_count_mismatch:expected={expected_frame_count}:actual={len(rows)}"
        )
    if len(ego_track) != len(rows):
        problems.append(
            f"m8_gt_prediction_frame_count_mismatch:gt={len(ego_track)}:prediction={len(rows)}"
        )
    for index, row in enumerate(rows):
        experiment = row.get("experiment") or {}
        if experiment.get("scenario_ir_sha256") != ir_sha256:
            problems.append(f"m8_frame_scenario_ir_binding_mismatch:{row['frame_id']}")
        if experiment.get("scene_id") != scenario_ir.get("scenario_id"):
            problems.append(f"m8_frame_scene_binding_mismatch:{row['frame_id']}")
        if int(row["frame_id"]) != index or index >= len(ego_track):
            problems.append(f"m8_frame_id_binding_mismatch:{row['frame_id']}")
            continue
        if abs(float(row["timestamp"]) - ego_track[index]["t_sec"]) > 0.02:
            problems.append(f"m8_frame_timestamp_binding_mismatch:{row['frame_id']}")
        pose = (row.get("inputs") or {}).get("ego_pose") or {}
        if not _pose_matches_state(pose, ego_track[index], tolerance=0.05):
            problems.append(f"m8_frame_ego_pose_binding_mismatch:{row['frame_id']}")
    return problems


def _trajectory_metrics(
    rows: list[Mapping[str, Any]],
    ego_track: list[dict[str, float]],
    *,
    output_name: str,
    spacing_sec: float,
) -> dict[str, Any]:
    errors: list[dict[str, Any]] = []
    clamped_count = 0
    point_count = 0
    for row in rows:
        points = (row.get("outputs") or {}).get(output_name)
        if not isinstance(points, list) or not points:
            return {"status": "unavailable", "reason": f"{output_name}_missing"}
        current = float(row["timestamp"])
        pose = (row.get("inputs") or {}).get("ego_pose") or {}
        for point_index, point in enumerate(points):
            if not isinstance(point, list) or len(point) != 2:
                return {
                    "status": "unavailable",
                    "reason": f"{output_name}_point_invalid",
                }
            horizon = (point_index + 1) * spacing_sec
            target_time = current + horizon
            truth, clamped = _state_at_time(ego_track, target_time)
            truth_point = _relative_model_point(pose, truth)
            predicted = [float(point[0]), float(point[1])]
            dx = predicted[0] - truth_point[0]
            dy = predicted[1] - truth_point[1]
            errors.append(
                {
                    "frame_id": int(row["frame_id"]),
                    "point_index": point_index,
                    "horizon_sec": horizon,
                    "truth_time_sec": target_time,
                    "truth_clamped_to_track": clamped,
                    "predicted_ego_m": predicted,
                    "ground_truth_ego_m": truth_point,
                    "distance_m": math.hypot(dx, dy),
                    "lateral_error_m": abs(dy),
                }
            )
            point_count += 1
            clamped_count += int(clamped)
    distances = [float(item["distance_m"]) for item in errors]
    lateral = [float(item["lateral_error_m"]) for item in errors]
    final_by_frame: list[dict[str, Any]] = []
    for frame_id in sorted({int(item["frame_id"]) for item in errors}):
        frame_errors = [item for item in errors if int(item["frame_id"]) == frame_id]
        final_by_frame.append(frame_errors[-1])
    return {
        "status": "evaluated",
        "metric_scope": "original_ego_reference_trajectory",
        "point_count": point_count,
        "frame_count": len(rows),
        "horizon_spacing_sec": spacing_sec,
        "clamped_ground_truth_point_count": clamped_count,
        "ade_m": _distribution(distances),
        "fde_m": _distribution([float(item["distance_m"]) for item in final_by_frame]),
        "lateral_error_m": _distribution(lateral),
        "per_point": errors,
    }


def _target_speed_metrics(
    rows: list[Mapping[str, Any]],
    ego_track: list[dict[str, float]],
    *,
    spacing_sec: float,
) -> dict[str, Any]:
    per_frame: list[dict[str, Any]] = []
    for row in rows:
        outputs = row.get("outputs") or {}
        probabilities = outputs.get("target_speed_probabilities")
        bins = outputs.get("target_speed_bins_mps")
        waypoints = outputs.get("waypoints_ego_m")
        if (
            not isinstance(probabilities, list)
            or not isinstance(bins, list)
            or len(probabilities) != len(bins)
            or not probabilities
            or not isinstance(waypoints, list)
            or not waypoints
        ):
            return {"status": "unavailable", "reason": "target_speed_binding_fields_missing"}
        horizon = len(waypoints) * spacing_sec
        truth, clamped = _state_at_time(ego_track, float(row["timestamp"]) + horizon)
        probability_values = [float(value) for value in probabilities]
        bin_values = [float(value) for value in bins]
        expected = sum(probability * speed for probability, speed in zip(probability_values, bin_values))
        truth_index = min(range(len(bin_values)), key=lambda index: abs(bin_values[index] - truth["speed_mps"]))
        per_frame.append(
            {
                "frame_id": int(row["frame_id"]),
                "reference_horizon_sec": horizon,
                "truth_time_sec": float(row["timestamp"]) + horizon,
                "truth_clamped_to_track": clamped,
                "ground_truth_speed_mps": truth["speed_mps"],
                "predicted_speed_mps": float(outputs["target_speed_mps"]),
                "probability_expected_speed_mps": expected,
                "ground_truth_nearest_bin_index": truth_index,
                "ground_truth_bin_probability": probability_values[truth_index],
                "argmax_bin_index": max(range(len(probability_values)), key=probability_values.__getitem__),
                "absolute_error_m": abs(float(outputs["target_speed_mps"]) - truth["speed_mps"]),
            }
        )
    errors = [float(item["absolute_error_m"]) for item in per_frame]
    cross_entropy = [
        -math.log(max(float(item["ground_truth_bin_probability"]), 1e-12))
        for item in per_frame
    ]
    accuracy = [
        int(item["argmax_bin_index"] == item["ground_truth_nearest_bin_index"])
        for item in per_frame
    ]
    return {
        "status": "evaluated",
        "metric_scope": "original_ego_speed_at_predicted_waypoint_horizon",
        "reference_horizon_policy": "last_predicted_waypoint",
        "frame_count": len(per_frame),
        "absolute_error_mps": _distribution(errors),
        "probability_cross_entropy_nats": _distribution(cross_entropy),
        "nearest_bin_accuracy": fmean(accuracy) if accuracy else None,
        "per_frame": per_frame,
    }


def _dynamic_bev_metrics(
    rows: list[Mapping[str, Any]],
    actors: list[dict[str, Any]],
    ego_track: list[dict[str, float]],
) -> dict[str, Any]:
    if not rows:
        return {"status": "unavailable", "reason": "trace_empty"}
    window = _model_bev_window(rows)
    if window is None:
        return {
            "status": "unavailable",
            "reason": "model_bev_window_missing_or_inconsistent",
        }

    frame_evidence: list[dict[str, Any]] = []
    visible_track_ids: set[str] = set()
    full_scene_sample_count = 0
    visible_gt_count = 0
    excluded_gt_count = 0
    prediction_count = 0
    invalid_prediction_count = 0
    all_primary_matches: list[dict[str, Any]] = []
    gt_by_class_frame: dict[int, dict[int, list[dict[str, Any]]]] = {
        class_id: {} for class_id in PREDICTION_CLASS_BY_ACTOR_TYPE.values()
    }
    predictions_by_class_frame: dict[int, dict[int, list[dict[str, Any]]]] = {
        class_id: {} for class_id in PREDICTION_CLASS_BY_ACTOR_TYPE.values()
    }

    for row in rows:
        frame_id = int(row["frame_id"])
        pose = (row.get("inputs") or {}).get("ego_pose") or {}
        all_truth = _actors_at_time(actors, float(row["timestamp"]), pose)
        visible_truth = [
            actor
            for actor in all_truth
            if _center_in_model_window(actor["center_ego_m"], window)
        ]
        full_scene_sample_count += len(all_truth)
        visible_gt_count += len(visible_truth)
        excluded_gt_count += len(all_truth) - len(visible_truth)
        visible_track_ids.update(str(actor["track_id"]) for actor in visible_truth)
        for actor in visible_truth:
            class_id = PREDICTION_CLASS_BY_ACTOR_TYPE[actor["actor_type"]]
            gt_by_class_frame[class_id].setdefault(frame_id, []).append(actor)

        predictions, invalid_count = _predicted_boxes(row)
        invalid_prediction_count += invalid_count
        prediction_count += len(predictions)
        for prediction in predictions:
            predictions_by_class_frame[prediction["class_id"]].setdefault(
                frame_id, []
            ).append(prediction)

        primary = _match_frame_boxes(
            visible_truth,
            predictions,
            iou_threshold=BBOX_PRIMARY_IOU_THRESHOLD,
        )
        all_primary_matches.extend(primary["matches"])
        frame_evidence.append(
            {
                "frame_id": frame_id,
                "timestamp": float(row["timestamp"]),
                "ground_truth": [_serialise_gt_box(item) for item in visible_truth],
                "predictions": [
                    _serialise_prediction_box(item) for item in predictions
                ],
                "matching_iou_threshold": BBOX_PRIMARY_IOU_THRESHOLD,
                "matches": primary["matches"],
                "false_positive_prediction_indices": primary["false_positive_prediction_indices"],
                "false_negative_track_ids": primary["false_negative_track_ids"],
            }
        )

    class_metrics: dict[str, dict[str, Any]] = {}
    ap50_by_class: dict[str, float | None] = {}
    ap25_by_class: dict[str, float | None] = {}
    for actor_type, class_id in PREDICTION_CLASS_BY_ACTOR_TYPE.items():
        gt_frames = gt_by_class_frame[class_id]
        prediction_frames = predictions_by_class_frame[class_id]
        primary = _evaluate_detection_set(
            gt_frames,
            prediction_frames,
            iou_threshold=BBOX_PRIMARY_IOU_THRESHOLD,
        )
        secondary = _evaluate_detection_set(
            gt_frames,
            prediction_frames,
            iou_threshold=BBOX_SECONDARY_IOU_THRESHOLD,
        )
        ap50 = _average_precision(
            gt_frames, prediction_frames, iou_threshold=BBOX_PRIMARY_IOU_THRESHOLD
        )
        ap25 = _average_precision(
            gt_frames, prediction_frames, iou_threshold=BBOX_SECONDARY_IOU_THRESHOLD
        )
        ap50_by_class[actor_type] = ap50
        ap25_by_class[actor_type] = ap25
        class_metrics[actor_type] = {
            "ground_truth_count": primary["ground_truth_count"],
            "prediction_count": primary["prediction_count"],
            "tp": primary["tp"],
            "fp": primary["fp"],
            "fn": primary["fn"],
            "precision": _safe_ratio(primary["tp"], primary["prediction_count"]),
            "recall": _safe_ratio(primary["tp"], primary["ground_truth_count"]),
            "iou_threshold": BBOX_PRIMARY_IOU_THRESHOLD,
            "iou_at_0_25": {
                "tp": secondary["tp"],
                "fp": secondary["fp"],
                "fn": secondary["fn"],
                "precision": _safe_ratio(secondary["tp"], secondary["prediction_count"]),
                "recall": _safe_ratio(secondary["tp"], secondary["ground_truth_count"]),
            },
            "ap50": ap50,
            "ap25": ap25,
            "iou": _distribution(primary["ious"]),
            "center_error_m": _distribution(primary["center_errors"]),
            "length_abs_error_m": _distribution(primary["length_errors"]),
            "width_abs_error_m": _distribution(primary["width_errors"]),
            "size_error_m": _distribution(primary["size_errors"]),
            "yaw_error_deg": _distribution(primary["yaw_errors_deg"]),
        }

    total_gt = sum(item["ground_truth_count"] for item in class_metrics.values())
    total_tp = sum(item["tp"] for item in class_metrics.values())
    total_fp = sum(item["fp"] for item in class_metrics.values())
    total_fn = sum(item["fn"] for item in class_metrics.values())
    primary_ious = [item["iou"] for item in all_primary_matches if item.get("matched")]
    primary_center_errors = [
        item["center_error_m"] for item in all_primary_matches if item.get("matched")
    ]
    primary_length_errors = [
        item["length_abs_error_m"] for item in all_primary_matches if item.get("matched")
    ]
    primary_width_errors = [
        item["width_abs_error_m"] for item in all_primary_matches if item.get("matched")
    ]
    primary_size_errors = [
        item["size_error_m"] for item in all_primary_matches if item.get("matched")
    ]
    primary_yaw_errors = [
        item["yaw_error_deg"] for item in all_primary_matches if item.get("matched")
    ]
    map50 = _mean_defined(ap50_by_class.values())
    map25 = _mean_defined(ap25_by_class.values())
    return {
        "status": "evaluated",
        "metric_scope": "dynamic_actor_oriented_bev_bbox",
        "coordinate_frame": "model_ego_carla_x_forward_y_right",
        "ground_truth_pose_source": "scenario_ir_actor_reference_trajectory",
        "gt_filter_policy": "actor_center_inside_model_bev_window",
        "model_bev_window": window,
        "full_scene_ground_truth_actor_sample_count": full_scene_sample_count,
        "ground_truth_actor_sample_count_in_model_window": visible_gt_count,
        "ground_truth_actor_sample_count_excluded_outside_model_window": excluded_gt_count,
        "raw_gt_actor_track_count": len(actors),
        "evaluated_gt_actor_track_count": len(visible_track_ids),
        "prediction_count_after_confidence_gate": prediction_count,
        "invalid_prediction_count": invalid_prediction_count,
        "box_detection": {
            "status": "evaluated",
            "ground_truth_count": total_gt,
            "prediction_count_after_confidence_gate": prediction_count,
            "matched_count": total_tp,
            "tp": total_tp,
            "fp": total_fp,
            "fn": total_fn,
            "recall": _safe_ratio(total_tp, total_gt),
            "precision": _safe_ratio(total_tp, prediction_count),
            "f1": _f1(total_tp, prediction_count, total_gt),
            "confidence_gate": BBOX_CONFIDENCE_GATE,
            "primary_iou_threshold": BBOX_PRIMARY_IOU_THRESHOLD,
            "secondary_iou_threshold": BBOX_SECONDARY_IOU_THRESHOLD,
            "matching": "same_frame_same_class_confidence_sorted_unique_oriented_rectangle_iou",
            "ap50": ap50_by_class,
            "ap25": ap25_by_class,
            "mAP50": map50,
            "mAP25": map25,
            "iou": _distribution(primary_ious),
            "center_error_m": _distribution(primary_center_errors),
            "length_abs_error_m": _distribution(primary_length_errors),
            "width_abs_error_m": _distribution(primary_width_errors),
            "size_error_m": _distribution(primary_size_errors),
            "yaw_error_deg": _distribution(primary_yaw_errors),
            "per_class": class_metrics,
            "per_frame": frame_evidence,
        },
        "full_scene_ground_truth": True,
    }


def _predicted_boxes(
    row: Mapping[str, Any],
) -> tuple[list[dict[str, float]], int]:
    result: list[dict[str, float]] = []
    invalid_count = 0
    for box in (row.get("outputs") or {}).get("bounding_boxes_ego") or []:
        if not isinstance(box, list) or len(box) < 9:
            invalid_count += 1
            continue
        try:
            confidence = float(box[-1])
            class_id = int(round(float(box[7])))
            center_x, center_y = float(box[0]), float(box[1])
            half_length, half_width = float(box[2]), float(box[3])
            yaw_rad = float(box[4])
        except (TypeError, ValueError, OverflowError):
            invalid_count += 1
            continue
        if (
            confidence < BBOX_CONFIDENCE_GATE
            or class_id not in {0, 1}
            or not all(math.isfinite(value) for value in (confidence, center_x, center_y, half_length, half_width, yaw_rad))
            or half_length <= 0.0
            or half_width <= 0.0
        ):
            invalid_count += 1
            continue
        result.append(
            {
                "x": center_x,
                "y": center_y,
                "half_length": half_length,
                "half_width": half_width,
                "yaw_rad": _wrap_angle(yaw_rad),
                "class_id": class_id,
                "confidence": confidence,
            }
        )
    return result, invalid_count


def _model_bev_window(rows: list[Mapping[str, Any]]) -> dict[str, Any] | None:
    """Return the common TF++ BEV crop used for the frame-level boxes."""

    values: tuple[float, float, float, float] | None = None
    shape: tuple[int, int] | None = None
    for row in rows:
        grid = (row.get("dynamic_bev_proxy") or {}).get("grid") or {}
        try:
            current = tuple(
                float(grid[name]) for name in ("min_x_m", "max_x_m", "min_y_m", "max_y_m")
            )
            current_shape = (int(grid["height"]), int(grid["width"]))
        except (KeyError, TypeError, ValueError, OverflowError):
            return None
        if (
            any(not math.isfinite(item) for item in current)
            or current[0] >= current[1]
            or current[2] >= current[3]
            or current_shape[0] <= 0
            or current_shape[1] <= 0
        ):
            return None
        if values is None:
            values = current
            shape = current_shape
        elif current != values or current_shape != shape:
            return None
    if values is None or shape is None:
        return None
    return {
        "min_x_m": values[0],
        "max_x_m": values[1],
        "min_y_m": values[2],
        "max_y_m": values[3],
        "height": shape[0],
        "width": shape[1],
        "center_policy": "inclusive",
        "source": "intermediate_record.dynamic_bev_proxy.grid_model_crop",
    }


def _center_in_model_window(center: list[float], window: Mapping[str, Any]) -> bool:
    return (
        float(window["min_x_m"]) <= float(center[0]) <= float(window["max_x_m"])
        and float(window["min_y_m"]) <= float(center[1]) <= float(window["max_y_m"])
    )


def _serialise_gt_box(actor: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "track_id": str(actor["track_id"]),
        "actor_type": str(actor["actor_type"]),
        "class_id": PREDICTION_CLASS_BY_ACTOR_TYPE[str(actor["actor_type"])],
        "center_ego_m": [float(actor["center_ego_m"][0]), float(actor["center_ego_m"][1])],
        "length_m": float(actor["length_m"]),
        "width_m": float(actor["width_m"]),
        "yaw_rad": float(actor["yaw_rad"]),
    }


def _serialise_prediction_box(prediction: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "class_id": int(prediction["class_id"]),
        "center_ego_m": [float(prediction["x"]), float(prediction["y"])],
        "length_m": float(prediction["half_length"]) * 2.0,
        "width_m": float(prediction["half_width"]) * 2.0,
        "yaw_rad": float(prediction["yaw_rad"]),
        "confidence": float(prediction["confidence"]),
    }


def _match_frame_boxes(
    ground_truth: list[Mapping[str, Any]],
    predictions: list[Mapping[str, Any]],
    *,
    iou_threshold: float,
) -> dict[str, Any]:
    """Match one frame using the standard score-ordered detection rule."""

    used_gt: set[int] = set()
    matches: list[dict[str, Any]] = []
    false_positive_indices: list[int] = []
    for prediction_index, prediction in sorted(
        enumerate(predictions),
        key=lambda item: (-float(item[1]["confidence"]), item[0]),
    ):
        candidates = [
            (gt_index, actor)
            for gt_index, actor in enumerate(ground_truth)
            if gt_index not in used_gt
            and PREDICTION_CLASS_BY_ACTOR_TYPE[str(actor["actor_type"])]
            == int(prediction["class_id"])
        ]
        best: tuple[int, Mapping[str, Any], float] | None = None
        for gt_index, actor in candidates:
            iou = _oriented_box_iou(actor, prediction)
            if best is None or iou > best[2]:
                best = (gt_index, actor, iou)
        if best is None or best[2] < iou_threshold:
            false_positive_indices.append(prediction_index)
            continue
        gt_index, actor, iou = best
        used_gt.add(gt_index)
        matches.append(
            {
                "prediction_index": prediction_index,
                "track_id": str(actor["track_id"]),
                "actor_type": str(actor["actor_type"]),
                "class_id": int(prediction["class_id"]),
                "matched": True,
                "iou": iou,
                **_box_error_fields(actor, prediction),
            }
        )
    false_negative_track_ids = [
        str(actor["track_id"])
        for index, actor in enumerate(ground_truth)
        if index not in used_gt
    ]
    return {
        "matches": matches,
        "false_positive_prediction_indices": false_positive_indices,
        "false_negative_track_ids": false_negative_track_ids,
    }


def _evaluate_detection_set(
    ground_truth_by_frame: Mapping[int, list[Mapping[str, Any]]],
    predictions_by_frame: Mapping[int, list[Mapping[str, Any]]],
    *,
    iou_threshold: float,
) -> dict[str, Any]:
    ground_truth_count = sum(len(items) for items in ground_truth_by_frame.values())
    prediction_count = sum(len(items) for items in predictions_by_frame.values())
    tp = 0
    ious: list[float] = []
    center_errors: list[float] = []
    length_errors: list[float] = []
    width_errors: list[float] = []
    size_errors: list[float] = []
    yaw_errors_deg: list[float] = []
    for frame_id in sorted(set(ground_truth_by_frame) | set(predictions_by_frame)):
        ground_truth = ground_truth_by_frame.get(frame_id, [])
        predictions = predictions_by_frame.get(frame_id, [])
        result = _match_frame_boxes(
            list(ground_truth), list(predictions), iou_threshold=iou_threshold
        )
        for match in result["matches"]:
            tp += 1
            ious.append(float(match["iou"]))
            center_errors.append(float(match["center_error_m"]))
            length_errors.append(float(match["length_abs_error_m"]))
            width_errors.append(float(match["width_abs_error_m"]))
            size_errors.append(float(match["size_error_m"]))
            yaw_errors_deg.append(float(match["yaw_error_deg"]))
    fp = prediction_count - tp
    fn = ground_truth_count - tp
    return {
        "ground_truth_count": ground_truth_count,
        "prediction_count": prediction_count,
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "ious": ious,
        "center_errors": center_errors,
        "length_errors": length_errors,
        "width_errors": width_errors,
        "size_errors": size_errors,
        "yaw_errors_deg": yaw_errors_deg,
    }


def _average_precision(
    ground_truth_by_frame: Mapping[int, list[Mapping[str, Any]]],
    predictions_by_frame: Mapping[int, list[Mapping[str, Any]]],
    *,
    iou_threshold: float,
) -> float | None:
    ground_truth_count = sum(len(items) for items in ground_truth_by_frame.values())
    if ground_truth_count == 0:
        return None
    detections = [
        (frame_id, prediction_index, prediction)
        for frame_id, predictions in predictions_by_frame.items()
        for prediction_index, prediction in enumerate(predictions)
    ]
    detections.sort(
        key=lambda item: (-float(item[2]["confidence"]), int(item[0]), int(item[1]))
    )
    used_by_frame: dict[int, set[int]] = {}
    true_positive: list[int] = []
    false_positive: list[int] = []
    for frame_id, _, prediction in detections:
        ground_truth = ground_truth_by_frame.get(frame_id, [])
        used = used_by_frame.setdefault(frame_id, set())
        candidates = [
            (index, actor)
            for index, actor in enumerate(ground_truth)
            if index not in used
        ]
        best: tuple[int, float] | None = None
        for index, actor in candidates:
            iou = _oriented_box_iou(actor, prediction)
            if best is None or iou > best[1]:
                best = (index, iou)
        if best is not None and best[1] >= iou_threshold:
            used.add(best[0])
            true_positive.append(1)
            false_positive.append(0)
        else:
            true_positive.append(0)
            false_positive.append(1)
    if not true_positive:
        return 0.0
    cumulative_tp = 0
    cumulative_fp = 0
    precisions: list[float] = []
    recalls: list[float] = []
    for tp, fp in zip(true_positive, false_positive):
        cumulative_tp += tp
        cumulative_fp += fp
        precisions.append(cumulative_tp / (cumulative_tp + cumulative_fp))
        recalls.append(cumulative_tp / ground_truth_count)
    # All-points interpolated AP, with the precision envelope used by common
    # detection toolkits. The score order is retained above for auditability.
    recall_points = [0.0, *recalls, 1.0]
    precision_points = [0.0, *precisions, 0.0]
    for index in range(len(precision_points) - 2, -1, -1):
        precision_points[index] = max(
            precision_points[index], precision_points[index + 1]
        )
    return sum(
        (recall_points[index + 1] - recall_points[index]) * precision_points[index + 1]
        for index in range(len(recall_points) - 1)
    )


def _oriented_box_iou(
    ground_truth: Mapping[str, Any], prediction: Mapping[str, Any]
) -> float:
    gt_polygon = _rectangle_polygon(
        ground_truth["center_ego_m"],
        float(ground_truth["length_m"]) / 2.0,
        float(ground_truth["width_m"]) / 2.0,
        float(ground_truth["yaw_rad"]),
    )
    prediction_polygon = _rectangle_polygon(
        [float(prediction["x"]), float(prediction["y"])],
        float(prediction["half_length"]),
        float(prediction["half_width"]),
        float(prediction["yaw_rad"]),
    )
    intersection = _convex_polygon_intersection(gt_polygon, prediction_polygon)
    intersection_area = _polygon_area(intersection)
    gt_area = _polygon_area(gt_polygon)
    prediction_area = _polygon_area(prediction_polygon)
    union = gt_area + prediction_area - intersection_area
    return intersection_area / union if union > 0.0 else 0.0


def _rectangle_polygon(
    center: list[float] | Mapping[str, Any],
    half_length: float,
    half_width: float,
    yaw_rad: float,
) -> list[tuple[float, float]]:
    if isinstance(center, Mapping):
        x, y = float(center["x"]), float(center["y"])
    else:
        x, y = float(center[0]), float(center[1])
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    corners = [
        (-half_length, -half_width),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width),
    ]
    return [
        (
            x + cos_yaw * local_x - sin_yaw * local_y,
            y + sin_yaw * local_x + cos_yaw * local_y,
        )
        for local_x, local_y in corners
    ]


def _convex_polygon_intersection(
    subject: list[tuple[float, float]], clip: list[tuple[float, float]]
) -> list[tuple[float, float]]:
    output = list(subject)
    if not output:
        return []
    clip_orientation = _signed_polygon_area(clip)
    sign = 1.0 if clip_orientation >= 0.0 else -1.0
    for clip_start, clip_end in zip(clip, [*clip[1:], clip[0]]):
        input_points = output
        output = []
        if not input_points:
            break
        for start, end in zip(input_points, [*input_points[1:], input_points[0]]):
            start_inside = _cross(clip_start, clip_end, start) * sign >= -1e-9
            end_inside = _cross(clip_start, clip_end, end) * sign >= -1e-9
            if end_inside:
                if not start_inside:
                    output.append(_line_intersection(start, end, clip_start, clip_end))
                output.append(end)
            elif start_inside:
                output.append(_line_intersection(start, end, clip_start, clip_end))
    return output


def _line_intersection(
    start: tuple[float, float],
    end: tuple[float, float],
    clip_start: tuple[float, float],
    clip_end: tuple[float, float],
) -> tuple[float, float]:
    ray = (end[0] - start[0], end[1] - start[1])
    clip_ray = (clip_end[0] - clip_start[0], clip_end[1] - clip_start[1])
    denominator = ray[0] * clip_ray[1] - ray[1] * clip_ray[0]
    if abs(denominator) <= 1e-12:
        return end
    offset = (clip_start[0] - start[0], clip_start[1] - start[1])
    numerator = offset[0] * clip_ray[1] - offset[1] * clip_ray[0]
    t = numerator / denominator
    return (start[0] + t * ray[0], start[1] + t * ray[1])


def _cross(
    start: tuple[float, float], end: tuple[float, float], point: tuple[float, float]
) -> float:
    return (end[0] - start[0]) * (point[1] - start[1]) - (end[1] - start[1]) * (
        point[0] - start[0]
    )


def _signed_polygon_area(polygon: list[tuple[float, float]]) -> float:
    if len(polygon) < 3:
        return 0.0
    return 0.5 * sum(
        start[0] * end[1] - end[0] * start[1]
        for start, end in zip(polygon, [*polygon[1:], polygon[0]])
    )


def _polygon_area(polygon: list[tuple[float, float]]) -> float:
    return abs(_signed_polygon_area(polygon))


def _box_error_fields(
    ground_truth: Mapping[str, Any], prediction: Mapping[str, Any]
) -> dict[str, float]:
    gt_center = ground_truth["center_ego_m"]
    center_error = math.hypot(
        float(prediction["x"]) - float(gt_center[0]),
        float(prediction["y"]) - float(gt_center[1]),
    )
    length_error = abs(float(prediction["half_length"]) * 2.0 - float(ground_truth["length_m"]))
    width_error = abs(float(prediction["half_width"]) * 2.0 - float(ground_truth["width_m"]))
    size_error = math.hypot(length_error, width_error)
    yaw_error_deg = math.degrees(
        abs(_wrap_angle(float(prediction["yaw_rad"]) - float(ground_truth["yaw_rad"])))
    )
    return {
        "center_error_m": center_error,
        "length_abs_error_m": length_error,
        "width_abs_error_m": width_error,
        "size_error_m": size_error,
        "yaw_error_deg": yaw_error_deg,
    }


def _safe_ratio(numerator: int, denominator: int) -> float | None:
    return float(numerator) / float(denominator) if denominator else None


def _f1(tp: int, prediction_count: int, ground_truth_count: int) -> float | None:
    precision = _safe_ratio(tp, prediction_count)
    recall = _safe_ratio(tp, ground_truth_count)
    if precision is None or recall is None or precision + recall == 0.0:
        return None
    return 2.0 * precision * recall / (precision + recall)


def _mean_defined(values: Iterable[float | None]) -> float | None:
    defined = [float(value) for value in values if value is not None]
    return fmean(defined) if defined else None


def _actor_truth(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        raise TransFuserPPM8Error("scenario_ir.actors must be a list")
    result = []
    for index, raw in enumerate(value):
        if not isinstance(raw, Mapping):
            raise TransFuserPPM8Error(f"scenario_ir.actors[{index}] must be an object")
        actor_type = str(raw.get("type") or "")
        if actor_type not in MODEL_CLASS_BY_ACTOR_TYPE:
            continue
        track_id = str(raw.get("source_track_id") or raw.get("actor_id") or "")
        dimensions = raw.get("dimensions") or {}
        try:
            length = float(dimensions["length"])
            width = float(dimensions["width"])
        except (KeyError, TypeError, ValueError) as exc:
            raise TransFuserPPM8Error(f"actor {track_id or index} dimensions are invalid") from exc
        if length <= 0.0 or width <= 0.0:
            raise TransFuserPPM8Error(f"actor {track_id or index} dimensions must be positive")
        result.append(
            {
                "track_id": track_id or f"actor-{index}",
                "actor_type": actor_type,
                "length_m": length,
                "width_m": width,
                "track": _track(raw.get("reference_trajectory"), f"actor[{index}].reference_trajectory"),
            }
        )
    return result


def _actors_at_time(
    actors: list[dict[str, Any]], timestamp: float, ego_pose: Mapping[str, Any]
) -> list[dict[str, Any]]:
    result = []
    for actor in actors:
        track = actor["track"]
        if timestamp < track[0]["t_sec"] - 0.02 or timestamp > track[-1]["t_sec"] + 0.02:
            continue
        state, _ = _state_at_time(track, timestamp)
        result.append(
            {
                "track_id": actor["track_id"],
                "actor_type": actor["actor_type"],
                "center_ego_m": _relative_model_point(ego_pose, state),
                "length_m": actor["length_m"],
                "width_m": actor["width_m"],
                "yaw_rad": _relative_model_yaw(ego_pose, state),
            }
        )
    return result


def _track(value: Any, label: str, *, require_speed: bool = False) -> list[dict[str, float]]:
    if not isinstance(value, list) or not value:
        raise TransFuserPPM8Error(f"{label} must be a non-empty list")
    result = []
    previous = -math.inf
    for index, raw in enumerate(value):
        if not isinstance(raw, Mapping):
            raise TransFuserPPM8Error(f"{label}[{index}] must be an object")
        try:
            state = {
                "t_sec": float(raw["t_sec"]),
                "x": float(raw["x"]),
                "y": float(raw["y"]),
                "z": float(raw.get("z", 0.0)),
                "yaw": float(raw.get("yaw", 0.0)),
                "speed_mps": float(raw.get("speed_mps", 0.0)),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise TransFuserPPM8Error(f"{label}[{index}] is invalid") from exc
        if any(not math.isfinite(item) for item in state.values()) or state["t_sec"] < previous:
            raise TransFuserPPM8Error(f"{label}[{index}] is non-finite or non-monotonic")
        if require_speed and "speed_mps" not in raw:
            raise TransFuserPPM8Error(f"{label}[{index}].speed_mps is required")
        previous = state["t_sec"]
        result.append(state)
    return result


def _state_at_time(track: list[dict[str, float]], timestamp: float) -> tuple[dict[str, float], bool]:
    if timestamp <= track[0]["t_sec"]:
        return deepcopy(track[0]), timestamp < track[0]["t_sec"]
    if timestamp >= track[-1]["t_sec"]:
        return deepcopy(track[-1]), timestamp > track[-1]["t_sec"]
    for left, right in zip(track, track[1:]):
        if left["t_sec"] <= timestamp <= right["t_sec"]:
            duration = right["t_sec"] - left["t_sec"]
            ratio = 0.0 if duration <= 0.0 else (timestamp - left["t_sec"]) / duration
            return (
                {
                    key: (
                        left[key]
                        + ratio * _shortest_angle_delta(left[key], right[key])
                        if key == "yaw"
                        else left[key] + ratio * (right[key] - left[key])
                    )
                    for key in left
                },
                False,
            )
    return deepcopy(track[-1]), True


def _relative_model_point(ego_pose: Mapping[str, Any], world_state: Mapping[str, float]) -> list[float]:
    ego_x = float(ego_pose.get("x", 0.0))
    ego_y = float(ego_pose.get("y", 0.0))
    yaw = math.radians(float(ego_pose.get("yaw", 0.0)))
    dx = float(world_state["x"]) - ego_x
    dy = float(world_state["y"]) - ego_y
    return [
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        math.sin(yaw) * dx - math.cos(yaw) * dy,
    ]


def _relative_model_yaw(
    ego_pose: Mapping[str, Any], world_state: Mapping[str, float]
) -> float:
    """Convert the Scenario IR left-handed heading convention to TF++ yaw."""

    relative_scene_yaw = math.radians(
        float(world_state.get("yaw", 0.0)) - float(ego_pose.get("yaw", 0.0))
    )
    # Scenario IR uses y-left while CARLA/TF++ uses y-right. Reflection changes
    # the sign of the relative heading; wrapping keeps IoU/error calculations
    # stable across the +/- pi boundary.
    return _wrap_angle(-relative_scene_yaw)


def _wrap_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _shortest_angle_delta(start: float, end: float) -> float:
    return _wrap_angle(float(end) - float(start))


def _pose_matches_state(
    pose: Mapping[str, Any], state: Mapping[str, float], *, tolerance: float
) -> bool:
    try:
        position_error = math.hypot(float(pose["x"]) - state["x"], float(pose["y"]) - state["y"])
        yaw_error = abs((float(pose["yaw"]) - state["yaw"] + 180.0) % 360.0 - 180.0)
    except (KeyError, TypeError, ValueError):
        return False
    return position_error <= tolerance and yaw_error <= max(1.0, tolerance * 10.0)


def _observed_input_source(rows: list[Mapping[str, Any]]) -> str:
    explicit = {
        str((row.get("provenance") or {}).get("input_source") or "")
        for row in rows
        if (row.get("provenance") or {}).get("input_source")
    }
    if explicit:
        return next(iter(explicit)) if len(explicit) == 1 else "mixed"
    versions = {str((row.get("experiment") or {}).get("scene_version") or "") for row in rows}
    if versions and all("nurec" in value.lower() for value in versions):
        return "nurec_stage_b_6cam_rgb_lidar"
    if versions and all(value == "open-loop-exchange-v1" for value in versions):
        return "carla_stage_a_native_rgb_lidar"
    return "unknown"


def _sensor_provenance(rows: list[Mapping[str, Any]]) -> dict[str, Any]:
    camera_hashes: list[str] = []
    lidar_hashes: list[str] = []
    source_frames: list[dict[str, Any]] = []
    for row in rows:
        inputs = row.get("inputs") or {}
        camera = inputs.get("camera_front") or {}
        lidar = inputs.get("lidar_top") or {}
        camera_hashes.append(str(camera.get("sha256") or ""))
        lidar_hashes.append(str(lidar.get("sha256") or ""))
        binding = (row.get("provenance") or {}).get("source_frame_binding")
        if isinstance(binding, Mapping):
            source_frames.append(deepcopy(dict(binding)))

    def sequence_digest(values: list[str]) -> str:
        return hashlib.sha256("\n".join(values).encode("ascii", errors="replace")).hexdigest()

    return {
        "frame_count": len(rows),
        "camera_front_payload_sequence_sha256": sequence_digest(camera_hashes),
        "lidar_top_payload_sequence_sha256": sequence_digest(lidar_hashes),
        "camera_front_payload_sha256": camera_hashes,
        "lidar_top_payload_sha256": lidar_hashes,
        "source_frame_binding": source_frames,
    }


def _distribution(values: list[float]) -> dict[str, Any]:
    if not values:
        return {
            "count": 0,
            "mean": None,
            "min": None,
            "max": None,
            "p50": None,
            "p95": None,
            "p99": None,
        }
    ordered = sorted(float(value) for value in values)
    return {
        "count": len(ordered),
        "mean": fmean(ordered),
        "min": ordered[0],
        "max": ordered[-1],
        "p50": _percentile(ordered, 50.0),
        "p95": _percentile(ordered, 95.0),
        "p99": _percentile(ordered, 99.0),
    }


def _percentile(values: list[float], percentile: float) -> float:
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def _sha256_file(path: Path) -> str:
    if not path.is_file():
        raise TransFuserPPM8Error(f"scenario_ir is unavailable: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
