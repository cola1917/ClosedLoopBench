from __future__ import annotations

import json
import math
from copy import deepcopy
from hashlib import sha256
from typing import Any, Mapping


VEHICLE_TRACK = "c1958768d48640948f6053d04cffd35b"
PEDESTRIAN_TRACK = "71603dd1a2ba4e9daf095535e38310ac"
FORMAL_SCENE_ID = "cc8c0bf57f984915a77078b10eb33198"
FORMAL_SCENE_VERSION = "formal40k-v1"
SUPPORTED_CASES = {
    "S0_original_replay",
    "S2_lead_hard_brake",
    "S4_pedestrian_early_crossing",
}
CASE_ACTOR_CONTROL_MODES = {
    "S0_original_replay": {
        VEHICLE_TRACK: "replay",
        PEDESTRIAN_TRACK: "replay",
    },
    "S2_lead_hard_brake": {
        VEHICLE_TRACK: "scripted",
        PEDESTRIAN_TRACK: "replay",
    },
    "S4_pedestrian_early_crossing": {
        VEHICLE_TRACK: "replay",
        PEDESTRIAN_TRACK: "scripted",
    },
}


class Scene0061VariantError(ValueError):
    """Raised when a light edit would leave the recorded actor corridor."""


def build_scene0061_variant(
    run_config: Mapping[str, Any],
    *,
    case_id: str,
    seed: int,
    event_timestamp_sec: float | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    if case_id not in SUPPORTED_CASES:
        raise Scene0061VariantError(f"unsupported focused TF++ case: {case_id}")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise Scene0061VariantError("seed must be a non-negative integer")
    source = deepcopy(dict(run_config))
    _validate_formal_source(source)
    # Every focused case must retain both formally bound actors even when only
    # one trajectory is edited.
    _actor_for_track(source, VEHICLE_TRACK)
    _actor_for_track(source, PEDESTRIAN_TRACK)
    source_hash = _digest(source)
    result = deepcopy(source)
    experiment = dict(result.get("experiment") or {})
    experiment.update(
        {
            "case_id": case_id,
            "seed": seed,
            "algorithm_id": "transfuserpp_v5",
            "algorithm_version": "carla_garage.leaderboard_2.transfuser_v5",
        }
    )
    result["experiment"] = experiment
    carla = dict(result.get("carla") or {})
    carla["seed"] = seed
    result["carla"] = carla

    delta: dict[str, Any] = {
        "case_id": case_id,
        "source_run_config_sha256": source_hash,
        "source_track_id": None,
        "event_timestamp_sec": event_timestamp_sec,
        "path_geometry_changed": False,
        "remote_validation_required": True,
    }
    if case_id == "S0_original_replay":
        delta["operation"] = "none"
    elif case_id == "S2_lead_hard_brake":
        if event_timestamp_sec is None or not math.isfinite(float(event_timestamp_sec)) or float(event_timestamp_sec) < 0.0:
            raise Scene0061VariantError("S2 requires a finite non-negative event_timestamp_sec")
        actor = _actor_for_track(result, VEHICLE_TRACK)
        actor["reference_trajectory"], edit = _hard_brake_trajectory(
            actor.get("reference_trajectory") or [],
            event_timestamp_sec=float(event_timestamp_sec),
            deceleration_mps2=5.0,
            duration_sec=1.0,
        )
        actor["counterfactual_edit"] = edit
        delta.update(edit)
        delta["source_track_id"] = VEHICLE_TRACK
    else:
        if event_timestamp_sec is None or not math.isfinite(float(event_timestamp_sec)):
            raise Scene0061VariantError(
                "S4 requires the baseline source-corridor crossing event timestamp"
            )
        actor = _actor_for_track(result, PEDESTRIAN_TRACK)
        edited, edit = _early_crossing_trajectory(
            actor.get("reference_trajectory") or [],
            source_crossing_timestamp_sec=float(event_timestamp_sec),
            crossing_advance_sec=1.0,
            anticipation_window_sec=2.0,
        )
        actor["reference_trajectory"] = edited
        actor["counterfactual_edit"] = edit
        delta.update(edit)
        delta["source_track_id"] = PEDESTRIAN_TRACK
        delta["event_timestamp_sec"] = edit["intervention_timestamp_sec"]

    event_evidence = _build_counterfactual_event_evidence(
        source,
        case_id=case_id,
        requested_event_timestamp_sec=event_timestamp_sec,
        delta=delta,
    )
    result["counterfactual_event_evidence"] = event_evidence

    actor_control_contract = _freeze_actor_control_modes(result, case_id)
    result["actor_control_contract"] = actor_control_contract
    delta["case_actor_control_mode"] = actor_control_contract[
        "case_actor_control_mode"
    ]
    delta["effective_actor_control_modes"] = deepcopy(
        actor_control_contract["effective_modes_by_track"]
    )

    result["counterfactual"] = deepcopy(delta)
    delta["edited_run_config_sha256"] = _digest(result)
    return result, delta


def _build_counterfactual_event_evidence(
    source: Mapping[str, Any],
    *,
    case_id: str,
    requested_event_timestamp_sec: float | None,
    delta: Mapping[str, Any],
) -> dict[str, Any]:
    if case_id == "S0_original_replay":
        return {
            "schema_version": "scene0061_counterfactual_event_evidence.v1",
            "case_id": case_id,
            "event_kind": "none",
            "source_actor_track_id": None,
            "source_trajectory_sha256": None,
            "requested_event_timestamp_sec": None,
            "status": "not_applicable",
        }
    track_id = VEHICLE_TRACK if case_id == "S2_lead_hard_brake" else PEDESTRIAN_TRACK
    actor = _actor_for_track(dict(source), track_id)
    trajectory = _validated_trajectory(actor.get("reference_trajectory") or [])
    timestamp = float(requested_event_timestamp_sec)
    event_pose, segment = _source_pose_at_timestamp(trajectory, timestamp)
    event_kind = (
        "baseline_lead_hard_brake_anchor"
        if case_id == "S2_lead_hard_brake"
        else "baseline_pedestrian_source_corridor_crossing_anchor"
    )
    return {
        "schema_version": "scene0061_counterfactual_event_evidence.v1",
        "case_id": case_id,
        "event_kind": event_kind,
        "source_actor_track_id": track_id,
        "source_trajectory_sha256": _digest(trajectory),
        "requested_event_timestamp_sec": timestamp,
        "source_event_pose": event_pose,
        "source_geometry": {
            "source": "scenario_ir.actor.reference_trajectory",
            "corridor": "source_reference_corridor",
            "interpolation": "linear_xy_on_bracketing_source_segment",
            "bracketing_segment": segment,
            "free_space_geometry_used": False,
        },
        "edit_event_timestamp_sec": delta.get("event_timestamp_sec"),
        "status": "source_trajectory_bound",
        "semantic_event_annotation": (
            "caller_declared_baseline_crossing_bound_to_hashed_formal_pedestrian_trajectory"
            if case_id == "S4_pedestrian_early_crossing"
            else "caller_declared_brake_anchor_bound_to_hashed_formal_vehicle_trajectory"
        ),
        "remote_validation_required": True,
    }


def _source_pose_at_timestamp(
    trajectory: list[dict[str, Any]], timestamp: float
) -> tuple[dict[str, float], dict[str, Any]]:
    if timestamp < float(trajectory[0]["t_sec"]) or timestamp > float(
        trajectory[-1]["t_sec"]
    ):
        raise Scene0061VariantError("event timestamp is outside the source trajectory")
    for index, point in enumerate(trajectory):
        if abs(float(point["t_sec"]) - timestamp) <= 1e-9:
            pose = {"t_sec": timestamp, "x": float(point["x"]), "y": float(point["y"])}
            return pose, {
                "left_index": index,
                "right_index": index,
                "left_timestamp_sec": timestamp,
                "right_timestamp_sec": timestamp,
            }
    for index in range(1, len(trajectory)):
        left, right = trajectory[index - 1], trajectory[index]
        left_time, right_time = float(left["t_sec"]), float(right["t_sec"])
        if left_time < timestamp < right_time:
            weight = (timestamp - left_time) / (right_time - left_time)
            pose = {
                "t_sec": timestamp,
                "x": float(left["x"]) + weight * (float(right["x"]) - float(left["x"])),
                "y": float(left["y"]) + weight * (float(right["y"]) - float(left["y"])),
            }
            return pose, {
                "left_index": index - 1,
                "right_index": index,
                "left_timestamp_sec": left_time,
                "right_timestamp_sec": right_time,
            }
    raise Scene0061VariantError("event timestamp cannot be bound to source geometry")


def _hard_brake_trajectory(
    trajectory: list[Any],
    *,
    event_timestamp_sec: float,
    deceleration_mps2: float,
    duration_sec: float,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    source = _validated_trajectory(trajectory)
    if event_timestamp_sec + duration_sec > float(source[-1]["t_sec"]):
        raise Scene0061VariantError(
            "S2 trajectory does not cover the complete hard-brake interval"
        )
    source = _insert_time_point(source, event_timestamp_sec)
    source = _insert_time_point(source, event_timestamp_sec + duration_sec)
    times = [float(row["t_sec"]) for row in source]
    if not times[0] <= event_timestamp_sec <= times[-1]:
        raise Scene0061VariantError("S2 event timestamp is outside the lead trajectory")
    distances = [0.0]
    for left, right in zip(source, source[1:]):
        distances.append(
            distances[-1]
            + math.hypot(float(right["x"]) - float(left["x"]), float(right["y"]) - float(left["y"]))
        )
    event_s = _interpolate_scalar(times, distances, event_timestamp_sec)
    source_speed = _speed_at(source, event_timestamp_sec)
    brake_end = event_timestamp_sec + duration_sec
    stop_time = min(duration_sec, source_speed / deceleration_mps2)
    brake_progress = max(
        0.0,
        source_speed * stop_time - 0.5 * deceleration_mps2 * stop_time**2,
    )
    baseline_end_s = _interpolate_scalar(times, distances, min(brake_end, times[-1]))
    lag_m = max(0.0, baseline_end_s - (event_s + brake_progress))
    result = []
    for point in source:
        t_sec = float(point["t_sec"])
        if t_sec < event_timestamp_sec:
            result.append(deepcopy(point))
            continue
        dt = t_sec - event_timestamp_sec
        if dt <= duration_sec:
            motion_dt = min(dt, stop_time)
            target_s = event_s + max(
                0.0,
                source_speed * motion_dt - 0.5 * deceleration_mps2 * motion_dt**2,
            )
            speed = max(0.0, source_speed - deceleration_mps2 * dt)
        else:
            target_s = event_s + brake_progress
            speed = 0.0
        edited = _point_at_distance(source, distances, min(target_s, distances[-1]))
        edited["t_sec"] = t_sec
        edited["speed_mps"] = float(speed)
        result.append(edited)
    return result, {
        "operation": "lead_hard_brake_corridor_retime",
        "event_timestamp_sec": event_timestamp_sec,
        "deceleration_mps2": deceleration_mps2,
        "duration_sec": duration_sec,
        "stop_time_sec": stop_time,
        "source_speed_mps": source_speed,
        "longitudinal_lag_m": lag_m,
        "corridor": "source_reference_corridor",
        "path_geometry_changed": False,
        "post_brake_behavior": "stopped_on_source_corridor",
    }


def _early_crossing_trajectory(
    trajectory: list[Any],
    *,
    source_crossing_timestamp_sec: float,
    crossing_advance_sec: float,
    anticipation_window_sec: float,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    source = _validated_trajectory(trajectory)
    intervention = source_crossing_timestamp_sec - anticipation_window_sec
    edited_crossing = source_crossing_timestamp_sec - crossing_advance_sec
    if crossing_advance_sec <= 0.0 or anticipation_window_sec <= crossing_advance_sec:
        raise Scene0061VariantError(
            "S4 crossing advance must be positive and shorter than anticipation"
        )
    if intervention < float(source[0]["t_sec"]):
        raise Scene0061VariantError(
            "S4 source trajectory lacks the required pre-intervention corridor"
        )
    if not intervention < edited_crossing < source_crossing_timestamp_sec <= float(
        source[-1]["t_sec"]
    ):
        raise Scene0061VariantError(
            "S4 crossing/intervention timestamps are outside the pedestrian trajectory"
        )
    source = _insert_time_point(source, intervention)
    source = _insert_time_point(source, source_crossing_timestamp_sec)
    time_scale = (edited_crossing - intervention) / (
        source_crossing_timestamp_sec - intervention
    )
    result = []
    for point in source:
        source_time = float(point["t_sec"])
        edited = deepcopy(point)
        if source_time <= intervention:
            edited_time = source_time
        elif source_time <= source_crossing_timestamp_sec:
            edited_time = intervention + (source_time - intervention) * time_scale
            if edited.get("speed_mps") is not None:
                edited["speed_mps"] = float(edited["speed_mps"]) / time_scale
        else:
            edited_time = source_time - crossing_advance_sec
        edited["t_sec"] = edited_time
        if result and edited_time <= float(result[-1]["t_sec"]):
            raise Scene0061VariantError("S4 retiming produced non-increasing timestamps")
        result.append(edited)
    return result, {
        "operation": "pedestrian_corridor_speedup",
        "time_shift_sec": -crossing_advance_sec,
        "anticipation_window_sec": anticipation_window_sec,
        "time_scale_before_crossing": time_scale,
        "speed_scale_before_crossing": 1.0 / time_scale,
        "intervention_timestamp_sec": intervention,
        "source_crossing_event_timestamp_sec": source_crossing_timestamp_sec,
        "edited_crossing_event_timestamp_sec": edited_crossing,
        "pre_intervention_trajectory_unchanged": True,
        "corridor": "source_reference_corridor",
        "free_space_path_allowed": False,
        "skeleton_edit_allowed": False,
        "path_geometry_changed": False,
    }


def _validate_formal_source(config: Mapping[str, Any]) -> None:
    experiment = config.get("experiment") or {}
    if experiment.get("scene_id") != FORMAL_SCENE_ID:
        raise Scene0061VariantError("run config is not the formal scene-0061 identity")
    if experiment.get("scene_version") != FORMAL_SCENE_VERSION:
        raise Scene0061VariantError("run config scene_version is not formal40k-v1")


def _freeze_actor_control_modes(
    config: dict[str, Any], case_id: str
) -> dict[str, Any]:
    expected = CASE_ACTOR_CONTROL_MODES[case_id]
    rows = []
    for track_id in (VEHICLE_TRACK, PEDESTRIAN_TRACK):
        actor = _actor_for_track(config, track_id)
        actor_type = "vehicle" if track_id == VEHICLE_TRACK else "pedestrian"
        effective_mode = expected[track_id]
        source_mode = str(actor.get("closed_loop_level") or "unspecified")
        execution_evidence = (
            f"trajectory_replay_{'vehicle' if actor_type == 'vehicle' else 'walker'}_control"
            if effective_mode == "replay"
            else f"scripted_{'vehicle' if actor_type == 'vehicle' else 'walker'}_control"
        )
        pose_source = (
            "scenario_ir_reference_trajectory"
            if effective_mode == "replay"
            else "carla_runtime_actor_pose"
        )
        pose_reference = (
            "source_track_frame"
            if effective_mode == "replay"
            else (
                "carla_bounding_box_bottom"
                if actor_type == "pedestrian"
                else "carla_bounding_box_center"
            )
        )
        actor["closed_loop_level"] = effective_mode
        closed_loop = dict(actor.get("closed_loop") or {})
        closed_loop["ego_responsive"] = effective_mode == "scripted"
        actor["closed_loop"] = closed_loop
        actor["effective_control_mode"] = effective_mode
        actor["control_mode_contract"] = {
            "schema_version": "scene0061_actor_control_mode.v1",
            "case_id": case_id,
            "source_track_id": track_id,
            "source_mode": source_mode,
            "effective_mode": effective_mode,
            "runner_executor": execution_evidence,
            "immutable": True,
        }
        binding = actor.get("binding")
        if isinstance(binding, dict):
            binding["effective_control_mode"] = effective_mode
            binding["sensor_pose_source"] = pose_source
            binding["sensor_pose_reference"] = pose_reference
        rows.append(
            {
                "actor_id": str(actor.get("actor_id") or ""),
                "source_track_id": track_id,
                "actor_type": actor_type,
                "source_mode": source_mode,
                "effective_mode": effective_mode,
                "runner_executor": execution_evidence,
                "sensor_pose_source": pose_source,
                "sensor_pose_reference": pose_reference,
            }
        )
    case_mode = "replay" if case_id == "S0_original_replay" else "scripted"
    return {
        "schema_version": "scene0061_actor_control_contract.v1",
        "case_id": case_id,
        "case_actor_control_mode": case_mode,
        "effective_modes_by_track": deepcopy(expected),
        "actors": rows,
        "runner_semantics": {
            "replay": "reference_trajectory_without_ego_responsive_behavior_planner",
            "scripted": "reference_corridor_with_ego_responsive_behavior_planner",
        },
        "remote_validation_required": True,
    }


def _insert_time_point(
    points: list[dict[str, Any]], timestamp: float
) -> list[dict[str, Any]]:
    if any(abs(float(point["t_sec"]) - timestamp) <= 1e-9 for point in points):
        return points
    for index in range(1, len(points)):
        left, right = points[index - 1], points[index]
        left_time, right_time = float(left["t_sec"]), float(right["t_sec"])
        if left_time < timestamp < right_time:
            weight = (timestamp - left_time) / (right_time - left_time)
            inserted = deepcopy(left)
            inserted["t_sec"] = float(timestamp)
            for name in ("x", "y", "z", "yaw", "roll", "pitch", "speed_mps"):
                if left.get(name) is not None and right.get(name) is not None:
                    inserted[name] = float(left[name]) + weight * (
                        float(right[name]) - float(left[name])
                    )
            return points[:index] + [inserted] + points[index:]
    raise Scene0061VariantError("cannot insert event sample outside trajectory")


def _actor_for_track(config: dict[str, Any], track_id: str) -> dict[str, Any]:
    matches = []
    for actor in config.get("actors") or []:
        candidates = {
            actor.get("source_track_id"),
            actor.get("track_id"),
            (actor.get("binding") or {}).get("nurec_track_id"),
            ((actor.get("binding") or {}).get("nurec") or {}).get("track_id"),
        }
        if track_id in candidates:
            matches.append(actor)
    if len(matches) != 1:
        raise Scene0061VariantError(
            f"expected exactly one actor for track {track_id}, found {len(matches)}"
        )
    return matches[0]


def _validated_trajectory(value: list[Any]) -> list[dict[str, Any]]:
    if not isinstance(value, list) or len(value) < 2:
        raise Scene0061VariantError("actor reference trajectory requires at least two points")
    result = []
    last_time = -math.inf
    for index, raw in enumerate(value):
        if not isinstance(raw, Mapping) or any(raw.get(name) is None for name in ("t_sec", "x", "y")):
            raise Scene0061VariantError(f"trajectory point {index} lacks t_sec/x/y")
        row = deepcopy(dict(raw))
        time_value = float(row["t_sec"])
        if not math.isfinite(time_value) or time_value <= last_time:
            raise Scene0061VariantError("trajectory timestamps must be finite and increasing")
        if not all(math.isfinite(float(row[name])) for name in ("x", "y")):
            raise Scene0061VariantError("trajectory coordinates must be finite")
        last_time = time_value
        result.append(row)
    return result


def _speed_at(points: list[dict[str, Any]], t_sec: float) -> float:
    nearest = min(points, key=lambda row: abs(float(row["t_sec"]) - t_sec))
    if nearest.get("speed_mps") is not None:
        return max(0.0, float(nearest["speed_mps"]))
    index = points.index(nearest)
    left = points[max(0, index - 1)]
    right = points[min(len(points) - 1, index + 1)]
    dt = float(right["t_sec"]) - float(left["t_sec"])
    return (
        math.hypot(float(right["x"]) - float(left["x"]), float(right["y"]) - float(left["y"])) / dt
        if dt > 0.0
        else 0.0
    )


def _point_at_distance(
    points: list[dict[str, Any]], distances: list[float], target: float
) -> dict[str, Any]:
    for index in range(1, len(distances)):
        if target <= distances[index]:
            span = distances[index] - distances[index - 1]
            weight = 0.0 if span <= 0.0 else (target - distances[index - 1]) / span
            left, right = points[index - 1], points[index]
            result = deepcopy(left)
            for name in ("x", "y", "z", "yaw", "roll", "pitch"):
                if left.get(name) is not None and right.get(name) is not None:
                    result[name] = float(left[name]) + weight * (float(right[name]) - float(left[name]))
            return result
    return deepcopy(points[-1])


def _interpolate_scalar(xs: list[float], ys: list[float], x: float) -> float:
    for index in range(1, len(xs)):
        if x <= xs[index]:
            span = xs[index] - xs[index - 1]
            weight = 0.0 if span <= 0.0 else (x - xs[index - 1]) / span
            return ys[index - 1] + weight * (ys[index] - ys[index - 1])
    return ys[-1]


def _digest(value: Any) -> str:
    return sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    ).hexdigest()
