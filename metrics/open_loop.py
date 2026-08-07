"""Metrics and report contract for open-loop multimodal evaluation."""

from __future__ import annotations

import math
from copy import deepcopy
from typing import Any, Mapping


OPEN_LOOP_REPORT_SCHEMA = "open_loop_multimodal_report.v1"


def score_open_loop_predictions(
    scenario_ir: Mapping[str, Any],
    predictions: Any,
    *,
    scenario_ir_path: str | None = None,
    scenario_ir_sha256: str | None = None,
    opendrive_path: str | None = None,
    opendrive_sha256: str | None = None,
) -> dict[str, Any]:
    """Score predicted future points against the pinned IR trajectory.

    Prediction rows use ``frame_id`` and ``predicted_waypoints``. Each waypoint
    is either ``{"horizon_sec", "x", "y", "yaw"}`` or ``[x, y]``. Horizons
    are relative to the source frame, so the scorer never treats an algorithm
    prediction as a new ego pose.
    """

    ego_track = _track(
        (scenario_ir.get("ego") or {}).get("reference_trajectory"),
        "ego.reference_trajectory",
    )
    actor_tracks = _actor_tracks(scenario_ir.get("actors", []))
    rows = _prediction_rows(predictions)
    expected_ids = set(range(len(ego_track)))
    seen_ids: set[int] = set()
    frame_mismatch_count = 0
    dropped_frame_count = 0
    matched_rows: list[tuple[int, Mapping[str, Any]]] = []
    latency_values: list[float] = []
    for row in rows:
        frame_id = _integer(row.get("frame_id"), "prediction.frame_id")
        observation_frame_id = row.get("observation_frame_id", frame_id)
        if (
            not isinstance(observation_frame_id, int)
            or isinstance(observation_frame_id, bool)
            or observation_frame_id != frame_id
            or frame_id not in expected_ids
            or frame_id in seen_ids
        ):
            frame_mismatch_count += 1
            continue
        seen_ids.add(frame_id)
        if row.get("dropped") is True or row.get("execution_status") in {
            "dropped",
            "fallback",
        }:
            dropped_frame_count += 1
            continue
        matched_rows.append((frame_id, row))
        if _is_finite(row.get("inference_ms")):
            latency_values.append(float(row["inference_ms"]))

    errors: list[dict[str, float]] = []
    collision_hits: list[dict[str, Any]] = []
    for frame_id, row in matched_rows:
        base_t = ego_track[frame_id]["t_sec"]
        waypoints = row.get("predicted_waypoints", row.get("waypoints", []))
        if not isinstance(waypoints, list):
            raise ValueError(f"prediction frame {frame_id} waypoints must be a list")
        for waypoint_index, raw_waypoint in enumerate(waypoints):
            point = _waypoint(raw_waypoint, f"prediction[{frame_id}].waypoints[{waypoint_index}]")
            horizon = _finite(
                raw_waypoint.get("horizon_sec", raw_waypoint.get("t_sec", 0.0))
                if isinstance(raw_waypoint, Mapping)
                else 0.0,
                f"prediction[{frame_id}].waypoints[{waypoint_index}].horizon_sec",
            )
            if horizon < 0.0:
                raise ValueError("prediction waypoint horizon_sec must be non-negative")
            absolute_t = base_t + horizon
            truth = _state_at_time(ego_track, absolute_t)
            dx = point["x"] - truth["x"]
            dy = point["y"] - truth["y"]
            distance = math.hypot(dx, dy)
            lateral = -math.sin(math.radians(truth["yaw"])) * dx + math.cos(
                math.radians(truth["yaw"])
            ) * dy
            error = {
                "frame_id": float(frame_id),
                "horizon_sec": horizon,
                "distance_m": distance,
                "lateral_error_m": lateral,
            }
            predicted_yaw = point.get("yaw")
            if predicted_yaw is not None:
                error["heading_error_deg"] = _angle_delta(float(predicted_yaw), truth["yaw"])
            errors.append(error)
            hit = _collision_proxy(
                point,
                absolute_t,
                actor_tracks,
                ego_radius_m=float(row.get("ego_radius_m", 2.5)),
            )
            if hit is not None:
                collision_hits.append(
                    {
                        "frame_id": frame_id,
                        "horizon_sec": horizon,
                        "actor_id": hit,
                    }
                )

    distances = [item["distance_m"] for item in errors]
    lateral_errors = [abs(item["lateral_error_m"]) for item in errors]
    heading_errors = [item["heading_error_deg"] for item in errors if "heading_error_deg" in item]
    fde = None
    if errors:
        fde = max(
            errors,
            key=lambda item: (item["frame_id"], item["horizon_sec"]),
        )["distance_m"]
    latency = {
        "count": len(latency_values),
        "mean_ms": _mean(latency_values),
        "p95_ms": _percentile(latency_values, 95.0),
        "max_ms": max(latency_values) if latency_values else None,
    }
    source = scenario_ir.get("source") or {}
    scenario_id = str(scenario_ir.get("scenario_id", ""))
    report = {
        "schema_version": OPEN_LOOP_REPORT_SCHEMA,
        "scene_id": source.get("scene_name") or scenario_id,
        "scenario_id": scenario_id,
        "scene_version": source.get("version"),
        "execution_status": "completed" if matched_rows else "failed",
        "evidence_classification": "open_loop_multimodal",
        "real_carla_nurec_closed_loop": False,
        "remote_validation_required": True,
        "ego_pose_source": "scenario_ir_reference_trajectory",
        "control_affects_next_ego_pose": False,
        "claims_m8": False,
        "claims_m9": False,
        "matrix_actor_ready_ir_bound": False,
        "metrics": {
            "ade_m": _mean(distances),
            "fde_m": fde,
            "lateral_error_p95_m": _percentile(lateral_errors, 95.0),
            "heading_error_p95_deg": _percentile(heading_errors, 95.0),
            "prediction_point_count": len(errors),
            "collision_proxy_count": len(collision_hits),
            "collision_proxy_hits": collision_hits,
            "latency_ms": latency,
        },
        "frame_sync": {
            "source_frame_count": len(ego_track),
            "prediction_frame_count": len(rows),
            "matched_frame_count": len(matched_rows),
            "dropped_frame_count": dropped_frame_count,
            "frame_mismatch_count": frame_mismatch_count,
            "scored_frame_mismatch_count": 0,
        },
        "artifacts": {
            "scenario_ir_path": scenario_ir_path,
            "scenario_ir_sha256": scenario_ir_sha256,
            "opendrive_path": opendrive_path,
            "opendrive_sha256": opendrive_sha256,
        },
        "per_frame": errors,
    }
    validate_open_loop_report(report)
    return report


def validate_open_loop_report(report: Mapping[str, Any]) -> None:
    required = {
        "schema_version",
        "scene_id",
        "scenario_id",
        "execution_status",
        "evidence_classification",
        "ego_pose_source",
        "control_affects_next_ego_pose",
        "claims_m8",
        "claims_m9",
        "metrics",
        "frame_sync",
    }
    missing = sorted(required - set(report))
    if missing:
        raise ValueError(f"open-loop report missing fields: {missing}")
    if report["schema_version"] != OPEN_LOOP_REPORT_SCHEMA:
        raise ValueError("open-loop report schema_version is invalid")
    if report["evidence_classification"] != "open_loop_multimodal":
        raise ValueError("open-loop report evidence_classification is invalid")
    if report["ego_pose_source"] != "scenario_ir_reference_trajectory":
        raise ValueError("open-loop report must identify the IR pose source")
    for field in ("control_affects_next_ego_pose", "claims_m8", "claims_m9"):
        if report[field] is not False:
            raise ValueError(f"open-loop report {field} must be false")
    sync = report["frame_sync"]
    if not isinstance(sync, Mapping) or sync.get("scored_frame_mismatch_count") != 0:
        raise ValueError("open-loop report must have zero scored frame mismatches")


def _prediction_rows(predictions: Any) -> list[Mapping[str, Any]]:
    value = predictions.get("frames", predictions.get("predictions", [])) if isinstance(predictions, Mapping) else predictions
    if not isinstance(value, list):
        raise ValueError("predictions must contain a list of frames")
    if any(not isinstance(row, Mapping) for row in value):
        raise ValueError("every prediction frame must be an object")
    return [dict(row) for row in value]


def _actor_tracks(actors: Any) -> dict[str, tuple[list[dict[str, float]], float]]:
    if not isinstance(actors, list):
        return {}
    result = {}
    for actor in actors:
        if not isinstance(actor, Mapping):
            continue
        actor_id = str(actor.get("actor_id", ""))
        track = actor.get("reference_trajectory")
        if not actor_id or not isinstance(track, list) or not track:
            continue
        dimensions = actor.get("dimensions") or {}
        length = _finite(dimensions.get("length", 4.5), "actor length")
        width = _finite(dimensions.get("width", 1.8), "actor width")
        result[actor_id] = (_track(track, f"actor[{actor_id}].reference_trajectory"), max(length, width, 0.1) / 2.0)
    return result


def _collision_proxy(
    point: Mapping[str, float],
    t_sec: float,
    actor_tracks: Mapping[str, tuple[list[dict[str, float]], float]],
    *,
    ego_radius_m: float,
) -> str | None:
    for actor_id, (track, actor_radius) in actor_tracks.items():
        actor = _state_at_time(track, t_sec)
        if math.hypot(point["x"] - actor["x"], point["y"] - actor["y"]) <= ego_radius_m + actor_radius:
            return actor_id
    return None


def _track(value: Any, label: str) -> list[dict[str, float]]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"{label} must be a non-empty list")
    result = []
    previous_t = -math.inf
    for index, raw in enumerate(value):
        if not isinstance(raw, Mapping):
            raise ValueError(f"{label}[{index}] must be an object")
        state = {
            "t_sec": _finite(raw.get("t_sec"), f"{label}[{index}].t_sec"),
            "x": _finite(raw.get("x"), f"{label}[{index}].x"),
            "y": _finite(raw.get("y"), f"{label}[{index}].y"),
            "yaw": _finite(raw.get("yaw", 0.0), f"{label}[{index}].yaw"),
        }
        if state["t_sec"] < previous_t:
            raise ValueError(f"{label} timestamps must be monotonic")
        previous_t = state["t_sec"]
        result.append(state)
    return result


def _state_at_time(track: list[dict[str, float]], t_sec: float) -> dict[str, float]:
    if t_sec <= track[0]["t_sec"]:
        return deepcopy(track[0])
    if t_sec >= track[-1]["t_sec"]:
        return deepcopy(track[-1])
    for left, right in zip(track, track[1:]):
        if left["t_sec"] <= t_sec <= right["t_sec"]:
            duration = right["t_sec"] - left["t_sec"]
            ratio = 0.0 if duration <= 0.0 else (t_sec - left["t_sec"]) / duration
            return {
                "t_sec": float(t_sec),
                "x": left["x"] + ratio * (right["x"] - left["x"]),
                "y": left["y"] + ratio * (right["y"] - left["y"]),
                "yaw": left["yaw"] + ratio * (right["yaw"] - left["yaw"]),
            }
    return deepcopy(track[-1])


def _waypoint(value: Any, label: str) -> dict[str, float]:
    if isinstance(value, Mapping):
        return {
            "x": _finite(value.get("x"), f"{label}.x"),
            "y": _finite(value.get("y"), f"{label}.y"),
            **(
                {"yaw": _finite(value["yaw"], f"{label}.yaw")}
                if value.get("yaw") is not None
                else {}
            ),
        }
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return {"x": _finite(value[0], f"{label}[0]"), "y": _finite(value[1], f"{label}[1]")}
    raise ValueError(f"{label} must contain x/y")


def _integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    return int(value)


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{label} must be finite")
    return float(value)


def _is_finite(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _angle_delta(left: float, right: float) -> float:
    return abs((left - right + 180.0) % 360.0 - 180.0)


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction
