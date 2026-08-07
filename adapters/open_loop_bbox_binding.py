"""Auditable dynamic-actor bindings for open-loop bbox evaluation.

The Scenario IR is the metric ground truth.  The formal NuRec USDZ is used to
prove that the same source track exists in the rendered scene and covers the
requested replay window.  No actor is selected from a nearest visual guess.
"""

from __future__ import annotations

import hashlib
import json
import math
import zipfile
from copy import deepcopy
from pathlib import Path
from typing import Any, Iterable, Mapping


MANIFEST_SCHEMA = "open_loop_bbox_actor_manifest.v1"
SUPPORTED_ACTOR_TYPES = frozenset({"vehicle", "pedestrian"})
VEHICLE_USDZ_LABELS = frozenset(
    {"automobile", "heavy_truck", "bus", "Other Vehicle - Construction Vehicle"}
)
PEDestrian_USDZ_LABELS = frozenset({"pedestrian"})
DEFAULT_FRAME_INTERVAL_SEC = 0.05
DEFAULT_USDZ_TIME_TOLERANCE_US = 120_000


class OpenLoopBBoxBindingError(ValueError):
    """Raised when actor geometry or temporal provenance is incomplete."""


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise OpenLoopBBoxBindingError(f"cannot read {path}: {exc}") from exc
    return digest.hexdigest()


def build_actor_manifest(
    scenario_ir: Mapping[str, Any],
    *,
    scenario_ir_path: str | Path,
    usdz_path: str | Path,
    frame_interval_sec: float = DEFAULT_FRAME_INTERVAL_SEC,
    usdz_time_tolerance_us: int = DEFAULT_USDZ_TIME_TOLERANCE_US,
) -> dict[str, Any]:
    """Build one actor manifest shared by all three open-loop routes."""

    _validate_scenario_ir(scenario_ir)
    scenario_path = Path(scenario_ir_path).resolve()
    artifact_path = Path(usdz_path).resolve()
    if not scenario_path.is_file():
        raise OpenLoopBBoxBindingError(f"Scenario IR is unavailable: {scenario_path}")
    if not artifact_path.is_file():
        raise OpenLoopBBoxBindingError(f"NuRec USDZ is unavailable: {artifact_path}")
    if (
        isinstance(frame_interval_sec, bool)
        or not isinstance(frame_interval_sec, (int, float))
        or not math.isfinite(float(frame_interval_sec))
        or float(frame_interval_sec) < 0.0
    ):
        raise OpenLoopBBoxBindingError("frame_interval_sec must be finite and non-negative")
    if (
        isinstance(usdz_time_tolerance_us, bool)
        or not isinstance(usdz_time_tolerance_us, int)
        or usdz_time_tolerance_us < 0
    ):
        raise OpenLoopBBoxBindingError("usdz_time_tolerance_us must be a non-negative integer")

    usdz = _load_usdz_tracks(artifact_path)
    ir_actors = _actor_rows(scenario_ir)
    selected: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    for index, actor in enumerate(ir_actors):
        actor_type = str(actor.get("type") or "")
        if actor_type not in SUPPORTED_ACTOR_TYPES:
            continue
        track_id = str(actor.get("source_track_id") or actor.get("actor_id") or "")
        if not track_id:
            raise OpenLoopBBoxBindingError(f"Scenario IR actor {index} has no source_track_id")
        track = usdz["tracks"].get(track_id)
        if track is None:
            rejected.append(
                {"actor_id": str(actor.get("actor_id") or track_id), "reason": "usdz_track_missing"}
            )
            continue
        expected_labels = (
            VEHICLE_USDZ_LABELS if actor_type == "vehicle" else PEDestrian_USDZ_LABELS
        )
        if track["flags"] != "DYNAMIC|CONTROLLABLE":
            rejected.append(
                {
                    "actor_id": str(actor.get("actor_id") or track_id),
                    "reason": "usdz_track_not_dynamic_controllable",
                    "flags": track["flags"],
                }
            )
            continue
        if track["label"] not in expected_labels:
            rejected.append(
                {
                    "actor_id": str(actor.get("actor_id") or track_id),
                    "reason": "usdz_label_type_mismatch",
                    "actor_type": actor_type,
                    "usdz_label": track["label"],
                }
            )
            continue
        dimensions = _dimensions(actor.get("dimensions"), f"actors[{index}].dimensions")
        dimension_delta = max(
            abs(left - right) for left, right in zip(dimensions, track["dimensions_m"])
        )
        if dimension_delta > 0.05:
            raise OpenLoopBBoxBindingError(
                f"actor {track_id} dimensions disagree with USDZ by {dimension_delta:.6f} m"
            )
        trajectory = _trajectory(
            actor.get("reference_trajectory"), f"actors[{index}].reference_trajectory"
        )
        if not trajectory:
            raise OpenLoopBBoxBindingError(f"actor {track_id} has an empty reference trajectory")
        selected.append(
            {
                "actor_id": str(actor.get("actor_id") or track_id),
                "source_track_id": track_id,
                "actor_type": actor_type,
                "category": str(actor.get("category") or ""),
                "dimensions_m": {
                    "length": dimensions[0],
                    "width": dimensions[1],
                    "height": dimensions[2],
                },
                "ir_trajectory": trajectory,
                "ir_active_window_sec": {
                    "first": trajectory[0]["t_sec"],
                    "last": trajectory[-1]["t_sec"],
                },
                "usdz_track": {
                    "label": track["label"],
                    "flags": track["flags"],
                    "pose_count": len(track["poses"]),
                    "first_timestamp_us": track["timestamps_us"][0],
                    "last_timestamp_us": track["timestamps_us"][-1],
                    "first_time_sec": (track["timestamps_us"][0] - usdz["time_origin_us"])
                    / 1_000_000.0,
                    "last_time_sec": (track["timestamps_us"][-1] - usdz["time_origin_us"])
                    / 1_000_000.0,
                    "dimensions_m": list(track["dimensions_m"]),
                },
            }
        )

    if not selected:
        raise OpenLoopBBoxBindingError("USDZ/Scenario IR intersection has no supported dynamic actors")
    selected.sort(key=lambda item: item["source_track_id"])
    actor_ids = [item["source_track_id"] for item in selected]
    if len(actor_ids) != len(set(actor_ids)):
        raise OpenLoopBBoxBindingError("selected actor source_track_id values are not unique")

    ego_track = _trajectory(
        (scenario_ir.get("ego") or {}).get("reference_trajectory"),
        "ego.reference_trajectory",
    )
    source_start_us = _source_start_us(scenario_ir)
    frames = []
    for frame_id, ego_state in enumerate(ego_track):
        timestamp = float(ego_state["t_sec"])
        interval_start = max(0.0, timestamp - float(frame_interval_sec))
        frame = _build_frame(
            selected,
            frame_id=frame_id,
            timestamp_sec=timestamp,
            interval_start_sec=interval_start,
            usdz=usdz,
            source_start_us=source_start_us,
            usdz_time_tolerance_us=usdz_time_tolerance_us,
        )
        frame["ego_timestamp_us"] = source_start_us + int(round(timestamp * 1_000_000.0))
        frames.append(frame)

    manifest = {
        "schema_version": MANIFEST_SCHEMA,
        "scene_id": str(scenario_ir["scenario_id"]),
        "scenario_id": str(scenario_ir["scenario_id"]),
        "scenario_ir": {
            "path": str(scenario_path),
            "sha256": sha256_file(scenario_path),
        },
        "usdz": {
            "path": str(artifact_path),
            "sha256": sha256_file(artifact_path),
            "byte_count": artifact_path.stat().st_size,
            "sequence_tracks_entry": usdz["sequence_tracks_entry"],
            "sequence_tracks_sha256": usdz["sequence_tracks_sha256"],
            "time_origin_us": usdz["time_origin_us"],
            "source_start_timestamp_us": source_start_us,
            "source_start_offset_us": usdz["time_origin_us"] - source_start_us,
        },
        "coordinate_frame": {
            "scenario_ir": "scene_local_ego_start_x_forward_y_left_z_up",
            "nurec_dynamic_object_input": "scene_local_ego_start_x_forward_y_left_z_up",
            "carla_runtime": "scene_local_ego_start_x_forward_y_right_z_up",
            "yaw_unit": "degree",
        },
        "policy": {
            "ground_truth_source": "scenario_ir_actor_reference_trajectory",
            "dynamic_input_source": "scenario_ir_pose_bound_to_usdz_sequence_track",
            "supported_actor_types": sorted(SUPPORTED_ACTOR_TYPES),
            "excluded_actor_types": ["object", "two_wheeler"],
            "active_window": "trajectory_first_last_timestamp_inclusive",
            "frame_pose_policy": "exact_ir_interpolation_no_out_of_window_extrapolation",
            "pose_pair_policy": "interpolate_when_interval_is_inside_track_else_exact_frame_pose",
            "same_dynamic_object_for_rgb_and_lidar": True,
            "fail_closed": True,
        },
        "actors": selected,
        "rejected_supported_ir_actors": rejected,
        "summary": {
            "actor_count": len(selected),
            "vehicle_count": sum(item["actor_type"] == "vehicle" for item in selected),
            "pedestrian_count": sum(item["actor_type"] == "pedestrian" for item in selected),
            "frame_count": len(frames),
            "source_ir_actor_count": len(ir_actors),
        },
        "frames": frames,
    }
    manifest["manifest_sha256"] = manifest_content_sha256(manifest)
    validate_actor_manifest(manifest)
    return manifest


def load_actor_manifest(
    path: str | Path,
    *,
    expected_scenario_ir_sha256: str | None = None,
    expected_scene_id: str | None = None,
    require_usdz_file: bool = True,
) -> dict[str, Any]:
    manifest_path = Path(path).resolve()
    try:
        value = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise OpenLoopBBoxBindingError(f"cannot read actor manifest {manifest_path}: {exc}") from exc
    if not isinstance(value, dict):
        raise OpenLoopBBoxBindingError("actor manifest must be a JSON object")
    validate_actor_manifest(value, require_usdz_file=require_usdz_file)
    if expected_scenario_ir_sha256:
        actual = (value.get("scenario_ir") or {}).get("sha256")
        if actual != expected_scenario_ir_sha256:
            raise OpenLoopBBoxBindingError("actor manifest Scenario IR SHA-256 does not match expected input")
    if expected_scene_id and value.get("scene_id") != expected_scene_id:
        raise OpenLoopBBoxBindingError("actor manifest scene_id does not match expected scene")
    value["manifest_path"] = str(manifest_path)
    value["manifest_file_sha256"] = sha256_file(manifest_path)
    return value


def write_actor_manifest(path: str | Path, manifest: Mapping[str, Any]) -> None:
    target = Path(path)
    if target.exists():
        raise OpenLoopBBoxBindingError(f"refusing to overwrite actor manifest: {target}")
    validate_actor_manifest(manifest)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def manifest_content_sha256(manifest: Mapping[str, Any]) -> str:
    value = deepcopy(dict(manifest))
    value.pop("manifest_sha256", None)
    value.pop("manifest_path", None)
    value.pop("manifest_file_sha256", None)
    return canonical_sha256(value)


def frame_binding(manifest: Mapping[str, Any], frame_id: int) -> dict[str, Any]:
    validate_actor_manifest(manifest, require_usdz_file=False)
    frames = manifest["frames"]
    if isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id < 0 or frame_id >= len(frames):
        raise OpenLoopBBoxBindingError(f"actor manifest has no frame {frame_id}")
    return deepcopy(frames[frame_id])


def frame_dynamic_objects(manifest: Mapping[str, Any], frame_id: int) -> list[dict[str, Any]]:
    """Return the exact simulator-frame actor payload for one frame."""

    return frame_binding(manifest, frame_id)["dynamic_objects"]


def actor_ground_truth_rows(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Return manifest actor rows in the shape consumed by the bbox evaluator."""

    validate_actor_manifest(manifest, require_usdz_file=False)
    result = []
    for actor in manifest["actors"]:
        result.append(
            {
                "track_id": actor["source_track_id"],
                "actor_id": actor["actor_id"],
                "actor_type": actor["actor_type"],
                "length_m": actor["dimensions_m"]["length"],
                "width_m": actor["dimensions_m"]["width"],
                "height_m": actor["dimensions_m"]["height"],
                "track": deepcopy(actor["ir_trajectory"]),
                "active_window_sec": deepcopy(actor["ir_active_window_sec"]),
            }
        )
    return result


def _build_frame(
    actors: list[Mapping[str, Any]],
    *,
    frame_id: int,
    timestamp_sec: float,
    interval_start_sec: float,
    usdz: Mapping[str, Any],
    source_start_us: int,
    usdz_time_tolerance_us: int,
) -> dict[str, Any]:
    dynamic_objects: list[dict[str, Any]] = []
    usdz_bindings: list[dict[str, Any]] = []
    gt_active_ids: list[str] = []
    for actor in actors:
        trajectory = actor["ir_trajectory"]
        first = float(actor["ir_active_window_sec"]["first"])
        last = float(actor["ir_active_window_sec"]["last"])
        active = first - 1e-6 <= timestamp_sec <= last + 1e-6
        if not active:
            continue
        track_id = str(actor["source_track_id"])
        ir_state = _state_at_time(trajectory, timestamp_sec, f"actor {track_id}")
        gt_active_ids.append(track_id)
        track = usdz["tracks"][track_id]
        absolute_us = source_start_us + int(round(timestamp_sec * 1_000_000.0))
        nearest_index, nearest_delta = _nearest_timestamp(track["timestamps_us"], absolute_us)
        if absolute_us < track["timestamps_us"][0] - usdz_time_tolerance_us:
            raise OpenLoopBBoxBindingError(
                f"actor {track_id} USDZ track does not cover frame {frame_id}: "
                f"target precedes first timestamp by "
                f"{track['timestamps_us'][0] - absolute_us} us"
            )
        if absolute_us > track["timestamps_us"][-1] + usdz_time_tolerance_us:
            raise OpenLoopBBoxBindingError(
                f"actor {track_id} USDZ track does not cover frame {frame_id}: "
                f"target follows last timestamp by "
                f"{absolute_us - track['timestamps_us'][-1]} us"
            )
        interval_inside = first - 1e-6 <= interval_start_sec <= last + 1e-6
        start_state = (
            _state_at_time(trajectory, interval_start_sec, f"actor {track_id}")
            if interval_inside
            else ir_state
        )
        dimensions = deepcopy(actor["dimensions_m"])
        dynamic_objects.append(
            {
                "actor_id": actor["actor_id"],
                "track_id": track_id,
                "actor_type": actor["actor_type"],
                "dimensions_m": dimensions,
                "pose_source": "scenario_ir_actor_reference_trajectory",
                "pose_reference": "actor_bbox_center",
                "pose_pair": {
                    "start": _pose(start_state),
                    "end": _pose(ir_state),
                },
                "usdz_track_id": track_id,
                "usdz_nearest_timestamp_us": track["timestamps_us"][nearest_index],
                "usdz_nearest_delta_us": nearest_delta,
                "usdz_pose_sampling": (
                    "exact_keyframe"
                    if nearest_delta <= usdz_time_tolerance_us
                    else "interpolated_inside_track_window"
                ),
                "pose_pair_interval_inside_ir_track": interval_inside,
            }
        )
        usdz_bindings.append(
            {
                "actor_id": actor["actor_id"],
                "track_id": track_id,
                "nearest_timestamp_us": track["timestamps_us"][nearest_index],
                "nearest_delta_us": nearest_delta,
                "track_first_timestamp_us": track["timestamps_us"][0],
                "track_last_timestamp_us": track["timestamps_us"][-1],
            }
        )
    dynamic_objects.sort(key=lambda item: str(item["track_id"]))
    usdz_bindings.sort(key=lambda item: str(item["track_id"]))
    active_ids = [str(item["track_id"]) for item in dynamic_objects]
    pose_digest_payload = [
        {
            "track_id": item["track_id"],
            "actor_type": item["actor_type"],
            "dimensions_m": item["dimensions_m"],
            "pose_pair": item["pose_pair"],
        }
        for item in dynamic_objects
    ]
    return {
        "frame_id": frame_id,
        "timestamp_sec": timestamp_sec,
        "interval_start_sec": interval_start_sec,
        "gt_active_actor_ids": sorted(gt_active_ids),
        "active_actor_ids": active_ids,
        "active_actor_set_sha256": canonical_sha256(active_ids),
        "pose_digest": canonical_sha256(pose_digest_payload),
        "dynamic_object_sha256": canonical_sha256(dynamic_objects),
        "usdz_track_bindings": usdz_bindings,
        "dynamic_objects": dynamic_objects,
    }


def validate_actor_manifest(
    manifest: Mapping[str, Any], *, require_usdz_file: bool = True
) -> None:
    if manifest.get("schema_version") != MANIFEST_SCHEMA:
        raise OpenLoopBBoxBindingError("unsupported actor manifest schema")
    scene_id = str(manifest.get("scene_id") or "")
    if not scene_id or scene_id != str(manifest.get("scenario_id") or ""):
        raise OpenLoopBBoxBindingError("actor manifest scene/scenario identity is invalid")
    scenario_ref = manifest.get("scenario_ir")
    usdz_ref = manifest.get("usdz")
    if not isinstance(scenario_ref, Mapping) or not _is_sha256(str(scenario_ref.get("sha256") or "")):
        raise OpenLoopBBoxBindingError("actor manifest Scenario IR reference is invalid")
    if not isinstance(usdz_ref, Mapping) or not _is_sha256(str(usdz_ref.get("sha256") or "")):
        raise OpenLoopBBoxBindingError("actor manifest USDZ reference is invalid")
    usdz_path = Path(str(usdz_ref.get("path") or ""))
    if require_usdz_file:
        if not usdz_path.is_file():
            raise OpenLoopBBoxBindingError(f"actor manifest USDZ is unavailable: {usdz_path}")
        if sha256_file(usdz_path) != usdz_ref["sha256"]:
            raise OpenLoopBBoxBindingError("actor manifest USDZ SHA-256 mismatch")
    actors = manifest.get("actors")
    frames = manifest.get("frames")
    if not isinstance(actors, list) or not actors:
        raise OpenLoopBBoxBindingError("actor manifest actors must be a non-empty list")
    if not isinstance(frames, list) or not frames:
        raise OpenLoopBBoxBindingError("actor manifest frames must be a non-empty list")
    source_ids: set[str] = set()
    for actor in actors:
        if not isinstance(actor, Mapping):
            raise OpenLoopBBoxBindingError("actor manifest actor entry must be an object")
        track_id = str(actor.get("source_track_id") or "")
        if not track_id or track_id in source_ids:
            raise OpenLoopBBoxBindingError("actor manifest source track IDs must be unique")
        source_ids.add(track_id)
        if actor.get("actor_type") not in SUPPORTED_ACTOR_TYPES:
            raise OpenLoopBBoxBindingError(f"unsupported actor type in manifest: {actor.get('actor_type')}")
        dimensions = actor.get("dimensions_m")
        if not isinstance(dimensions, Mapping) or any(
            not _positive_finite(dimensions.get(name)) for name in ("length", "width", "height")
        ):
            raise OpenLoopBBoxBindingError(f"actor {track_id} dimensions are invalid")
        _trajectory(actor.get("ir_trajectory"), f"actor {track_id}.ir_trajectory")
    for expected_frame_id, frame in enumerate(frames):
        _validate_frame(frame, expected_frame_id, source_ids)
    expected_manifest_sha = manifest.get("manifest_sha256")
    if not _is_sha256(str(expected_manifest_sha or "")):
        raise OpenLoopBBoxBindingError("actor manifest manifest_sha256 is missing or invalid")
    if expected_manifest_sha != manifest_content_sha256(manifest):
        raise OpenLoopBBoxBindingError("actor manifest content SHA-256 mismatch")


def _validate_frame(frame: Any, expected_frame_id: int, source_ids: set[str]) -> None:
    if not isinstance(frame, Mapping) or frame.get("frame_id") != expected_frame_id:
        raise OpenLoopBBoxBindingError(f"actor manifest frame identity failed at {expected_frame_id}")
    objects = frame.get("dynamic_objects")
    if not isinstance(objects, list):
        raise OpenLoopBBoxBindingError(f"actor manifest frame {expected_frame_id} dynamic_objects is invalid")
    ids = [str(item.get("track_id") or "") for item in objects if isinstance(item, Mapping)]
    if len(ids) != len(objects) or len(ids) != len(set(ids)) or any(item not in source_ids for item in ids):
        raise OpenLoopBBoxBindingError(f"actor manifest frame {expected_frame_id} actor IDs are invalid")
    if frame.get("active_actor_ids") != ids:
        raise OpenLoopBBoxBindingError(f"actor manifest frame {expected_frame_id} active set is inconsistent")
    if frame.get("active_actor_set_sha256") != canonical_sha256(ids):
        raise OpenLoopBBoxBindingError(f"actor manifest frame {expected_frame_id} active set digest is invalid")
    if frame.get("dynamic_object_sha256") != canonical_sha256(objects):
        raise OpenLoopBBoxBindingError(f"actor manifest frame {expected_frame_id} dynamic digest is invalid")
    pose_payload = [
        {
            "track_id": item["track_id"],
            "actor_type": item["actor_type"],
            "dimensions_m": item["dimensions_m"],
            "pose_pair": item["pose_pair"],
        }
        for item in objects
    ]
    if frame.get("pose_digest") != canonical_sha256(pose_payload):
        raise OpenLoopBBoxBindingError(f"actor manifest frame {expected_frame_id} pose digest is invalid")
    if not isinstance(frame.get("gt_active_actor_ids"), list):
        raise OpenLoopBBoxBindingError(f"actor manifest frame {expected_frame_id} GT active set is invalid")
    for item in objects:
        if not isinstance(item, Mapping):
            raise OpenLoopBBoxBindingError("dynamic object must be an object")
        if not item.get("actor_id") or not item.get("track_id"):
            raise OpenLoopBBoxBindingError("dynamic object actor_id and track_id are required")
        _pose_pair(item.get("pose_pair"), f"frame {expected_frame_id} actor {item.get('track_id')}")


def _load_usdz_tracks(path: Path) -> dict[str, Any]:
    try:
        with zipfile.ZipFile(path) as archive:
            names = [name for name in archive.namelist() if name.endswith("sequence_tracks.json")]
            if len(names) != 1:
                raise OpenLoopBBoxBindingError(
                    f"USDZ must contain exactly one sequence_tracks.json, found {len(names)}"
                )
            body = archive.read(names[0])
    except (OSError, zipfile.BadZipFile, KeyError) as exc:
        raise OpenLoopBBoxBindingError(f"cannot read USDZ sequence tracks: {path}: {exc}") from exc
    try:
        document = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as exc:
        raise OpenLoopBBoxBindingError("USDZ sequence_tracks.json is invalid JSON") from exc
    track_rows: list[dict[str, Any]] = []
    chunks = _track_chunks(document)
    if not chunks:
        raise OpenLoopBBoxBindingError("USDZ sequence_tracks.json has no tracks_data")
    for chunk in chunks:
        tracks_data = chunk.get("tracks_data")
        cuboids_data = chunk.get("cuboidtracks_data") or {}
        if not isinstance(tracks_data, Mapping):
            raise OpenLoopBBoxBindingError("USDZ tracks_data must be an object")
        ids = tracks_data.get("tracks_id")
        poses = tracks_data.get("tracks_poses")
        timestamps = tracks_data.get("tracks_timestamps_us")
        labels = tracks_data.get("tracks_label_class")
        flags = tracks_data.get("tracks_flags")
        dimensions = cuboids_data.get("cuboids_dims")
        arrays = (ids, poses, timestamps, labels, flags, dimensions)
        if any(not isinstance(value, list) for value in arrays):
            raise OpenLoopBBoxBindingError("USDZ tracks_data arrays are incomplete")
        lengths = {len(value) for value in arrays}
        if len(lengths) != 1:
            raise OpenLoopBBoxBindingError("USDZ tracks_data arrays have different lengths")
        for index in range(len(ids)):
            track_id = str(ids[index] or "")
            if not track_id:
                raise OpenLoopBBoxBindingError("USDZ track ID is empty")
            pose_rows = poses[index]
            timestamp_rows = timestamps[index]
            if not isinstance(pose_rows, list) or not isinstance(timestamp_rows, list):
                raise OpenLoopBBoxBindingError(f"USDZ track {track_id} pose/timestamp arrays are invalid")
            if len(pose_rows) != len(timestamp_rows) or not pose_rows:
                raise OpenLoopBBoxBindingError(f"USDZ track {track_id} has invalid temporal coverage")
            parsed_poses = [_usdz_pose(row, track_id) for row in pose_rows]
            parsed_timestamps = [int(value) for value in timestamp_rows]
            if parsed_timestamps != sorted(parsed_timestamps):
                raise OpenLoopBBoxBindingError(f"USDZ track {track_id} timestamps are not monotonic")
            dims = dimensions[index]
            if not isinstance(dims, list) or len(dims) != 3 or any(not _positive_finite(item) for item in dims):
                raise OpenLoopBBoxBindingError(f"USDZ track {track_id} dimensions are invalid")
            track_rows.append(
                {
                    "track_id": track_id,
                    "poses": parsed_poses,
                    "timestamps_us": parsed_timestamps,
                    "label": str(labels[index] or ""),
                    "flags": str(flags[index] or ""),
                    "dimensions_m": [float(item) for item in dims],
                }
            )
    tracks: dict[str, dict[str, Any]] = {}
    for row in track_rows:
        if row["track_id"] in tracks:
            raise OpenLoopBBoxBindingError(f"duplicate USDZ track ID: {row['track_id']}")
        tracks[row["track_id"]] = row
    time_origin_us = min(row["timestamps_us"][0] for row in track_rows)
    return {
        "tracks": tracks,
        "time_origin_us": time_origin_us,
        "sequence_tracks_entry": names[0],
        "sequence_tracks_sha256": hashlib.sha256(body).hexdigest(),
    }


def _track_chunks(document: Any) -> list[Mapping[str, Any]]:
    if not isinstance(document, Mapping):
        return []
    if isinstance(document.get("tracks_data"), Mapping):
        return [document]
    result = []
    for value in document.values():
        if isinstance(value, Mapping) and isinstance(value.get("tracks_data"), Mapping):
            result.append(value)
    return result


def _usdz_pose(value: Any, track_id: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 7:
        raise OpenLoopBBoxBindingError(f"USDZ track {track_id} pose must contain 7 values")
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise OpenLoopBBoxBindingError(f"USDZ track {track_id} pose is not numeric") from exc
    if any(not math.isfinite(item) for item in result):
        raise OpenLoopBBoxBindingError(f"USDZ track {track_id} pose contains non-finite values")
    quaternion_norm = math.sqrt(sum(item * item for item in result[3:]))
    if quaternion_norm <= 1e-6:
        raise OpenLoopBBoxBindingError(f"USDZ track {track_id} quaternion is empty")
    return result


def _actor_rows(scenario_ir: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    actors = scenario_ir.get("actors")
    if not isinstance(actors, list):
        raise OpenLoopBBoxBindingError("scenario_ir.actors must be a list")
    return actors


def _validate_scenario_ir(scenario_ir: Mapping[str, Any]) -> None:
    if not isinstance(scenario_ir, Mapping) or scenario_ir.get("schema_version") != "scenario_ir.v1":
        raise OpenLoopBBoxBindingError("actor manifest requires scenario_ir.v1")
    if not str(scenario_ir.get("scenario_id") or ""):
        raise OpenLoopBBoxBindingError("Scenario IR scenario_id is required")


def _source_start_us(scenario_ir: Mapping[str, Any]) -> int:
    source = scenario_ir.get("source") or {}
    value = source.get("start_timestamp_us")
    if isinstance(value, bool) or not isinstance(value, int):
        raise OpenLoopBBoxBindingError("Scenario IR source.start_timestamp_us is required")
    return value


def _dimensions(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, Mapping):
        raise OpenLoopBBoxBindingError(f"{label} must be an object")
    try:
        # Scenario IR stores width before length in its serialized actor rows;
        # normalize the named fields here so all downstream geometry is L/W/H.
        result = tuple(float(value[name]) for name in ("length", "width", "height"))
    except (KeyError, TypeError, ValueError) as exc:
        try:
            result = tuple(float(value[name]) for name in ("width", "length", "height"))
            result = (result[1], result[0], result[2])
        except (KeyError, TypeError, ValueError) as fallback_exc:
            raise OpenLoopBBoxBindingError(f"{label} must contain length/width/height") from fallback_exc
    if any(not _positive_finite(item) for item in result):
        raise OpenLoopBBoxBindingError(f"{label} must be positive and finite")
    return result


def _trajectory(value: Any, label: str) -> list[dict[str, float]]:
    if not isinstance(value, list):
        raise OpenLoopBBoxBindingError(f"{label} must be a list")
    result = []
    previous = -math.inf
    for index, row in enumerate(value):
        if not isinstance(row, Mapping):
            raise OpenLoopBBoxBindingError(f"{label}[{index}] must be an object")
        try:
            state = {
                "t_sec": float(row["t_sec"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row.get("z", 0.0)),
                "yaw": float(row.get("yaw", 0.0)),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise OpenLoopBBoxBindingError(f"{label}[{index}] is incomplete") from exc
        if any(not math.isfinite(item) for item in state.values()) or state["t_sec"] < previous:
            raise OpenLoopBBoxBindingError(f"{label}[{index}] is non-finite or not monotonic")
        previous = state["t_sec"]
        result.append(state)
    return result


def _state_at_time(track: list[Mapping[str, Any]], timestamp: float, label: str) -> dict[str, float]:
    if not track or timestamp < float(track[0]["t_sec"]) - 1e-6 or timestamp > float(track[-1]["t_sec"]) + 1e-6:
        raise OpenLoopBBoxBindingError(f"{label} has no in-window pose at {timestamp:.6f} sec")
    if timestamp <= float(track[0]["t_sec"]):
        return {key: float(track[0][key]) for key in ("t_sec", "x", "y", "z", "yaw")}
    if timestamp >= float(track[-1]["t_sec"]):
        return {key: float(track[-1][key]) for key in ("t_sec", "x", "y", "z", "yaw")}
    for left, right in zip(track, track[1:]):
        left_t = float(left["t_sec"])
        right_t = float(right["t_sec"])
        if left_t <= timestamp <= right_t:
            ratio = 0.0 if right_t <= left_t else (timestamp - left_t) / (right_t - left_t)
            return {
                key: float(left[key]) + ratio * (float(right[key]) - float(left[key]))
                for key in ("x", "y", "z", "yaw")
            } | {"t_sec": float(timestamp)}
    raise OpenLoopBBoxBindingError(f"{label} interpolation failed at {timestamp:.6f} sec")


def _pose(state: Mapping[str, Any]) -> dict[str, float]:
    return {name: float(state.get(name, 0.0)) for name in ("x", "y", "z", "yaw")}


def _nearest_timestamp(values: list[int], target: int) -> tuple[int, int]:
    best_index = min(range(len(values)), key=lambda index: abs(values[index] - target))
    return best_index, abs(values[best_index] - target)


def _positive_finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value)) and float(value) > 0.0


def _pose_pair(value: Any, label: str) -> None:
    if not isinstance(value, Mapping):
        raise OpenLoopBBoxBindingError(f"{label} pose_pair must be an object")
    for endpoint in ("start", "end"):
        pose = value.get(endpoint)
        if not isinstance(pose, Mapping):
            raise OpenLoopBBoxBindingError(f"{label} {endpoint} pose is missing")
        for name in ("x", "y", "z", "yaw"):
            if not isinstance(pose.get(name), (int, float)) or isinstance(pose.get(name), bool) or not math.isfinite(float(pose[name])):
                raise OpenLoopBBoxBindingError(f"{label} {endpoint}.{name} is invalid")


def _is_sha256(value: str) -> bool:
    return len(value) == 64 and all(item in "0123456789abcdef" for item in value)
