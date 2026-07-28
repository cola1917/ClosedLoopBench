from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.lidar_world_support import (
    LidarWorldSupportError,
    expected_lidar_support_from_physical_boxes,
)
from runners.build_m8_lidar_occupancy import _lidar_sensor_to_ego


DEFAULT_MAX_RANGE_M = 80.0


def build_expected_lidar_support(
    runtime_rows: list[Mapping[str, Any]],
    run_config: Mapping[str, Any],
    *,
    max_range_m: float = DEFAULT_MAX_RANGE_M,
    nurec_rows: list[Mapping[str, Any]] | None = None,
    source_lidar_support: Mapping[str, Any] | None = None,
    source_lidar_support_sha256: str | None = None,
    static_source_lidar_support: Mapping[str, Any] | None = None,
    static_source_lidar_support_sha256: str | None = None,
) -> list[dict[str, Any]]:
    """Build explicit same-tick CARLA and source-LiDAR expectations."""

    sensor_to_ego = _lidar_sensor_to_ego(run_config)
    source_mode = nurec_rows is not None or source_lidar_support is not None
    if static_source_lidar_support is not None and not source_mode:
        raise LidarWorldSupportError(
            "static source-backed M8 LiDAR support requires the dynamic source trace"
        )
    if source_mode and (nurec_rows is None or source_lidar_support is None):
        raise LidarWorldSupportError(
            "source-backed M8 LiDAR expectations require both NuRec and source LiDAR traces"
        )
    if source_mode and not _is_sha256(source_lidar_support_sha256):
        raise LidarWorldSupportError("source-backed M8 LiDAR support requires its SHA-256")
    alignments = _nurec_alignment_by_frame(nurec_rows or []) if source_mode else {}
    source_frames = _source_support_by_frame(source_lidar_support) if source_mode else {}
    actor_sources, static_object_sources = (
        _registered_object_sources(run_config) if source_mode else ({}, set())
    )
    if source_mode and static_object_sources and static_source_lidar_support is None:
        raise LidarWorldSupportError(
            "source-backed M8 LiDAR expectations require static source LiDAR support"
        )
    if static_source_lidar_support is not None and not _is_sha256(static_source_lidar_support_sha256):
        raise LidarWorldSupportError("static source-backed M8 LiDAR support requires its SHA-256")
    static_source_frames = (
        _source_support_by_frame(static_source_lidar_support, label="static source LiDAR")
        if static_source_lidar_support is not None
        else {}
    )
    manifest_sha256 = _native_scan_manifest_sha256(run_config) if source_mode else None
    rows = []
    seen = set()
    for runtime in runtime_rows:
        frame_id = runtime.get("frame_id")
        if not isinstance(frame_id, int) or isinstance(frame_id, bool) or frame_id in seen:
            raise LidarWorldSupportError("M8 runtime frames require unique integer frame_id")
        seen.add(frame_id)
        ego_state = runtime.get("ego_state")
        states = runtime.get("object_states")
        if not isinstance(ego_state, Mapping) or not isinstance(states, list):
            raise LidarWorldSupportError(f"M8 runtime frame {frame_id} lacks ego_state or object_states")
        expected_objects = expected_lidar_support_from_physical_boxes(
            ego_pose=ego_state.get("pose"),
            sensor_to_ego=sensor_to_ego,
            object_states=states,
            max_range_m=max_range_m,
        )
        row: dict[str, Any] = {
            "schema_version": "m8_expected_lidar_support.v1",
            "frame_id": frame_id,
            "simulation_time_sec": runtime.get("simulation_time_sec"),
            "sensor_to_ego": sensor_to_ego,
            "expected_world_objects": expected_objects,
        }
        if source_mode:
            alignment = alignments.get(frame_id)
            if alignment is None:
                raise LidarWorldSupportError(f"M8 frame {frame_id} has no passed NuRec temporal alignment")
            if alignment["manifest_sha256"] != manifest_sha256:
                raise LidarWorldSupportError(f"M8 frame {frame_id} has a foreign native scan manifest")
            frame_end_us = alignment["wire_end_us"]
            source_support = source_frames.get(frame_end_us)
            if source_support is None:
                raise LidarWorldSupportError(
                    f"M8 frame {frame_id} lacks source LiDAR support at {frame_end_us}"
                )
            static_source_support = static_source_frames.get(frame_end_us)
            if static_object_sources and static_source_support is None:
                raise LidarWorldSupportError(
                    f"M8 frame {frame_id} lacks static source LiDAR support at {frame_end_us}"
                )
            _apply_source_dynamic_support(
                expected_objects,
                source_support,
                actor_sources=actor_sources,
                static_object_sources=static_object_sources,
                static_source_support=static_source_support or {},
                frame_id=frame_id,
            )
            row["schema_version"] = "m8_expected_lidar_support.v2"
            row["source_lidar_alignment"] = {
                **alignment,
                "source_lidar_support_sha256": source_lidar_support_sha256,
                "static_source_lidar_support_sha256": static_source_lidar_support_sha256,
            }
        rows.append(row)
    if not rows:
        raise LidarWorldSupportError("M8 expected LiDAR support requires at least one runtime frame")
    return rows


def _is_sha256(value: str | None) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(char in "0123456789abcdef" for char in value)
    )


def _registered_object_sources(run_config: Mapping[str, Any]) -> tuple[dict[str, str], dict[str, str]]:
    actors = run_config.get("actors")
    if not isinstance(actors, list):
        raise LidarWorldSupportError("source-backed M8 LiDAR expectations require run_config.actors")
    result: dict[str, str] = {}
    for actor in actors:
        if not isinstance(actor, Mapping):
            raise LidarWorldSupportError("M8 actor registration must contain objects")
        actor_id = str(actor.get("actor_id") or "")
        source_track_id = str(actor.get("source_track_id") or "")
        if not actor_id or not source_track_id or actor_id in result:
            raise LidarWorldSupportError("M8 actor registration has invalid or duplicate source track bindings")
        result[actor_id] = source_track_id
    static_obstacles = run_config.get("static_obstacles") or []
    if not isinstance(static_obstacles, list):
        raise LidarWorldSupportError("M8 static_obstacles must be a list")
    static_sources: dict[str, str] = {}
    for item in static_obstacles:
        if not isinstance(item, Mapping):
            raise LidarWorldSupportError("M8 static_obstacles must contain objects")
        object_id = str(item.get("object_id") or "")
        source = item.get("source")
        source_track_id = str(source.get("source_track_id") or "") if isinstance(source, Mapping) else ""
        if not object_id or not source_track_id or object_id in static_sources:
            raise LidarWorldSupportError(
                "M8 static obstacle registration has invalid or duplicate source track bindings"
            )
        if object_id in result:
            raise LidarWorldSupportError(f"M8 object {object_id} is registered as both dynamic and static")
        static_sources[object_id] = source_track_id
    return result, static_sources


def _native_scan_manifest_sha256(run_config: Mapping[str, Any]) -> str:
    runtime = run_config.get("nurec_runtime")
    manifest = runtime.get("native_scan_manifest") if isinstance(runtime, Mapping) else None
    value = manifest.get("sha256") if isinstance(manifest, Mapping) else None
    if not _is_sha256(value):
        raise LidarWorldSupportError("M8 run config has no valid native scan manifest SHA-256")
    return str(value)


def _nurec_alignment_by_frame(rows: list[Mapping[str, Any]]) -> dict[int, dict[str, Any]]:
    result: dict[int, dict[str, Any]] = {}
    for row in rows:
        frame_id = row.get("frame_id")
        if not isinstance(frame_id, int) or isinstance(frame_id, bool):
            raise LidarWorldSupportError("NuRec trace row has no integer frame_id")
        if row.get("status") != "passed":
            raise LidarWorldSupportError(f"M8 frame {frame_id} NuRec trace did not pass")
        dispatch = row.get("dispatch")
        alignment = dispatch.get("temporal_alignment") if isinstance(dispatch, Mapping) else None
        if not isinstance(alignment, Mapping) or alignment.get("status") != "aligned":
            raise LidarWorldSupportError(f"M8 frame {frame_id} has no aligned native LiDAR scan")
        manifest_sha256 = alignment.get("manifest_sha256")
        wire_start_us, wire_end_us = alignment.get("wire_start_us"), alignment.get("wire_end_us")
        if (
            not _is_sha256(manifest_sha256)
            or not isinstance(wire_start_us, int)
            or isinstance(wire_start_us, bool)
            or not isinstance(wire_end_us, int)
            or isinstance(wire_end_us, bool)
            or wire_start_us >= wire_end_us
            or frame_id in result
        ):
            raise LidarWorldSupportError(f"M8 frame {frame_id} has invalid native LiDAR alignment")
        result[frame_id] = {
            "status": "aligned",
            "native_scan_index": alignment.get("native_scan_index"),
            "wire_start_us": wire_start_us,
            "wire_end_us": wire_end_us,
            "midpoint_error_us": alignment.get("midpoint_error_us"),
            "max_midpoint_error_us": alignment.get("max_midpoint_error_us"),
            "manifest_sha256": manifest_sha256,
        }
    return result


def _source_support_by_frame(
    source_lidar_support: Mapping[str, Any],
    *,
    label: str = "source LiDAR",
) -> dict[int, dict[str, Mapping[str, Any]]]:
    if not isinstance(source_lidar_support, Mapping):
        raise LidarWorldSupportError(f"M8 {label} support must be a JSON object")
    schema_version = source_lidar_support.get("schema_version")
    if schema_version not in {
        "ncore_dynamic_lidar_support_audit.v2",
        "ncore_static_lidar_support_audit.v2",
    }:
        raise LidarWorldSupportError(
            f"M8 {label} support must use an ncore *_lidar_support_audit.v2 schema"
        )
    raw_frames = source_lidar_support.get("source_lidar_frames")
    if not isinstance(raw_frames, list) or not raw_frames:
        raise LidarWorldSupportError("M8 source LiDAR support has no same-tick frame records")
    result: dict[int, dict[str, Mapping[str, Any]]] = {}
    for frame in raw_frames:
        if not isinstance(frame, Mapping):
            raise LidarWorldSupportError("M8 source LiDAR frame record must be an object")
        end_us = frame.get("source_lidar_frame_end_us")
        rows = frame.get("track_support")
        if not isinstance(end_us, int) or isinstance(end_us, bool) or not isinstance(rows, list) or end_us in result:
            raise LidarWorldSupportError("M8 source LiDAR frame record is invalid or duplicate")
        track_rows: dict[str, Mapping[str, Any]] = {}
        for item in rows:
            if not isinstance(item, Mapping):
                raise LidarWorldSupportError("M8 source LiDAR track support must be an object")
            track_id = str(item.get("track_id") or "")
            if not track_id or track_id in track_rows:
                raise LidarWorldSupportError("M8 source LiDAR track support has invalid or duplicate track_id")
            track_rows[track_id] = item
        result[end_us] = track_rows
    return result


def _apply_source_dynamic_support(
    expected_objects: list[dict[str, Any]],
    source_support: Mapping[str, Mapping[str, Any]],
    *,
    actor_sources: Mapping[str, str],
    static_object_sources: Mapping[str, str],
    static_source_support: Mapping[str, Mapping[str, Any]],
    frame_id: int,
) -> None:
    for item in expected_objects:
        object_id = str(item.get("object_id") or "")
        source_track_id = actor_sources.get(object_id)
        if source_track_id is None and object_id in static_object_sources:
            _apply_source_static_support(
                item,
                static_source_support.get(static_object_sources[object_id]),
                source_track_id=static_object_sources[object_id],
                frame_id=frame_id,
            )
            continue
        if source_track_id is None:
            raise LidarWorldSupportError(
                f"M8 frame {frame_id} object {object_id} is absent from the full scene registry"
            )
        source = source_support.get(source_track_id)
        if source is None or source.get("source_cuboid_available") is not True:
            raise LidarWorldSupportError(
                f"M8 frame {frame_id} active dynamic object {object_id} lacks a source cuboid"
            )
        exact_hits = source.get("exact_box_hit_points")
        padded_hits = source.get("padded_box_hit_points")
        if (
            not isinstance(exact_hits, int)
            or isinstance(exact_hits, bool)
            or exact_hits < 0
            or not isinstance(padded_hits, int)
            or isinstance(padded_hits, bool)
            or padded_hits < exact_hits
        ):
            raise LidarWorldSupportError(
                f"M8 frame {frame_id} dynamic object {object_id} has invalid source LiDAR counts"
            )
        source_expected = exact_hits > 0
        carla_expected = bool(item["expected_lidar_support"])
        source_carla_conflict = source_expected and not carla_expected
        # Preserve source/CARLA contradictions in the immutable expectation
        # row so the four-stream audit can report the exact failing object and
        # tick. Aborting here would discard collision/lane/visibility evidence
        # and turn a measured geometry failure into an unavailable metric.
        item["expected_lidar_support"] = carla_expected and source_expected
        item["source"] = "carla_physical_box_occlusion_and_ncore_same_tick_dynamic.v1"
        item["source_lidar_observability"] = {
            "kind": "ncore_same_tick_dynamic_cuboid",
            "source_track_id": source_track_id,
            "annotation_status": source.get("annotation_status"),
            "exact_box_hit_points": exact_hits,
            "padded_box_hit_points": padded_hits,
            "status": "source_observed_carla_occluded" if source_carla_conflict else "available",
            "issues": ["source_observed_carla_occluded"] if source_carla_conflict else [],
        }


def _apply_source_static_support(
    item: dict[str, Any],
    source: Mapping[str, Any] | None,
    *,
    source_track_id: str,
    frame_id: int,
) -> None:
    """Gate static-object LiDAR expectations on same-tick source observability.

    Static registry objects remain required for RGB visibility and CARLA collision
    auditing even when the source scan has no cuboid at this tick.  In that case
    only the LiDAR expectation is disabled, avoiding an unsupported assertion.
    """

    item["source"] = "carla_physical_box_occlusion_and_ncore_same_tick_static.v1"
    if source is None or source.get("source_cuboid_available") is not True:
        item["expected_lidar_support"] = False
        item["source_lidar_observability"] = {
            "kind": "ncore_same_tick_static_cuboid",
            "source_track_id": source_track_id,
            "status": "unavailable",
            "annotation_status": (source or {}).get("annotation_status") if isinstance(source, Mapping) else "missing_source_track",
        }
        return
    exact_hits = source.get("exact_box_hit_points")
    padded_hits = source.get("padded_box_hit_points")
    if (
        not isinstance(exact_hits, int)
        or isinstance(exact_hits, bool)
        or exact_hits < 0
        or not isinstance(padded_hits, int)
        or isinstance(padded_hits, bool)
        or padded_hits < exact_hits
    ):
        raise LidarWorldSupportError(
            f"M8 frame {frame_id} static object {item.get('object_id')} has invalid source LiDAR counts"
        )
    source_expected = exact_hits > 0
    carla_expected = bool(item["expected_lidar_support"])
    item["expected_lidar_support"] = carla_expected and source_expected
    conflict = source_expected and not carla_expected
    item["source_lidar_observability"] = {
        "kind": "ncore_same_tick_static_cuboid",
        "source_track_id": source_track_id,
        "status": "source_observed_carla_occluded" if conflict else "available",
        "annotation_status": source.get("annotation_status"),
        "exact_box_hit_points": exact_hits,
        "padded_box_hit_points": padded_hits,
        "issues": ["source_observed_carla_occluded"] if conflict else [],
    }


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a JSON object")
    return value


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _json_object_from_path(path: Path) -> Mapping[str, Any]:
    value = json.loads(path.read_bytes())
    if not isinstance(value, Mapping):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build occlusion-aware CARLA LiDAR support expectations for M8.")
    parser.add_argument("--m8-runtime-trace", required=True, type=Path)
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--max-range-m", type=float, default=DEFAULT_MAX_RANGE_M)
    parser.add_argument("--nurec-multimodal-trace", type=Path)
    parser.add_argument("--source-lidar-support", type=Path)
    parser.add_argument("--static-source-lidar-support", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite M8 expected LiDAR support: {args.output}")
        if bool(args.nurec_multimodal_trace) != bool(args.source_lidar_support):
            raise ValueError(
                "--nurec-multimodal-trace and --source-lidar-support must be supplied together"
            )
        source_body = (
            args.source_lidar_support.read_bytes() if args.source_lidar_support else None
        )
        source_support = json.loads(source_body) if source_body is not None else None
        if source_support is not None and not isinstance(source_support, Mapping):
            raise ValueError("--source-lidar-support must contain a JSON object")
        rows = build_expected_lidar_support(
            _read_jsonl(args.m8_runtime_trace),
            _read_json(args.run_config),
            max_range_m=args.max_range_m,
            nurec_rows=(
                _read_jsonl(args.nurec_multimodal_trace)
                if args.nurec_multimodal_trace
                else None
            ),
            source_lidar_support=source_support,
            source_lidar_support_sha256=(
                hashlib.sha256(source_body).hexdigest()
                if source_body is not None
                else None
            ),
            static_source_lidar_support=(
                _json_object_from_path(args.static_source_lidar_support)
                if args.static_source_lidar_support
                else None
            ),
            static_source_lidar_support_sha256=(
                hashlib.sha256(args.static_source_lidar_support.read_bytes()).hexdigest()
                if args.static_source_lidar_support
                else None
            ),
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text("".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in rows), encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, LidarWorldSupportError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output": str(args.output), "frame_count": len(rows)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
