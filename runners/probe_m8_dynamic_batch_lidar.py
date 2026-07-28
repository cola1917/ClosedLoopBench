#!/usr/bin/env python3
"""Measure NuRec LiDAR dynamic-object support as request batch size increases.

This is a diagnostic-only tool.  Every case is derived from one persisted
pre-dispatch NuRec frame, so CARLA poses, sensor poses, source scan alignment,
and the renderer artifact remain fixed while only the requested dynamic-object
set changes.  A passing subset never promotes M8; the full-object M8 audit
remains the acceptance gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not all(isinstance(row, dict) for row in rows):
        raise ValueError(f"{path} must contain JSON objects")
    return rows


def _unique_frame(rows: list[Mapping[str, Any]], frame_id: int, label: str) -> Mapping[str, Any]:
    matches = [row for row in rows if row.get("frame_id") == frame_id]
    if len(matches) != 1:
        raise ValueError(f"{label} requires exactly one frame_id={frame_id} row")
    return matches[0]


def _source_expected_dynamic_ids(expected_row: Mapping[str, Any]) -> set[str]:
    objects = expected_row.get("expected_world_objects")
    if not isinstance(objects, list):
        raise ValueError("source-backed M8 expectation has no expected_world_objects")
    result = set()
    for item in objects:
        if not isinstance(item, Mapping):
            raise ValueError("source-backed M8 expectation has invalid object row")
        source = item.get("source_lidar_observability")
        if (
            bool(item.get("expected_lidar_support"))
            and isinstance(source, Mapping)
            and isinstance(source.get("source_track_id"), str)
        ):
            object_id = str(item.get("object_id") or "")
            if not object_id:
                raise ValueError("source-backed M8 expectation has no object_id")
            result.add(object_id)
    if not result:
        raise ValueError("source-backed M8 expectation has no source-observed dynamic objects")
    return result


def _case_dynamic_actor_ids(
    frame: Mapping[str, Any],
    source_expected_ids: set[str],
    requested_sizes: list[int | str],
) -> list[tuple[str, list[str]]]:
    dynamic = frame.get("shared_dynamic_objects")
    if not isinstance(dynamic, list):
        raise ValueError("NuRec frame has no shared_dynamic_objects")
    full_ids = [str(item.get("actor_id") or "") for item in dynamic if isinstance(item, Mapping)]
    if not all(full_ids) or len(full_ids) != len(dynamic) or len(full_ids) != len(set(full_ids)):
        raise ValueError("NuRec frame dynamic actor IDs are invalid")
    priority_ids = [actor_id for actor_id in full_ids if actor_id in source_expected_ids]
    if not set(priority_ids) == source_expected_ids:
        raise ValueError("source-observed dynamic objects are absent from the NuRec frame")
    result = []
    seen = set()
    for requested in requested_sizes:
        if requested == "full":
            case_name, ids = f"full_{len(full_ids):03d}", list(full_ids)
        else:
            size = int(requested)
            if size < 1 or size > len(priority_ids):
                raise ValueError(
                    f"batch size {size} must be between 1 and source-observed dynamic count {len(priority_ids)}"
                )
            case_name, ids = f"source_prefix_{size:03d}", priority_ids[:size]
        if case_name not in seen:
            seen.add(case_name)
            result.append((case_name, ids))
    return result


def _reduced_frame(
    full_frame: Mapping[str, Any],
    actor_ids: list[str],
) -> dict[str, Any]:
    from adapters.nurec_multimodal import _digest, validate_nurec_multimodal_frame

    frame = deepcopy(dict(full_frame))
    selected = set(actor_ids)
    dynamic = [
        item
        for item in frame["shared_dynamic_objects"]
        if str(item.get("actor_id") or "") in selected
    ]
    if len(dynamic) != len(actor_ids):
        raise ValueError("requested dynamic actors were lost while building batch case")
    frame["shared_dynamic_objects"] = dynamic
    digest = _digest(dynamic)
    frame["shared_dynamic_object_sha256"] = digest
    for modality in ("rgb", "lidar"):
        requests = frame["modalities"][modality]["requests"]
        if not isinstance(requests, list) or not requests:
            raise ValueError(f"NuRec frame has no {modality} requests")
        # This is an RPC-cost reduction for the diagnostic only. The remaining
        # RGB request and LiDAR request keep their original calibrated poses.
        frame["modalities"][modality]["requests"] = [deepcopy(requests[0])]
        frame["modalities"][modality]["requests"][0]["dynamic_object_sha256"] = digest
    validate_nurec_multimodal_frame(frame)
    return frame


def _lidar_payload(evidence: Mapping[str, Any]) -> dict[str, Any]:
    records = evidence.get("records")
    if not isinstance(records, list):
        raise ValueError("NuRec batch evidence has no response records")
    matches = [
        item
        for item in records
        if isinstance(item, Mapping)
        and item.get("modality") == "lidar"
        and item.get("status") == "passed"
    ]
    if len(matches) != 1:
        raise ValueError("NuRec batch evidence has no passed lidar response")
    metadata = matches[0].get("response_metadata")
    materialized = metadata.get("materialized_payload") if isinstance(metadata, Mapping) else None
    if not isinstance(materialized, Mapping):
        raise ValueError("NuRec batch LiDAR response was not materialized")
    path, digest = materialized.get("path"), materialized.get("sha256")
    if not isinstance(path, str) or not isinstance(digest, str) or len(digest) != 64:
        raise ValueError("NuRec batch LiDAR materialization has no identity")
    return dict(materialized)


def run_probe(
    *,
    run_config: Mapping[str, Any],
    full_frame: Mapping[str, Any],
    runtime_row: Mapping[str, Any],
    source_expected_row: Mapping[str, Any],
    output_dir: Path,
    requested_sizes: list[int | str],
) -> dict[str, Any]:
    from adapters.lidar_world_support import (
        lidar_occupancy_from_xyzi,
        summarize_xyzi_payload,
    )
    from adapters.nurec_260_client import build_nurec_260_client
    from adapters.nurec_multimodal import validate_nurec_multimodal_frame
    from runners.build_m8_lidar_occupancy import _lidar_sensor_to_ego

    validate_nurec_multimodal_frame(full_frame)
    frame_id = full_frame.get("frame_id")
    if not isinstance(frame_id, int) or runtime_row.get("frame_id") != frame_id or source_expected_row.get("frame_id") != frame_id:
        raise ValueError("batch probe inputs must share one integer frame_id")
    object_states = runtime_row.get("object_states")
    ego_state = runtime_row.get("ego_state")
    if not isinstance(object_states, list) or not isinstance(ego_state, Mapping):
        raise ValueError("M8 runtime row lacks physical object states or ego state")
    source_expected_ids = _source_expected_dynamic_ids(source_expected_row)
    cases = _case_dynamic_actor_ids(full_frame, source_expected_ids, requested_sizes)
    state_by_id = {
        str(item.get("object_id") or ""): item
        for item in object_states
        if isinstance(item, Mapping)
    }
    sensor_to_ego = _lidar_sensor_to_ego(run_config)
    rows = []
    for case_name, actor_ids in cases:
        case_dir = output_dir / "cases" / case_name
        case_dir.mkdir(parents=True, exist_ok=False)
        frame = _reduced_frame(full_frame, actor_ids)
        client = build_nurec_260_client(run_config, payload_output_dir=case_dir)
        try:
            evidence = client.dispatch_frame(frame)
        finally:
            client.close()
        payload = _lidar_payload(evidence)
        body = Path(str(payload["path"])).read_bytes()
        if hashlib.sha256(body).hexdigest() != payload["sha256"]:
            raise ValueError(f"NuRec batch LiDAR SHA-256 mismatch: {payload['path']}")
        expected_ids = [actor_id for actor_id in actor_ids if actor_id in source_expected_ids]
        states = [state_by_id[actor_id] for actor_id in expected_ids if actor_id in state_by_id]
        if len(states) != len(expected_ids):
            raise ValueError(f"M8 runtime frame {frame_id} lacks requested dynamic physical boxes")
        occupancy = lidar_occupancy_from_xyzi(
            body,
            ego_pose=ego_state.get("pose"),
            sensor_to_ego=sensor_to_ego,
            object_states=states,
        )
        observed_ids = sorted(
            str(item["object_id"])
            for item in occupancy
            if isinstance(item.get("point_count"), int) and item["point_count"] > 0
        )
        expected_set = set(expected_ids)
        rows.append(
            {
                "case": case_name,
                "status": "passed" if expected_set.issubset(observed_ids) else "failed",
                "requested_dynamic_actor_ids": actor_ids,
                "requested_dynamic_actor_count": len(actor_ids),
                "source_expected_dynamic_ids": sorted(expected_set),
                "source_expected_dynamic_count": len(expected_set),
                "observed_source_expected_dynamic_ids": observed_ids,
                "observed_source_expected_dynamic_count": len(observed_ids),
                "missing_source_expected_dynamic_ids": sorted(expected_set - set(observed_ids)),
                "lidar_payload": payload,
                "lidar_payload_summary": summarize_xyzi_payload(body),
                "nurec_evidence": evidence,
            }
        )
    return {
        "schema_version": "m8_dynamic_batch_lidar_probe.v1",
        "status": "diagnostic_only",
        "frame_id": frame_id,
        "full_dynamic_actor_count": len(full_frame["shared_dynamic_objects"]),
        "source_expected_dynamic_count": len(source_expected_ids),
        "cases": rows,
        "summary": {
            "case_count": len(rows),
            "passing_case_count": sum(item["status"] == "passed" for item in rows),
            "failing_case_count": sum(item["status"] == "failed" for item in rows),
        },
    }


def _parse_sizes(value: str) -> list[int | str]:
    result: list[int | str] = []
    for item in value.split(","):
        normalized = item.strip().lower()
        if not normalized:
            continue
        result.append("full" if normalized == "full" else int(normalized))
    if not result:
        raise ValueError("--batch-sizes must contain at least one size")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--nurec-frame-trace", required=True, type=Path)
    parser.add_argument("--m8-runtime-trace", required=True, type=Path)
    parser.add_argument("--source-expected-lidar", required=True, type=Path)
    parser.add_argument("--frame-id", required=True, type=int)
    parser.add_argument("--batch-sizes", default="1,4,8,16,full")
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output_dir.exists():
            raise ValueError(f"refusing to overwrite batch probe directory: {args.output_dir}")
        sizes = _parse_sizes(args.batch_sizes)
        output_dir = args.output_dir
        output_dir.mkdir(parents=True)
        result = run_probe(
            run_config=_load_object(args.run_config),
            full_frame=_unique_frame(_load_jsonl(args.nurec_frame_trace), args.frame_id, "NuRec frame trace"),
            runtime_row=_unique_frame(_load_jsonl(args.m8_runtime_trace), args.frame_id, "M8 runtime trace"),
            source_expected_row=_unique_frame(_load_jsonl(args.source_expected_lidar), args.frame_id, "source expected LiDAR"),
            output_dir=output_dir,
            requested_sizes=sizes,
        )
        (output_dir / "batch_summary.v1.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps(result["summary"], ensure_ascii=False))


if __name__ == "__main__":
    main()
