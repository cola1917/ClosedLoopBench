from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_multimodal import (
    NuRecMultimodalError,
    validate_nurec_multimodal_evidence,
)


class MultimodalClosedLoopError(RuntimeError):
    """Raised when a run overstates actor or RGB/LiDAR closure evidence."""


def validate_multimodal_closed_loop_result(result: Mapping[str, Any]) -> dict[str, Any]:
    if result.get("status") != "interactive_closed_loop":
        raise MultimodalClosedLoopError("run must complete as interactive_closed_loop")
    report = result.get("report")
    if not isinstance(report, Mapping):
        raise MultimodalClosedLoopError("closed-loop report is missing")
    runtime = report.get("runtime") or {}
    binding = runtime.get("actor_runtime_binding") or {}
    if binding.get("status") != "passed" or not binding.get("records"):
        raise MultimodalClosedLoopError("runtime actor binding evidence did not pass")
    binding_records = binding["records"]
    if any(record.get("status") != "passed" for record in binding_records):
        raise MultimodalClosedLoopError("one or more runtime actor identities failed")
    for record in binding_records:
        if set(record.get("required_modalities") or []) != {"rgb", "lidar"}:
            raise MultimodalClosedLoopError(
                f"actor {record.get('actor_id')} is not bound to RGB and LiDAR"
            )
        runtime_actor_id = (record.get("carla") or {}).get("runtime_actor_id")
        if not isinstance(runtime_actor_id, int) or isinstance(runtime_actor_id, bool):
            raise MultimodalClosedLoopError(
                f"actor {record.get('actor_id')} lacks a CARLA runtime actor id"
            )
        if not record.get("source_track_id") or not record.get("nurec_track_id"):
            raise MultimodalClosedLoopError(
                f"actor {record.get('actor_id')} lacks source/NuRec identity"
            )
        if record["source_track_id"] != record["nurec_track_id"]:
            raise MultimodalClosedLoopError(
                f"actor {record.get('actor_id')} source/NuRec identity changed"
            )
        expected_reference = (
            "carla_actor_origin"
            if record.get("actor_type") == "pedestrian"
            else "carla_bounding_box_center"
        )
        if record.get("sensor_pose_reference") != expected_reference:
            raise MultimodalClosedLoopError(
                f"actor {record.get('actor_id')} has an invalid NuRec pose reference"
            )
    runtime_actor_ids = [record["carla"]["runtime_actor_id"] for record in binding_records]
    if len(runtime_actor_ids) != len(set(runtime_actor_ids)):
        raise MultimodalClosedLoopError("two bindings reference the same CARLA runtime actor")

    sensor = runtime.get("multimodal_sensor") or {}
    if not sensor.get("required"):
        raise MultimodalClosedLoopError("multimodal sensor evidence was not fail-closed")
    if sensor.get("status") != "passed" or not sensor.get("sensor_closed_loop"):
        raise MultimodalClosedLoopError("NuRec multimodal sensor loop did not pass")
    if set(sensor.get("modalities") or []) != {"rgb", "lidar"}:
        raise MultimodalClosedLoopError("sensor loop does not include both RGB and LiDAR")

    frame_trace_count = int(runtime.get("frame_trace_count") or 0)
    evidence_trace = result.get("nurec_multimodal_trace")
    if not isinstance(evidence_trace, list) or not evidence_trace:
        raise MultimodalClosedLoopError("per-frame NuRec evidence trace is missing")
    if len(evidence_trace) != frame_trace_count or sensor.get("frame_count") != frame_trace_count:
        raise MultimodalClosedLoopError("NuRec evidence does not cover every CARLA frame")
    frame_ids = []
    digests = set()
    for evidence in evidence_trace:
        try:
            validate_nurec_multimodal_evidence(evidence)
        except NuRecMultimodalError as exc:
            raise MultimodalClosedLoopError(
                f"invalid NuRec frame evidence: {exc}"
            ) from exc
        if evidence.get("status") != "passed":
            raise MultimodalClosedLoopError(
                f"NuRec frame {evidence.get('frame_id')} did not pass"
            )
        frame_id = evidence.get("frame_id")
        if not isinstance(frame_id, int):
            raise MultimodalClosedLoopError("NuRec evidence has no integer frame id")
        frame_ids.append(frame_id)
        digest = str(evidence.get("dynamic_object_sha256") or "")
        if not _is_sha256(digest):
            raise MultimodalClosedLoopError(f"NuRec frame {frame_id} has invalid actor digest")
        digests.add(digest)
        if int(evidence.get("dynamic_object_count") or 0) < 1:
            raise MultimodalClosedLoopError(f"NuRec frame {frame_id} has no dynamic actor")
        modalities = evidence.get("modalities") or {}
        for modality in ("rgb", "lidar"):
            summary = modalities.get(modality) or {}
            requested = int(summary.get("requested_count") or 0)
            passed = int(summary.get("passed_count") or 0)
            if requested < 1 or passed != requested:
                raise MultimodalClosedLoopError(
                    f"NuRec frame {frame_id} has incomplete {modality} responses"
                )
    if any(current <= previous for previous, current in zip(frame_ids, frame_ids[1:])):
        raise MultimodalClosedLoopError("NuRec evidence frame IDs are not strictly increasing")

    physical = runtime.get("actor_physical_response") or {}
    interactive_ids = {
        str(record["actor_id"])
        for record in binding_records
        if record.get("sensor_pose_source") == "carla_runtime_actor_pose"
    }
    missing_physical = sorted(interactive_ids - set(physical))
    if missing_physical:
        raise MultimodalClosedLoopError(
            "bound interactive actors lack physical response: " + ", ".join(missing_physical)
        )

    decisions_by_actor: dict[str, list[Mapping[str, Any]]] = {
        actor_id: [] for actor_id in interactive_ids
    }
    for row in report.get("metrics") or []:
        for actor_id, decision in (row.get("actor_decisions") or {}).items():
            if actor_id in decisions_by_actor and isinstance(decision, Mapping):
                decisions_by_actor[actor_id].append(decision)
    actor_types = {str(record["actor_id"]): record.get("actor_type") for record in binding_records}
    for actor_id, decisions in decisions_by_actor.items():
        if not decisions:
            raise MultimodalClosedLoopError(
                f"interactive actor {actor_id} has no explainable decisions"
            )
        for decision in decisions:
            if not decision.get("reason") or not decision.get("motion_constraint"):
                raise MultimodalClosedLoopError(
                    f"interactive actor {actor_id} decision lacks reason/constraint"
                )
            if actor_types.get(actor_id) == "pedestrian":
                if decision.get("motion_constraint") != "source_reference_corridor":
                    raise MultimodalClosedLoopError(
                        f"pedestrian actor {actor_id} left the source corridor contract"
                    )
                if set(decision.get("allowed_actions") or []) != {
                    "speed",
                    "pause",
                    "yield",
                    "abort",
                }:
                    raise MultimodalClosedLoopError(
                        f"pedestrian actor {actor_id} exposes unsupported edit actions"
                    )

    return {
        "schema_version": "multimodal_closed_loop_acceptance.v1",
        "scene_id": result.get("scenario_id"),
        "status": "passed",
        "frame_count": frame_trace_count,
        "actor_ids": sorted(str(record["actor_id"]) for record in binding_records),
        "interactive_actor_ids": sorted(interactive_ids),
        "dynamic_object_digest_count": len(digests),
        "modalities": ["rgb", "lidar"],
        "pose_references": {
            str(record["actor_id"]): record["sensor_pose_reference"]
            for record in binding_records
        },
        "explainable_decision_actor_count": sum(bool(value) for value in decisions_by_actor.values()),
    }


def _is_sha256(value: str) -> bool:
    return len(value) == 64 and all(character in "0123456789abcdef" for character in value)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Fail closed unless one run proves actor identity, physics, decisions, RGB and LiDAR."
    )
    parser.add_argument("--runtime-result", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        result = json.loads(args.runtime_result.read_text(encoding="utf-8"))
        evidence = validate_multimodal_closed_loop_result(result)
    except (OSError, ValueError, MultimodalClosedLoopError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(evidence, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
