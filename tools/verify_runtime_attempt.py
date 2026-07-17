#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def _json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _jsonl(path: Path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("attempt_dir", type=Path)
    parser.add_argument("bundle_dir", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    attempt = args.attempt_dir.resolve()
    bundle = args.bundle_dir.resolve()

    report = _json(attempt / "closed_loop_report.json")
    cleanup = _json(attempt / "cleanup_audit.json")
    frames = _jsonl(attempt / "frame_trace.jsonl")
    metrics = _jsonl(attempt / "metrics_trace.jsonl")
    plan_path = next(attempt.glob("*_plan.json"))
    plan = _json(plan_path)

    assert report["status"] == "ego_closed_loop"
    assert report["evaluation"]["overall_result"] == "pass"
    assert report["summary"]["collision_count"] == 0
    assert report["summary"]["route_progress"] >= 0.95
    runtime = report["runtime"]
    assert runtime["termination_reason"] == "route_complete"
    assert runtime["collision_sensor_available"] is True
    assert runtime["route_binding"]["source"] == "carla_runtime_topology"
    assert runtime["route_binding"]["source_trajectory_alignment"] == "not_claimed"
    controls = runtime["ego_driver_diagnostics"]["control_count"]
    assert controls == len(frames) == len(metrics) == runtime["frame_trace_count"]
    assert controls > 0
    assert any(
        abs(float(row["ego_control"][name])) > 1e-6
        for row in frames
        for name in ("throttle", "brake", "steer")
    )
    frame_ids = [int(row["world_tick_frame"]) for row in frames]
    assert all(right == left + 1 for left, right in zip(frame_ids, frame_ids[1:]))
    assert all(row["world_tick_frame"] == row["snapshot_frame"] for row in frames)
    assert cleanup["succeeded"] is True
    assert cleanup["actions"] and all(row["status"] == "succeeded" for row in cleanup["actions"])

    reconstruction_ref = plan["reconstruction_package"]
    assert reconstruction_ref["enabled"] is True
    scene_package = _json(bundle / "scene_package.json")
    assert scene_package["visual"]["nurec_usdz"] == "reconstruction/last.usdz"
    reconstruction_package = _json(bundle / "reconstruction_package.json")
    artifact = next(item for item in reconstruction_package["artifacts"] if item["role"] == "nurec_usdz")
    usdz = bundle / artifact["path"]
    assert usdz.stat().st_size == artifact["size_bytes"]
    digest = _sha256(usdz)
    assert digest == artifact["sha256"]

    result = {
        "schema_version": "closed_loop_runtime_verification.v1",
        "status": "passed",
        "attempt_dir": str(attempt),
        "ticks": controls,
        "first_frame": frame_ids[0],
        "last_frame": frame_ids[-1],
        "route_progress": report["summary"]["route_progress"],
        "collision_count": report["summary"]["collision_count"],
        "termination_reason": runtime["termination_reason"],
        "cleanup_actions": len(cleanup["actions"]),
        "nurec_usdz": str(usdz),
        "nurec_usdz_sha256": digest,
        "visual_binding": "not_claimed_by_native_carla_runtime",
    }
    rendered = json.dumps(result, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
