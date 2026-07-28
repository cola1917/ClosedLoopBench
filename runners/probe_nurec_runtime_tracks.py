from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_260_client import build_nurec_260_client
from adapters.nurec_multimodal import NuRecMultimodalError
from adapters.nurec_pose_probe import run_nurec_dynamic_pose_ab_probe
from runners.prepare_nurec_pose_probe_frames import prepare_probe_frames


def run_runtime_track_probes(
    *,
    mapping_path: Path,
    dataroot: Path,
    config_path: Path,
    output_dir: Path,
    scene_name: str,
    version: str,
    runtime_scene_start_us: int,
    track_ids: list[str] | None = None,
    delta_m: float = 1.0,
    moved_target_distance_m: float | None = None,
    resume: bool = False,
) -> dict[str, Any]:
    """Probe every selected loaded runtime track without promoting failed tracks.

    A track receives a report even when its source annotation is unavailable or
    the render RPC fails.  This makes the batch artifact an auditable inventory
    input rather than treating an interrupted probe as an implicit success.
    """

    mapping = _load_object(mapping_path)
    tracks = mapping.get("tracks")
    if mapping.get("schema_version") != "nurec_actor_mapping_observation.v1" or not isinstance(tracks, list):
        raise ValueError("mapping must be a nurec_actor_mapping_observation.v1 tracks array")
    selected = {str(track_id) for track_id in (track_ids or [])}
    if selected and not selected.issubset({str(row.get("track_id") or "") for row in tracks}):
        unknown = sorted(selected - {str(row.get("track_id") or "") for row in tracks})
        raise ValueError("selected tracks absent from mapping: " + ", ".join(unknown))
    runtime_dynamic_rows = [
        row
        for row in tracks
        if re.fullmatch(r"[0-9a-f]{32}", str(row.get("track_id") or ""))
    ]
    if selected and not selected.issubset(
        {str(row.get("track_id") or "") for row in runtime_dynamic_rows}
    ):
        unknown = sorted(
            selected - {str(row.get("track_id") or "") for row in runtime_dynamic_rows}
        )
        raise ValueError("selected tracks are not runtime dynamic tracks: " + ", ".join(unknown))
    rows = [
        row
        for row in runtime_dynamic_rows
        if not selected or str(row.get("track_id")) in selected
    ]
    if not rows:
        raise ValueError("no runtime tracks selected")
    if output_dir.exists() and not resume:
        raise FileExistsError(f"refusing to overwrite probe directory: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=resume)
    probe_dir = output_dir / "probes"
    frame_dir = output_dir / "frames"
    probe_dir.mkdir(exist_ok=resume)
    frame_dir.mkdir(exist_ok=resume)

    config = _load_object(config_path)
    client = build_nurec_260_client(config)
    result_rows: list[dict[str, Any]] = []
    try:
        for row in sorted(rows, key=lambda item: str(item.get("track_id") or "")):
            track_id = str(row.get("track_id") or "")
            report_path = probe_dir / f"{track_id}.json"
            if report_path.exists():
                if not resume:
                    raise FileExistsError(f"refusing to overwrite probe report: {report_path}")
                report = _load_object(report_path)
                result_rows.append(_summary_row(track_id, report, skipped=True))
                continue
            actor_type = _actor_type(str(row.get("runtime_type_id") or ""))
            try:
                baseline, moved, context = prepare_probe_frames(
                    dataroot,
                    version=version,
                    scene_name=scene_name,
                    track_id=track_id,
                    actor_type=actor_type,
                    runtime_scene_start_us=runtime_scene_start_us,
                    delta_m=delta_m,
                    moved_target_distance_m=moved_target_distance_m,
                )
                track_frames = frame_dir / track_id
                track_frames.mkdir()
                _write_json(track_frames / "baseline.json", baseline)
                _write_json(track_frames / "moved.json", moved)
                _write_json(track_frames / "context.json", context)
                report = run_nurec_dynamic_pose_ab_probe(
                    track_id, baseline, moved, dispatch_frame=client.dispatch_frame
                )
            except (OSError, ValueError, NuRecMultimodalError, RuntimeError) as exc:
                report = {
                    "schema_version": "nurec_dynamic_pose_ab_probe.v1",
                    "track_id": track_id,
                    "status": "failed",
                    "issues": ["preparation_or_dispatch_error"],
                    "detail": str(exc),
                    "probe": None,
                }
            _write_json(report_path, report)
            result_rows.append(_summary_row(track_id, report, skipped=False))
    finally:
        client.close()

    summary = {
        "schema_version": "nurec_runtime_track_probe_batch.v1",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "mapping": _file_identity(mapping_path),
        "config": _file_identity(config_path),
        "scene_name": scene_name,
        "version": version,
        "runtime_scene_start_us": runtime_scene_start_us,
        "pose_delta_m": delta_m,
        "moved_target_distance_m": moved_target_distance_m,
        "tracks": result_rows,
        "summary": {
            "selected_track_count": len(result_rows),
            "passed_track_count": sum(item["status"] == "passed" for item in result_rows),
            "failed_track_count": sum(item["status"] == "failed" for item in result_rows),
            "skipped_existing_count": sum(item["skipped_existing"] for item in result_rows),
        },
    }
    _write_json(output_dir / "batch_summary.v1.json", summary)
    return summary


def _actor_type(runtime_type_id: str) -> str:
    if runtime_type_id.startswith("vehicle."):
        return "vehicle"
    if runtime_type_id.startswith("walker."):
        return "pedestrian"
    raise ValueError(f"unsupported runtime actor type: {runtime_type_id}")


def _summary_row(track_id: str, report: dict[str, Any], *, skipped: bool) -> dict[str, Any]:
    return {
        "track_id": track_id,
        "status": str(report.get("status") or "failed"),
        "issues": list(report.get("issues") or []),
        "report": f"probes/{track_id}.json",
        "skipped_existing": skipped,
    }


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _file_identity(path: Path) -> dict[str, Any]:
    return {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Batch A/A/B probe NuRec loaded runtime tracks.")
    parser.add_argument("--actor-mapping", required=True, type=Path)
    parser.add_argument("--dataroot", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument("--runtime-scene-start-us", required=True, type=int)
    parser.add_argument("--track-id", action="append", default=[])
    parser.add_argument("--delta-m", type=float, default=1.0)
    parser.add_argument("--moved-target-distance-m", type=float)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args(argv)
    try:
        summary = run_runtime_track_probes(
            mapping_path=args.actor_mapping,
            dataroot=args.dataroot,
            config_path=args.config,
            output_dir=args.output_dir,
            scene_name=args.scene,
            version=args.version,
            runtime_scene_start_us=args.runtime_scene_start_us,
            track_ids=args.track_id or None,
            delta_m=args.delta_m,
            moved_target_distance_m=args.moved_target_distance_m,
            resume=args.resume,
        )
    except (OSError, ValueError, NuRecMultimodalError, RuntimeError) as exc:
        parser.error(str(exc))
    print(json.dumps(summary["summary"], ensure_ascii=False))
    return 0 if summary["summary"]["failed_track_count"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
