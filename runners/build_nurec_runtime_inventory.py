from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_inventory import build_nurec_runtime_track_inventory


def build_inventory_from_files(
    mapping_path: str | Path,
    artifact_path: str | Path,
    probe_paths: list[str | Path],
    *,
    renderer_version: str,
) -> dict[str, Any]:
    mapping = _load_object(Path(mapping_path))
    if mapping.get("schema_version") != "nurec_actor_mapping_observation.v1":
        raise ValueError("actor mapping must use nurec_actor_mapping_observation.v1")
    if mapping.get("source") != "loaded_nurec_scenario.actor_mapping":
        raise ValueError("actor mapping source must be loaded_nurec_scenario.actor_mapping")
    tracks = mapping.get("tracks")
    if not isinstance(tracks, list) or not tracks:
        raise ValueError("actor mapping observation must contain tracks")

    artifact = Path(artifact_path).resolve()
    observed_artifact = Path(str(mapping.get("usdz_path") or "")).resolve()
    if observed_artifact != artifact:
        raise ValueError(
            f"actor mapping USDZ {observed_artifact} does not match artifact {artifact}"
        )

    runtime_mapping = {}
    for row in tracks:
        if not isinstance(row, dict):
            raise ValueError("actor mapping tracks must be objects")
        track_id = str(row.get("track_id") or "")
        if not track_id or track_id in runtime_mapping:
            raise ValueError("actor mapping track IDs must be non-empty and unique")
        actor_id = row.get("runtime_actor_id")
        actor_type = str(row.get("runtime_type_id") or "")
        if not isinstance(actor_id, int) or isinstance(actor_id, bool) or not actor_type:
            raise ValueError(f"actor mapping track {track_id} lacks runtime identity")
        runtime_mapping[track_id] = SimpleNamespace(
            actor_inst=SimpleNamespace(id=actor_id, type_id=actor_type)
        )

    probe_results = {}
    for raw_path in probe_paths:
        report = _load_object(Path(raw_path))
        if report.get("schema_version") != "nurec_dynamic_pose_ab_probe.v1":
            raise ValueError(f"probe must use nurec_dynamic_pose_ab_probe.v1: {raw_path}")
        if report.get("status") != "passed":
            raise ValueError(f"pose probe did not pass: {raw_path}")
        track_id = str(report.get("track_id") or "")
        probe = report.get("probe")
        if not track_id or not isinstance(probe, dict):
            raise ValueError(f"pose probe lacks track_id or probe payload: {raw_path}")
        if track_id in probe_results:
            raise ValueError(f"duplicate pose probe for track: {track_id}")
        probe_results[track_id] = probe

    return build_nurec_runtime_track_inventory(
        runtime_mapping,
        artifact_path=artifact,
        renderer_version=renderer_version,
        probe_results=probe_results,
    )


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Combine a loaded CARLA NuRec actor mapping with RGB/LiDAR pose probes."
    )
    parser.add_argument("--actor-mapping", required=True, type=Path)
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--renderer-version", required=True)
    parser.add_argument("--pose-probe", action="append", default=[], type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    try:
        result = build_inventory_from_files(
            args.actor_mapping,
            args.artifact,
            args.pose_probe,
            renderer_version=args.renderer_version,
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(result["summary"], ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
