from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_object_visibility import SceneObjectVisibilityError, build_visibility_manifest


def physical_frames_from_m8_runtime(rows: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Adapt raw M8 CARLA truth to the calibrated projection contract."""

    frames = []
    seen = set()
    for row in rows:
        frame_id = row.get("frame_id")
        ego = row.get("ego_state")
        states = row.get("object_states")
        if not isinstance(frame_id, int) or frame_id in seen:
            raise ValueError("M8 runtime frames require unique integer frame_id")
        if not isinstance(ego, Mapping) or not isinstance(ego.get("pose"), Mapping):
            raise ValueError(f"M8 runtime frame {frame_id} has no ego physical pose")
        if not isinstance(states, list):
            raise ValueError(f"M8 runtime frame {frame_id} has no physical object states")
        actor_states = {}
        for state in states:
            if not isinstance(state, Mapping):
                raise ValueError(f"M8 runtime frame {frame_id} has invalid object state")
            object_id = str(state.get("object_id") or "")
            if not object_id or object_id in actor_states:
                raise ValueError(f"M8 runtime frame {frame_id} has duplicate object state")
            actor_states[object_id] = {
                "pose": dict(state.get("pose") or {}),
                "extent_m": dict(state.get("extent_m") or {}),
            }
        seen.add(frame_id)
        frames.append(
            {
                "world_tick_frame": frame_id,
                "simulation_time_sec": row.get("simulation_time_sec"),
                "ego_pose": dict(ego["pose"]),
                "actor_states": actor_states,
            }
        )
    return frames


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a JSON object")
    return value


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Project physical M8 CARLA boxes into exact NuRec RGB payloads.")
    parser.add_argument("--scene-object-registry", type=Path, required=True)
    parser.add_argument("--run-config", type=Path, required=True)
    parser.add_argument("--m8-runtime-trace", type=Path, required=True)
    parser.add_argument("--nurec-multimodal-trace", type=Path, required=True)
    parser.add_argument("--camera-calibration-capture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite M8 expected visibility: {args.output}")
        manifest = build_visibility_manifest(
            _load_json(args.scene_object_registry),
            _load_json(args.run_config),
            physical_frames_from_m8_runtime(_load_jsonl(args.m8_runtime_trace)),
            _load_jsonl(args.nurec_multimodal_trace),
            _load_json(args.camera_calibration_capture),
            source_files={
                "registry": args.scene_object_registry,
                "run_config": args.run_config,
                "m8_runtime_trace": args.m8_runtime_trace,
                "nurec_multimodal_trace": args.nurec_multimodal_trace,
                "camera_calibration_capture": args.camera_calibration_capture,
            },
        )
        manifest["producer"]["scope"] = "m8_physical_box_expected_visibility"
        manifest["producer"]["limitations"].append(
            "Projection is a physical expected-visible candidate, not an occlusion-complete LiDAR expectation."
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, SceneObjectVisibilityError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output": str(args.output), "summary": manifest["summary"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
