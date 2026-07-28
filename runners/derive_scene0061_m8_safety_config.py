from __future__ import annotations

import argparse
import hashlib
import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping


def derive_m8_safety_config(
    source: Mapping[str, Any], *, lidar_instant_sampling: bool = False
) -> dict[str, Any]:
    """Freeze M8 gates without changing replay/control semantics."""

    config = deepcopy(dict(source))
    if config.get("schema_version") != "carla_run_config.mvp.v0":
        raise ValueError("M8 derivation requires carla_run_config.mvp.v0")
    runtime = dict(config.get("runtime") or {})
    if runtime.get("m7_actor_pose_audit_required") is not True:
        raise ValueError("M8 derivation requires an M7 runtime-pose config")
    actors = config.get("actors")
    if not isinstance(actors, list) or not actors:
        raise ValueError("M8 derivation requires full replay actor configuration")
    for actor in actors:
        if not isinstance(actor, Mapping):
            raise ValueError("M8 actor configuration must contain objects")
        if str(actor.get("closed_loop_level") or "") != "replay":
            raise ValueError("M8 baseline must retain replay actor behaviour")
    runtime["m8_safety_audit_required"] = True
    runtime["m8_safety_contract"] = "carla_nurec_independent_evidence.v1"
    # M8 compares CARLA physical boxes with NuRec observations. A dynamic
    # actor outside its source annotation window must therefore exist in neither.
    runtime["dynamic_actor_lifecycle"] = "source_annotation_window"
    config["nurec_runtime"] = dict(config.get("nurec_runtime") or {})
    config["nurec_runtime"]["lidar_instant_sampling"] = bool(lidar_instant_sampling)
    config["runtime"] = runtime
    config["run_id"] = str(config.get("run_id") or "scene0061") + "-m8-safety-probe"
    return config


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Derive immutable M8 Scene-0061 safety-audit config.")
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--lidar-instant-sampling", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite M8 config: {args.output}")
        source = json.loads(args.run_config.read_text(encoding="utf-8"))
        if not isinstance(source, Mapping):
            raise ValueError("run config must be a JSON object")
        derived = derive_m8_safety_config(
            source, lidar_instant_sampling=args.lidar_instant_sampling
        )
        derived["config_identity"] = {
            "schema_version": "m8_safety_config_identity.v1",
            "source_path": str(args.run_config.resolve()),
            "source_sha256": hashlib.sha256(args.run_config.read_bytes()).hexdigest(),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(derived, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output": str(args.output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
