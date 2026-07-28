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


def freeze_replay_actor_bindings(
    run_config: Mapping[str, Any], binding_set: Mapping[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Freeze selected NuRec bindings to the M6 replay pose contract."""

    selected = (run_config.get("actor_binding") or {}).get("selected_actor_ids")
    if not isinstance(selected, list) or not selected:
        raise ValueError("run config requires actor_binding.selected_actor_ids")
    selected_ids = [str(item) for item in selected]
    if len(set(selected_ids)) != len(selected_ids):
        raise ValueError("selected actor IDs must be unique")
    frozen_config = deepcopy(dict(run_config))
    frozen_bindings = deepcopy(dict(binding_set))
    sidecar_by_id = {
        str(item.get("actor_id") or ""): item
        for item in frozen_bindings.get("bindings") or []
        if isinstance(item, dict) and str(item.get("actor_id") or "")
    }
    actors_by_id = {
        str(item.get("actor_id") or ""): item
        for item in frozen_config.get("actors") or []
        if isinstance(item, dict) and str(item.get("actor_id") or "")
    }
    for actor_id in selected_ids:
        sidecar = sidecar_by_id.get(actor_id)
        actor = actors_by_id.get(actor_id)
        if sidecar is None or actor is None or not isinstance(actor.get("binding"), dict):
            raise ValueError(f"selected actor has no matching embedded/sidecar binding: {actor_id}")
        if str(actor.get("closed_loop_level") or "") != "replay":
            raise ValueError(f"M6 binding freeze requires replay actor: {actor_id}")
        control = dict(sidecar.get("control") or {})
        control["mode"] = "replay"
        control["ego_responsive"] = False
        sidecar["control"] = control
        sync = dict(sidecar.get("sensor_sync") or {})
        sync["pose_source"] = "scenario_ir_reference_trajectory"
        sync["pose_reference"] = "source_track_frame"
        sidecar["sensor_sync"] = sync
        embedded = actor["binding"]
        embedded["sensor_pose_source"] = sync["pose_source"]
        embedded["sensor_pose_reference"] = sync["pose_reference"]
        embedded["effective_control_mode"] = "replay"
    summary = dict(frozen_bindings.get("summary") or {})
    summary["interactive_count"] = 0
    frozen_bindings["summary"] = summary
    return frozen_config, frozen_bindings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Freeze selected Scene-0061 NuRec bindings to M6 replay semantics.")
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--output-config", required=True, type=Path)
    parser.add_argument("--output-actor-bindings", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output_config.exists() or args.output_actor_bindings.exists():
            raise ValueError("refusing to overwrite M6 binding-derived output")
        config = json.loads(args.run_config.read_text(encoding="utf-8"))
        nurec = config.get("nurec_runtime") if isinstance(config, dict) else None
        source_path = Path(str((nurec or {}).get("actor_bindings") or ""))
        binding_set = json.loads(source_path.read_text(encoding="utf-8"))
        frozen_config, frozen_bindings = freeze_replay_actor_bindings(config, binding_set)
        args.output_actor_bindings.parent.mkdir(parents=True, exist_ok=True)
        binding_bytes = (json.dumps(frozen_bindings, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
        args.output_actor_bindings.write_bytes(binding_bytes)
        frozen_config["nurec_runtime"] = dict(frozen_config["nurec_runtime"])
        frozen_config["nurec_runtime"]["actor_bindings"] = str(args.output_actor_bindings.resolve())
        frozen_config["nurec_runtime"]["actor_bindings_sha256"] = hashlib.sha256(binding_bytes).hexdigest()
        args.output_config.parent.mkdir(parents=True, exist_ok=True)
        args.output_config.write_text(json.dumps(frozen_config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output_config": str(args.output_config), "output_actor_bindings": str(args.output_actor_bindings)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
