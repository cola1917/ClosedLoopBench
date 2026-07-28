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


def derive_m7_physical_pose_bindings(
    run_config: Mapping[str, Any], binding_set: Mapping[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Derive an immutable M7 config without promoting replay actors to control.

    Replay remains the CARLA behaviour policy.  Only the NuRec request pose is
    changed from source annotation to measured CARLA runtime geometry, which is
    the minimum contract needed to test physical/render agreement.
    """

    config = deepcopy(dict(run_config))
    bindings = deepcopy(dict(binding_set))
    selected = (config.get("actor_binding") or {}).get("selected_actor_ids")
    if not isinstance(selected, list) or not selected:
        raise ValueError("run config requires actor_binding.selected_actor_ids")
    selected_ids = [str(value) for value in selected]
    if len(selected_ids) != len(set(selected_ids)):
        raise ValueError("selected actor IDs must be unique")
    sidecars = {
        str(item.get("actor_id") or ""): item
        for item in bindings.get("bindings") or []
        if isinstance(item, dict)
    }
    actors = {
        str(item.get("actor_id") or ""): item
        for item in config.get("actors") or []
        if isinstance(item, dict)
    }
    for actor_id in selected_ids:
        sidecar = sidecars.get(actor_id)
        actor = actors.get(actor_id)
        if sidecar is None or actor is None or not isinstance(actor.get("binding"), dict):
            raise ValueError(f"selected actor has no matching binding: {actor_id}")
        # nuScenes track translations are cuboid centres for vehicles and
        # pedestrians. NuRec's DynamicObject API consumes that same reference.
        reference = "carla_bounding_box_center"
        sync = dict(sidecar.get("sensor_sync") or {})
        sync["pose_source"] = "carla_runtime_actor_pose"
        sync["pose_reference"] = reference
        sync["replay_render_pose_mode"] = "carla_runtime_physical"
        sidecar["sensor_sync"] = sync
        embedded = actor["binding"]
        embedded["sensor_pose_source"] = "carla_runtime_actor_pose"
        embedded["sensor_pose_reference"] = reference
        actor_contract = dict(actor.get("control_mode_contract") or {})
        actor_contract["sensor_pose_source"] = "carla_runtime_actor_pose"
        actor_contract["sensor_pose_reference"] = reference
        actor["control_mode_contract"] = actor_contract
        # Keep M6 replay behaviour, capabilities and control contract intact.
        if str(actor.get("closed_loop_level") or "") != "replay":
            raise ValueError(f"M7 baseline derivation requires replay actor: {actor_id}")
    config["runtime"] = dict(config.get("runtime") or {})
    config["runtime"]["m7_actor_pose_audit_required"] = True
    config["runtime"]["m7_pose_contract"] = "carla_runtime_actor_pose.v1"
    return config, bindings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Derive Scene-0061 M7 physical NuRec pose bindings.")
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--output-config", required=True, type=Path)
    parser.add_argument("--output-actor-bindings", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output_config.exists() or args.output_actor_bindings.exists():
            raise ValueError("refusing to overwrite M7 derived output")
        config = json.loads(args.run_config.read_text(encoding="utf-8"))
        nurec = config.get("nurec_runtime") if isinstance(config, dict) else None
        source_path = Path(str((nurec or {}).get("actor_bindings") or ""))
        binding_set = json.loads(source_path.read_text(encoding="utf-8"))
        derived_config, derived_bindings = derive_m7_physical_pose_bindings(config, binding_set)
        args.output_actor_bindings.parent.mkdir(parents=True, exist_ok=True)
        binding_bytes = (json.dumps(derived_bindings, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
        args.output_actor_bindings.write_bytes(binding_bytes)
        derived_config["nurec_runtime"] = dict(derived_config["nurec_runtime"])
        derived_config["nurec_runtime"]["actor_bindings"] = str(args.output_actor_bindings.resolve())
        derived_config["nurec_runtime"]["actor_bindings_sha256"] = hashlib.sha256(binding_bytes).hexdigest()
        args.output_config.parent.mkdir(parents=True, exist_ok=True)
        args.output_config.write_text(json.dumps(derived_config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output_config": str(args.output_config), "output_actor_bindings": str(args.output_actor_bindings)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
