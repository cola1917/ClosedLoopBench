from __future__ import annotations

import argparse
import hashlib
import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping


def _m8_actor_type(actor: Mapping[str, Any]) -> str:
    value = str(actor.get("type") or actor.get("actor_type") or "").lower()
    return {"walker": "pedestrian", "person": "pedestrian"}.get(value, value)


def _bind_all_m8_dynamic_replay_actors(config: dict[str, Any]) -> None:
    """Bind each physical replay actor to the identically named NuRec track."""

    for actor in config.get("actors") or []:
        if not isinstance(actor, dict):
            raise ValueError("M8 actor configuration must contain objects")
        actor_id = str(actor.get("actor_id") or "")
        source_track_id = str(actor.get("source_track_id") or actor_id)
        if not actor_id or not source_track_id:
            raise ValueError("M8 dynamic actors require actor_id and source_track_id")
        actor_type = _m8_actor_type(actor)
        if actor_type not in {"vehicle", "pedestrian", "two_wheeler"}:
            raise ValueError(
                f"M8 actor {actor_id} has unsupported dynamic type: {actor_type!r}"
            )
        binding = dict(actor.get("binding") or {})
        prior_track_id = binding.get("nurec_track_id")
        if prior_track_id not in {None, "", source_track_id}:
            raise ValueError(f"M8 actor {actor_id} has mismatched NuRec track binding")
        binding.update(
            {
                "schema_version": "actor_runtime_binding.v1",
                "nurec_track_id": source_track_id,
                "sensor_pose_source": "carla_runtime_actor_pose",
                "sensor_pose_reference": (
                    "carla_bounding_box_bottom"
                    if actor_type == "pedestrian"
                    else "carla_bounding_box_center"
                ),
                "required_modalities": ["rgb", "lidar"],
                "same_dynamic_object_for_all_modalities": True,
                "declared_status": "ready",
                "effective_control_mode": str(
                    actor.get("effective_control_mode")
                    or binding.get("effective_control_mode")
                    or "replay"
                ),
            }
        )
        actor["binding"] = binding


def build_m8_actor_binding_manifest(config: Mapping[str, Any]) -> dict[str, Any]:
    """Return a hashable closure manifest for every M8 dynamic replay binding."""

    bindings = []
    for actor in config.get("actors") or []:
        if not isinstance(actor, Mapping):
            raise ValueError("M8 actor configuration must contain objects")
        binding = actor.get("binding")
        if not isinstance(binding, Mapping):
            raise ValueError(f"M8 actor {actor.get('actor_id')} lacks runtime binding")
        bindings.append(
            {
                "actor_id": str(actor.get("actor_id") or ""),
                "source_track_id": str(actor.get("source_track_id") or ""),
                "actor_type": _m8_actor_type(actor),
                "nurec_track_id": str(binding.get("nurec_track_id") or ""),
                "sensor_pose_source": str(binding.get("sensor_pose_source") or ""),
                "sensor_pose_reference": str(binding.get("sensor_pose_reference") or ""),
                "required_modalities": list(binding.get("required_modalities") or []),
                "same_dynamic_object_for_all_modalities": binding.get(
                    "same_dynamic_object_for_all_modalities"
                ),
                "effective_control_mode": str(binding.get("effective_control_mode") or ""),
                "declared_status": str(binding.get("declared_status") or ""),
            }
        )
    if not bindings or any(not row["actor_id"] or not row["source_track_id"] for row in bindings):
        raise ValueError("M8 binding manifest requires non-empty actor and source-track IDs")
    return {
        "schema_version": "m8_full_dynamic_actor_binding_set.v1",
        "scene_id": str(config.get("scenario_id") or ""),
        "run_id": str(config.get("run_id") or ""),
        "dynamic_actor_count": len(bindings),
        "control_selection_actor_ids": list(
            (config.get("actor_binding") or {}).get("selected_actor_ids") or []
        ),
        "bindings": sorted(bindings, key=lambda row: row["actor_id"]),
    }


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
    _bind_all_m8_dynamic_replay_actors(config)
    runtime["m8_safety_audit_required"] = True
    runtime["m8_safety_contract"] = "carla_nurec_independent_evidence.v1"
    # M8 compares CARLA physical boxes with NuRec observations. A dynamic
    # actor outside its source annotation window must therefore exist in neither.
    runtime["dynamic_actor_lifecycle"] = "source_annotation_window"
    runtime["static_obstacle_lifecycle"] = "source_annotation_window"
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
    parser.add_argument(
        "--actor-bindings-output",
        type=Path,
        help="Write the immutable full M8 dynamic actor-to-NuRec binding manifest.",
    )
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite M8 config: {args.output}")
        if (
            args.actor_bindings_output is not None
            and args.actor_bindings_output.exists()
        ):
            raise ValueError(
                "refusing to overwrite M8 actor bindings: "
                f"{args.actor_bindings_output}"
            )
        source = json.loads(args.run_config.read_text(encoding="utf-8"))
        if not isinstance(source, Mapping):
            raise ValueError("run config must be a JSON object")
        derived = derive_m8_safety_config(
            source, lidar_instant_sampling=args.lidar_instant_sampling
        )
        manifest_bytes = None
        if args.actor_bindings_output is not None:
            manifest = build_m8_actor_binding_manifest(derived)
            manifest_bytes = (
                json.dumps(manifest, ensure_ascii=False, indent=2) + "\n"
            ).encode("utf-8")
            runtime = dict(derived.get("nurec_runtime") or {})
            runtime["actor_bindings"] = str(args.actor_bindings_output.resolve())
            runtime["actor_bindings_sha256"] = hashlib.sha256(manifest_bytes).hexdigest()
            derived["nurec_runtime"] = runtime
        derived["config_identity"] = {
            "schema_version": "m8_safety_config_identity.v1",
            "source_path": str(args.run_config.resolve()),
            "source_sha256": hashlib.sha256(args.run_config.read_bytes()).hexdigest(),
        }
        if args.actor_bindings_output is not None and manifest_bytes is not None:
            args.actor_bindings_output.parent.mkdir(parents=True, exist_ok=True)
            args.actor_bindings_output.write_bytes(manifest_bytes)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(derived, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output": str(args.output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
