"""Derive a provenance-closed formal40k Scene-0061 base configuration.

A measured smoke live tick is a physical source, never a formal result.  This
command creates a *new* formal execution config only after every immutable
input matches the frozen matrix.  It never inherits earlier LiDAR evidence; a
fresh formal-identity live tick must produce that evidence before a separate
write-once binding is permitted.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agents.plugin_contract import strict_json_loads
from runtime.scene0061_counterfactual import PEDESTRIAN_TRACK, VEHICLE_TRACK, validate_scene0061_counterfactual_matrix
from runtime.scene0061_lidar_axis_normalization import LiDARAxisNormalizationError, validate_lidar_axis_normalization


DERIVATION_SCHEMA = "scene0061_formal_base_derivation.v1"
EVIDENCE_BINDING_SCHEMA = "scene0061_formal_lidar_evidence_binding.v1"
FORMAL_VERSION = "formal40k-v1"
RUNTIME_SCENE = "scene-0061"
CAMERAS = {
    "camera_front", "camera_front_left", "camera_front_right",
    "camera_back", "camera_back_left", "camera_back_right",
}
ROLES = {
    "nurec_usdz", "runtime_validated_scene_package", "actor_ready_scenario_ir",
    "actor_selection", "opendrive",
}
EXECUTION_ROLES = {
    "runtime_scene_package", "runtime_actor_bindings", "runtime_native_scan_manifest",
    "runtime_opendrive",
}
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class Scene0061FormalBaseError(ValueError):
    """The source or formal provenance chain cannot be trusted."""


def _sha256(body: bytes) -> str:
    return hashlib.sha256(body).hexdigest()


def _identity(path: Path, label: str) -> dict[str, Any]:
    target = path.expanduser().resolve()
    if not target.is_file():
        raise Scene0061FormalBaseError(f"{label} does not exist: {target}")
    body = target.read_bytes()
    return {"path": str(target), "sha256": _sha256(body), "byte_count": len(body)}


def _read_object(path: Path, label: str) -> tuple[dict[str, Any], dict[str, Any]]:
    identity = _identity(path, label)
    try:
        value = strict_json_loads(Path(identity["path"]).read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, ValueError, json.JSONDecodeError) as exc:
        raise Scene0061FormalBaseError(f"cannot read strict {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise Scene0061FormalBaseError(f"{label} must be an object")
    return value, identity


def _require_hash(value: Any, label: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise Scene0061FormalBaseError(f"{label} must be a lowercase SHA-256")
    return value


def _matrix_contract(matrix: Mapping[str, Any]) -> tuple[Mapping[str, Any], dict[str, str]]:
    try:
        validate_scene0061_counterfactual_matrix(dict(matrix))
    except ValueError as exc:
        raise Scene0061FormalBaseError(f"invalid formal matrix: {exc}") from exc
    scene = matrix.get("scene_identity")
    if not isinstance(scene, Mapping) or scene.get("scene_version") != FORMAL_VERSION:
        raise Scene0061FormalBaseError("matrix is not formal40k Scene-0061")
    rows = matrix.get("immutable_inputs")
    if not isinstance(rows, list):
        raise Scene0061FormalBaseError("matrix requires immutable inputs")
    expected: dict[str, str] = {}
    for row in rows:
        if not isinstance(row, Mapping):
            raise Scene0061FormalBaseError("matrix immutable input is not an object")
        role = str(row.get("role") or "")
        if not role or role in expected:
            raise Scene0061FormalBaseError("matrix immutable input roles must be unique")
        expected[role] = _require_hash(row.get("sha256"), f"matrix input {role}")
    if set(expected) != ROLES:
        raise Scene0061FormalBaseError("matrix immutable inputs are incomplete")
    return scene, expected


def _validate_physical_source(source: Mapping[str, Any], scene: Mapping[str, Any]) -> None:
    if source.get("scenario_id") != scene.get("scene_id"):
        raise Scene0061FormalBaseError("source scenario_id does not match formal scene")
    experiment = source.get("experiment")
    if not isinstance(experiment, Mapping):
        raise Scene0061FormalBaseError("source requires experiment identity")
    if experiment.get("scene_version") == FORMAL_VERSION:
        raise Scene0061FormalBaseError("source is already formal40k; refusing re-derivation")
    if (experiment.get("identity") or {}).get("artifact_sha256") != scene.get("artifact_sha256"):
        raise Scene0061FormalBaseError("source artifact SHA-256 does not match formal matrix")
    if not isinstance(source.get("config_derivation"), Mapping):
        raise Scene0061FormalBaseError("source must be an axis-bound measured physical config")
    if float((source.get("carla") or {}).get("fixed_delta_seconds", -1.0)) != 0.05:
        raise Scene0061FormalBaseError("source requires CARLA fixed_delta_seconds=0.05")
    runtime = source.get("nurec_runtime")
    if not isinstance(runtime, Mapping) or runtime.get("runtime_scene_id") != RUNTIME_SCENE:
        raise Scene0061FormalBaseError("source runtime_scene_id must be scene-0061")
    if not str(runtime.get("scene_package") or ""):
        raise Scene0061FormalBaseError("source lacks the actual NRE scene-package wrapper")
    if runtime.get("lidar_coordinate_validation") is not None:
        raise Scene0061FormalBaseError("source must not inherit old LiDAR coordinate evidence")
    try:
        validate_lidar_axis_normalization(runtime.get("lidar_axis_normalization"))
    except (LiDARAxisNormalizationError, TypeError, ValueError) as exc:
        raise Scene0061FormalBaseError(f"source LiDAR normalization is not verified: {exc}") from exc
    cameras = runtime.get("camera_specs")
    if not isinstance(cameras, list) or len(cameras) != 6:
        raise Scene0061FormalBaseError("source requires exactly six camera specs")
    if {str(row.get("sensor_id") or "") for row in cameras if isinstance(row, Mapping)} != CAMERAS:
        raise Scene0061FormalBaseError("source camera IDs do not match formal six-camera contract")
    if any(not isinstance(row, Mapping) or row.get("width") != 1600 or row.get("height") != 900 for row in cameras):
        raise Scene0061FormalBaseError("source cameras must all be 1600x900")
    lidars = runtime.get("lidar_specs")
    if not isinstance(lidars, list) or len(lidars) != 1 or not isinstance(lidars[0], Mapping):
        raise Scene0061FormalBaseError("source requires exactly one LiDAR")
    if lidars[0].get("sensor_id") != "lidar_top" or str(lidars[0].get("model") or "").upper() not in {"AT128", "PANDAR128"}:
        raise Scene0061FormalBaseError("source LiDAR must be lidar_top AT128 or PANDAR128")
    selected = {str(value) for value in ((source.get("actor_binding") or {}).get("selected_actor_ids") or [])}
    expected_tracks = {VEHICLE_TRACK, PEDESTRIAN_TRACK}
    if selected != expected_tracks:
        raise Scene0061FormalBaseError("source must select both formal actor tracks")
    observed = {
        str((actor.get("binding") or {}).get("nurec_track_id") or actor.get("source_track_id") or "")
        for actor in (source.get("actors") or []) if isinstance(actor, Mapping)
    }
    if not expected_tracks.issubset(observed):
        raise Scene0061FormalBaseError("source actor records do not bind both formal tracks")


def _validated_execution_inputs(
    source: Mapping[str, Any],
    execution_inputs: Mapping[str, Mapping[str, Any]],
    matrix_opendrive_sha256: str,
) -> list[dict[str, Any]]:
    """Bind the source's actual CARLA/NRE runtime files, not just formal assets.

    The canonical scene package and source OpenDRIVE are immutable formal
    inputs.  NRE consumes a resolved wrapper, while CARLA consumes a converted
    OpenDRIVE.  Both must be declared, re-hashed, and related to the exact
    physical source config so a new formal live tick cannot silently switch
    either runtime input.
    """

    if set(execution_inputs) != EXECUTION_ROLES:
        raise Scene0061FormalBaseError("provided runtime execution inputs are incomplete")
    runtime = source["nurec_runtime"]
    experiment_identity = (source.get("experiment") or {}).get("identity") or {}
    native_scan = runtime.get("native_scan_manifest") or {}
    expected = {
        "runtime_scene_package": {
            "path": runtime.get("scene_package"),
            "sha256": None,
        },
        "runtime_actor_bindings": {
            "path": runtime.get("actor_bindings"),
            "sha256": runtime.get("actor_bindings_sha256"),
        },
        "runtime_native_scan_manifest": {
            "path": native_scan.get("path"),
            "sha256": native_scan.get("sha256"),
        },
        "runtime_opendrive": {
            "path": None,
            "sha256": experiment_identity.get("runtime_opendrive_sha256"),
        },
    }
    source_xodr_hash = _require_hash(
        experiment_identity.get("runtime_opendrive_source_sha256"),
        "source runtime OpenDRIVE source",
    )
    if source_xodr_hash != matrix_opendrive_sha256:
        raise Scene0061FormalBaseError(
            "source runtime OpenDRIVE source SHA-256 does not match formal matrix"
        )
    records: list[dict[str, Any]] = []
    for role in sorted(EXECUTION_ROLES):
        record = dict(execution_inputs[role])
        actual_hash = _require_hash(record.get("sha256"), f"provided {role}")
        expected_hash = expected[role]["sha256"]
        if expected_hash is not None and actual_hash != _require_hash(expected_hash, f"source {role}"):
            raise Scene0061FormalBaseError(
                f"provided {role} SHA-256 does not match physical source config"
            )
        expected_path = expected[role]["path"]
        if expected_path is not None and Path(str(expected_path)).expanduser().resolve() != Path(str(record.get("path") or "")).resolve():
            raise Scene0061FormalBaseError(
                f"provided {role} path does not match physical source config"
            )
        records.append({"role": role, **record})
    return records


def derive_formal_base(
    source: Mapping[str, Any], source_identity: Mapping[str, Any],
    matrix: Mapping[str, Any], matrix_identity: Mapping[str, Any],
    immutable_inputs: Mapping[str, Mapping[str, Any]],
    execution_inputs: Mapping[str, Mapping[str, Any]],
    formal_run_id: str,
) -> dict[str, Any]:
    if not str(formal_run_id).strip():
        raise Scene0061FormalBaseError("formal run ID must be non-empty")
    scene, expected_hashes = _matrix_contract(matrix)
    _validate_physical_source(source, scene)
    if set(immutable_inputs) != ROLES:
        raise Scene0061FormalBaseError("provided immutable input roles are incomplete")
    records = []
    for role in sorted(ROLES):
        record = dict(immutable_inputs[role])
        if _require_hash(record.get("sha256"), f"provided {role}") != expected_hashes[role]:
            raise Scene0061FormalBaseError(f"provided {role} SHA-256 does not match matrix")
        records.append({"role": role, **record})
    execution_records = _validated_execution_inputs(
        source, execution_inputs, expected_hashes["opendrive"]
    )
    result = deepcopy(dict(source))
    result["run_id"] = str(formal_run_id)
    experiment = dict(result["experiment"])
    experiment.update({
        "run_id": str(formal_run_id), "scene_id": scene["scene_id"],
        "scene_version": scene["scene_version"], "artifact_sha256": scene["artifact_sha256"],
        "scene_package_sha256": scene["scene_package_sha256"],
        "scenario_ir_sha256": scene["scenario_ir_sha256"],
        "immutable_matrix_sha256": matrix["immutable_matrix_sha256"],
    })
    nested = dict(experiment.get("identity") or {})
    nested.update({name: experiment[name] for name in ("artifact_sha256", "scene_package_sha256", "scenario_ir_sha256", "immutable_matrix_sha256")})
    experiment["identity"] = nested
    result["experiment"] = experiment
    runtime = dict(result["nurec_runtime"])
    runtime.update({
        "lidar_response_coordinate_frame": "sensor_local",
        "lidar_axis_convention": "carla_sensor",
        "lidar_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
    })
    runtime.pop("lidar_coordinate_validation", None)
    result["nurec_runtime"] = runtime
    result["formal_base_derivation"] = {
        "schema_version": DERIVATION_SCHEMA,
        "kind": "new_formal40k_execution_config_from_measured_physical_source",
        "source_run_config": dict(source_identity),
        "formal_matrix": dict(matrix_identity),
        "immutable_inputs": records,
        "runtime_execution_inputs": execution_records,
        "physical_evidence_status": "pending_new_formal_identity_live_tick",
        "prohibited": [
            "inherit_preformal_lidar_coordinate_evidence",
            "overwrite_historical_output",
            "promote_smoke_result_by_matrix_metadata_only",
        ],
    }
    return result


def _write_once(path: Path, value: Mapping[str, Any]) -> dict[str, Any]:
    target = path.expanduser().resolve()
    if target.exists():
        raise FileExistsError(f"refusing to overwrite output: {target}")
    body = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("xb") as stream:
        stream.write(body)
    return {"path": str(target), "sha256": _sha256(body), "byte_count": len(body)}


def derive_formal_base_file(*, source_config: Path, matrix: Path, immutable_paths: Mapping[str, Path], execution_paths: Mapping[str, Path], formal_run_id: str, output: Path) -> dict[str, Any]:
    source, source_identity = _read_object(source_config, "source run config")
    frozen_matrix, matrix_identity = _read_object(matrix, "formal matrix")
    inputs = {role: _identity(path, f"immutable input {role}") for role, path in immutable_paths.items()}
    execution_inputs = {role: _identity(path, f"runtime execution input {role}") for role, path in execution_paths.items()}
    result = derive_formal_base(source, source_identity, frozen_matrix, matrix_identity, inputs, execution_inputs, formal_run_id)
    return {"status": "passed", "output": _write_once(output, result), "formal_base_derivation": result["formal_base_derivation"]}


def _validate_bound_file_identity(value: object, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise Scene0061FormalBaseError(f"LiDAR evidence lacks {label} identity")
    path = Path(str(value.get("path") or "")).expanduser().resolve()
    actual = _identity(path, f"LiDAR evidence {label}")
    if any(actual.get(name) != value.get(name) for name in ("path", "sha256", "byte_count")):
        raise Scene0061FormalBaseError(f"LiDAR evidence {label} identity no longer matches")
    return actual


def _validate_formal_evidence(
    evidence: Mapping[str, Any],
    experiment: Mapping[str, Any],
    runtime: Mapping[str, Any],
    derivation: Mapping[str, Any],
    base_identity: Mapping[str, Any],
    evidence_identity: Mapping[str, Any],
) -> None:
    # The full native-anchor replay is performed by scene0061_live_tick before
    # this file is written. Binding rechecks the formal identity and passed
    # status rather than pretending an independent JSON-only replay is enough.
    if evidence.get("schema_version") != "scene0061_lidar_coordinate_validation.v1" or evidence.get("status") != "passed":
        raise Scene0061FormalBaseError("LiDAR evidence is not a passed coordinate-validation result")
    expected = {
        "scene_id": experiment.get("scene_id"),
        "runtime_scene_id": runtime.get("runtime_scene_id"),
        "artifact_sha256": experiment.get("artifact_sha256"),
        "sensor_id": "lidar_top",
        "response_coordinate_frame": "sensor_local",
        "axis_convention": "carla_sensor",
        "sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
    }
    for name, value in expected.items():
        if evidence.get(name) != value:
            raise Scene0061FormalBaseError(f"LiDAR evidence {name} does not match formal base")
    axis = evidence.get("axis_validation")
    if not isinstance(axis, Mapping) or axis.get("status") != "passed":
        raise Scene0061FormalBaseError("LiDAR evidence lacks a passing native-anchor axis validation")
    replay = evidence.get("gate_replay")
    if not isinstance(replay, Mapping) or replay.get("status") != "passed":
        raise Scene0061FormalBaseError("LiDAR evidence lacks a passing production gate replay")
    provenance = evidence.get("live_tick_provenance")
    if not isinstance(provenance, Mapping) or provenance.get("schema_version") != (
        "scene0061_lidar_live_tick_provenance.v1"
    ):
        raise Scene0061FormalBaseError("LiDAR evidence lacks live-tick provenance")
    if provenance.get("run_id") != experiment.get("run_id"):
        raise Scene0061FormalBaseError("LiDAR evidence run_id does not match formal base")
    selected = _validate_bound_file_identity(
        provenance.get("selected_config"), "selected config"
    )
    for name in ("path", "sha256", "byte_count"):
        if selected.get(name) != base_identity.get(name):
            raise Scene0061FormalBaseError(
                "LiDAR evidence selected config does not match formal base"
            )
    snapshot = _validate_bound_file_identity(
        provenance.get("runtime_config"), "runtime config snapshot"
    )
    if any(snapshot.get(name) != base_identity.get(name) for name in ("sha256", "byte_count")):
        raise Scene0061FormalBaseError(
            "LiDAR evidence runtime config snapshot does not match formal base"
        )
    opendrive = _validate_bound_file_identity(provenance.get("opendrive"), "OpenDRIVE")
    validation = _validate_bound_file_identity(
        provenance.get("live_tick_validation"), "live-tick validation"
    )
    if provenance["live_tick_validation"].get("status") != "passed":
        raise Scene0061FormalBaseError("LiDAR evidence live-tick validation is not passed")
    try:
        validation_payload = strict_json_loads(Path(validation["path"]).read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, ValueError, json.JSONDecodeError) as exc:
        raise Scene0061FormalBaseError(f"cannot read strict live-tick validation: {exc}") from exc
    if not isinstance(validation_payload, Mapping) or validation_payload.get("status") != "passed":
        raise Scene0061FormalBaseError("persisted live-tick validation is not passed")
    evidence_path = Path(str(evidence_identity.get("path") or "")).expanduser().resolve()
    output_root = evidence_path.parent
    if evidence_path.name != "lidar_axis_evidence.json":
        raise Scene0061FormalBaseError("LiDAR evidence must be the live-tick output artifact")
    if snapshot["path"] != str((output_root / "runtime_run_config.json").resolve()):
        raise Scene0061FormalBaseError("LiDAR evidence runtime config is not from the same output directory")
    if validation["path"] != str((output_root / "live_tick_validation.json").resolve()):
        raise Scene0061FormalBaseError("LiDAR evidence validation is not from the same output directory")
    rows = derivation.get("runtime_execution_inputs")
    declared_opendrive = [
        row for row in rows or []
        if isinstance(row, Mapping) and row.get("role") == "runtime_opendrive"
    ]
    if len(declared_opendrive) != 1 or any(
        opendrive.get(name) != declared_opendrive[0].get(name)
        for name in ("path", "sha256", "byte_count")
    ):
        raise Scene0061FormalBaseError("LiDAR evidence OpenDRIVE does not match formal base")


def bind_lidar_evidence(base: Mapping[str, Any], base_identity: Mapping[str, Any], evidence: Mapping[str, Any], evidence_identity: Mapping[str, Any], bound_run_id: str) -> dict[str, Any]:
    experiment, runtime = base.get("experiment"), base.get("nurec_runtime")
    derivation = base.get("formal_base_derivation")
    if not str(bound_run_id).strip():
        raise Scene0061FormalBaseError("bound run ID must be non-empty")
    if not isinstance(experiment, Mapping) or experiment.get("scene_version") != FORMAL_VERSION or not isinstance(derivation, Mapping):
        raise Scene0061FormalBaseError("LiDAR binding requires a derived formal40k base config")
    if not isinstance(runtime, Mapping) or runtime.get("lidar_coordinate_validation") is not None:
        raise Scene0061FormalBaseError("formal base must have unbound LiDAR coordinate evidence")
    _validate_formal_evidence(
        evidence, experiment, runtime, derivation, base_identity, evidence_identity
    )
    result = deepcopy(dict(base))
    result["run_id"] = str(bound_run_id)
    result["experiment"] = {**dict(experiment), "run_id": str(bound_run_id)}
    result["nurec_runtime"] = {**dict(runtime), "lidar_coordinate_validation": {"evidence_path": evidence_identity["path"], "evidence_sha256": evidence_identity["sha256"]}}
    result["formal_lidar_evidence_binding"] = {
        "schema_version": EVIDENCE_BINDING_SCHEMA,
        "base_run_config": dict(base_identity),
        "lidar_coordinate_evidence": dict(evidence_identity),
        "evidence_status": evidence.get("status"),
    }
    return result


def bind_lidar_evidence_file(*, base_config: Path, lidar_evidence: Path, bound_run_id: str, output: Path) -> dict[str, Any]:
    base, base_identity = _read_object(base_config, "formal base config")
    evidence, evidence_identity = _read_object(lidar_evidence, "LiDAR evidence")
    result = bind_lidar_evidence(base, base_identity, evidence, evidence_identity, bound_run_id)
    return {"status": "passed", "output": _write_once(output, result), "formal_lidar_evidence_binding": result["formal_lidar_evidence_binding"]}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    derive = commands.add_parser("derive", help="create a new formal physical-execution config")
    derive.add_argument("--source-config", type=Path, required=True)
    derive.add_argument("--matrix", type=Path, required=True)
    derive.add_argument("--formal-run-id", required=True)
    derive.add_argument("--output", type=Path, required=True)
    derive.add_argument("--scene-package", type=Path, required=True)
    derive.add_argument("--scenario-ir", type=Path, required=True)
    derive.add_argument("--actor-selection", type=Path, required=True)
    derive.add_argument("--opendrive", type=Path, required=True)
    derive.add_argument("--nurec-usdz", type=Path, required=True)
    derive.add_argument("--runtime-scene-package", type=Path, required=True)
    derive.add_argument("--runtime-actor-bindings", type=Path, required=True)
    derive.add_argument("--runtime-native-scan-manifest", type=Path, required=True)
    derive.add_argument("--runtime-opendrive", type=Path, required=True)
    bind = commands.add_parser("bind-lidar-evidence", help="bind one fresh passing formal LiDAR gate")
    bind.add_argument("--base-config", type=Path, required=True)
    bind.add_argument("--lidar-evidence", type=Path, required=True)
    bind.add_argument("--bound-run-id", required=True)
    bind.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "derive":
            result = derive_formal_base_file(
                source_config=args.source_config, matrix=args.matrix, formal_run_id=args.formal_run_id, output=args.output,
                immutable_paths={
                    "runtime_validated_scene_package": args.scene_package,
                    "actor_ready_scenario_ir": args.scenario_ir,
                    "actor_selection": args.actor_selection,
                    "opendrive": args.opendrive,
                    "nurec_usdz": args.nurec_usdz,
                },
                execution_paths={
                    "runtime_scene_package": args.runtime_scene_package,
                    "runtime_actor_bindings": args.runtime_actor_bindings,
                    "runtime_native_scan_manifest": args.runtime_native_scan_manifest,
                    "runtime_opendrive": args.runtime_opendrive,
                },
            )
        else:
            result = bind_lidar_evidence_file(base_config=args.base_config, lidar_evidence=args.lidar_evidence, bound_run_id=args.bound_run_id, output=args.output)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
