from __future__ import annotations

import base64
import hashlib
import json
import math
import re
from copy import deepcopy
from typing import Any, Mapping

from agents.plugin_contract import canonical_sha256
from agents.transfuserpp_contract import (
    ALGORITHM_ID,
    ALGORITHM_VERSION,
    camera_adaptation_contract,
    cuda_runtime_identity,
)
from runtime.scene0061_counterfactual import validate_scene0061_counterfactual_matrix
from runtime.scene0061_variants import (
    CASE_ACTOR_CONTROL_MODES,
    SUPPORTED_CASES,
    build_scene0061_variant,
)


FORMAL_CAMERAS = {
    "camera_front",
    "camera_front_left",
    "camera_front_right",
    "camera_back",
    "camera_back_left",
    "camera_back_right",
}


class Scene0061TransFuserPPRemoteError(ValueError):
    """Raised when a remote run bundle would rely on an unverified binding."""


def freeze_actor_binding_set_for_case(
    binding_set: Mapping[str, Any], case_id: str
) -> dict[str, Any]:
    """Freeze an actor binding-set file to a focused case's control modes.

    ``build_scene0061_variant`` freezes the per-actor binding blocks embedded
    in the run config, but the sidecar binding-set FILE the NuRec handler
    cross-checks against kept its interactive defaults — the first live
    S-case run failed on exactly that contract mismatch. The frozen file must
    agree with the embedded contract: replay actors render at scenario-IR
    poses (source_track_frame), scripted actors at CARLA runtime poses.
    """

    if case_id not in CASE_ACTOR_CONTROL_MODES:
        raise Scene0061TransFuserPPRemoteError(f"unsupported focused case: {case_id}")
    expected_modes = CASE_ACTOR_CONTROL_MODES[case_id]
    frozen = deepcopy(dict(binding_set))
    bindings = frozen.get("bindings")
    if not isinstance(bindings, list) or not bindings:
        raise Scene0061TransFuserPPRemoteError("actor binding set has no bindings")
    interactive_count = 0
    for item in bindings:
        track_id = str(item.get("source_track_id") or item.get("actor_id") or "")
        if track_id not in expected_modes:
            raise Scene0061TransFuserPPRemoteError(
                f"binding set track is not part of the focused case: {track_id}"
            )
        mode = expected_modes[track_id]
        control = dict(item.get("control") or {})
        control["mode"] = mode
        control["ego_responsive"] = mode == "scripted"
        item["control"] = control
        sync = dict(item.get("sensor_sync") or {})
        if mode == "scripted":
            interactive_count += 1
            sync["pose_source"] = "carla_runtime_actor_pose"
            sync["pose_reference"] = "carla_bounding_box_center"
        else:
            sync["pose_source"] = "scenario_ir_reference_trajectory"
            sync["pose_reference"] = "source_track_frame"
        item["sensor_sync"] = sync
    summary = dict(frozen.get("summary") or {})
    summary["interactive_count"] = interactive_count
    frozen["summary"] = summary
    return frozen


def prepare_scene0061_transfuserpp_remote_run(
    base_run_config: Mapping[str, Any],
    runtime_template: Mapping[str, Any],
    matrix: Mapping[str, Any],
    *,
    case_id: str,
    seed: int,
    event_timestamp_sec: float | None,
    container_payload_root: str = "/sim-data",
    base_actor_bindings: Mapping[str, Any] | None = None,
    actor_bindings_out_path: str | None = None,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any] | None]:
    frozen_matrix = deepcopy(dict(matrix))
    validate_scene0061_counterfactual_matrix(frozen_matrix)
    if case_id not in SUPPORTED_CASES:
        raise Scene0061TransFuserPPRemoteError(f"unsupported focused case: {case_id}")
    if not any(row.get("case_id") == case_id for row in frozen_matrix.get("cases") or []):
        raise Scene0061TransFuserPPRemoteError("case is absent from the immutable matrix")
    matrix_case = next(
        row for row in frozen_matrix["cases"] if row.get("case_id") == case_id
    )
    expected_case_mode = "replay" if case_id == "S0_original_replay" else "scripted"
    if matrix_case.get("actor_control_mode") != expected_case_mode:
        raise Scene0061TransFuserPPRemoteError(
            "counterfactual matrix actor_control_mode conflicts with the focused case"
        )

    scene = frozen_matrix["scene_identity"]
    source = deepcopy(dict(base_run_config))
    _validate_formal_base_identity(source, scene)
    source_experiment = dict(source.get("experiment") or {})
    source_experiment.update(
        {
            "scene_id": scene["scene_id"],
            "scene_version": scene["scene_version"],
        }
    )
    source["experiment"] = source_experiment
    _validate_formal_nurec_sensors(source)
    fixed_delta = (source.get("carla") or {}).get("fixed_delta_seconds")
    if not isinstance(fixed_delta, (int, float)) or abs(float(fixed_delta) - 0.05) > 1e-9:
        raise Scene0061TransFuserPPRemoteError(
            "TransFuser++ formal run requires carla.fixed_delta_seconds=0.05 (20 Hz)"
        )

    variant, delta = build_scene0061_variant(
        source,
        case_id=case_id,
        seed=seed,
        event_timestamp_sec=event_timestamp_sec,
    )
    _validate_actor_control_contract(variant, case_id, expected_case_mode)
    frozen_actor_bindings: dict[str, Any] | None = None
    if base_actor_bindings is not None:
        if not actor_bindings_out_path:
            raise Scene0061TransFuserPPRemoteError(
                "freezing the actor binding set requires actor_bindings_out_path"
            )
        frozen = freeze_actor_binding_set_for_case(base_actor_bindings, case_id)
        # The runtime handler re-hashes the binding-set FILE BYTES
        # (nurec_260_client), so hash the exact serialization the caller
        # must write to actor_bindings_out_path.
        frozen_bytes = (
            json.dumps(frozen, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        ).encode("utf-8")
        frozen_actor_bindings = {
            "binding_set": frozen,
            "file_bytes_b64": base64.b64encode(frozen_bytes).decode("ascii"),
            "file_sha256": hashlib.sha256(frozen_bytes).hexdigest(),
            "path": str(actor_bindings_out_path),
        }
        variant_nurec = dict(variant.get("nurec_runtime") or {})
        variant_nurec["actor_bindings"] = str(actor_bindings_out_path)
        variant_nurec["actor_bindings_sha256"] = frozen_actor_bindings["file_sha256"]
        variant["nurec_runtime"] = variant_nurec
    event_evidence = variant.get("counterfactual_event_evidence") or {}
    if (
        event_evidence.get("case_id") != case_id
        or event_evidence.get("schema_version")
        != "scene0061_counterfactual_event_evidence.v1"
        or (
            case_id != "S0_original_replay"
            and event_evidence.get("status") != "source_trajectory_bound"
        )
    ):
        raise Scene0061TransFuserPPRemoteError(
            "counterfactual event evidence is absent or not source-trajectory-bound"
        )
    nurec = variant["nurec_runtime"]
    camera_spec = next(
        row for row in nurec["camera_specs"] if row.get("sensor_id") == "camera_front"
    )
    lidar_spec = next(
        row for row in nurec["lidar_specs"] if row.get("sensor_id") == "lidar_top"
    )
    run_id = f"scene0061-tfpp-{case_id}-seed-{seed}"
    variant["run_id"] = run_id
    ego = dict(variant.get("ego") or {})
    role_name = str(ego.get("role_name") or "ego_vehicle")
    control_topic = str(
        ego.get("control_topic") or f"/carla/{role_name}/vehicle_control_cmd"
    )
    observation_topic = str(
        ego.get("observation_topic") or "/closed_loop/ego/observation"
    )
    ego.update(
        {
            "driver": "ros2_observation_control",
            "algorithm_id": ALGORITHM_ID,
            "algorithm_version": ALGORITHM_VERSION,
            "control_topic": control_topic,
            "observation_topic": observation_topic,
            "algorithm_sensor_binding": {
                "camera_sensor_id": "camera_front",
                "camera_source_width": camera_spec["width"],
                "camera_source_height": camera_spec["height"],
                "camera_sensor_to_ego": deepcopy(camera_spec["sensor_to_ego"]),
                "camera_sensor_to_ego_coordinate_frame": (
                    "carla_x_forward_y_right_z_up"
                ),
                "camera_adaptation": camera_adaptation_contract(),
                "lidar_sensor_id": "lidar_top",
                "lidar_sensor_to_ego": deepcopy(lidar_spec["sensor_to_ego"]),
                "lidar_axis_convention": "carla_sensor",
                "lidar_sensor_to_ego_coordinate_frame": (
                    "carla_x_forward_y_right_z_up"
                ),
                "container_payload_root": str(container_payload_root),
                "host_reference_scope": "triplicate_output_root",
            },
        }
    )
    variant["ego"] = ego

    experiment = dict(variant.get("experiment") or {})
    experiment.update(
        {
            "scene_id": scene["scene_id"],
            "scene_version": scene["scene_version"],
            "case_id": case_id,
            "seed": seed,
            "algorithm_id": ALGORITHM_ID,
            "algorithm_version": ALGORITHM_VERSION,
            "artifact_sha256": scene["artifact_sha256"],
            "scene_package_sha256": scene["scene_package_sha256"],
            "scenario_ir_sha256": scene["scenario_ir_sha256"],
            "immutable_matrix_sha256": frozen_matrix["immutable_matrix_sha256"],
            "source_run_config_sha256": delta["source_run_config_sha256"],
        }
    )
    variant["experiment"] = experiment
    relative_intermediate_root = f"transfuserpp_intermediates/{case_id}/seed_{seed}"
    variant["algorithm_evidence_contract"] = {
        "schema_version": "transfuserpp_acceptance_evidence_contract.v1",
        "intermediate_root_relative": relative_intermediate_root,
        "backend_failure_count": 0,
        "non_initialization_fallback_count": 0,
        "mismatched_control_count": 0,
        "require_frame_complete_intermediate_trace": True,
    }
    variant["algorithm_runtime_identity"] = cuda_runtime_identity(runtime_template)
    variant["algorithm_gpu_validation"] = {
        "status": "pending",
        "binding_required_before_acceptance": True,
    }
    variant_hash_payload = deepcopy(variant)
    variant_hash_payload.pop("algorithm_gpu_validation", None)
    variant_hash_payload["experiment"].pop("variant_config_sha256", None)
    experiment["variant_config_sha256"] = canonical_sha256(variant_hash_payload)
    experiment["identity"] = {
        name: experiment[name]
        for name in (
            "artifact_sha256",
            "scene_package_sha256",
            "scenario_ir_sha256",
            "immutable_matrix_sha256",
            "source_run_config_sha256",
            "variant_config_sha256",
        )
    }
    variant["experiment"] = experiment
    run_config_sha256 = canonical_sha256(
        {
            key: value
            for key, value in variant.items()
            if key not in {"config_identity", "algorithm_gpu_validation"}
        }
    )
    variant["config_identity"] = {
        "schema_version": "closedloopbench_run_config_identity.v1",
        "canonical_sha256": run_config_sha256,
        "hash_scope": "whole_run_config_excluding_config_identity_and_algorithm_gpu_validation",
    }

    runtime_config = deepcopy(dict(runtime_template))
    runtime_config.update(
        {
            "schema_version": "transfuserpp_runtime_config.v1",
            "algorithm_id": ALGORITHM_ID,
            "intermediate_output_dir": (
                f"{str(container_payload_root).rstrip('/')}/{relative_intermediate_root}"
            ),
            "control_topic": control_topic,
            "observation_topic": observation_topic,
            "scene_id": scene["scene_id"],
            "case_id": case_id,
            "seed": seed,
            "experiment": {
                name: experiment[name]
                for name in (
                    "scene_id",
                    "scene_version",
                    "case_id",
                    "seed",
                    "artifact_sha256",
                    "scene_package_sha256",
                    "scenario_ir_sha256",
                    "immutable_matrix_sha256",
                    "source_run_config_sha256",
                    "variant_config_sha256",
                )
            }
            | {"run_config_sha256": run_config_sha256},
        }
    )
    runtime_config.pop("real_checkpoint_loaded", None)

    bundle = {
        "schema_version": "scene0061_transfuserpp_remote_run_bundle.v1",
        "status": "remote_validation_required",
        "run_id": run_id,
        "case_id": case_id,
        "seed": seed,
        "run_config_sha256": run_config_sha256,
        "runtime_config_sha256": canonical_sha256(runtime_config),
        "delta": deepcopy(delta),
        "counterfactual_event_evidence": deepcopy(event_evidence),
        "counterfactual_event_evidence_sha256": canonical_sha256(event_evidence),
        "container_mount_contract": {
            "SIM_DATA_HOST_PATH": "triplicate_output_root",
            "container_path": str(container_payload_root),
            "payload_paths": "attempt-relative and remapped by the observation driver",
        },
        "transport_contract": {
            "control_topic": control_topic,
            "observation_topic": observation_topic,
            "control_frequency_hz": 20.0,
            "fixed_delta_seconds": 0.05,
        },
        "required_remote_gates": [
            "transfuserpp_runtime_manifest_prepared",
            "cuda_preflight_evidence_bound",
            "real_checkpoint_loaded",
            "nurec_lidar_coordinate_frame_verified",
            "backend_failure_count_zero",
            "intermediate_trace_valid",
            "render_quality_report_identity_match",
            "multimodal_closed_loop_passed",
        ],
    }
    return variant, runtime_config, bundle, frozen_actor_bindings


def _validate_formal_base_identity(
    source: Mapping[str, Any], scene: Mapping[str, Any]
) -> None:
    """Reject a smoke/diagnostic config before matrix metadata can overwrite it.

    A formal matrix describes the target experiment; it is not authority to
    promote a physically different source run.  In particular, the r19 live
    tick is useful LiDAR evidence but carries a smoke scene version and must
    never become a formal TransFuser++ base merely because this preparation
    function subsequently fills matrix identity fields.
    """

    experiment = source.get("experiment")
    if not isinstance(experiment, Mapping):
        raise Scene0061TransFuserPPRemoteError(
            "formal base run config requires an experiment identity"
        )
    if experiment.get("scene_id") != scene.get("scene_id"):
        raise Scene0061TransFuserPPRemoteError(
            "base run config scene_id does not match the formal matrix"
        )
    if experiment.get("scene_version") != scene.get("scene_version"):
        raise Scene0061TransFuserPPRemoteError(
            "base run config scene_version does not match the formal matrix"
        )
    if source.get("scenario_id") != scene.get("scene_id"):
        raise Scene0061TransFuserPPRemoteError(
            "base run config scenario_id does not match the formal matrix"
        )


def _validate_formal_nurec_sensors(config: Mapping[str, Any]) -> None:
    nurec = config.get("nurec_runtime")
    if not isinstance(nurec, Mapping):
        raise Scene0061TransFuserPPRemoteError("base run config requires nurec_runtime")
    if not str(nurec.get("runtime_scene_id") or "").strip():
        raise Scene0061TransFuserPPRemoteError(
            "formal run requires a non-empty NRE runtime_scene_id"
        )
    cameras = nurec.get("camera_specs")
    if (
        not isinstance(cameras, list)
        or len(cameras) != 6
        or any(not isinstance(row, Mapping) for row in cameras)
    ):
        raise Scene0061TransFuserPPRemoteError(
            "formal run requires exactly six camera specification records"
        )
    camera_ids = [str(row.get("sensor_id") or "") for row in cameras]
    if len(set(camera_ids)) != 6 or set(camera_ids) != FORMAL_CAMERAS:
        raise Scene0061TransFuserPPRemoteError(
            "formal run must contain six unique scene-0061 camera IDs"
        )
    for row in cameras:
        width = row.get("width", row.get("resolution_w"))
        height = row.get("height", row.get("resolution_h"))
        if width != 1600 or height != 900:
            raise Scene0061TransFuserPPRemoteError(
                f"formal physical camera {row.get('sensor_id')} must be 1600x900"
            )
        _validated_sensor_to_ego(
            row.get("sensor_to_ego"), f"formal camera {row.get('sensor_id')}"
        )
    lidars = nurec.get("lidar_specs")
    if (
        not isinstance(lidars, list)
        or len(lidars) != 1
        or not isinstance(lidars[0], Mapping)
        or lidars[0].get("sensor_id") != "lidar_top"
    ):
        raise Scene0061TransFuserPPRemoteError(
            "formal run requires exactly one LiDAR specification named lidar_top"
        )
    _validated_sensor_to_ego(lidars[0].get("sensor_to_ego"), "formal lidar_top")
    if str(lidars[0].get("model") or "").upper() not in {"AT128", "PANDAR128"}:
        raise Scene0061TransFuserPPRemoteError(
            "formal lidar_top model must be AT128 or PANDAR128"
        )
    if nurec.get("lidar_response_coordinate_frame") != "sensor_local":
        raise Scene0061TransFuserPPRemoteError(
            "NRE LiDAR response coordinate frame must be explicitly verified as sensor_local"
        )
    if nurec.get("lidar_axis_convention") != "carla_sensor":
        raise Scene0061TransFuserPPRemoteError(
            "NRE LiDAR axis convention must be explicitly verified as carla_sensor"
        )
    if nurec.get("lidar_sensor_to_ego_coordinate_frame") != (
        "carla_x_forward_y_right_z_up"
    ):
        raise Scene0061TransFuserPPRemoteError(
            "lidar sensor_to_ego must be verified in CARLA x-forward/y-right/z-up"
        )
    evidence = nurec.get("lidar_coordinate_validation")
    evidence_sha256 = str((evidence or {}).get("evidence_sha256") or "")
    if (
        not isinstance(evidence, Mapping)
        or not evidence.get("evidence_path")
        or re.fullmatch(r"[0-9a-f]{64}", evidence_sha256) is None
    ):
        raise Scene0061TransFuserPPRemoteError(
            "NRE LiDAR coordinate validation evidence path and lowercase SHA-256 are required"
        )


def _validate_actor_control_contract(
    config: Mapping[str, Any], case_id: str, expected_case_mode: str
) -> None:
    contract = config.get("actor_control_contract")
    if (
        not isinstance(contract, Mapping)
        or contract.get("schema_version") != "scene0061_actor_control_contract.v1"
        or contract.get("case_id") != case_id
        or contract.get("case_actor_control_mode") != expected_case_mode
    ):
        raise Scene0061TransFuserPPRemoteError(
            "scene-0061 actor control contract is absent or conflicts with the case"
        )
    expected_modes = CASE_ACTOR_CONTROL_MODES[case_id]
    if contract.get("effective_modes_by_track") != expected_modes:
        raise Scene0061TransFuserPPRemoteError(
            "scene-0061 effective actor control modes are not frozen"
        )
    rows = contract.get("actors")
    if not isinstance(rows, list) or len(rows) != 2:
        raise Scene0061TransFuserPPRemoteError(
            "scene-0061 actor control contract must contain both formal actors"
        )
    actors = config.get("actors") or []
    for track_id, expected_mode in expected_modes.items():
        actor_matches = [
            actor
            for actor in actors
            if isinstance(actor, Mapping)
            and track_id
            in {
                actor.get("source_track_id"),
                actor.get("track_id"),
                (actor.get("binding") or {}).get("nurec_track_id"),
            }
        ]
        row_matches = [row for row in rows if row.get("source_track_id") == track_id]
        if len(actor_matches) != 1 or len(row_matches) != 1:
            raise Scene0061TransFuserPPRemoteError(
                f"actor control contract does not uniquely bind track {track_id}"
            )
        actor = actor_matches[0]
        row = row_matches[0]
        control_contract = actor.get("control_mode_contract")
        binding = actor.get("binding")
        if not isinstance(control_contract, Mapping) or not isinstance(binding, Mapping):
            raise Scene0061TransFuserPPRemoteError(
                f"actor control contract or binding is absent for track {track_id}"
            )
        if (
            actor.get("closed_loop_level") != expected_mode
            or actor.get("effective_control_mode") != expected_mode
            or row.get("effective_mode") != expected_mode
        ):
            raise Scene0061TransFuserPPRemoteError(
                f"actor control mode mismatch for track {track_id}"
            )
        pose_source = row.get("sensor_pose_source")
        pose_reference = row.get("sensor_pose_reference")
        if (
            pose_source != binding.get("sensor_pose_source")
            or pose_source != control_contract.get("sensor_pose_source")
            or pose_reference != binding.get("sensor_pose_reference")
            or pose_reference != control_contract.get("sensor_pose_reference")
        ):
            raise Scene0061TransFuserPPRemoteError(
                f"actor pose contract mismatch for track {track_id}"
            )


def _validated_sensor_to_ego(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 16:
        raise Scene0061TransFuserPPRemoteError(
            f"{label} requires a complete 16-value sensor_to_ego matrix"
        )
    if any(
        isinstance(item, bool)
        or not isinstance(item, (int, float))
        or not math.isfinite(float(item))
        for item in value
    ):
        raise Scene0061TransFuserPPRemoteError(
            f"{label} sensor_to_ego matrix must contain finite numeric values"
        )
    matrix = [float(item) for item in value]
    if any(
        abs(matrix[index] - expected) > 1e-6
        for index, expected in zip((12, 13, 14, 15), (0.0, 0.0, 0.0, 1.0))
    ):
        raise Scene0061TransFuserPPRemoteError(
            f"{label} sensor_to_ego must be a homogeneous rigid transform"
        )
    rotation = [matrix[0:3], matrix[4:7], matrix[8:11]]
    for left_index, left in enumerate(rotation):
        for right_index, right in enumerate(rotation):
            dot = sum(a * b for a, b in zip(left, right))
            expected = 1.0 if left_index == right_index else 0.0
            if abs(dot - expected) > 1e-4:
                raise Scene0061TransFuserPPRemoteError(
                    f"{label} sensor_to_ego rotation is not orthonormal"
                )
    determinant = (
        rotation[0][0]
        * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1]
        * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2]
        * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > 1e-4:
        raise Scene0061TransFuserPPRemoteError(
            f"{label} sensor_to_ego rotation determinant must be +1"
        )
    return matrix
