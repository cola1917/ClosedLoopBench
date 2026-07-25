from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

from agents.plugin_contract import canonical_sha256, file_sha256


ALGORITHM_ID = "transfuserpp_v5"
ALGORITHM_VERSION = "carla_garage.leaderboard_2.transfuser_v5"
UPSTREAM_REPOSITORY = "https://github.com/autonomousvision/carla_garage"
UPSTREAM_BRANCH = "leaderboard_2"
UPSTREAM_REFERENCE = "refs/remotes/origin/leaderboard_2"
REQUIRED_UPSTREAM_FILES = (
    "team_code/config.py",
    "team_code/data.py",
    "team_code/model.py",
    "team_code/transfuser_utils.py",
)
REQUIRED_CARLA_NAVIGATION_FILES = (
    "navigation/__init__.py",
    "navigation/global_route_planner.py",
    "navigation/local_planner.py",
)
ADAPTER_SOURCE_FILES = (
    "agents/algorithm_backend.py",
    "agents/plugin_contract.py",
    "agents/transfuserpp_contract.py",
    "agents/transfuserpp_plugin.py",
    "agents/transfuserpp_ros2_backend.py",
    "agents/transfuserpp_runtime.py",
    "runners/build_transfuserpp_runtime_manifest.py",
    "runners/run_algorithm_container.py",
    "runners/run_transfuserpp_cuda_preflight.py",
)
REQUIRED_RGB_CAMERAS = ("camera_front",)
REQUIRED_LIDARS = ("lidar_top",)
FORMAL_CAMERA_SOURCE_SIZE = (1600, 900)
MODEL_CAMERA_INPUT_SIZE = (800, 450)
CAMERA_ADAPTATION_SCHEMA_VERSION = "transfuserpp_camera_adaptation.v1"
CAMERA_ADAPTATION_EVIDENCE_SCHEMA_VERSION = (
    "transfuserpp_camera_adaptation_evidence.v1"
)
CAMERA_ADAPTATION_METHOD = "resize_linear"
REQUIRED_DENSE_KEYS = (
    "bev_semantic_labels",
    "perspective_semantic_labels",
    "depth",
    "target_speed_probabilities",
)
FOCUSED_CASES = {
    "S0_original_replay",
    "S2_lead_hard_brake",
    "S4_pedestrian_early_crossing",
}
SUPPORTED_ROUTE_COMMANDS = {
    "LEFT": 1,
    "RIGHT": 2,
    "STRAIGHT": 3,
    "LANE_FOLLOW": 4,
    "CHANGE_LANE_LEFT": 5,
    "CHANGE_LANE_RIGHT": 6,
}
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
GIT_REVISION_RE = re.compile(r"^[0-9a-f]{40}$")


class TransFuserPPContractError(ValueError):
    """Raised when the TF++ runtime boundary cannot be proven safe."""


def capability() -> dict[str, Any]:
    return {
        "algorithm_id": ALGORITHM_ID,
        "algorithm_version": ALGORITHM_VERSION,
        "uses_route": True,
        "uses_ego_state": True,
        "required_rgb_cameras": list(REQUIRED_RGB_CAMERAS),
        "requires_lidar": True,
        "required_lidars": list(REQUIRED_LIDARS),
        "is_perception_algorithm": True,
        "requires_gpu": True,
        "checkpoint_identity": "external_required",
        "supported_control_hz": 20.0,
        "timeout_sec": 0.5,
        "intermediate_outputs": [
            "perspective_semantic",
            "bev_semantic",
            "depth",
            "bounding_boxes",
            "waypoints",
            "route_checkpoints",
            "target_speed_distribution",
            "target_speed_mps",
            "vehicle_control",
        ],
        "full_3d_occupancy_output": False,
    }


def camera_adaptation_contract() -> dict[str, Any]:
    """Return the immutable physical-camera-to-model-input boundary.

    NuRec remains the source of the physical 1600x900 RGB frame. The adapter
    deterministically resizes that source into the 800x450 TF++ input canvas
    before applying the upstream model crop. The hash covers all static
    transform parameters so a frame trace cannot silently use another resize.
    """

    source_width, source_height = FORMAL_CAMERA_SOURCE_SIZE
    target_width, target_height = MODEL_CAMERA_INPUT_SIZE
    payload = {
        "schema_version": CAMERA_ADAPTATION_SCHEMA_VERSION,
        "method": CAMERA_ADAPTATION_METHOD,
        "source_width": source_width,
        "source_height": source_height,
        "target_width": target_width,
        "target_height": target_height,
        "source_coordinate_frame": "camera_optical",
        "source_payload_sha256_binding": "camera_front.sha256",
        "interpolation": "linear",
    }
    return {**payload, "contract_sha256": canonical_sha256(payload)}


def validate_camera_adaptation_contract(value: Any, label: str) -> dict[str, Any]:
    expected = camera_adaptation_contract()
    if not isinstance(value, Mapping) or dict(value) != expected:
        raise TransFuserPPContractError(
            f"{label} must exactly match the 1600x900-to-800x450 camera adaptation contract"
        )
    return deepcopy(expected)


def camera_adaptation_evidence(
    *,
    contract: Mapping[str, Any],
    source_payload: Mapping[str, Any],
    model_sensor_width: int,
    model_sensor_height: int,
    center_crop_xyxy: list[int],
    model_crop_applied_by_upstream: bool,
) -> dict[str, Any]:
    """Bind a materialized physical RGB payload to its resize provenance."""

    checked = validate_camera_adaptation_contract(contract, "camera adaptation")
    if not SHA256_RE.fullmatch(str(source_payload.get("sha256") or "")):
        raise TransFuserPPContractError("camera adaptation source payload SHA-256 is invalid")
    byte_count = source_payload.get("byte_count")
    if isinstance(byte_count, bool) or not isinstance(byte_count, int) or byte_count < 0:
        raise TransFuserPPContractError("camera adaptation source payload byte_count is invalid")
    if (
        isinstance(model_sensor_width, bool)
        or isinstance(model_sensor_height, bool)
        or not isinstance(model_sensor_width, int)
        or not isinstance(model_sensor_height, int)
        or model_sensor_width <= 0
        or model_sensor_height <= 0
    ):
        raise TransFuserPPContractError("camera adaptation model sensor size is invalid")
    expected_crop = camera_center_crop_window(
        checked["target_width"],
        checked["target_height"],
        model_sensor_width,
        model_sensor_height,
    )
    if center_crop_xyxy != expected_crop:
        raise TransFuserPPContractError("camera adaptation center crop is inconsistent")
    payload = {
        "schema_version": CAMERA_ADAPTATION_EVIDENCE_SCHEMA_VERSION,
        "contract_sha256": checked["contract_sha256"],
        "source_payload_sha256": str(source_payload["sha256"]),
        "source_payload_byte_count": byte_count,
        "source_width": checked["source_width"],
        "source_height": checked["source_height"],
        "target_width": checked["target_width"],
        "target_height": checked["target_height"],
        "method": checked["method"],
        "interpolation": checked["interpolation"],
        "model_sensor_width": model_sensor_width,
        "model_sensor_height": model_sensor_height,
        "center_crop_xyxy": list(center_crop_xyxy),
        "model_crop_applied_by_upstream": bool(model_crop_applied_by_upstream),
    }
    return {**payload, "evidence_sha256": canonical_sha256(payload)}


def validate_camera_adaptation_evidence(
    value: Any,
    *,
    source_payload: Mapping[str, Any],
    calibration: Mapping[str, Any],
    label: str,
) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TransFuserPPContractError(f"{label} is required")
    contract = validate_camera_adaptation_contract(
        calibration.get("camera_adaptation"), f"{label}.contract"
    )
    try:
        expected = camera_adaptation_evidence(
            contract=contract,
            source_payload=source_payload,
            model_sensor_width=value["model_sensor_width"],
            model_sensor_height=value["model_sensor_height"],
            center_crop_xyxy=value["center_crop_xyxy"],
            model_crop_applied_by_upstream=value["model_crop_applied_by_upstream"],
        )
    except KeyError as exc:
        raise TransFuserPPContractError(f"{label} is incomplete: {exc.args[0]}") from exc
    if dict(value) != expected:
        raise TransFuserPPContractError(f"{label} hash or transform fields are invalid")
    return deepcopy(expected)


def runtime_config_schema() -> dict[str, Any]:
    return {
        "schema_version": "transfuserpp_runtime_config.v1",
        "algorithm_id": ALGORITHM_ID,
        "upstream": {
            "repository": UPSTREAM_REPOSITORY,
            "branch": UPSTREAM_BRANCH,
            "reference": UPSTREAM_REFERENCE,
            "family_version": "TransFuser v5 / Leaderboard 2.0",
        },
        "required": [
            "repo_path",
            "repo_revision",
            "upstream_reference",
            "repo_sha256",
            "checkpoint_path",
            "checkpoint_sha256",
            "model_config_path",
            "model_config_sha256",
            "intermediate_output_dir",
            "carla_agents_path",
            "carla_agents_sha256",
            "container_image_digest",
        ],
        "sensor_contract": {
            "rgb": list(REQUIRED_RGB_CAMERAS),
            "lidar": list(REQUIRED_LIDARS),
            "nurec_formal_cameras_still_required": 6,
            "algorithm_camera_count": 1,
            "lidar_encoding": "float32_xyzi_little_endian",
            "lidar_coordinate_frame": "sensor_local_with_explicit_sensor_to_ego",
        },
        "navigation_contract": {
            "speed_source": "carla_actor_velocity",
            "compass_source": "carla_actor_transform_yaw",
            "gps_source": "bypassed",
            "target_point_source": "closedloopbench_route_planner_ego_frame",
            "official_leaderboard_sensor_equivalent": False,
        },
        "defaults": {
            "device": "cuda:0",
            "supported_control_hz": 20.0,
            "timeout_sec": 0.5,
            "max_synchronization_error_ms": 1.0,
            "jpeg_roundtrip": False,
            "camera_adaptation": "center_crop_to_model_aspect_then_resize",
            "compile_model": False,
            "single_checkpoint_only": True,
        },
        "formal_identity_required": [
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
            "run_config_sha256",
        ],
    }


def build_runtime_manifest(config: Mapping[str, Any]) -> dict[str, Any]:
    cfg = deepcopy(dict(config))
    problems: list[str] = []
    repo = Path(str(cfg.get("repo_path") or "")).expanduser()
    checkpoint = Path(str(cfg.get("checkpoint_path") or "")).expanduser()
    model_config = Path(str(cfg.get("model_config_path") or "")).expanduser()
    output_dir = Path(str(cfg.get("intermediate_output_dir") or "")).expanduser()
    carla_agents = Path(str(cfg.get("carla_agents_path") or "")).expanduser()

    if cfg.get("schema_version") != "transfuserpp_runtime_config.v1":
        problems.append("runtime_schema_version_mismatch")
    if cfg.get("algorithm_id") != ALGORITHM_ID:
        problems.append("runtime_algorithm_id_mismatch")

    if not repo.is_dir():
        problems.append("repo_path_unavailable")
    else:
        for relative in REQUIRED_UPSTREAM_FILES:
            if not (repo / relative).is_file():
                problems.append(f"upstream_file_missing:{relative}")
    if not checkpoint.is_file():
        problems.append("checkpoint_path_unavailable")
    if not model_config.is_file():
        problems.append("model_config_path_unavailable")
    if not carla_agents.is_dir():
        problems.append("carla_agents_path_unavailable")
    else:
        for relative in REQUIRED_CARLA_NAVIGATION_FILES:
            if not (carla_agents / relative).is_file():
                problems.append(f"carla_navigation_file_missing:{relative}")
    if not str(cfg.get("intermediate_output_dir") or ""):
        problems.append("intermediate_output_dir_missing")
    elif output_dir.exists() and not output_dir.is_dir():
        problems.append("intermediate_output_dir_not_directory")

    _validate_declared_hash(
        cfg.get("repo_sha256"), "repo_sha256", problems, allow_content_only=True
    )
    _validate_declared_hash(cfg.get("checkpoint_sha256"), "checkpoint_sha256", problems)
    _validate_declared_hash(cfg.get("model_config_sha256"), "model_config_sha256", problems)
    _validate_declared_hash(cfg.get("carla_agents_sha256"), "carla_agents_sha256", problems)
    if re.fullmatch(r"sha256:[0-9a-f]{64}", str(cfg.get("container_image_digest") or "")) is None:
        problems.append("container_image_digest_invalid")
    revision = str(cfg.get("repo_revision") or "")
    if not GIT_REVISION_RE.fullmatch(revision):
        problems.append("repo_revision_invalid")

    actual_revision = _git_revision(repo) if repo.is_dir() else None
    if repo.is_dir() and actual_revision is None:
        problems.append("repo_revision_unverified")
    if actual_revision and GIT_REVISION_RE.fullmatch(revision) and actual_revision != revision:
        problems.append("repo_revision_mismatch")
    actual_origin = _git_origin(repo) if repo.is_dir() else None
    if repo.is_dir() and actual_origin is None:
        problems.append("repo_origin_unverified")
    elif actual_origin and actual_origin != UPSTREAM_REPOSITORY:
        problems.append("repo_origin_mismatch")
    configured_reference = str(cfg.get("upstream_reference") or "")
    if configured_reference != UPSTREAM_REFERENCE:
        problems.append("upstream_reference_invalid")
    resolved_reference = (
        _git_reference_commit(repo, configured_reference)
        if repo.is_dir() and configured_reference == UPSTREAM_REFERENCE
        else None
    )
    if repo.is_dir() and configured_reference == UPSTREAM_REFERENCE and resolved_reference is None:
        problems.append("upstream_reference_unverified")
    revision_in_reference = (
        _git_is_ancestor(repo, revision, resolved_reference)
        if resolved_reference and GIT_REVISION_RE.fullmatch(revision)
        else None
    )
    if resolved_reference and revision_in_reference is not True:
        problems.append("repo_revision_not_in_upstream_reference_history")
    worktree_clean = _git_worktree_clean(repo) if repo.is_dir() else None
    if repo.is_dir() and worktree_clean is not True:
        problems.append("repo_worktree_not_clean")
    actual_repo_hash = repository_snapshot_sha256(repo) if actual_revision else None
    if repo.is_dir() and actual_repo_hash is None:
        problems.append("repo_sha256_unverified")
    if actual_repo_hash and SHA256_RE.fullmatch(str(cfg.get("repo_sha256") or "")):
        if actual_repo_hash != cfg["repo_sha256"]:
            problems.append("repo_sha256_mismatch")
    if checkpoint.is_file() and SHA256_RE.fullmatch(str(cfg.get("checkpoint_sha256") or "")):
        if file_sha256(checkpoint) != cfg["checkpoint_sha256"]:
            problems.append("checkpoint_sha256_mismatch")
    if model_config.is_file() and SHA256_RE.fullmatch(str(cfg.get("model_config_sha256") or "")):
        if file_sha256(model_config) != cfg["model_config_sha256"]:
            problems.append("model_config_sha256_mismatch")
    actual_carla_agents_hash = directory_snapshot_sha256(carla_agents)
    if carla_agents.is_dir() and actual_carla_agents_hash is None:
        problems.append("carla_agents_sha256_unverified")
    if actual_carla_agents_hash and SHA256_RE.fullmatch(
        str(cfg.get("carla_agents_sha256") or "")
    ):
        if actual_carla_agents_hash != cfg["carla_agents_sha256"]:
            problems.append("carla_agents_sha256_mismatch")
    _validate_experiment_identity(cfg.get("experiment"), problems)
    adapter_source_hash = adapter_source_snapshot_sha256()
    if adapter_source_hash is None:
        problems.append("adapter_source_sha256_unverified")

    return {
        "schema_version": "transfuserpp_runtime_manifest.v1",
        "algorithm_id": ALGORITHM_ID,
        "algorithm_version": ALGORITHM_VERSION,
        "execution_status": "prepared" if not problems else "blocked",
        "evidence_classification": "remote_validation_required",
        "real_checkpoint_loaded": False,
        "remote_gpu_validation_required": True,
        "official_leaderboard_sensor_equivalent": False,
        "upstream_sensor_agent_safety_controllers": "bypassed",
        "upstream": {
            "repository": UPSTREAM_REPOSITORY,
            "branch": UPSTREAM_BRANCH,
            "configured_reference": configured_reference or "unavailable",
            "resolved_reference_commit": resolved_reference or "unavailable",
            "reference_binding_sha256": (
                canonical_sha256(
                    {
                        "repository": UPSTREAM_REPOSITORY,
                        "reference": configured_reference,
                        "resolved_commit": resolved_reference,
                    }
                )
                if resolved_reference
                else "unavailable"
            ),
            "repo_revision_in_reference_history": revision_in_reference is True,
            "repo_revision": revision or "unavailable",
            "detected_repo_revision": actual_revision or "unavailable",
            "detected_repo_sha256": actual_repo_hash or "unavailable",
            "detected_repo_origin": actual_origin or "unavailable",
            "worktree_clean_including_untracked": worktree_clean is True,
        },
        "identity": {
            "repo_sha256": cfg.get("repo_sha256") or "unavailable",
            "checkpoint_sha256": cfg.get("checkpoint_sha256") or "unavailable",
            "model_config_sha256": cfg.get("model_config_sha256") or "unavailable",
            "carla_agents_sha256": cfg.get("carla_agents_sha256") or "unavailable",
            "adapter_source_sha256": adapter_source_hash or "unavailable",
            "container_image_digest": cfg.get("container_image_digest") or "unavailable",
            "container_image_selection": "docker_compose_service_image_id",
            "runtime_config_sha256": canonical_sha256(_identity_config(cfg)),
        },
        "capability": capability(),
        "problems": sorted(set(problems)),
        "remote_actions_remaining": [
            "install_or_build_transfuserpp_runtime",
            "bind_pinned_carla_garage_repository",
            "bind_real_checkpoint_and_model_config",
            "bind_matching_carla_pythonapi_agents_navigation",
            "verify_cuda_inference_and_memory",
            "bind_live_nurec_rgb_lidar_payloads",
            "run_scene0061_s0_s2_s4",
        ],
    }


def assert_runtime_prepared(config: Mapping[str, Any]) -> dict[str, Any]:
    manifest = build_runtime_manifest(config)
    if manifest["execution_status"] != "prepared":
        raise TransFuserPPContractError(
            "TransFuser++ runtime is not prepared: " + ", ".join(manifest["problems"])
        )
    return manifest


def validate_observation(
    observation: Mapping[str, Any], *, max_synchronization_error_ms: float = 1.0
) -> dict[str, Any]:
    if not isinstance(observation, Mapping):
        raise TransFuserPPContractError("observation must be an object")
    frame_id = observation.get("frame_id")
    if isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id < 0:
        raise TransFuserPPContractError("frame_id must be a non-negative integer")
    timestamp = observation.get("timestamp", observation.get("t_sec"))
    if not _finite_nonnegative(timestamp):
        raise TransFuserPPContractError("timestamp must be finite and non-negative")

    rgb = observation.get("rgb")
    if not isinstance(rgb, Mapping):
        raise TransFuserPPContractError("rgb must be an object")
    front = _validate_payload_ref(rgb.get("camera_front"), "camera_front")
    lidar = _validate_payload_ref(observation.get("lidar"), "lidar_top")
    if front.get("encoding") != "jpeg" or front.get("coordinate_frame") != "camera_optical":
        raise TransFuserPPContractError(
            "camera_front must be a JPEG payload in camera_optical coordinates"
        )
    calibration = observation.get("calibration")
    if not isinstance(calibration, Mapping):
        raise TransFuserPPContractError("observation calibration is required")
    if calibration.get("camera_sensor_id") != "camera_front":
        raise TransFuserPPContractError("camera calibration sensor ID mismatch")
    validate_camera_adaptation_contract(
        calibration.get("camera_adaptation"), "camera calibration adaptation"
    )
    _rigid_matrix(
        calibration.get("camera_sensor_to_ego"),
        "calibration.camera_sensor_to_ego",
    )
    if calibration.get("camera_sensor_to_ego_coordinate_frame") != (
        "carla_x_forward_y_right_z_up"
    ):
        raise TransFuserPPContractError("camera calibration coordinate frame is invalid")
    if calibration.get("lidar_sensor_id") != "lidar_top":
        raise TransFuserPPContractError("LiDAR calibration sensor ID mismatch")
    if lidar.get("encoding") != "float32_xyzi_little_endian":
        raise TransFuserPPContractError("lidar_top encoding must be float32_xyzi_little_endian")
    if lidar.get("coordinate_frame") not in {"sensor_local", "carla_ego"}:
        raise TransFuserPPContractError("lidar_top coordinate_frame must be sensor_local or carla_ego")
    if lidar.get("coordinate_frame") == "sensor_local":
        lidar_matrix = _rigid_matrix(lidar.get("sensor_to_ego"), "lidar_top.sensor_to_ego")
        calibrated_lidar_matrix = _rigid_matrix(
            calibration.get("lidar_sensor_to_ego"),
            "calibration.lidar_sensor_to_ego",
        )
        if lidar_matrix != calibrated_lidar_matrix:
            raise TransFuserPPContractError(
                "LiDAR payload and calibration sensor_to_ego mismatch"
            )
        if lidar.get("axis_convention") != "carla_sensor":
            raise TransFuserPPContractError(
                "sensor-local lidar_top axis_convention must be carla_sensor"
            )

    ego = observation.get("ego_state")
    if not isinstance(ego, Mapping) or not _finite_nonnegative(ego.get("speed_mps")):
        raise TransFuserPPContractError("ego_state.speed_mps is required")
    pose = ego.get("pose")
    if not isinstance(pose, Mapping) or any(
        not _finite(pose.get(name)) for name in ("x", "y", "yaw")
    ):
        raise TransFuserPPContractError("ego_state.pose x/y/yaw is required")

    route = observation.get("route")
    if not isinstance(route, Mapping):
        raise TransFuserPPContractError("route is required")
    target = route.get("target_point_ego_m", route.get("target_point"))
    if (
        not isinstance(target, (list, tuple))
        or len(target) != 2
        or any(not _finite(value) for value in target)
    ):
        raise TransFuserPPContractError("route.target_point_ego_m must contain finite x/y")
    command = str(route.get("route_command") or "")
    if command not in SUPPORTED_ROUTE_COMMANDS:
        raise TransFuserPPContractError(f"unsupported route_command: {command}")
    if route.get("target_point_coordinate_frame", "carla_ego") != "carla_ego":
        raise TransFuserPPContractError("target point must use the carla_ego coordinate frame")

    sync = observation.get("synchronization")
    if not isinstance(sync, Mapping):
        raise TransFuserPPContractError("synchronization metadata is required")
    if sync.get("frame_id") != frame_id:
        raise TransFuserPPContractError("frame_mismatch: synchronization frame differs")
    error_ms = sync.get("error_ms")
    if not _finite_nonnegative(error_ms):
        raise TransFuserPPContractError("synchronization.error_ms is invalid")
    if float(error_ms) > float(max_synchronization_error_ms):
        raise TransFuserPPContractError("synchronization_error_threshold_exceeded")
    if not SHA256_RE.fullmatch(str(sync.get("dynamic_object_sha256") or "")):
        raise TransFuserPPContractError("synchronization.dynamic_object_sha256 is invalid")
    run_context = observation.get("run_context")
    if not isinstance(run_context, Mapping) or not re.fullmatch(
        r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", str(run_context.get("run_id") or "")
    ):
        raise TransFuserPPContractError("run_context.run_id is required")
    run_identity = run_context.get("identity")
    if not isinstance(run_identity, Mapping) or any(
        not SHA256_RE.fullmatch(str(run_identity.get(name) or ""))
        for name in (
            "artifact_sha256",
            "scene_package_sha256",
            "scenario_ir_sha256",
            "immutable_matrix_sha256",
            "source_run_config_sha256",
            "variant_config_sha256",
            "run_config_sha256",
        )
    ):
        raise TransFuserPPContractError("run_context formal identity is incomplete")

    result = deepcopy(dict(observation))
    result["rgb"] = {"camera_front": front}
    result["lidar"] = lidar
    result["calibration"] = deepcopy(dict(calibration))
    result["route"] = dict(route)
    result["route"]["target_point_ego_m"] = [float(target[0]), float(target[1])]
    return result


def validate_intermediate_record(record: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(record, Mapping) or record.get("schema_version") != "transfuserpp_intermediate_frame.v1":
        raise TransFuserPPContractError("unsupported TransFuser++ intermediate frame schema")
    if record.get("algorithm_id") != ALGORITHM_ID:
        raise TransFuserPPContractError("intermediate algorithm_id mismatch")
    if record.get("algorithm_version") != ALGORITHM_VERSION:
        raise TransFuserPPContractError("intermediate algorithm_version mismatch")
    if (
        isinstance(record.get("frame_id"), bool)
        or not isinstance(record.get("frame_id"), int)
        or int(record["frame_id"]) < 0
    ):
        raise TransFuserPPContractError("intermediate frame_id is required")
    if not _finite_nonnegative(record.get("timestamp")):
        raise TransFuserPPContractError("intermediate timestamp is invalid")
    identity = record.get("identity")
    if not isinstance(identity, Mapping):
        raise TransFuserPPContractError("intermediate identity is required")
    for name in (
        "repo_sha256",
        "checkpoint_sha256",
        "model_config_sha256",
        "runtime_config_sha256",
        "carla_agents_sha256",
        "adapter_source_sha256",
    ):
        if not SHA256_RE.fullmatch(str(identity.get(name) or "")):
            raise TransFuserPPContractError(f"intermediate {name} is invalid")
    if not GIT_REVISION_RE.fullmatch(str(identity.get("repo_revision") or "")):
        raise TransFuserPPContractError("intermediate repo_revision is invalid")
    if re.fullmatch(
        r"sha256:[0-9a-f]{64}",
        str(identity.get("container_image_digest") or ""),
    ) is None:
        raise TransFuserPPContractError(
            "intermediate container_image_digest is invalid"
        )
    experiment_problems: list[str] = []
    _validate_experiment_identity(record.get("experiment"), experiment_problems)
    if experiment_problems:
        raise TransFuserPPContractError(
            "intermediate experiment identity invalid: " + ", ".join(experiment_problems)
        )
    if not re.fullmatch(
        r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}",
        str((record.get("experiment") or {}).get("run_id") or ""),
    ):
        raise TransFuserPPContractError("intermediate experiment run_id is invalid")
    provenance = record.get("provenance")
    if not isinstance(provenance, Mapping) or provenance.get("real_checkpoint_loaded") is not True:
        raise TransFuserPPContractError("intermediate real checkpoint provenance is required")
    if provenance.get("execution_mode") != "remote_model_inference":
        raise TransFuserPPContractError("intermediate execution mode is invalid")
    inputs = record.get("inputs")
    if not isinstance(inputs, Mapping):
        raise TransFuserPPContractError("intermediate input provenance is required")
    intermediate_camera = _validate_payload_ref(
        inputs.get("camera_front"), "intermediate.camera_front"
    )
    intermediate_lidar = _validate_payload_ref(
        inputs.get("lidar_top"), "intermediate.lidar_top"
    )
    if (
        intermediate_camera.get("encoding") != "jpeg"
        or intermediate_camera.get("coordinate_frame") != "camera_optical"
    ):
        raise TransFuserPPContractError("intermediate camera payload semantics are invalid")
    if (
        intermediate_lidar.get("encoding") != "float32_xyzi_little_endian"
        or intermediate_lidar.get("coordinate_frame") not in {"sensor_local", "carla_ego"}
    ):
        raise TransFuserPPContractError("intermediate LiDAR payload semantics are invalid")
    calibration = inputs.get("calibration")
    if not isinstance(calibration, Mapping):
        raise TransFuserPPContractError("intermediate calibration is required")
    if calibration.get("camera_sensor_id") != "camera_front":
        raise TransFuserPPContractError("intermediate camera calibration is invalid")
    validate_camera_adaptation_contract(
        calibration.get("camera_adaptation"), "intermediate camera adaptation"
    )
    _rigid_matrix(
        calibration.get("camera_sensor_to_ego"),
        "intermediate.calibration.camera_sensor_to_ego",
    )
    if calibration.get("camera_sensor_to_ego_coordinate_frame") != (
        "carla_x_forward_y_right_z_up"
    ):
        raise TransFuserPPContractError(
            "intermediate camera calibration coordinate frame is invalid"
        )
    validate_camera_adaptation_evidence(
        inputs.get("camera_adaptation"),
        source_payload=intermediate_camera,
        calibration=calibration,
        label="intermediate camera adaptation evidence",
    )
    if calibration.get("lidar_sensor_id") != "lidar_top":
        raise TransFuserPPContractError("intermediate LiDAR calibration sensor ID is invalid")
    calibrated_lidar_matrix = _rigid_matrix(
        calibration.get("lidar_sensor_to_ego"),
        "intermediate.calibration.lidar_sensor_to_ego",
    )
    if intermediate_lidar.get("coordinate_frame") == "sensor_local":
        intermediate_lidar_matrix = _rigid_matrix(
            intermediate_lidar.get("sensor_to_ego"),
            "intermediate.lidar_top.sensor_to_ego",
        )
        if intermediate_lidar.get("axis_convention") != "carla_sensor":
            raise TransFuserPPContractError(
                "intermediate sensor-local LiDAR axis convention is invalid"
            )
        if intermediate_lidar_matrix != calibrated_lidar_matrix:
            raise TransFuserPPContractError(
                "intermediate LiDAR payload/calibration extrinsic mismatch"
            )
    if inputs.get("model_ego_coordinate_frame") != "carla_x_forward_y_right_z_up":
        raise TransFuserPPContractError("intermediate model ego coordinate frame is invalid")
    if inputs.get("ego_pose_coordinate_frame") != (
        "closedloopbench_scene_x_forward_y_left_z_up"
    ):
        raise TransFuserPPContractError("intermediate ego pose coordinate frame is invalid")
    ego_pose = inputs.get("ego_pose")
    if not isinstance(ego_pose, Mapping) or any(
        not _finite(ego_pose.get(name)) for name in ("x", "y", "yaw")
    ):
        raise TransFuserPPContractError("intermediate ego pose is invalid")
    outputs = record.get("outputs")
    if not isinstance(outputs, Mapping):
        raise TransFuserPPContractError("intermediate outputs are required")
    for name in ("waypoints_ego_m", "route_checkpoints_ego_m"):
        points = outputs.get(name)
        if not isinstance(points, list) or not points or any(
            not isinstance(point, list)
            or len(point) != 2
            or any(not _finite(value) for value in point)
            for point in points
        ):
            raise TransFuserPPContractError(f"intermediate {name} is invalid")
    if not _finite_nonnegative(outputs.get("target_speed_mps")):
        raise TransFuserPPContractError("intermediate target_speed_mps is invalid")
    probabilities = outputs.get("target_speed_probabilities")
    bins = outputs.get("target_speed_bins_mps")
    if (
        not isinstance(probabilities, list)
        or not isinstance(bins, list)
        or not probabilities
        or len(probabilities) != len(bins)
        or any(not _finite_nonnegative(value) for value in probabilities + bins)
        or abs(sum(float(value) for value in probabilities) - 1.0) > 1e-4
    ):
        raise TransFuserPPContractError("intermediate target-speed distribution is invalid")
    selection_mode = outputs.get("target_speed_selection_mode")
    selected_index = outputs.get("target_speed_selected_index")
    threshold = outputs.get("target_speed_brake_uncertainty_threshold")
    if not _finite(threshold) or not 0.0 <= float(threshold) <= 1.0:
        raise TransFuserPPContractError(
            "intermediate target-speed uncertainty threshold is invalid"
        )
    if selection_mode == "weighted_expectation":
        expected_speed = sum(
            float(probability) * float(speed)
            for probability, speed in zip(probabilities, bins)
        )
        if selected_index is not None or float(probabilities[0]) > float(threshold):
            raise TransFuserPPContractError(
                "weighted target-speed selection metadata is inconsistent"
            )
    elif selection_mode == "argmax":
        expected_index = max(
            range(len(probabilities)),
            key=lambda index: float(probabilities[index]),
        )
        if selected_index != expected_index:
            raise TransFuserPPContractError(
                "argmax target-speed selected index is inconsistent"
            )
        expected_speed = float(bins[expected_index])
    elif selection_mode == "brake_uncertainty_override":
        if selected_index != 0 or float(probabilities[0]) <= float(threshold):
            raise TransFuserPPContractError(
                "brake uncertainty override metadata is inconsistent"
            )
        expected_speed = float(bins[0])
    else:
        raise TransFuserPPContractError(
            "intermediate target-speed selection mode is invalid"
        )
    if abs(float(outputs["target_speed_mps"]) - expected_speed) > 1e-4:
        raise TransFuserPPContractError(
            "intermediate target_speed_mps does not match selection mode"
        )
    boxes = outputs.get("bounding_boxes_ego")
    if not isinstance(boxes, list) or any(
        not isinstance(box, list)
        or len(box) < 9
        or any(not _finite(value) for value in box)
        for box in boxes
    ):
        raise TransFuserPPContractError("intermediate bounding_boxes_ego is invalid")
    control = outputs.get("control")
    if not isinstance(control, Mapping):
        raise TransFuserPPContractError("intermediate control is required")
    for name, lower, upper in (
        ("throttle", 0.0, 1.0),
        ("steer", -1.0, 1.0),
        ("brake", 0.0, 1.0),
    ):
        if not _finite(control.get(name)) or not lower <= float(control[name]) <= upper:
            raise TransFuserPPContractError(f"intermediate control {name} is invalid")
    latency = record.get("latency_ms")
    if not isinstance(latency, Mapping) or not _finite_nonnegative(latency.get("inference")):
        raise TransFuserPPContractError("intermediate inference latency is invalid")
    semantics = record.get("semantics")
    if not isinstance(semantics, Mapping):
        raise TransFuserPPContractError("intermediate semantics declaration is required")
    if semantics.get("occupancy_evaluation") != "dynamic_bev_proxy_only":
        raise TransFuserPPContractError("full 3D occupancy must not be implied")
    if semantics.get("full_3d_occupancy_ground_truth_available") is not False:
        raise TransFuserPPContractError("full 3D occupancy availability must fail closed")
    synchronization = record.get("synchronization")
    if (
        not isinstance(synchronization, Mapping)
        or synchronization.get("frame_id") != record.get("frame_id")
        or not _finite_nonnegative(synchronization.get("error_ms"))
    ):
        raise TransFuserPPContractError("intermediate synchronization is invalid")
    dense = record.get("dense_outputs")
    if not isinstance(dense, Mapping):
        raise TransFuserPPContractError("intermediate dense output reference is required")
    if dense.get("encoding") != "numpy_npz" or not SHA256_RE.fullmatch(
        str(dense.get("sha256") or "")
    ):
        raise TransFuserPPContractError("intermediate dense output identity is invalid")
    if list(dense.get("required_keys") or []) != list(REQUIRED_DENSE_KEYS):
        raise TransFuserPPContractError("intermediate dense output keys are incomplete")
    proxy = record.get("dynamic_bev_proxy")
    grid = (proxy or {}).get("grid") if isinstance(proxy, Mapping) else None
    if not isinstance(grid, Mapping):
        raise TransFuserPPContractError("dynamic BEV grid declaration is required")
    if grid.get("row_axis") != "ego_x_forward" or grid.get("column_axis") != "ego_y_right":
        raise TransFuserPPContractError("dynamic BEV grid axes are invalid")
    if (proxy.get("class_mapping") or {}) != {"vehicle": 9, "pedestrian": 10}:
        raise TransFuserPPContractError("dynamic BEV class mapping is invalid")
    return deepcopy(dict(record))


def camera_center_crop_window(
    source_width: int,
    source_height: int,
    target_width: int,
    target_height: int,
) -> list[int]:
    """Return a deterministic center crop before resizing to the model sensor size."""

    if min(source_width, source_height, target_width, target_height) <= 0:
        raise TransFuserPPContractError("camera dimensions must be positive")
    target_aspect = target_width / target_height
    source_aspect = source_width / source_height
    if source_aspect > target_aspect:
        crop_width = max(1, int(round(source_height * target_aspect)))
        left = (source_width - crop_width) // 2
        return [left, 0, left + crop_width, source_height]
    crop_height = max(1, int(round(source_width / target_aspect)))
    top = (source_height - crop_height) // 2
    return [0, top, source_width, top + crop_height]


def route_command_index(command: str) -> int:
    try:
        return SUPPORTED_ROUTE_COMMANDS[str(command)]
    except KeyError as exc:
        raise TransFuserPPContractError(f"unsupported route_command: {command}") from exc


def repository_snapshot_sha256(repo: str | Path) -> str | None:
    """Hash tracked paths and bytes without depending on the checkout location."""

    root = Path(repo).resolve()
    if not (root / ".git").exists():
        return None
    try:
        result = subprocess.run(
            ["git", "-C", str(root), "ls-files", "-z"],
            check=True,
            capture_output=True,
            timeout=30.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    digest = hashlib.sha256()
    for encoded in sorted(item for item in result.stdout.split(b"\0") if item):
        relative = encoded.decode("utf-8", errors="strict")
        path = root / relative
        if not path.is_file():
            return None
        digest.update(encoded)
        digest.update(b"\0")
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        digest.update(b"\0")
    return digest.hexdigest()


def directory_snapshot_sha256(root: str | Path) -> str | None:
    """Hash Python source paths/bytes for a mounted CARLA agents package."""

    directory = Path(root).resolve()
    if not directory.is_dir():
        return None
    paths = sorted(
        path
        for path in directory.rglob("*.py")
        if path.is_file() and "__pycache__" not in path.parts
    )
    if not paths:
        return None
    digest = hashlib.sha256()
    for path in paths:
        relative = path.relative_to(directory).as_posix().encode("utf-8")
        digest.update(relative)
        digest.update(b"\0")
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        digest.update(b"\0")
    return digest.hexdigest()


def adapter_source_snapshot_sha256() -> str | None:
    project_root = Path(__file__).resolve().parents[1]
    digest = hashlib.sha256()
    for relative in ADAPTER_SOURCE_FILES:
        path = project_root / relative
        if not path.is_file():
            return None
        encoded = relative.encode("utf-8")
        digest.update(encoded)
        digest.update(b"\0")
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        digest.update(b"\0")
    return digest.hexdigest()


def _validate_payload_ref(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TransFuserPPContractError(f"{name} payload reference is required")
    path = str(value.get("path") or "")
    digest = str(value.get("sha256") or "")
    if not path:
        raise TransFuserPPContractError(f"{name} path is required")
    if not SHA256_RE.fullmatch(digest):
        raise TransFuserPPContractError(f"{name} sha256 is invalid")
    byte_count = value.get("byte_count")
    if isinstance(byte_count, bool) or not isinstance(byte_count, int) or byte_count <= 0:
        raise TransFuserPPContractError(f"{name} byte_count must be a positive integer")
    result = deepcopy(dict(value))
    result["path"] = path
    result["sha256"] = digest
    return result


def _validate_declared_hash(
    value: Any, name: str, problems: list[str], *, allow_content_only: bool = False
) -> None:
    if not SHA256_RE.fullmatch(str(value or "")):
        problems.append(f"{name}_invalid")
    if allow_content_only:
        return


def _identity_config(config: Mapping[str, Any]) -> dict[str, Any]:
    injected_transport_keys = {
        "plugin",
        "shared_data_path",
        "runtime_config_path",
        "carla_host",
        "carla_port",
        "control_topic",
        "observation_topic",
        "ros_domain_id",
    }
    return {
        key: value
        for key, value in dict(config).items()
        if key not in {"intermediate_output_dir", *injected_transport_keys}
        and not callable(value)
    }


def _validate_experiment_identity(value: Any, problems: list[str]) -> None:
    if not isinstance(value, Mapping):
        problems.append("experiment_identity_missing")
        return
    if not re.fullmatch(r"^[0-9a-f]{32}$", str(value.get("scene_id") or "")):
        problems.append("experiment_scene_id_invalid")
    if not str(value.get("scene_version") or ""):
        problems.append("experiment_scene_version_missing")
    if value.get("case_id") not in FOCUSED_CASES:
        problems.append("experiment_case_id_invalid")
    seed = value.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        problems.append("experiment_seed_invalid")
    for name in (
        "artifact_sha256",
        "scene_package_sha256",
        "scenario_ir_sha256",
        "immutable_matrix_sha256",
        "source_run_config_sha256",
        "variant_config_sha256",
        "run_config_sha256",
    ):
        if not SHA256_RE.fullmatch(str(value.get(name) or "")):
            problems.append(f"experiment_{name}_invalid")


def _git_revision(repo: Path) -> str | None:
    if not (repo / ".git").exists():
        return None
    try:
        result = subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
        revision = result.stdout.strip().lower()
        return revision if GIT_REVISION_RE.fullmatch(revision) else None
    except (OSError, subprocess.SubprocessError):
        return None


def _git_origin(repo: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo), "remote", "get-url", "origin"],
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    value = result.stdout.strip().replace("git@github.com:", "https://github.com/")
    if value.endswith(".git"):
        value = value[:-4]
    return value or None


def _git_reference_commit(repo: Path, reference: str) -> str | None:
    if reference != UPSTREAM_REFERENCE:
        return None
    try:
        result = subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "--verify", f"{reference}^{{commit}}"],
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    commit = result.stdout.strip().lower()
    return commit if GIT_REVISION_RE.fullmatch(commit) else None


def _git_is_ancestor(repo: Path, revision: str, reference_commit: str) -> bool | None:
    if not GIT_REVISION_RE.fullmatch(str(revision)) or not GIT_REVISION_RE.fullmatch(
        str(reference_commit)
    ):
        return None
    try:
        result = subprocess.run(
            ["git", "-C", str(repo), "merge-base", "--is-ancestor", revision, reference_commit],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode == 0:
        return True
    if result.returncode == 1:
        return False
    return None


def cuda_runtime_identity(config: Mapping[str, Any]) -> dict[str, Any]:
    identity = {
        "algorithm_id": ALGORITHM_ID,
        "repo_revision": config.get("repo_revision"),
        "repo_sha256": config.get("repo_sha256"),
        "upstream_reference": config.get("upstream_reference"),
        "checkpoint_sha256": config.get("checkpoint_sha256"),
        "model_config_sha256": config.get("model_config_sha256"),
        "carla_agents_sha256": config.get("carla_agents_sha256"),
        "container_image_digest": config.get("container_image_digest"),
        "device": config.get("device", "cuda:0"),
        "cuda_gate": deepcopy(dict(config.get("cuda_gate") or {})),
    }
    return identity | {"canonical_sha256": canonical_sha256(identity)}


def _git_worktree_clean(repo: Path) -> bool | None:
    try:
        result = subprocess.run(
            [
                "git",
                "-C",
                str(repo),
                "status",
                "--porcelain=v1",
                "--untracked-files=all",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=30.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return not bool(result.stdout.strip())


def _rigid_matrix(value: Any, name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 16 or any(not _finite(item) for item in value):
        raise TransFuserPPContractError(f"{name} must be a finite row-major 4x4 matrix")
    matrix = [float(item) for item in value]
    if any(abs(matrix[index]) > 1e-6 for index in (12, 13, 14)) or abs(matrix[15] - 1.0) > 1e-6:
        raise TransFuserPPContractError(f"{name} must be a homogeneous rigid transform")
    rotation = [matrix[0:3], matrix[4:7], matrix[8:11]]
    for left_index, left in enumerate(rotation):
        for right_index, right in enumerate(rotation):
            dot = sum(a * b for a, b in zip(left, right))
            expected = 1.0 if left_index == right_index else 0.0
            if abs(dot - expected) > 1e-4:
                raise TransFuserPPContractError(f"{name} rotation is not orthonormal")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > 1e-4:
        raise TransFuserPPContractError(f"{name} rotation determinant must be +1")
    return matrix


def _finite(value: Any) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value))


def _finite_nonnegative(value: Any) -> bool:
    return _finite(value) and float(value) >= 0.0
