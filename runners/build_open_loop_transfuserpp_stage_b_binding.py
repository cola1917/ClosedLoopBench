"""Bind the real TransFuser++ runtime to the M6 NuRec open-loop inputs.

M6 deliberately has no CARLA dynamic-actor or control-owned pose binding.  The
Scenario IR exchange pin owns ego poses, NuRec owns the static render, and the
small input bundle below records the formal Scene Package, sensor calibration,
and LiDAR response-axis conversion used by the capture runner.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agents.plugin_contract import canonical_sha256, strict_json_loads
from agents.transfuserpp_contract import (
    cuda_runtime_identity,
    directory_snapshot_sha256,
    repository_snapshot_sha256,
)
from runners.run_open_loop_gt_replay import load_pinned_inputs
from runtime.scene0061_lidar_axis_normalization import validate_lidar_axis_normalization


UPSTREAM_REFERENCE = "refs/remotes/origin/leaderboard_2"
UPSTREAM_REPOSITORY = "https://github.com/autonomousvision/carla_garage"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
IMAGE_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
REVISION_RE = re.compile(r"^[0-9a-f]{40}$")

DEFAULT_NORMALIZATION = {
    "schema_version": "nre_lidar_axis_normalization.v1",
    "source_coordinate_frame": "nre_26_04_lidar_sensor",
    "source_axis_convention": "nre_26_04_render_axes",
    "target_coordinate_frame": "sensor_local",
    "target_axis_convention": "carla_sensor",
    "response_to_sensor": [
        0.0,
        0.0,
        -1.0,
        0.0,
        0.0,
        -1.0,
        0.0,
        0.0,
        -1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ],
}


class StageBBindingError(ValueError):
    """Raised when an M6 runtime cannot be bound to immutable inputs."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git(repo: Path, *args: str) -> str:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo), *args],
            check=True,
            capture_output=True,
            text=True,
            timeout=30.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise StageBBindingError(f"git command failed in {repo}: {args}") from exc
    return result.stdout.strip()


def _require_file(path: Path, label: str) -> Path:
    target = path.expanduser().resolve()
    if not target.is_file():
        raise StageBBindingError(f"{label} is unavailable: {target}")
    return target


def _require_hash(value: str, label: str, pattern: re.Pattern[str] = SHA256_RE) -> str:
    if pattern.fullmatch(value) is None:
        raise StageBBindingError(f"{label} is not a valid pinned hash")
    return value


def _copy_input(source: Path, destination: Path, label: str) -> Path:
    source = _require_file(source, label)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise StageBBindingError(f"refusing to overwrite M6 input: {destination}")
    shutil.copyfile(source, destination)
    if _sha256(source) != _sha256(destination):
        raise StageBBindingError(f"copied {label} changed its SHA-256")
    return destination


def _load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = strict_json_loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise StageBBindingError(f"{label} is not valid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise StageBBindingError(f"{label} must be a JSON object: {path}")
    return value


def _sensor_specs(value: Mapping[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    cameras = value.get("camera_specs")
    lidars = value.get("lidar_specs")
    if not isinstance(cameras, list) or not isinstance(lidars, list):
        raise StageBBindingError("M6 sensor spec bundle must contain camera_specs and lidar_specs")
    camera_rows = [dict(item) for item in cameras if isinstance(item, Mapping)]
    lidar_rows = [dict(item) for item in lidars if isinstance(item, Mapping)]
    camera_ids = sorted(str(item.get("sensor_id") or "") for item in camera_rows)
    expected = sorted(
        (
            "camera_front",
            "camera_front_left",
            "camera_front_right",
            "camera_back",
            "camera_back_left",
            "camera_back_right",
        )
    )
    if camera_ids != expected:
        raise StageBBindingError("M6 sensor bundle does not contain exactly the six formal cameras")
    if len(lidar_rows) != 1 or lidar_rows[0].get("sensor_id") != "lidar_top":
        raise StageBBindingError("M6 sensor bundle must contain exactly one lidar_top")
    for item in camera_rows + lidar_rows:
        matrix = item.get("sensor_to_ego")
        if not isinstance(matrix, list) or len(matrix) != 16:
            raise StageBBindingError(f"sensor {item.get('sensor_id')} lacks a 4x4 sensor_to_ego")
    for item in camera_rows:
        if int(item.get("width", 0)) != 1600 or int(item.get("height", 0)) != 900:
            raise StageBBindingError(f"camera {item.get('sensor_id')} is not 1600x900")
    if str(lidar_rows[0].get("device_type") or lidar_rows[0].get("model")) != "PANDAR128":
        raise StageBBindingError("M6 lidar_top must use the verified PANDAR128 device type")
    return camera_rows, lidar_rows


def build_binding(
    *,
    runtime_template: Path,
    scenario_ir: Path,
    opendrive: Path,
    repo_host_path: Path,
    checkpoint_host_path: Path,
    model_config_host_path: Path,
    carla_agents_host_path: Path,
    artifact_path: Path,
    scene_package_path: Path,
    sensor_specs_path: Path,
    output: Path,
    input_bundle_dir: Path,
    image_digest: str,
    repo_revision: str,
    case_id: str,
    seed: int,
    run_id_suffix: str | None = None,
    nurec_target: str = "127.0.0.1:46443",
    nurec_health_target: str = "127.0.0.1:46444",
    nurec_scene_id: str = "scene-0061",
    nurec_scene_start_us: int = 1532402927598150,
    nurec_python_api_path: str = "/home/cwadmin/sim-env/data/CARLA_0.9.16/PythonAPI/examples/nvidia/nurec",
    container_data_root: str = "/sim-data",
) -> dict[str, Any]:
    if output.exists():
        raise StageBBindingError(f"refusing to overwrite output: {output}")
    if input_bundle_dir.exists():
        raise StageBBindingError(f"refusing to overwrite M6 input bundle: {input_bundle_dir}")
    if case_id not in {"S0_original_replay", "S2_lead_hard_brake", "S4_pedestrian_early_crossing"}:
        raise StageBBindingError(f"unsupported open-loop case: {case_id}")
    if isinstance(seed, bool) or seed < 0:
        raise StageBBindingError("seed must be a non-negative integer")
    if run_id_suffix is not None and re.fullmatch(
        r"[A-Za-z0-9][A-Za-z0-9_.-]{0,31}", run_id_suffix
    ) is None:
        raise StageBBindingError("run_id_suffix is unsafe")
    _require_hash(image_digest, "container image digest", IMAGE_RE)
    _require_hash(repo_revision, "repo revision", REVISION_RE)
    if isinstance(nurec_scene_start_us, bool) or int(nurec_scene_start_us) < 0:
        raise StageBBindingError("nurec_scene_start_us must be non-negative")

    template = _load_object(runtime_template.expanduser().resolve(), "runtime template")
    inputs = load_pinned_inputs(scenario_ir, opendrive)
    repo = repo_host_path.expanduser().resolve()
    checkpoint = _require_file(checkpoint_host_path, "checkpoint")
    model_config = _require_file(model_config_host_path, "model config")
    artifact = _require_file(artifact_path, "NuRec USDZ artifact")
    source_package = _require_file(scene_package_path, "formal NuRec Scene Package")
    source_specs = _require_file(sensor_specs_path, "NuRec sensor specs")
    specs_value = _load_object(source_specs, "NuRec sensor specs")
    camera_specs, lidar_specs = _sensor_specs(specs_value)
    package_value = _load_object(source_package, "formal NuRec Scene Package")
    if package_value.get("scene_id") != inputs.scenario_ir["scenario_id"]:
        raise StageBBindingError("NuRec Scene Package scene_id does not match Scenario IR")
    if (package_value.get("alignment") or {}).get("status") != "runtime_validated":
        raise StageBBindingError("M6 requires a runtime_validated NuRec Scene Package")
    normalization = validate_lidar_axis_normalization(DEFAULT_NORMALIZATION)

    agents = carla_agents_host_path.expanduser().resolve()
    if not repo.is_dir() or not (repo / ".git").exists():
        raise StageBBindingError(f"CARLA Garage checkout is unavailable: {repo}")
    if not agents.is_dir():
        raise StageBBindingError(f"CARLA agents source is unavailable: {agents}")
    detected_revision = _git(repo, "rev-parse", "HEAD")
    if detected_revision != repo_revision:
        raise StageBBindingError(
            f"repo revision mismatch: expected {repo_revision}, detected {detected_revision}"
        )
    origin = _git(repo, "remote", "get-url", "origin").replace("git@github.com:", "https://github.com/")
    if origin.endswith(".git"):
        origin = origin[:-4]
    if origin != UPSTREAM_REPOSITORY:
        raise StageBBindingError(f"unexpected CARLA Garage origin: {origin}")
    repo_hash = repository_snapshot_sha256(repo)
    agents_hash = directory_snapshot_sha256(agents)
    if repo_hash is None or agents_hash is None:
        raise StageBBindingError("repo or CARLA agents snapshot is incomplete")

    bundle_package = _copy_input(
        source_package,
        input_bundle_dir / "scene_package.runtime-validated.nurec-usdz.formal40k.json",
        "formal NuRec Scene Package",
    )
    bundle_specs = _copy_input(
        source_specs,
        input_bundle_dir / "nurec_sensor_specs.scene0061.v1.json",
        "NuRec sensor specs",
    )
    normalization_path = input_bundle_dir / "lidar_axis_normalization.nre-to-carla.v1.json"
    normalization_path.parent.mkdir(parents=True, exist_ok=True)
    normalization_path.write_text(
        json.dumps(normalization, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    normalization_hash = _sha256(normalization_path)

    package_hash = _sha256(bundle_package)
    specs_hash = _sha256(bundle_specs)
    artifact_hash = _sha256(artifact)
    input_descriptor = {
        "schema_version": "open_loop_transfuserpp_m6_input_set.v1",
        "scene_id": inputs.scenario_ir["scenario_id"],
        "scene_name": "scene-0061",
        "scenario_ir_sha256": inputs.scenario_ir_sha256,
        "opendrive_sha256": inputs.opendrive_sha256,
        "nurec_artifact_sha256": artifact_hash,
        "scene_package_sha256": package_hash,
        "sensor_specs_sha256": specs_hash,
        "lidar_normalization_sha256": normalization_hash,
        "ego_pose_source": "scenario_ir_reference_trajectory",
        "dynamic_actor_creation": False,
        "control_affects_next_ego_pose": False,
        "matrix_actor_ready_ir_bound": False,
    }
    input_set_hash = canonical_sha256(input_descriptor)
    input_manifest_path = input_bundle_dir / "m6_open_loop_input_set.v1.json"
    input_manifest_path.write_text(
        json.dumps(
            {**input_descriptor, "input_set_sha256": input_set_hash},
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    source_descriptor = {
        "schema_version": "open_loop_transfuserpp_stage_b_source.v1",
        "input_set_sha256": input_set_hash,
        "scenario_ir_sha256": inputs.scenario_ir_sha256,
        "opendrive_sha256": inputs.opendrive_sha256,
        "artifact_sha256": artifact_hash,
        "scene_package_sha256": package_hash,
        "sensor_specs_sha256": specs_hash,
        "lidar_normalization_sha256": normalization_hash,
        "ego_pose_source": "scenario_ir_reference_trajectory",
        "sensor_source": "nurec_stage_b_6cam_rgb_lidar",
        "dynamic_actor_creation": False,
        "control_affects_next_ego_pose": False,
        "matrix_actor_ready_ir_bound": False,
    }
    source_hash = canonical_sha256(source_descriptor)
    variant_descriptor = {
        "schema_version": "open_loop_transfuserpp_stage_b_variant.v1",
        "source_descriptor_sha256": source_hash,
        "case_id": case_id,
        "seed": seed,
        "actor_mode": "none_static_nurec_scene",
        "sensor_source": "nurec_stage_b_6cam_rgb_lidar",
        "control_affects_next_ego_pose": False,
        "dynamic_actor_creation": False,
    }
    variant_hash = canonical_sha256(variant_descriptor)
    base_run_id = f"scene0061-tfpp-{case_id}-seed-{seed}-stage-b"
    run_id = f"{base_run_id}-{run_id_suffix}" if run_id_suffix else base_run_id
    experiment = {
        "scene_id": inputs.scenario_ir["scenario_id"],
        "scene_version": "open-loop-nurec-stage-b-v1",
        "case_id": case_id,
        "seed": seed,
        "artifact_sha256": artifact_hash,
        "scene_package_sha256": package_hash,
        "scenario_ir_sha256": inputs.scenario_ir_sha256,
        # The existing TF++ contract requires this field. For M6 it is bound to
        # the immutable open-loop input set, never to actor-ready matrix IR.
        "immutable_matrix_sha256": input_set_hash,
        "immutable_matrix_role": "open_loop_input_set_not_actor_ready_matrix",
        "source_run_config_sha256": source_hash,
        "variant_config_sha256": variant_hash,
    }
    try:
        shared_data_root = output.parent.parent.resolve()
        package_relative = bundle_package.resolve().relative_to(shared_data_root).as_posix()
        specs_relative = bundle_specs.resolve().relative_to(shared_data_root).as_posix()
    except ValueError as exc:
        raise StageBBindingError(
            "M6 input bundle must live beside the runtime directory under one shared data root"
        ) from exc
    nurec_runtime = {
        "python_api_path": nurec_python_api_path,
        "target": nurec_target,
        "health_target": nurec_health_target,
        "runtime_scene_id": nurec_scene_id,
        "scene_start_us": int(nurec_scene_start_us),
        "timeout_sec": 120.0,
        "scene_package": str(bundle_package.resolve()),
        "scene_package_container_path": f"{container_data_root}/{package_relative}",
        "sensor_specs_path": str(bundle_specs.resolve()),
        "sensor_specs_container_path": f"{container_data_root}/{specs_relative}",
        "camera_specs": camera_specs,
        "lidar_specs": lidar_specs,
        "lidar_response_coordinate_frame": normalization["target_coordinate_frame"],
        "lidar_axis_convention": normalization["target_axis_convention"],
        "lidar_axis_normalization": normalization,
        "dynamic_actor_creation": False,
        "dynamic_object_count": 0,
        "api": "SensorsimService/26.04",
    }
    config = copy.deepcopy(template)
    config.update(
        {
            "schema_version": "transfuserpp_runtime_config.v1",
            "algorithm_id": "transfuserpp_v5",
            "repo_path": "/opt/algorithm/carla_garage",
            "repo_revision": repo_revision,
            "upstream_reference": UPSTREAM_REFERENCE,
            "repo_sha256": repo_hash,
            "checkpoint_path": "/opt/algorithm/checkpoints/model_0030_0.pth",
            "checkpoint_sha256": _sha256(checkpoint),
            "model_config_path": "/opt/algorithm/checkpoints/config.json",
            "model_config_sha256": _sha256(model_config),
            "carla_agents_path": "/opt/carla-pythonapi/agents",
            "carla_agents_sha256": agents_hash,
            "container_image_digest": image_digest,
            "intermediate_output_dir": (
                f"{container_data_root}/transfuserpp_intermediates/{case_id}/seed_{seed}"
            ),
            "scene_id": inputs.scenario_ir["scenario_id"],
            "case_id": case_id,
            "seed": seed,
            "run_id": run_id,
            "run_attempt": run_id_suffix or "base",
            "experiment": experiment,
            "nurec_runtime": nurec_runtime,
            "open_loop": {
                "evidence_classification": "open_loop_multimodal",
                "scenario_ir_sha256": inputs.scenario_ir_sha256,
                "opendrive_sha256": inputs.opendrive_sha256,
                "control_affects_next_ego_pose": False,
                "claims_m8": False,
                "claims_m9": False,
                "real_carla_nurec_closed_loop": False,
                "real_carla_stage_b_open_loop": True,
                "sensor_source": "nurec_stage_b_6cam_rgb_lidar",
                "ego_pose_source": "scenario_ir_reference_trajectory",
                "dynamic_actor_creation": False,
                "dynamic_object_count": 0,
                "matrix_actor_ready_ir_bound": False,
                "input_set_sha256": input_set_hash,
                "scene_package_sha256": package_hash,
                "sensor_specs_sha256": specs_hash,
                "lidar_normalization_sha256": normalization_hash,
                "source_descriptor": source_descriptor,
                "variant_descriptor": variant_descriptor,
            },
            "real_checkpoint_loaded": False,
            "remote_gpu_validation_required": True,
        }
    )
    config.pop("config_identity", None)
    config.pop("algorithm_gpu_validation", None)
    config["experiment"].pop("run_config_sha256", None)
    config["algorithm_runtime_identity"] = cuda_runtime_identity(config)
    run_hash_payload = copy.deepcopy(config)
    run_hash = canonical_sha256(run_hash_payload)
    config["experiment"]["run_config_sha256"] = run_hash
    config_identity_payload = {
        key: value
        for key, value in config.items()
        if key not in {"config_identity", "algorithm_gpu_validation"}
    }
    config["config_identity"] = {
        "schema_version": "closedloopbench_run_config_identity.v1",
        "canonical_sha256": canonical_sha256(config_identity_payload),
        "hash_scope": (
            "whole_run_config_excluding_config_identity_and_algorithm_gpu_validation"
        ),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(config, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return {
        "status": "written",
        "output": str(output),
        "runtime_config_sha256": _sha256(output),
        "run_config_sha256": run_hash,
        "input_set_sha256": input_set_hash,
        "scene_package_sha256": package_hash,
        "sensor_specs_sha256": specs_hash,
        "lidar_normalization_sha256": normalization_hash,
        "artifact_sha256": artifact_hash,
        "repo_sha256": repo_hash,
        "checkpoint_sha256": config["checkpoint_sha256"],
        "model_config_sha256": config["model_config_sha256"],
        "carla_agents_sha256": agents_hash,
        "container_image_digest": image_digest,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-template", type=Path, required=True)
    parser.add_argument("--scenario-ir", type=Path, required=True)
    parser.add_argument("--opendrive", type=Path, required=True)
    parser.add_argument("--repo-host-path", type=Path, required=True)
    parser.add_argument("--checkpoint-host-path", type=Path, required=True)
    parser.add_argument("--model-config-host-path", type=Path, required=True)
    parser.add_argument("--carla-agents-host-path", type=Path, required=True)
    parser.add_argument("--artifact-path", type=Path, required=True)
    parser.add_argument("--scene-package-path", type=Path, required=True)
    parser.add_argument("--sensor-specs-path", type=Path, required=True)
    parser.add_argument("--input-bundle-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--image-digest", required=True)
    parser.add_argument("--repo-revision", required=True)
    parser.add_argument("--case-id", default="S0_original_replay")
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--run-id-suffix")
    parser.add_argument("--nurec-target", default="127.0.0.1:46443")
    parser.add_argument("--nurec-health-target", default="127.0.0.1:46444")
    parser.add_argument("--nurec-scene-id", default="scene-0061")
    parser.add_argument("--nurec-scene-start-us", type=int, default=1532402927598150)
    parser.add_argument(
        "--nurec-python-api-path",
        default="/home/cwadmin/sim-env/data/CARLA_0.9.16/PythonAPI/examples/nvidia/nurec",
    )
    args = parser.parse_args(argv)
    try:
        result = build_binding(
            runtime_template=args.runtime_template,
            scenario_ir=args.scenario_ir,
            opendrive=args.opendrive,
            repo_host_path=args.repo_host_path,
            checkpoint_host_path=args.checkpoint_host_path,
            model_config_host_path=args.model_config_host_path,
            carla_agents_host_path=args.carla_agents_host_path,
            artifact_path=args.artifact_path,
            scene_package_path=args.scene_package_path,
            sensor_specs_path=args.sensor_specs_path,
            input_bundle_dir=args.input_bundle_dir,
            output=args.output,
            image_digest=args.image_digest,
            repo_revision=args.repo_revision,
            case_id=args.case_id,
            seed=args.seed,
            run_id_suffix=args.run_id_suffix,
            nurec_target=args.nurec_target,
            nurec_health_target=args.nurec_health_target,
            nurec_scene_id=args.nurec_scene_id,
            nurec_scene_start_us=args.nurec_scene_start_us,
            nurec_python_api_path=args.nurec_python_api_path,
        )
    except (OSError, ValueError, json.JSONDecodeError, StageBBindingError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}))
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
