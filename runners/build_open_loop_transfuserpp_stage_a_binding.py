"""Bind a real TransFuser++ Stage A runtime to the local open-loop pins.

This is intentionally separate from the formal scene-0061 matrix bundle
builder.  Stage A uses the pinned exchange-v2 IR and native CARLA OpenDRIVE
listed in ``docs/open_loop_multimodal_eval.md``; the resulting identity keeps
that boundary explicit and never promotes it to the formal actor-ready matrix.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import re
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


UPSTREAM_REFERENCE = "refs/remotes/origin/leaderboard_2"
UPSTREAM_REPOSITORY = "https://github.com/autonomousvision/carla_garage"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
IMAGE_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
REVISION_RE = re.compile(r"^[0-9a-f]{40}$")


class StageABindingError(ValueError):
    """Raised when an M5 runtime cannot be bound to real immutable inputs."""


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
        raise StageABindingError(f"git command failed in {repo}: {args}") from exc
    return result.stdout.strip()


def _require_file(path: Path, label: str) -> Path:
    target = path.expanduser().resolve()
    if not target.is_file():
        raise StageABindingError(f"{label} is unavailable: {target}")
    return target


def _require_hash(value: str, label: str, pattern: re.Pattern[str] = SHA256_RE) -> str:
    if pattern.fullmatch(value) is None:
        raise StageABindingError(f"{label} is not a lowercase SHA-256/image/revision")
    return value


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
    matrix_path: Path,
    output: Path,
    image_digest: str,
    repo_revision: str,
    case_id: str,
    seed: int,
    run_id_suffix: str | None = None,
    repo_container_path: str = "/opt/algorithm/carla_garage",
    checkpoint_container_path: str = "/opt/algorithm/checkpoints/model_0030_0.pth",
    model_config_container_path: str = "/opt/algorithm/checkpoints/config.json",
    agents_container_path: str = "/opt/carla-pythonapi/agents",
    container_data_root: str = "/sim-data",
) -> dict[str, Any]:
    if output.exists():
        raise StageABindingError(f"refusing to overwrite output: {output}")
    if case_id != "S0_original_replay":
        raise StageABindingError("M5 Stage A binding currently requires S0_original_replay")
    if isinstance(seed, bool) or seed < 0:
        raise StageABindingError("seed must be a non-negative integer")
    if run_id_suffix is not None and re.fullmatch(
        r"[A-Za-z0-9][A-Za-z0-9_.-]{0,31}", run_id_suffix
    ) is None:
        raise StageABindingError("run_id_suffix is unsafe")
    _require_hash(image_digest, "container image digest", IMAGE_RE)
    _require_hash(repo_revision, "repo revision", REVISION_RE)

    template = strict_json_loads(runtime_template.read_text(encoding="utf-8"))
    if not isinstance(template, dict):
        raise StageABindingError("runtime template must be a JSON object")
    inputs = load_pinned_inputs(scenario_ir, opendrive)
    repo = repo_host_path.expanduser().resolve()
    checkpoint = _require_file(checkpoint_host_path, "checkpoint")
    model_config = _require_file(model_config_host_path, "model config")
    artifact = _require_file(artifact_path, "scene artifact")
    scene_package = _require_file(scene_package_path, "scene package")
    matrix = _require_file(matrix_path, "counterfactual matrix")
    agents = carla_agents_host_path.expanduser().resolve()
    if not repo.is_dir() or not (repo / ".git").exists():
        raise StageABindingError(f"CARLA Garage checkout is unavailable: {repo}")
    if not agents.is_dir():
        raise StageABindingError(f"CARLA agents source is unavailable: {agents}")

    detected_revision = _git(repo, "rev-parse", "HEAD")
    if detected_revision != repo_revision:
        raise StageABindingError(
            f"repo revision mismatch: expected {repo_revision}, detected {detected_revision}"
        )
    repo_hash = repository_snapshot_sha256(repo)
    agents_hash = directory_snapshot_sha256(agents)
    if repo_hash is None or agents_hash is None:
        raise StageABindingError("repo or CARLA agents snapshot is incomplete")
    origin = _git(repo, "remote", "get-url", "origin").replace("git@github.com:", "https://github.com/")
    if origin.endswith(".git"):
        origin = origin[:-4]
    if origin != UPSTREAM_REPOSITORY:
        raise StageABindingError(f"unexpected CARLA Garage origin: {origin}")

    source_descriptor = {
        "schema_version": "open_loop_transfuserpp_stage_a_source.v1",
        "scenario_ir_sha256": inputs.scenario_ir_sha256,
        "opendrive_sha256": inputs.opendrive_sha256,
        "artifact_sha256": _sha256(artifact),
        "scene_package_sha256": _sha256(scene_package),
        "matrix_file_sha256": _sha256(matrix),
        "ego_pose_source": "scenario_ir_reference_trajectory",
        "control_affects_next_ego_pose": False,
        "matrix_actor_ready_ir_bound": False,
    }
    source_hash = canonical_sha256(source_descriptor)
    variant_descriptor = {
        "schema_version": "open_loop_transfuserpp_stage_a_variant.v1",
        "source_descriptor_sha256": source_hash,
        "case_id": case_id,
        "seed": seed,
        "actor_mode": "scenario_ir_replay",
        "control_affects_next_ego_pose": False,
        "sensor_source": "carla_stage_a_native_rgb_lidar",
    }
    variant_hash = canonical_sha256(variant_descriptor)
    experiment = {
        "scene_id": inputs.scenario_ir["scenario_id"],
        "scene_version": "open-loop-exchange-v1",
        "case_id": case_id,
        "seed": seed,
        "artifact_sha256": source_descriptor["artifact_sha256"],
        "scene_package_sha256": source_descriptor["scene_package_sha256"],
        "scenario_ir_sha256": inputs.scenario_ir_sha256,
        "immutable_matrix_sha256": source_descriptor["matrix_file_sha256"],
        "source_run_config_sha256": source_hash,
        "variant_config_sha256": variant_hash,
    }
    config = copy.deepcopy(template)
    base_run_id = f"scene0061-tfpp-{case_id}-seed-{seed}-stage-a"
    run_id = f"{base_run_id}-{run_id_suffix}" if run_id_suffix else base_run_id
    config.update(
        {
            "schema_version": "transfuserpp_runtime_config.v1",
            "algorithm_id": "transfuserpp_v5",
            "repo_path": repo_container_path,
            "repo_revision": repo_revision,
            "upstream_reference": UPSTREAM_REFERENCE,
            "repo_sha256": repo_hash,
            "checkpoint_path": checkpoint_container_path,
            "checkpoint_sha256": _sha256(checkpoint),
            "model_config_path": model_config_container_path,
            "model_config_sha256": _sha256(model_config),
            "carla_agents_path": agents_container_path,
            "carla_agents_sha256": agents_hash,
            "container_image_digest": image_digest,
            "intermediate_output_dir": (
                f"{container_data_root}/transfuserpp_intermediates/{case_id}/seed_{seed}"
            ),
            "scene_id": inputs.scenario_ir["scenario_id"],
            "case_id": case_id,
            "seed": seed,
            "run_id": run_id,
            "experiment": experiment,
            "open_loop": {
                "evidence_classification": "open_loop_multimodal",
                "scenario_ir_sha256": inputs.scenario_ir_sha256,
                "opendrive_sha256": inputs.opendrive_sha256,
                "control_affects_next_ego_pose": False,
                "claims_m8": False,
                "claims_m9": False,
                "sensor_source": "carla_stage_a_native_rgb_lidar",
                "waypoint_spacing_sec": 0.5,
                "matrix_actor_ready_ir_bound": False,
                "source_descriptor": source_descriptor,
                "variant_descriptor": variant_descriptor,
            },
        }
    )
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
        "repo_sha256": repo_hash,
        "checkpoint_sha256": config["checkpoint_sha256"],
        "model_config_sha256": config["model_config_sha256"],
        "carla_agents_sha256": agents_hash,
        "source_run_config_sha256": source_hash,
        "variant_config_sha256": variant_hash,
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
    parser.add_argument("--matrix-path", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--image-digest", required=True)
    parser.add_argument("--repo-revision", required=True)
    parser.add_argument("--case-id", default="S0_original_replay")
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--run-id-suffix")
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
            matrix_path=args.matrix_path,
            output=args.output,
            image_digest=args.image_digest,
            repo_revision=args.repo_revision,
            case_id=args.case_id,
            seed=args.seed,
            run_id_suffix=args.run_id_suffix,
        )
    except (OSError, ValueError, json.JSONDecodeError, StageABindingError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}))
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
