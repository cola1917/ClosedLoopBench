from __future__ import annotations

import json
import pickletools
import zipfile
from pathlib import Path
from typing import Any


class ReconstructionPlanningError(ValueError):
    """Raised when a reconstruction cannot enter a ClosedLoopBench plan."""


def build_reconstruction_integration_plan(
    scenario_ir: dict[str, Any],
    package: dict[str, Any],
    *,
    expected_camera_ids: tuple[str, ...] = (
        "camera_front",
        "camera_front_left",
        "camera_front_right",
    ),
    expected_global_step: int = 1000,
    expected_samples_per_epoch: int = 1000,
    expected_max_epochs: int = 1,
) -> dict[str, Any]:
    """Validate a NuRec training result and describe its runtime boundary.

    Package integrity and path containment are intentionally owned by
    ``load_reconstruction_package``. This function verifies the training gate
    that makes the current artifact eligible for downstream planning.
    """

    scene_id = str(scenario_ir.get("scenario_id") or "")
    source = scenario_ir.get("source") or {}
    if not scene_id or source.get("scene_token") != scene_id:
        raise ReconstructionPlanningError(
            "Scenario IR must use its nuScenes scene token as scenario_id"
        )
    if package.get("scene_id") != scene_id:
        raise ReconstructionPlanningError("reconstruction and Scenario IR scene identities differ")

    artifacts = package.get("_resolved_artifacts")
    if not isinstance(artifacts, dict):
        raise ReconstructionPlanningError(
            "package must be loaded with load_reconstruction_package first"
        )
    required_roles = ("nurec_usdz", "nurec_checkpoint", "nurec_config")
    missing = [role for role in required_roles if role not in artifacts]
    if missing:
        raise ReconstructionPlanningError(
            "reconstruction is missing required artifact roles: " + ", ".join(missing)
        )

    config = _load_yaml(Path(artifacts["nurec_config"]))
    checks = {
        "camera_ids": (list(expected_camera_ids), _as_cameras),
        "n_samples_per_epoch": (expected_samples_per_epoch, _as_int),
        "max_epochs": (expected_max_epochs, _as_int),
    }
    matched_config: dict[str, Any] = {}
    for key, (expected, normalize) in checks.items():
        candidates = _find_values(config, key)
        match = next(
            ((path, normalize(value)) for path, value in candidates if normalize(value) == expected),
            None,
        )
        if match is None:
            rendered = ", ".join(f"{path}={value!r}" for path, value in candidates) or "<not found>"
            raise ReconstructionPlanningError(
                f"NuRec config gate failed: expected {key}={expected!r}; found {rendered}"
            )
        matched_config[key] = {"path": match[0], "value": match[1]}

    checkpoint_step = _read_checkpoint_global_step(Path(artifacts["nurec_checkpoint"]))
    if checkpoint_step != expected_global_step:
        raise ReconstructionPlanningError(
            f"checkpoint gate failed: expected global_step={expected_global_step}, "
            f"found {checkpoint_step}"
        )

    inventory = {item["role"]: item for item in package["artifacts"]}
    return {
        "schema_version": "reconstruction_integration_plan.v0",
        "scene_id": scene_id,
        "source": {
            "dataset": source.get("dataset"),
            "scene_name": source.get("scene_name"),
            "scene_token": source.get("scene_token"),
        },
        "reconstruction": {
            "backend": package.get("backend"),
            "package_path": str(package["_package_path"]),
            "nurec_usdz": inventory["nurec_usdz"],
            "nurec_checkpoint": inventory["nurec_checkpoint"],
            "nurec_config": inventory["nurec_config"],
        },
        "validation": {
            "status": "passed",
            "artifact_integrity": "passed",
            "scene_identity": "passed",
            "training_gate": {
                "status": "passed",
                "global_step": checkpoint_step,
                "config": matched_config,
            },
        },
        "closed_loop_plan": {
            "motion_and_map": "build_closed_loop_scene_package",
            "visual_artifact": "nurec_usdz",
            "runtime_alignment": "requires_measured_landmarks",
            "carla_visual_binding": "not_implemented",
            "carla_visual_binding_reason": (
                "CARLA 0.9.16 cannot load a NuRec USDZ as a native map or sensor renderer; "
                "a renderer adapter and measured runtime alignment are still required"
            ),
        },
    }


def _load_yaml(path: Path) -> Any:
    text = path.read_text(encoding="utf-8")
    try:
        import yaml
    except ImportError as exc:
        try:
            result = json.loads(text)
        except json.JSONDecodeError:
            raise ReconstructionPlanningError(
                "PyYAML is required to validate a non-JSON NuRec parsed configuration"
            ) from exc
    else:
        result = yaml.safe_load(text)
    if not isinstance(result, dict):
        raise ReconstructionPlanningError("NuRec parsed config is not a mapping")
    return result


def _find_values(node: Any, key: str, path: str = "") -> list[tuple[str, Any]]:
    found: list[tuple[str, Any]] = []
    if isinstance(node, dict):
        for child_key, child in node.items():
            child_path = f"{path}.{child_key}" if path else str(child_key)
            if child_key == key:
                found.append((child_path, child))
            found.extend(_find_values(child, key, child_path))
    elif isinstance(node, (list, tuple)):
        for index, child in enumerate(node):
            found.extend(_find_values(child, key, f"{path}[{index}]"))
    return found


def _as_cameras(value: Any) -> list[str] | None:
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value]
    if isinstance(value, str):
        return [item.strip().strip("\"'") for item in value.strip("[]").split(",") if item.strip()]
    return None


def _as_int(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _read_checkpoint_global_step(path: Path) -> int:
    try:
        with zipfile.ZipFile(path) as archive:
            candidates = [
                name
                for name in archive.namelist()
                if name.endswith("/data.pkl") or name == "data.pkl"
            ]
            if len(candidates) != 1:
                raise ReconstructionPlanningError(
                    f"checkpoint gate failed: expected one data.pkl, found {candidates!r}"
                )
            payload = archive.read(candidates[0])
    except zipfile.BadZipFile as exc:
        raise ReconstructionPlanningError("checkpoint is not a supported PyTorch ZIP archive") from exc

    expecting_value = False
    integer_ops = {"BININT", "BININT1", "BININT2", "INT", "LONG", "LONG1", "LONG4"}
    memo_ops = {"BINPUT", "LONG_BINPUT", "MEMOIZE", "PUT"}
    string_ops = {"BINUNICODE", "BINUNICODE8", "SHORT_BINUNICODE", "UNICODE"}
    for opcode, argument, _ in pickletools.genops(payload):
        if expecting_value:
            if opcode.name in memo_ops:
                continue
            if opcode.name in integer_ops:
                return int(argument)
            raise ReconstructionPlanningError(
                f"checkpoint global_step has unsupported opcode {opcode.name}"
            )
        if opcode.name in string_ops and argument == "global_step":
            expecting_value = True
    raise ReconstructionPlanningError("checkpoint global_step was not found")
