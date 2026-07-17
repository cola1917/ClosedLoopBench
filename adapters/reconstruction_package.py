from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path
from typing import Any


class ReconstructionPackageError(ValueError):
    """Raised when a Reconstruction Package is unsafe or inconsistent."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_reconstruction_package(path: Path, *, expected_scene_id: str) -> dict[str, Any]:
    path = Path(path).resolve()
    package = json.loads(path.read_text(encoding="utf-8"))
    if package.get("schema_version") != "reconstruction_package.v1":
        raise ReconstructionPackageError("unsupported reconstruction package version")
    if package.get("scene_id") != expected_scene_id:
        raise ReconstructionPackageError("reconstruction package scene token mismatch")
    source = package.get("source") or {}
    if source.get("scene_token") != expected_scene_id:
        raise ReconstructionPackageError("reconstruction source.scene_token mismatch")
    artifacts = package.get("artifacts")
    if not isinstance(artifacts, list):
        raise ReconstructionPackageError("reconstruction artifacts must be an array")
    resolved: dict[str, Path] = {}
    for item in artifacts:
        role = item.get("role")
        relative = item.get("path")
        if not isinstance(role, str) or not isinstance(relative, str):
            raise ReconstructionPackageError("reconstruction artifact role/path is invalid")
        candidate = Path(relative)
        if candidate.is_absolute() or candidate.drive or ".." in candidate.parts:
            raise ReconstructionPackageError("reconstruction artifact path is unsafe")
        artifact = (path.parent / candidate).resolve()
        try:
            artifact.relative_to(path.parent)
        except ValueError as exc:
            raise ReconstructionPackageError("reconstruction artifact escapes package root") from exc
        if not artifact.is_file():
            raise ReconstructionPackageError(f"reconstruction artifact is missing: {relative}")
        if artifact.stat().st_size != item.get("size_bytes"):
            raise ReconstructionPackageError(f"reconstruction artifact size mismatch: {relative}")
        if _sha256(artifact) != item.get("sha256"):
            raise ReconstructionPackageError(f"reconstruction artifact digest mismatch: {relative}")
        if role in resolved:
            raise ReconstructionPackageError(f"duplicate reconstruction artifact role: {role}")
        resolved[role] = artifact
    package["_resolved_artifacts"] = resolved
    package["_package_path"] = path
    return package


def load_reconstruction_result(
    path: Path,
    *,
    exchange_root: Path,
    expected_scene_id: str,
) -> dict[str, Any]:
    result = json.loads(Path(path).read_text(encoding="utf-8"))
    if result.get("schema_version") != "reconstruction_result.v1":
        raise ReconstructionPackageError("unsupported reconstruction result version")
    payload = result.get("payload") or {}
    if payload.get("scene_id") != expected_scene_id:
        raise ReconstructionPackageError("reconstruction result scene token mismatch")
    if payload.get("status") not in {"succeeded", "partial"}:
        raise ReconstructionPackageError("reconstruction result is not consumable")
    reference = next(
        (
            item
            for item in payload.get("artifacts", [])
            if item.get("role") == "reconstruction_package"
        ),
        None,
    )
    if reference is None:
        raise ReconstructionPackageError("reconstruction result has no Reconstruction Package")
    relative = Path(str(reference.get("path", "")))
    if relative.is_absolute() or relative.drive or ".." in relative.parts:
        raise ReconstructionPackageError("reconstruction package reference is unsafe")
    root = Path(exchange_root).resolve()
    package_path = (root / relative).resolve()
    try:
        package_path.relative_to(root)
    except ValueError as exc:
        raise ReconstructionPackageError("reconstruction package escapes exchange root") from exc
    if not package_path.is_file():
        raise ReconstructionPackageError("referenced Reconstruction Package does not exist")
    if package_path.stat().st_size != reference.get("size_bytes"):
        raise ReconstructionPackageError("Reconstruction Package size mismatch")
    if _sha256(package_path) != reference.get("sha256"):
        raise ReconstructionPackageError("Reconstruction Package digest mismatch")
    return load_reconstruction_package(package_path, expected_scene_id=expected_scene_id)


def materialize_reconstruction_package(
    package: dict[str, Any],
    destination: Path,
) -> dict[str, str]:
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    source_path = Path(package["_package_path"])
    copied: dict[str, str] = {}
    for item in package["artifacts"]:
        source = package["_resolved_artifacts"][item["role"]]
        relative = Path(item["path"])
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if source.resolve() != target.resolve():
            shutil.copy2(source, target)
        copied[item["role"]] = target.relative_to(destination).as_posix()
    target_package = destination / "reconstruction_package.json"
    public_package = {key: value for key, value in package.items() if not key.startswith("_")}
    target_package.write_text(
        json.dumps(public_package, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    copied["reconstruction_package"] = target_package.relative_to(destination).as_posix()
    return copied
