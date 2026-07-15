from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import uuid
from pathlib import Path
from typing import Any


PACKAGE_NAME = "scene_package.json"
READY_NAME = "READY.json"
READY_SCHEMA_VERSION = "closed_loop_scene_ready.v0"
_SAFE_SEGMENT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class SceneExchangeError(ValueError):
    """Raised when a scene exchange package is unsafe or incomplete."""


class SceneVersionExistsError(FileExistsError):
    """Raised when an immutable scene version has already been published."""


def _safe_segment(value: str, label: str) -> str:
    value = str(value)
    if not _SAFE_SEGMENT.fullmatch(value) or value in {".", ".."}:
        raise SceneExchangeError(f"invalid {label}: {value!r}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _artifact_inventory(version_dir: Path) -> dict[str, dict[str, Any]]:
    inventory: dict[str, dict[str, Any]] = {}
    for path in sorted(version_dir.rglob("*")):
        if path.is_symlink():
            raise SceneExchangeError(f"symbolic links are not allowed in scene packages: {path}")
        if not path.is_file() or path.name == READY_NAME:
            continue
        relative = path.relative_to(version_dir).as_posix()
        inventory[relative] = {"sha256": _sha256(path), "size_bytes": path.stat().st_size}
    return inventory


def _validate_schema_value(value: Any, schema: dict[str, Any], path: str) -> None:
    expected = schema.get("type")
    expected_types = expected if isinstance(expected, list) else [expected]
    type_checks = {
        "object": lambda item: isinstance(item, dict),
        "array": lambda item: isinstance(item, list),
        "string": lambda item: isinstance(item, str),
        "null": lambda item: item is None,
        "number": lambda item: isinstance(item, (int, float)) and not isinstance(item, bool),
        "integer": lambda item: isinstance(item, int) and not isinstance(item, bool),
        "boolean": lambda item: isinstance(item, bool),
    }
    if expected and not any(type_checks[item](value) for item in expected_types):
        raise SceneExchangeError(f"{path} must have type {expected!r}")
    if "const" in schema and value != schema["const"]:
        raise SceneExchangeError(f"{path} must equal {schema['const']!r}")
    if isinstance(value, str) and len(value) < schema.get("minLength", 0):
        raise SceneExchangeError(f"{path} is too short")
    if isinstance(value, str) and "pattern" in schema:
        if re.search(str(schema["pattern"]), value) is None:
            raise SceneExchangeError(f"{path} does not match the required pattern")
    if isinstance(value, dict):
        for name in schema.get("required", []):
            if name not in value:
                raise SceneExchangeError(f"{path}.{name} is required")
        for name, child_schema in schema.get("properties", {}).items():
            if name in value:
                _validate_schema_value(value[name], child_schema, f"{path}.{name}")


def _load_package_schema() -> dict[str, Any]:
    from adapters.shared_protocol_validation import schema_path

    path = schema_path("closed_loop_scene_package.v1")
    return json.loads(path.read_text(encoding="utf-8"))


def _scene_root_path(exchange_root: Path, scene_id: str, *, create: bool) -> Path:
    exchange_root = Path(exchange_root).resolve()
    scene_root = exchange_root / "scenes" / scene_id
    if create:
        scene_root.mkdir(parents=True, exist_ok=True)
    resolved = scene_root.resolve()
    try:
        resolved.relative_to(exchange_root)
    except ValueError as error:
        raise SceneExchangeError("scene directory escapes the exchange root") from error
    return resolved


def _contained_version_dir(scene_root: Path, version: str) -> Path:
    version_dir = scene_root / version
    resolved = version_dir.resolve()
    try:
        resolved.relative_to(scene_root)
    except ValueError as error:
        raise SceneExchangeError("scene version escapes the scene directory") from error
    return resolved


def _resolve_package_file(package_dir: Path, relative: str, label: str) -> Path:
    candidate = Path(relative)
    if candidate.is_absolute() or candidate.drive or ".." in candidate.parts:
        raise SceneExchangeError(f"{label} must be a contained relative path: {relative!r}")
    package_root = package_dir.resolve()
    resolved = (package_root / candidate).resolve()
    try:
        resolved.relative_to(package_root)
    except ValueError as error:
        raise SceneExchangeError(f"{label} escapes the scene version directory") from error
    if not resolved.is_file():
        raise SceneExchangeError(f"{label} does not exist: {relative!r}")
    return resolved


def validate_scene_package(
    package_dir: Path,
    *,
    expected_scene_id: str | None = None,
) -> dict[str, Any]:
    """Validate the shared schema and every local artifact reference."""

    package_dir = Path(package_dir)
    package_path = package_dir / PACKAGE_NAME
    if not package_path.is_file():
        raise SceneExchangeError(f"missing {PACKAGE_NAME}: {package_dir}")
    try:
        package = json.loads(package_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SceneExchangeError(f"invalid {PACKAGE_NAME}: {error}") from error

    from adapters.shared_protocol_validation import SharedProtocolValidationError, validate_document

    try:
        validate_document(package)
    except SharedProtocolValidationError as error:
        raise SceneExchangeError(f"invalid Scene Package contract: {error}") from error
    if expected_scene_id is not None and package["scene_id"] != expected_scene_id:
        raise SceneExchangeError(
            f"scene id mismatch: directory={expected_scene_id!r}, package={package['scene_id']!r}"
        )

    references = {
        "motion.scene_ir": package["motion"]["scene_ir"],
        "scenario.openscenario": package["scenario"]["openscenario"],
        "map.opendrive": package["map"].get("opendrive"),
        "visual.nurec_usdz": (package.get("visual") or {}).get("nurec_usdz"),
        "visual.nurec_checkpoint": (package.get("visual") or {}).get("nurec_checkpoint"),
        "visual.reconstruction_package": (package.get("visual") or {}).get("reconstruction_package"),
    }
    for label, relative in references.items():
        if relative is not None:
            if not isinstance(relative, str) or not relative:
                raise SceneExchangeError(f"{label} must be a non-empty relative path or null")
            _resolve_package_file(package_dir, relative, label)
    return package


def _ready_payload(version_dir: Path, scene_id: str, version: str) -> dict[str, Any]:
    package_path = version_dir / PACKAGE_NAME
    return {
        "schema_version": READY_SCHEMA_VERSION,
        "scene_id": scene_id,
        "version": version,
        "manifest": PACKAGE_NAME,
        "manifest_sha256": _sha256(package_path),
        "artifacts": _artifact_inventory(version_dir),
    }


def validate_ready_version(
    version_dir: Path,
    *,
    expected_scene_id: str | None = None,
    expected_version: str | None = None,
) -> dict[str, Any]:
    version_dir = Path(version_dir)
    marker_path = version_dir / READY_NAME
    if not marker_path.is_file():
        raise SceneExchangeError(f"scene version is not ready: {version_dir}")
    try:
        marker = json.loads(marker_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SceneExchangeError(f"invalid {READY_NAME}: {error}") from error
    if marker.get("schema_version") != READY_SCHEMA_VERSION:
        raise SceneExchangeError(f"unsupported ready marker: {marker.get('schema_version')!r}")
    if expected_scene_id is not None and marker.get("scene_id") != expected_scene_id:
        raise SceneExchangeError("READY scene id does not match the requested scene")
    if expected_version is not None and marker.get("version") != expected_version:
        raise SceneExchangeError("READY version does not match the requested version")
    if marker.get("manifest") != PACKAGE_NAME:
        raise SceneExchangeError("READY manifest name is invalid")
    package_path = version_dir / PACKAGE_NAME
    if not package_path.is_file() or marker.get("manifest_sha256") != _sha256(package_path):
        raise SceneExchangeError("scene package changed after publication")
    expected_artifacts = marker.get("artifacts")
    if not isinstance(expected_artifacts, dict):
        raise SceneExchangeError("READY artifact inventory is missing")
    if expected_artifacts != _artifact_inventory(version_dir):
        raise SceneExchangeError("scene artifacts changed after publication")
    return validate_scene_package(version_dir, expected_scene_id=expected_scene_id)


def publish_scene_version(
    bundle_dir: Path,
    exchange_root: Path,
    scene_id: str | None,
    version: str,
) -> Path:
    """Publish an immutable version atomically into ``root/scenes``."""

    bundle_dir = Path(bundle_dir).resolve()
    package = validate_scene_package(bundle_dir)
    package_scene_id = _safe_segment(package["scene_id"], "package scene id")
    if scene_id is None:
        scene_id = package_scene_id
    else:
        scene_id = _safe_segment(scene_id, "scene id")
        if scene_id != package_scene_id:
            raise SceneExchangeError(
                f"scene id mismatch: requested={scene_id!r}, package={package_scene_id!r}"
            )
    version = _safe_segment(version, "version")
    validate_scene_package(bundle_dir, expected_scene_id=scene_id)

    scene_root = _scene_root_path(exchange_root, scene_id, create=True)
    target = _contained_version_dir(scene_root, version)
    if target.exists():
        raise SceneVersionExistsError(f"scene version already exists: {target}")

    staging = scene_root / f".publishing-{version}-{uuid.uuid4().hex}"
    try:
        shutil.copytree(bundle_dir, staging, symlinks=False)
        validate_scene_package(staging, expected_scene_id=scene_id)
        (staging / READY_NAME).write_text(
            json.dumps(_ready_payload(staging, scene_id, version), indent=2) + "\n",
            encoding="utf-8",
        )
        try:
            os.rename(staging, target)
        except FileExistsError as error:
            raise SceneVersionExistsError(f"scene version already exists: {target}") from error
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    return target


def list_ready_versions(exchange_root: Path, scene_id: str) -> list[str]:
    scene_id = _safe_segment(scene_id, "scene id")
    scene_root = _scene_root_path(exchange_root, scene_id, create=False)
    if not scene_root.is_dir():
        return []
    versions = []
    for path in scene_root.iterdir():
        if not path.is_dir() or path.name.startswith("."):
            continue
        try:
            _safe_segment(path.name, "version")
            version_dir = _contained_version_dir(scene_root, path.name)
            validate_ready_version(
                version_dir,
                expected_scene_id=scene_id,
                expected_version=path.name,
            )
        except SceneExchangeError:
            continue
        versions.append(path.name)
    return sorted(versions)


def consume_scene_version(
    exchange_root: Path,
    scene_id: str,
    version: str | None = None,
) -> dict[str, Any]:
    """Resolve and validate one ready scene version for a runtime consumer."""

    scene_id = _safe_segment(scene_id, "scene id")
    if version is None:
        ready = list_ready_versions(exchange_root, scene_id)
        if not ready:
            raise SceneExchangeError(f"no ready version for scene {scene_id!r}")
        version = ready[-1]
    version = _safe_segment(version, "version")
    scene_root = _scene_root_path(exchange_root, scene_id, create=False)
    version_dir = _contained_version_dir(scene_root, version)
    package = validate_ready_version(
        version_dir,
        expected_scene_id=scene_id,
        expected_version=version,
    )

    def resolved(relative: str | None) -> str | None:
        return str(_resolve_package_file(version_dir, relative, "artifact")) if relative else None

    return {
        "scene_id": scene_id,
        "version": version,
        "version_dir": str(version_dir),
        "scene_package": str(version_dir / PACKAGE_NAME),
        "scene_ir": resolved(package["motion"]["scene_ir"]),
        "openscenario": resolved(package["scenario"]["openscenario"]),
        "opendrive": resolved(package["map"].get("opendrive")),
        "nurec_usdz": resolved((package.get("visual") or {}).get("nurec_usdz")),
        "nurec_checkpoint": resolved((package.get("visual") or {}).get("nurec_checkpoint")),
    }
