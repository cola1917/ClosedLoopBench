"""Strict, parameterized Scene-0061 one-tick diagnostic runner.

This runner deliberately does not reuse or rewrite historical repro scripts.
Every run snapshots the selected configuration and records the selected inputs
before attempting CARLA execution, so a later failure can be attributed to the
configuration that was actually supplied.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runners.run_carla_basic_agent import (
    _import_basic_agent_cls,
    build_basic_agent_plan,
    run_basic_agent,
)
from runners.validate_multimodal_closed_loop import (
    MultimodalClosedLoopError,
    validate_multimodal_closed_loop_result,
)
from adapters.nurec_multimodal import (
    NuRecMultimodalError,
    validate_nurec_multimodal_evidence,
)
from adapters.nurec_260_client import _validate_runtime_actor_binding_contract
from runtime.scene0061_carla_lidar_probe import CarlaNativeLidarProbe
from runtime.scene0061_lidar_axis_collector import (
    LiDARAxisCollectionError,
    collect_lidar_axis_evidence,
)


class Scene0061LiveTickError(RuntimeError):
    """Raised when a one-tick diagnostic cannot prove its own provenance."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Scene0061LiveTickError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise Scene0061LiveTickError(f"JSON document must be an object: {path}")
    return value


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _git_commit(project_root: Path = PROJECT_ROOT) -> str | None:
    try:
        completed = subprocess.run(
            ["git", "-C", str(project_root), "rev-parse", "HEAD"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    commit = completed.stdout.strip()
    return commit or None


def _python_runtime_identity() -> dict[str, Any]:
    """Return the interpreter identity that will execute CARLA/NuRec code.

    Generated NuRec protobuf modules are coupled to the protobuf runtime.  The
    identity is captured during preparation and checked again before execution
    so a shell's system Python cannot silently replace the env_build interpreter.
    """

    protobuf: dict[str, Any] = {"available": False}
    try:
        import google.protobuf  # type: ignore[import-not-found]

        protobuf = {
            "available": True,
            "version": str(getattr(google.protobuf, "__version__", "")),
            "path": str(Path(str(getattr(google.protobuf, "__file__", ""))).resolve()),
        }
    except Exception as exc:  # Evidence must say why an interpreter cannot load protobuf.
        protobuf["detail"] = f"{type(exc).__name__}: {exc}"
    return {
        "executable": str(Path(sys.executable).resolve()),
        "version": sys.version.split()[0],
        "protobuf": protobuf,
    }


def _carla_basic_agent_identity(carla_python_api_path: Path) -> dict[str, Any]:
    """Identify the explicit CARLA PythonAPI tree used for BasicAgent.

    The separately-installed ``carla`` wheel does not contain CARLA's
    ``agents.navigation`` package.  Recording the PythonAPI tree and the exact
    BasicAgent source file makes that otherwise implicit dependency auditable.
    """

    python_api = carla_python_api_path.expanduser().resolve()
    basic_agent = python_api / "agents" / "navigation" / "basic_agent.py"
    identity = _file_identity(basic_agent, required=True)
    assert identity is not None
    return {
        "python_api_path": str(python_api),
        "basic_agent": identity,
    }


def _preflight_carla_basic_agent(carla_python_api_path: Path) -> dict[str, Any]:
    """Load the explicit CARLA BasicAgent without connecting to CARLA."""

    identity = _carla_basic_agent_identity(carla_python_api_path)
    try:
        basic_agent_cls = _import_basic_agent_cls(identity["python_api_path"])
        module = sys.modules.get(str(getattr(basic_agent_cls, "__module__", "")))
        module_path = Path(str(getattr(module, "__file__", ""))).resolve()
        expected_path = Path(str(identity["basic_agent"]["path"])).resolve()
        if module_path != expected_path:
            raise Scene0061LiveTickError(
                f"BasicAgent resolved to {module_path}, expected {expected_path}"
            )
    except Exception as exc:
        raise Scene0061LiveTickError(
            f"CARLA BasicAgent preflight failed under {Path(sys.executable).resolve()}: {exc}"
        ) from exc
    return {"status": "passed", **identity}


def _file_identity(path: Path | None, *, required: bool = False) -> dict[str, Any] | None:
    if path is None:
        if required:
            raise Scene0061LiveTickError("a required file path is missing")
        return None
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise Scene0061LiveTickError(f"required file does not exist: {resolved}")
    return {
        "path": str(resolved),
        "sha256": _sha256_file(resolved),
        "byte_count": resolved.stat().st_size,
    }


def _configured_file(config_path: Path, value: Any) -> Path | None:
    text = str(value or "").strip()
    if not text:
        return None
    candidate = Path(text).expanduser()
    if not candidate.is_absolute():
        candidate = config_path.parent / candidate
    return candidate.resolve()


def _actor_binding_identity(config: Mapping[str, Any], config_path: Path) -> dict[str, Any] | None:
    runtime = config.get("nurec_runtime")
    if not isinstance(runtime, Mapping):
        return None
    identity = _file_identity(_configured_file(config_path, runtime.get("actor_bindings")))
    if identity is None:
        return None
    declared = runtime.get("actor_bindings_sha256")
    if declared is not None:
        declared = str(declared)
        identity["declared_sha256"] = declared
        if identity["sha256"] != declared:
            raise Scene0061LiveTickError(
                "nurec_runtime.actor_bindings_sha256 does not match the selected sidecar"
            )
    return identity


def _native_scan_manifest_identity(config: Mapping[str, Any], config_path: Path) -> dict[str, Any] | None:
    runtime = config.get("nurec_runtime")
    if not isinstance(runtime, Mapping):
        return None
    reference = runtime.get("native_scan_manifest")
    if not isinstance(reference, Mapping):
        return None
    identity = _file_identity(_configured_file(config_path, reference.get("path")))
    if identity is None:
        return None
    declared = reference.get("sha256")
    if declared is not None:
        declared = str(declared)
        identity["declared_sha256"] = declared
        if identity["sha256"] != declared:
            raise Scene0061LiveTickError(
                "nurec_runtime.native_scan_manifest.sha256 does not match the selected manifest"
            )
    return identity


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


_EVIDENCE_ARTIFACTS = (
    "basic_agent_plan.json",
    "runtime_result.json",
    "frame_trace.jsonl",
    "nurec_multimodal_trace.jsonl",
    "metrics_trace.jsonl",
    "cleanup_audit.json",
    "closed_loop_report.json",
    "lidar_axis_evidence.json",
)


def _write_artifact_manifest(output_dir: Path) -> dict[str, Any]:
    """Write a non-circular inventory of every required diagnostic output.

    The environment and the manifest are deliberately excluded from this list:
    both are updated after execution, so adding either would create a mutable
    self-hash.  The environment records the manifest identity instead.
    """

    root = output_dir.expanduser().resolve()
    artifacts: list[dict[str, Any]] = []
    missing: list[str] = []
    for name in _EVIDENCE_ARTIFACTS:
        path = root / name
        if not path.is_file():
            missing.append(name)
            continue
        identity = _file_identity(path, required=True)
        assert identity is not None
        artifacts.append({"name": name, **identity})
    manifest = {
        "schema_version": "scene0061_live_tick_artifact_manifest.v1",
        "generated_at_utc": _utc_now(),
        "status": "complete" if not missing else "incomplete",
        "artifacts": artifacts,
        "missing_artifacts": missing,
    }
    _write_json(root / "artifact_manifest.json", manifest)
    return manifest


def _read_jsonl_objects(path: Path, name: str, problems: list[str]) -> list[dict[str, Any]]:
    if not path.is_file() or not path.read_text(encoding="utf-8").strip():
        problems.append(f"missing persisted evidence: {name}")
        return []
    try:
        values = [
            json.loads(line)
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    except json.JSONDecodeError:
        problems.append(f"invalid JSONL evidence: {name}")
        return []
    if not all(isinstance(value, dict) for value in values):
        problems.append(f"JSONL evidence must contain objects: {name}")
        return []
    return values


def _is_within(path: Path, root: Path) -> bool:
    """Return whether a resolved artifact is contained by its run directory."""

    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        return False
    return True


def _validate_materialized_payload(
    record: Mapping[str, Any],
    *,
    output_dir: Path,
    seen_paths: set[Path],
    problems: list[str],
) -> None:
    """Re-hash a response payload rather than trusting its JSON pointer.

    The NuRec evidence contract deliberately permits metadata without a local
    payload.  That is useful for generic clients, but insufficient for this
    physical G0 diagnostic: its RGB/LiDAR response must remain independently
    inspectable after CARLA and NRE have been torn down.
    """

    modality = str(record.get("modality") or "unknown")
    sensor_id = str(record.get("sensor_id") or "unknown")
    label = f"{modality}:{sensor_id}"
    metadata = record.get("response_metadata")
    if not isinstance(metadata, Mapping):
        problems.append(f"{label} has no response metadata")
        return
    materialized = metadata.get("materialized_payload")
    if not isinstance(materialized, Mapping):
        problems.append(f"{label} has no materialized response payload")
        return
    declared_path = str(materialized.get("path") or "").strip()
    if not declared_path:
        problems.append(f"{label} materialized payload path is empty")
        return
    payload_path = Path(declared_path).expanduser().resolve()
    payload_root = output_dir.resolve() / "algorithm_sensor_payloads"
    if not _is_within(payload_path, payload_root):
        problems.append(f"{label} materialized payload is outside this run")
        return
    if not payload_path.is_file():
        problems.append(f"{label} materialized payload does not exist")
        return
    if payload_path in seen_paths:
        problems.append(f"{label} reuses a materialized payload path")
        return
    seen_paths.add(payload_path)
    byte_count = materialized.get("byte_count")
    if byte_count != payload_path.stat().st_size:
        problems.append(f"{label} materialized payload byte count does not match")
    if str(materialized.get("sha256") or "") != _sha256_file(payload_path):
        problems.append(f"{label} materialized payload SHA-256 does not match")


def _validate_physical_multimodal_evidence(
    evidence: Mapping[str, Any],
    *,
    output_dir: Path,
    environment: Mapping[str, Any] | None,
    problems: list[str],
) -> None:
    """Apply Scene-0061 G0 checks that are stricter than the shared schema.

    This is intentionally an acceptance-layer validator.  The shared contract
    supports any non-empty RGB/LiDAR request set and clients that do not retain
    payload files; Scene-0061 G0 specifically requires one live six-camera and
    one-LiDAR frame whose local artifacts can be re-hashed.
    """

    try:
        validate_nurec_multimodal_evidence(evidence)
    except NuRecMultimodalError as exc:
        problems.append(f"persisted NuRec frame is invalid: {exc}")
        return

    records = evidence.get("records")
    if not isinstance(records, list):
        problems.append("persisted NuRec frame has no response records")
        return
    rgb_records = [record for record in records if record.get("modality") == "rgb"]
    lidar_records = [record for record in records if record.get("modality") == "lidar"]
    if len(rgb_records) != 6:
        problems.append("persisted NuRec frame does not contain exactly six RGB responses")
    if len(lidar_records) != 1:
        problems.append("persisted NuRec frame does not contain exactly one LiDAR response")
    sensor_ids = [str(record.get("sensor_id") or "") for record in records]
    if not all(sensor_ids) or len(sensor_ids) != len(set(sensor_ids)):
        problems.append("persisted NuRec frame has missing or duplicate sensor IDs")
    seen_paths: set[Path] = set()
    for record in records:
        _validate_materialized_payload(
            record, output_dir=output_dir, seen_paths=seen_paths, problems=problems
        )

    dispatch = evidence.get("dispatch")
    if not isinstance(dispatch, Mapping):
        problems.append("persisted NuRec frame has no NRE dispatch evidence")
        return
    if dispatch.get("nre_api") != "SensorsimService/26.04":
        problems.append("persisted NuRec frame is not bound to NRE SensorsimService/26.04")
    if dispatch.get("response_validation") != "injected_modality_specific_inspector":
        problems.append("persisted NuRec frame lacks modality-specific NRE response validation")
    if dispatch.get("canonical_scene_id") != evidence.get("scene_id"):
        problems.append("persisted NuRec dispatch canonical scene does not match evidence")

    expected_runtime = (environment or {}).get("nurec_runtime")
    if isinstance(expected_runtime, Mapping):
        expected_scene = expected_runtime.get("runtime_scene_id")
        if expected_scene and dispatch.get("runtime_scene_id") != expected_scene:
            problems.append("persisted NuRec dispatch runtime scene does not match prepared config")

    native_scan = (environment or {}).get("native_scan_manifest")
    if isinstance(native_scan, Mapping):
        alignment = dispatch.get("temporal_alignment")
        if not isinstance(alignment, Mapping):
            problems.append("persisted NuRec frame has no native-scan alignment")
        else:
            if alignment.get("source") != "hashed_native_scan_manifest":
                problems.append("persisted NuRec alignment is not sourced from a hashed native scan")
            if alignment.get("manifest_sha256") != native_scan.get("sha256"):
                problems.append("persisted NuRec alignment manifest SHA-256 does not match prepared input")
            try:
                if int(alignment.get("midpoint_error_us")) > int(
                    alignment.get("max_midpoint_error_us")
                ):
                    problems.append("persisted NuRec native-scan alignment exceeds its threshold")
            except (TypeError, ValueError):
                problems.append("persisted NuRec native-scan alignment has invalid midpoint evidence")


def _validate_static_actor_binding(config: Mapping[str, Any], sidecar_identity: Mapping[str, Any] | None) -> None:
    """Reject a config/sidecar contract drift before a CARLA process is touched."""

    runtime = config.get("nurec_runtime")
    selected = (config.get("actor_binding") or {}).get("selected_actor_ids")
    if not isinstance(runtime, Mapping) or not selected:
        return
    if sidecar_identity is None:
        raise Scene0061LiveTickError("selected actor bindings require a readable sidecar")
    try:
        sidecar = _load_object(Path(str(sidecar_identity["path"])))
        resolved_config = json.loads(json.dumps(config))
        resolved_config.setdefault("nurec_runtime", {})["actor_bindings"] = sidecar_identity["path"]
        _validate_runtime_actor_binding_contract(
            resolved_config, resolved_config["nurec_runtime"], sidecar
        )
    except (NuRecMultimodalError, OSError, KeyError, TypeError, ValueError) as exc:
        raise Scene0061LiveTickError(
            f"static actor-binding contract is not executable: {exc}"
        ) from exc


def _preflight_sensor_handler(
    config: dict[str, Any], output_dir: Path, sensor_handler_factory: Callable[[dict[str, Any], Path], Any]
) -> dict[str, Any]:
    """Import generated protobufs and construct the handler without touching CARLA."""

    try:
        handler = sensor_handler_factory(config, output_dir)
        if not callable(handler):
            raise Scene0061LiveTickError("sensor handler factory returned a non-callable")
        if hasattr(handler, "close"):
            handler.close()
    except Exception as exc:
        raise Scene0061LiveTickError(
            f"NuRec sensor handler preflight failed under {Path(sys.executable).resolve()}: {exc}"
        ) from exc
    return {
        "status": "passed",
        "factory": f"{sensor_handler_factory.__module__}:{sensor_handler_factory.__name__}",
    }


def prepare_live_tick(
    *,
    config_path: Path,
    output_dir: Path,
    run_id: str,
    opendrive_path: Path,
    host: str = "127.0.0.1",
    port: int = 2000,
    timeout_sec: float = 60.0,
    ego_driver: str = "basic_agent",
    require_multimodal: bool = True,
    carla_python_api_path: Path | None = None,
    capture_native_lidar: bool = False,
) -> dict[str, Any]:
    """Snapshot explicit inputs and create a plan for exactly one CARLA tick.

    ``output_dir`` must be new.  This prevents a later run from accidentally
    treating a trace from an earlier configuration as its own evidence.
    """

    if not str(run_id).strip():
        raise Scene0061LiveTickError("run_id must be non-empty")
    if not isinstance(capture_native_lidar, bool):
        raise Scene0061LiveTickError("capture_native_lidar must be a boolean")
    source_config = config_path.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    xodr = opendrive_path.expanduser().resolve()
    if output_dir.exists():
        raise Scene0061LiveTickError(f"output directory already exists: {output_dir}")
    config_identity = _file_identity(source_config, required=True)
    xodr_identity = _file_identity(xodr, required=True)
    config = _load_object(source_config)
    sidecar_identity = _actor_binding_identity(config, source_config)
    native_scan_manifest = _native_scan_manifest_identity(config, source_config)
    _validate_static_actor_binding(config, sidecar_identity)
    carla_basic_agent = (
        _preflight_carla_basic_agent(carla_python_api_path)
        if ego_driver == "basic_agent" and carla_python_api_path is not None
        else None
    )

    output_dir.mkdir(parents=True, exist_ok=False)
    runtime_config_path = output_dir / "runtime_run_config.json"
    runtime_config_path.write_bytes(source_config.read_bytes())
    runtime_config_identity = _file_identity(runtime_config_path, required=True)
    if runtime_config_identity["sha256"] != config_identity["sha256"]:
        raise Scene0061LiveTickError("runtime configuration snapshot hash mismatch")

    report_path = output_dir / "closed_loop_report.json"
    plan = build_basic_agent_plan(
        config,
        host=host,
        port=port,
        timeout_sec=timeout_sec,
        max_ticks=1,
        synchronous=True,
        output=str(report_path),
        acceptance_evidence=True,
        multimodal_sensor_required=require_multimodal,
        opendrive_path=str(xodr),
        ego_driver=ego_driver,
        snap_to_map=True,
    )
    # The config itself stays byte-for-byte intact.  The diagnostic run ID is
    # an execution identity, not an unrecorded mutation of user configuration.
    plan["run_id"] = str(run_id)
    plan.setdefault("experiment", {})["run_id"] = str(run_id)
    if carla_basic_agent is not None:
        plan.setdefault("runtime", {})["carla_python_api_path"] = carla_basic_agent[
            "python_api_path"
        ]
    environment_path = output_dir / "runtime_environment.json"
    plan.setdefault("artifacts", {})["runtime_environment"] = str(environment_path)
    plan.setdefault("runtime", {})["provenance"] = {
        "schema_version": "scene0061_live_tick_provenance.v1",
        "runtime_environment": str(environment_path),
        "selected_config_path": config_identity["path"],
        "selected_config_sha256": config_identity["sha256"],
        "runtime_config_path": runtime_config_identity["path"],
        "runtime_config_sha256": runtime_config_identity["sha256"],
        "capture_native_lidar_requested": capture_native_lidar,
        **(
            {"carla_basic_agent": carla_basic_agent}
            if carla_basic_agent is not None
            else {}
        ),
    }
    plan_path = output_dir / "basic_agent_plan.json"
    _write_json(plan_path, plan)

    environment: dict[str, Any] = {
        "schema_version": "scene0061_live_tick_environment.v1",
        "status": "prepared",
        "prepared_at_utc": _utc_now(),
        "run_id": str(run_id),
        "git_commit": _git_commit(),
        "python_runtime": _python_runtime_identity(),
        "config": config_identity,
        "runtime_config": runtime_config_identity,
        "opendrive": xodr_identity,
        "actor_bindings": sidecar_identity,
        "native_scan_manifest": native_scan_manifest,
        "nurec_runtime": {
            "runtime_scene_id": str(
                ((config.get("nurec_runtime") or {}).get("runtime_scene_id") or "")
            )
        },
        "carla_basic_agent": carla_basic_agent,
        "physical_lidar_probe": {
            "requested": capture_native_lidar,
            "mode": "same_frame_carla_native_lidar" if capture_native_lidar else "not_requested",
        },
        "artifacts": {
            "basic_agent_plan": str(plan_path.resolve()),
            "runtime_environment": str(environment_path.resolve()),
            "runtime_result": str((output_dir / "runtime_result.json").resolve()),
            "live_tick_validation": str((output_dir / "live_tick_validation.json").resolve()),
            "artifact_manifest": str((output_dir / "artifact_manifest.json").resolve()),
        },
    }
    _write_json(environment_path, environment)
    return environment


def verify_prepared_live_tick(output_dir: Path) -> dict[str, Any]:
    """Fail before CARLA execution when any recorded input drifted."""

    root = output_dir.expanduser().resolve()
    environment_path = root / "runtime_environment.json"
    environment = _load_object(environment_path)
    if environment.get("schema_version") != "scene0061_live_tick_environment.v1":
        raise Scene0061LiveTickError("unsupported runtime_environment schema")
    prepared_python = environment.get("python_runtime")
    current_python = _python_runtime_identity()
    if prepared_python != current_python:
        raise Scene0061LiveTickError(
            "prepared interpreter/protobuf identity does not match the executing process"
        )
    for name in ("config", "runtime_config", "opendrive", "actor_bindings", "native_scan_manifest"):
        identity = environment.get(name)
        if identity is None:
            continue
        if not isinstance(identity, Mapping):
            raise Scene0061LiveTickError(f"runtime_environment.{name} must be an object")
        path = Path(str(identity.get("path") or ""))
        expected = str(identity.get("sha256") or "")
        if not path.is_file() or _sha256_file(path) != expected:
            raise Scene0061LiveTickError(f"recorded {name} path/SHA-256 no longer matches")
    selected_config = environment.get("config")
    runtime_config = environment.get("runtime_config")
    if not isinstance(selected_config, Mapping) or not isinstance(runtime_config, Mapping):
        raise Scene0061LiveTickError(
            "prepared environment requires selected and runtime configuration identities"
        )
    if (
        selected_config.get("sha256") != runtime_config.get("sha256")
        or selected_config.get("byte_count") != runtime_config.get("byte_count")
    ):
        raise Scene0061LiveTickError(
            "runtime configuration snapshot does not match the selected config bytes"
        )
    carla_basic_agent = environment.get("carla_basic_agent")
    if carla_basic_agent is not None:
        if not isinstance(carla_basic_agent, Mapping) or carla_basic_agent.get("status") != "passed":
            raise Scene0061LiveTickError("prepared environment has no successful CARLA BasicAgent preflight")
        try:
            current_basic_agent = _carla_basic_agent_identity(
                Path(str(carla_basic_agent.get("python_api_path") or ""))
            )
        except Scene0061LiveTickError as exc:
            raise Scene0061LiveTickError(
                "recorded CARLA BasicAgent path/SHA-256 no longer matches"
            ) from exc
        if (
            current_basic_agent.get("python_api_path") != carla_basic_agent.get("python_api_path")
            or current_basic_agent.get("basic_agent") != carla_basic_agent.get("basic_agent")
        ):
            raise Scene0061LiveTickError(
                "recorded CARLA BasicAgent path/SHA-256 no longer matches"
            )
    runtime_config = runtime_config or {}
    plan_path = Path(str((environment.get("artifacts") or {}).get("basic_agent_plan") or ""))
    plan = _load_object(plan_path)
    provenance = ((plan.get("runtime") or {}).get("provenance") or {})
    if (
        provenance.get("runtime_config_path") != runtime_config.get("path")
        or provenance.get("runtime_config_sha256") != runtime_config.get("sha256")
    ):
        raise Scene0061LiveTickError(
            "basic_agent_plan runtime config path/SHA-256 does not match runtime_environment"
        )
    selected_config = selected_config or {}
    if (
        provenance.get("selected_config_path") != selected_config.get("path")
        or provenance.get("selected_config_sha256") != selected_config.get("sha256")
    ):
        raise Scene0061LiveTickError(
            "basic_agent_plan selected config path/SHA-256 does not match runtime_environment"
        )
    physical_lidar_probe = environment.get("physical_lidar_probe")
    if not isinstance(physical_lidar_probe, Mapping) or not isinstance(
        physical_lidar_probe.get("requested"), bool
    ):
        raise Scene0061LiveTickError(
            "prepared environment has no boolean physical LiDAR probe request"
        )
    if provenance.get("capture_native_lidar_requested") is not physical_lidar_probe.get("requested"):
        raise Scene0061LiveTickError(
            "basic_agent_plan native LiDAR probe request does not match runtime_environment"
        )
    handler_preflight = environment.get("sensor_handler_preflight")
    if not isinstance(handler_preflight, Mapping) or handler_preflight.get("status") != "passed":
        raise Scene0061LiveTickError(
            "prepared environment has no successful NuRec sensor-handler preflight"
        )
    return environment


def _load_callable(spec: str) -> Callable[..., Any]:
    if ":" not in spec:
        raise Scene0061LiveTickError("sensor handler factory must use module:callable")
    module_name, attribute = spec.split(":", 1)
    value = getattr(importlib.import_module(module_name), attribute, None)
    if not callable(value):
        raise Scene0061LiveTickError(f"sensor handler factory is not callable: {spec}")
    return value


def validate_live_tick_result(result: Mapping[str, Any], output_dir: Path) -> dict[str, Any]:
    """Validate the first physical multimodal frame and persisted evidence."""

    problems: list[str] = []
    environment: Mapping[str, Any] | None = None
    environment_path = output_dir / "runtime_environment.json"
    if environment_path.is_file():
        try:
            environment = _load_object(environment_path)
        except Scene0061LiveTickError as exc:
            problems.append(f"cannot read persisted runtime environment: {exc}")
    detail = str(result.get("detail") or "")
    expected_one_tick_termination = (
        result.get("status") == "failed"
        and result.get("reason") == "basic_agent_runtime_failed"
        and detail.startswith("route_incomplete:")
    )
    if result.get("status") not in {"ego_closed_loop", "interactive_closed_loop"} and not expected_one_tick_termination:
        problems.append(f"runtime_status={result.get('status')!r}")
    report = result.get("report") or {}
    runtime = report.get("runtime") or {}
    if result.get("cleanup_succeeded") is not True:
        problems.append("CARLA cleanup did not succeed")
    frame_trace_path = output_dir / "frame_trace.jsonl"
    multimodal_trace_path = output_dir / "nurec_multimodal_trace.jsonl"
    cleanup_path = output_dir / "cleanup_audit.json"
    persisted_rows = {
        "frame_trace.jsonl": _read_jsonl_objects(frame_trace_path, "frame_trace.jsonl", problems),
        "nurec_multimodal_trace.jsonl": _read_jsonl_objects(
            multimodal_trace_path, "nurec_multimodal_trace.jsonl", problems
        ),
    }
    if len(persisted_rows.get("frame_trace.jsonl", [])) != 1:
        problems.append("persisted frame trace does not contain exactly one frame")
    else:
        frame = persisted_rows["frame_trace.jsonl"][0]
        if not isinstance(frame.get("world_tick_frame") or frame.get("snapshot_frame"), int):
            problems.append("persisted frame trace has no CARLA frame identity")
        control = frame.get("ego_control")
        if not isinstance(control, Mapping) or not {"throttle", "steer", "brake"}.issubset(control):
            problems.append("persisted frame trace has no BasicAgent control")
    if len(persisted_rows.get("nurec_multimodal_trace.jsonl", [])) != 1:
        problems.append("persisted NuRec trace does not contain exactly one frame")
    else:
        nurec_frame = persisted_rows["nurec_multimodal_trace.jsonl"][0]
        _validate_physical_multimodal_evidence(
            nurec_frame,
            output_dir=output_dir,
            environment=environment,
            problems=problems,
        )
        frame = persisted_rows.get("frame_trace.jsonl", [{}])[0]
        trace_frame_id = nurec_frame.get("frame_id")
        carla_frame_id = frame.get("world_tick_frame") or frame.get("snapshot_frame")
        if trace_frame_id != carla_frame_id:
            problems.append("persisted CARLA and NuRec frame identities diverge")
    if not cleanup_path.is_file() or not cleanup_path.read_text(encoding="utf-8").strip():
        problems.append("missing persisted evidence: cleanup_audit.json")
    else:
        try:
            cleanup = json.loads(cleanup_path.read_text(encoding="utf-8"))
            actions = cleanup.get("actions")
            if cleanup.get("succeeded") is not True:
                problems.append("persisted cleanup audit did not succeed")
            if not isinstance(actions, list) or not actions:
                problems.append("persisted cleanup audit has no actions")
            elif any(not isinstance(action, Mapping) or action.get("status") != "succeeded" for action in actions):
                problems.append("one or more persisted cleanup actions failed")
        except json.JSONDecodeError:
            problems.append("invalid JSON evidence: cleanup_audit.json")
    if not problems and result.get("status") == "interactive_closed_loop":
        try:
            validate_multimodal_closed_loop_result(dict(result))
        except MultimodalClosedLoopError as exc:
            problems.append(f"multimodal evidence invalid: {exc}")
    memory_trace = result.get("nurec_multimodal_trace")
    disk_trace = persisted_rows.get("nurec_multimodal_trace.jsonl")
    if isinstance(memory_trace, list) and disk_trace is not None and memory_trace != disk_trace:
        problems.append("persisted NuRec trace diverges from runtime result")
    return {
        "schema_version": "scene0061_live_tick_validation.v1",
        "status": "passed" if not problems else "failed",
        "completion_class": (
            "one_tick_physical_multimodal_smoke"
            if expected_one_tick_termination
            else "closed_loop"
        ),
        "expected_one_tick_termination": expected_one_tick_termination,
        "frame_trace_count": int(runtime.get("frame_trace_count") or 0),
        "cleanup_succeeded": result.get("cleanup_succeeded") is True,
        "problems": problems,
    }


def _physical_lidar_probe_factory(
    config: Mapping[str, Any], output_dir: Path
) -> Callable[..., CarlaNativeLidarProbe]:
    """Bind a CARLA-native probe only to the configured ``lidar_top`` rig.

    The factory is built from the immutable runtime config used by the NuRec
    handler.  It never reads an historical output or infers a transform from a
    renderer response.
    """

    runtime = config.get("nurec_runtime")
    specs = runtime.get("lidar_specs") if isinstance(runtime, Mapping) else None
    rows = [
        row
        for row in specs or []
        if isinstance(row, Mapping) and row.get("sensor_id") == "lidar_top"
    ]
    if len(rows) != 1:
        raise Scene0061LiveTickError(
            "physical LiDAR probe requires exactly one configured lidar_top spec"
        )
    spec = rows[0]
    matrix = spec.get("sensor_to_ego")
    attributes = spec.get("carla_native_probe_attributes")
    if attributes is not None and not isinstance(attributes, Mapping):
        raise Scene0061LiveTickError(
            "lidar_top.carla_native_probe_attributes must be an object when supplied"
        )

    def factory(*, carla_module: Any, world: Any, ego_vehicle: Any, plan: Any) -> CarlaNativeLidarProbe:
        del plan
        return CarlaNativeLidarProbe(
            carla_module=carla_module,
            world=world,
            ego_vehicle=ego_vehicle,
            output_dir=output_dir,
            sensor_to_ego=matrix,
            blueprint_attributes=attributes,
        )

    return factory


def _collect_lidar_axis_evidence(
    *,
    output_dir: Path,
    config: Mapping[str, Any],
    environment: Mapping[str, Any],
    capture_native_lidar: bool,
) -> dict[str, Any]:
    """Try a physical coordinate proof, preserving an explicit failure otherwise.

    A non-passing collection is evidence of a real blocker, not a reason to
    mutate NuRec's coordinate metadata.  This function therefore catches only
    expected collection failures and records the exact reason beside the run.
    """

    base = {
        "schema_version": "scene0061_lidar_coordinate_validation.v1",
        "collection_source": "same_frame_carla_native_lidar_probe",
    }
    if not capture_native_lidar:
        # The default G0 diagnostic is not silently changed by an auxiliary
        # sensor.  Record the absence explicitly so it cannot be mistaken for
        # either a physical axis pass or a collection failure.
        return {
            **base,
            "status": "not_requested",
            "reason": "native_lidar_capture_not_requested",
        }
    try:
        frame_rows = _read_jsonl_objects(output_dir / "frame_trace.jsonl", "frame_trace.jsonl", [])
        nurec_rows = _read_jsonl_objects(
            output_dir / "nurec_multimodal_trace.jsonl",
            "nurec_multimodal_trace.jsonl",
            [],
        )
        if len(frame_rows) != 1 or len(nurec_rows) != 1:
            raise LiDARAxisCollectionError(
                "physical axis collection requires exactly one frame and one NuRec trace"
            )
        capture = frame_rows[0].get("native_lidar_capture")
        if not isinstance(capture, Mapping) or not capture.get("capture_path"):
            raise LiDARAxisCollectionError(
                "physical axis collection has no same-frame CARLA native LiDAR capture"
            )
        native_scan = environment.get("native_scan_manifest")
        if not isinstance(native_scan, Mapping) or not native_scan.get("path"):
            raise LiDARAxisCollectionError(
                "physical axis collection has no prepared native-scan manifest"
            )
        evidence = collect_lidar_axis_evidence(
            run_config=config,
            nurec_evidence=nurec_rows[0],
            frame_trace=frame_rows[0],
            native_capture_path=Path(str(capture["capture_path"])),
            native_scan_manifest_path=Path(str(native_scan["path"])),
        )
        evidence["collection_source"] = base["collection_source"]
        return evidence
    except (LiDARAxisCollectionError, OSError, ValueError) as exc:
        return {
            **base,
            "status": "failed",
            "reason": "physical_axis_collection_failed",
            "detail": str(exc),
        }


def execute_live_tick(
    output_dir: Path,
    *,
    sensor_handler_factory: Callable[[dict[str, Any], Path], Any],
    execute: Callable[..., dict[str, Any]] = run_basic_agent,
) -> dict[str, Any]:
    """Execute a prepared run, retaining evidence even when the gate fails."""

    root = output_dir.expanduser().resolve()
    environment = verify_prepared_live_tick(root)
    expected_factory = (environment.get("sensor_handler_preflight") or {}).get("factory")
    actual_factory = f"{sensor_handler_factory.__module__}:{sensor_handler_factory.__name__}"
    if expected_factory != actual_factory:
        raise Scene0061LiveTickError(
            "prepared NuRec sensor-handler factory does not match the executing factory"
        )
    environment_path = root / "runtime_environment.json"
    environment["status"] = "running"
    environment["execution_started_at_utc"] = _utc_now()
    _write_json(environment_path, environment)
    # The handler must consume the byte-for-byte runtime snapshot that was
    # verified above, rather than re-reading the operator's source path.  This
    # keeps the archived run input and the actual NuRec request configuration
    # identical at execution time.
    config = _load_object(
        Path(str((environment.get("runtime_config") or {}).get("path")))
    )
    plan = _load_object(Path(str((environment.get("artifacts") or {}).get("basic_agent_plan") or "")))
    physical_lidar_probe = environment.get("physical_lidar_probe")
    assert isinstance(physical_lidar_probe, Mapping)
    capture_native_lidar = physical_lidar_probe["requested"]
    assert isinstance(capture_native_lidar, bool)
    result_path = Path(str((environment.get("artifacts") or {}).get("runtime_result") or ""))
    validation_path = Path(str((environment.get("artifacts") or {}).get("live_tick_validation") or ""))
    try:
        handler = sensor_handler_factory(config, root)
        if not callable(handler):
            raise Scene0061LiveTickError("sensor handler factory returned a non-callable")
        if execute is run_basic_agent and capture_native_lidar:
            result = execute(
                plan,
                sensor_frame_handler=handler,
                physical_frame_probe_factory=_physical_lidar_probe_factory(config, root),
            )
        else:
            # Test and alternate integrations retain their established callable
            # contract.  The production BasicAgent path is the only one allowed
            # to make a physical CARLA coordinate claim.
            result = execute(plan, sensor_frame_handler=handler)
    except Exception as exc:
        result = {
            "status": "failed",
            "reason": "scene0061_live_tick_execution_failed",
            "detail": str(exc),
        }
    _write_json(result_path, result)
    axis_evidence = _collect_lidar_axis_evidence(
        output_dir=root,
        config=config,
        environment=environment,
        capture_native_lidar=capture_native_lidar,
    )
    _write_json(root / "lidar_axis_evidence.json", axis_evidence)
    validation = validate_live_tick_result(result, root)
    _write_json(validation_path, validation)
    environment["status"] = validation["status"]
    environment["execution_finished_at_utc"] = _utc_now()
    environment["validation"] = validation
    environment["lidar_axis_evidence"] = {
        **_file_identity(root / "lidar_axis_evidence.json", required=True),
        "status": axis_evidence["status"],
    }
    _write_json(environment_path, environment)
    manifest = _write_artifact_manifest(root)
    manifest_path = root / "artifact_manifest.json"
    manifest_identity = _file_identity(manifest_path, required=True)
    assert manifest_identity is not None
    environment["artifact_manifest"] = {
        **manifest_identity,
        "status": manifest["status"],
        "missing_artifacts": manifest["missing_artifacts"],
    }
    _write_json(environment_path, environment)
    return {
        "environment": environment,
        "result": result,
        "validation": validation,
        "artifact_manifest": manifest,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run one provenance-bound Scene-0061 CARLA/NuRec diagnostic tick."
    )
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--opendrive", required=True, type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--ego-driver", default="basic_agent")
    parser.add_argument(
        "--carla-python-api",
        type=Path,
        help="Explicit CARLA/PythonAPI/carla directory containing agents/navigation/basic_agent.py.",
    )
    parser.add_argument("--sensor-handler-factory", default="adapters.nurec_260_client:build_nurec_260_handler")
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--execute-prepared", action="store_true")
    parser.add_argument(
        "--capture-native-lidar",
        action="store_true",
        help=(
            "Explicitly arm a same-frame CARLA native LiDAR capture for the physical axis gate. "
            "The prepared and executing values must match."
        ),
    )
    args = parser.parse_args(argv)
    try:
        if args.prepare_only and args.execute_prepared:
            raise Scene0061LiveTickError("--prepare-only and --execute-prepared are mutually exclusive")
        if args.ego_driver == "basic_agent" and args.carla_python_api is None:
            raise Scene0061LiveTickError(
                "--carla-python-api is required for a BasicAgent live-tick diagnostic"
            )
        if args.execute_prepared:
            environment = verify_prepared_live_tick(args.output_dir)
            if (
                environment.get("run_id") != args.run_id
                or (environment.get("config") or {}).get("path") != str(args.config.expanduser().resolve())
                or (environment.get("opendrive") or {}).get("path") != str(args.opendrive.expanduser().resolve())
                or (
                    (environment.get("carla_basic_agent") or {}).get("python_api_path")
                    != str(args.carla_python_api.expanduser().resolve())
                )
                or (
                    ((environment.get("physical_lidar_probe") or {}).get("requested")
                    is not args.capture_native_lidar)
                )
            ):
                raise Scene0061LiveTickError("explicit execute-prepared inputs do not match the prepared environment")
        else:
            environment = prepare_live_tick(
                config_path=args.config,
                output_dir=args.output_dir,
                run_id=args.run_id,
                opendrive_path=args.opendrive,
                host=args.host,
                port=args.port,
                timeout_sec=args.timeout_sec,
                ego_driver=args.ego_driver,
                carla_python_api_path=args.carla_python_api,
                capture_native_lidar=args.capture_native_lidar,
            )
        if args.prepare_only:
            config = _load_object(
                Path(str((environment.get("runtime_config") or {}).get("path")))
            )
            environment["sensor_handler_preflight"] = _preflight_sensor_handler(
                config, args.output_dir.expanduser().resolve(), _load_callable(args.sensor_handler_factory)
            )
            _write_json(args.output_dir.expanduser().resolve() / "runtime_environment.json", environment)
        if args.prepare_only:
            print(json.dumps(environment, ensure_ascii=False, indent=2))
            return 0
        outcome = execute_live_tick(
            args.output_dir, sensor_handler_factory=_load_callable(args.sensor_handler_factory)
        )
    except (ImportError, OSError, Scene0061LiveTickError, ValueError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2
    print(json.dumps(outcome, ensure_ascii=False, indent=2))
    return 0 if outcome["validation"]["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
