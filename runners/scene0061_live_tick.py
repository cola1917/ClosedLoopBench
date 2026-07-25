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

from runners.run_carla_basic_agent import build_basic_agent_plan, run_basic_agent
from runners.validate_multimodal_closed_loop import (
    MultimodalClosedLoopError,
    validate_multimodal_closed_loop_result,
)
from adapters.nurec_multimodal import (
    NuRecMultimodalError,
    validate_nurec_multimodal_evidence,
)
from adapters.nurec_260_client import _validate_runtime_actor_binding_contract


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
) -> dict[str, Any]:
    """Snapshot explicit inputs and create a plan for exactly one CARLA tick.

    ``output_dir`` must be new.  This prevents a later run from accidentally
    treating a trace from an earlier configuration as its own evidence.
    """

    if not str(run_id).strip():
        raise Scene0061LiveTickError("run_id must be non-empty")
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
    environment_path = output_dir / "runtime_environment.json"
    plan.setdefault("artifacts", {})["runtime_environment"] = str(environment_path)
    plan.setdefault("runtime", {})["provenance"] = {
        "schema_version": "scene0061_live_tick_provenance.v1",
        "runtime_environment": str(environment_path),
        "selected_config_path": config_identity["path"],
        "selected_config_sha256": config_identity["sha256"],
        "runtime_config_path": runtime_config_identity["path"],
        "runtime_config_sha256": runtime_config_identity["sha256"],
    }
    plan_path = output_dir / "basic_agent_plan.json"
    _write_json(plan_path, plan)

    environment: dict[str, Any] = {
        "schema_version": "scene0061_live_tick_environment.v1",
        "status": "prepared",
        "prepared_at_utc": _utc_now(),
        "run_id": str(run_id),
        "git_commit": _git_commit(),
        "config": config_identity,
        "runtime_config": runtime_config_identity,
        "opendrive": xodr_identity,
        "actor_bindings": sidecar_identity,
        "native_scan_manifest": native_scan_manifest,
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
    runtime_config = environment.get("runtime_config") or {}
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
    selected_config = environment.get("config") or {}
    if (
        provenance.get("selected_config_path") != selected_config.get("path")
        or provenance.get("selected_config_sha256") != selected_config.get("sha256")
    ):
        raise Scene0061LiveTickError(
            "basic_agent_plan selected config path/SHA-256 does not match runtime_environment"
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
        try:
            validate_nurec_multimodal_evidence(nurec_frame)
        except NuRecMultimodalError as exc:
            problems.append(f"persisted NuRec frame is invalid: {exc}")
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


def execute_live_tick(
    output_dir: Path,
    *,
    sensor_handler_factory: Callable[[dict[str, Any], Path], Any],
    execute: Callable[..., dict[str, Any]] = run_basic_agent,
) -> dict[str, Any]:
    """Execute a prepared run, retaining evidence even when the gate fails."""

    root = output_dir.expanduser().resolve()
    environment = verify_prepared_live_tick(root)
    environment_path = root / "runtime_environment.json"
    environment["status"] = "running"
    environment["execution_started_at_utc"] = _utc_now()
    _write_json(environment_path, environment)
    # Build the handler from the explicit source path supplied by the operator.
    # This preserves relative paths in the source config; the byte-for-byte
    # snapshot is archival evidence and is verified separately above.
    config = _load_object(Path(str((environment.get("config") or {}).get("path"))))
    plan = _load_object(Path(str((environment.get("artifacts") or {}).get("basic_agent_plan") or "")))
    result_path = Path(str((environment.get("artifacts") or {}).get("runtime_result") or ""))
    validation_path = Path(str((environment.get("artifacts") or {}).get("live_tick_validation") or ""))
    try:
        handler = sensor_handler_factory(config, root)
        if not callable(handler):
            raise Scene0061LiveTickError("sensor handler factory returned a non-callable")
        result = execute(plan, sensor_frame_handler=handler)
    except Exception as exc:
        result = {
            "status": "failed",
            "reason": "scene0061_live_tick_execution_failed",
            "detail": str(exc),
        }
    _write_json(result_path, result)
    validation = validate_live_tick_result(result, root)
    _write_json(validation_path, validation)
    environment["status"] = validation["status"]
    environment["execution_finished_at_utc"] = _utc_now()
    environment["validation"] = validation
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
    parser.add_argument("--sensor-handler-factory", default="adapters.nurec_260_client:build_nurec_260_handler")
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--execute-prepared", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.prepare_only and args.execute_prepared:
            raise Scene0061LiveTickError("--prepare-only and --execute-prepared are mutually exclusive")
        if args.execute_prepared:
            environment = verify_prepared_live_tick(args.output_dir)
            if (
                environment.get("run_id") != args.run_id
                or (environment.get("config") or {}).get("path") != str(args.config.expanduser().resolve())
                or (environment.get("opendrive") or {}).get("path") != str(args.opendrive.expanduser().resolve())
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
            )
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
