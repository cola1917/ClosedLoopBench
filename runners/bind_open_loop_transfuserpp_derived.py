"""Bind reusable TF++ outputs to a newer, auditable observation trace.

This tool is intentionally a derived-artifact operation.  It never runs the
model and never edits the source records.  It only succeeds when every source
frame can be proven to use the same RGB/LiDAR payload hashes and source-frame
binding as the target trace.  The target trace supplies the current dynamic
actor provenance and synchronization digest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import tempfile
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

from adapters.open_loop_bbox_binding import frame_binding, load_actor_manifest
from agents.plugin_contract import strict_json_loads
from agents.transfuserpp_contract import validate_intermediate_record
from metrics.open_loop import validate_open_loop_report


BINDING_SCHEMA = "transfuserpp_derived_binding.v1"
EXPECTED_SOURCES = frozenset(
    {
        "reconstructed_rgb_lidar",
        "harmonized_rgb_reconstructed_lidar",
    }
)


class DerivedBindingError(ValueError):
    """Raised when a reusable output cannot be bound without guessing."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise DerivedBindingError(f"cannot read {path}: {exc}") from exc
    return digest.hexdigest()


def bind_derived_artifacts(
    *,
    source_dir: Path,
    source_report: Path,
    target_trace: Path,
    actor_manifest_path: Path,
    evidence_root: Path,
    output_dir: Path,
    output_report: Path,
    binding_audit: Path,
    expected_source: str,
    run_id: str,
) -> dict[str, Any]:
    """Create a new derived intermediate directory and bound route report."""

    if expected_source not in EXPECTED_SOURCES:
        raise DerivedBindingError(f"unsupported derived input source: {expected_source}")
    source_dir = source_dir.resolve()
    source_report = source_report.resolve()
    target_trace = target_trace.resolve()
    actor_manifest_path = actor_manifest_path.resolve()
    evidence_root = evidence_root.resolve()
    output_dir = output_dir.resolve()
    output_report = output_report.resolve()
    binding_audit = binding_audit.resolve()
    for path, label in (
        (source_dir, "source intermediate directory"),
        (source_report, "source route report"),
        (target_trace, "target observation trace"),
        (actor_manifest_path, "actor manifest"),
        (evidence_root, "evidence root"),
    ):
        if not path.exists():
            raise DerivedBindingError(f"{label} is unavailable: {path}")
    if not source_dir.is_dir():
        raise DerivedBindingError(f"source intermediate path is not a directory: {source_dir}")
    if output_dir.exists() or output_report.exists() or binding_audit.exists():
        raise DerivedBindingError(
            "refusing to overwrite derived output: "
            + ", ".join(str(path) for path in (output_dir, output_report, binding_audit) if path.exists())
        )

    report = _load_object(source_report, "source route report")
    try:
        validate_open_loop_report(report)
    except ValueError as exc:
        raise DerivedBindingError(f"source route report is invalid: {exc}") from exc
    route = report.get("input_route")
    if not isinstance(route, Mapping) or route.get("route_id") not in {
        "reconstructed" if expected_source == "reconstructed_rgb_lidar" else "harmonized"
    }:
        raise DerivedBindingError("source route report does not match the expected derived source")

    trace = _load_object(target_trace, "target observation trace")
    observations = trace.get("frames")
    if not isinstance(observations, list) or not observations:
        raise DerivedBindingError("target observation trace has no frames")
    if trace.get("source") != expected_source:
        raise DerivedBindingError(
            f"target trace source mismatch: expected {expected_source}, observed {trace.get('source')}"
        )

    manifest = load_actor_manifest(
        actor_manifest_path,
        expected_scenario_ir_sha256=str(
            ((trace.get("scenario_ir") or {}).get("sha256")) or ""
        ),
        expected_scene_id=str((trace.get("capture") or {}).get("scene_id") or "") or None,
    )
    source_records = _load_records(source_dir)
    if len(source_records) != len(observations):
        raise DerivedBindingError(
            f"source/target frame count mismatch: source={len(source_records)} target={len(observations)}"
        )
    source_report_frames = report.get("frames")
    if not isinstance(source_report_frames, list) or len(source_report_frames) != len(observations):
        raise DerivedBindingError("source route report frame rows are incomplete")

    evidence_root.mkdir(parents=True, exist_ok=True)
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    output_report.parent.mkdir(parents=True, exist_ok=True)
    binding_audit.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output_dir.name}.staging-", dir=str(output_dir.parent)))
    derived_records: list[dict[str, Any]] = []
    report_frames: list[dict[str, Any]] = []
    source_dynamic_hashes: list[str] = []
    target_dynamic_hashes: list[str] = []
    try:
        for index, (source_record, observation, source_report_frame) in enumerate(
            zip(source_records, observations, source_report_frames)
        ):
            if not isinstance(observation, Mapping) or not isinstance(source_report_frame, Mapping):
                raise DerivedBindingError(f"frame {index} is not an object")
            _validate_frame_binding(
                source_record,
                observation,
                source_report_frame,
                index=index,
                expected_source=expected_source,
                manifest=manifest,
                evidence_root=evidence_root,
            )
            dense_source = _resolve_reference(
                source_record.get("dense_outputs"),
                evidence_root=evidence_root,
                label=f"source frame {index} dense output",
            )
            dense_declared_sha = str(
                ((source_record.get("dense_outputs") or {}).get("sha256")) or ""
            )
            _assert_hash(dense_source, dense_declared_sha, f"source frame {index} dense output")

            frame_name = f"frame_{index:08d}"
            dense_target = staging / f"{frame_name}.dense.npz"
            shutil.copy2(dense_source, dense_target)
            dense_sha = sha256_file(dense_target)
            if dense_sha != dense_declared_sha:
                raise DerivedBindingError(f"dense copy hash changed at frame {index}")

            derived = _rebind_record(
                source_record,
                observation,
                evidence_root=evidence_root,
                final_output_dir=output_dir,
                dense_target=dense_target,
                source_record_path=source_dir / f"{frame_name}.intermediate.json",
                target_trace=target_trace,
                actor_manifest_path=actor_manifest_path,
                actor_manifest=manifest,
                expected_source=expected_source,
                index=index,
            )
            try:
                validate_intermediate_record(derived)
            except ValueError as exc:
                raise DerivedBindingError(f"derived intermediate frame {index} is invalid: {exc}") from exc
            record_path = staging / f"{frame_name}.intermediate.json"
            _write_json(record_path, derived)
            record_sha = sha256_file(record_path)
            derived_records.append(
                {
                    "path": _container_path(record_path, evidence_root),
                    "host_path": str((output_dir / record_path.name).resolve()),
                    "relative_path": _relative_path(output_dir / record_path.name, evidence_root),
                    "sha256": record_sha,
                }
            )
            source_dynamic_hashes.append(
                str((source_record.get("synchronization") or {}).get("dynamic_object_sha256") or "")
            )
            target_dynamic_hashes.append(
                str((observation.get("synchronization") or {}).get("dynamic_object_sha256") or "")
            )

            bound_frame = deepcopy(dict(source_report_frame))
            bound_frame["timestamp"] = float(observation["timestamp"])
            bound_frame["input_provenance"] = deepcopy(dict(observation.get("provenance") or {}))
            bound_frame["input_payloads"] = {
                "camera_front": deepcopy(dict(((observation.get("rgb") or {}).get("camera_front")) or {})),
                "lidar_top": deepcopy(dict(observation.get("lidar") or {})),
            }
            bound_frame["intermediate_record_ref"] = derived_records[-1]
            report_frames.append(bound_frame)

        derived_report = _rebind_report(
            report,
            trace=trace,
            target_trace=target_trace,
            actor_manifest_path=actor_manifest_path,
            actor_manifest=manifest,
            report_frames=report_frames,
            evidence_root=evidence_root,
            run_id=run_id,
            expected_source=expected_source,
            intermediate_count=len(derived_records),
        )
        try:
            validate_open_loop_report(derived_report)
        except ValueError as exc:
            raise DerivedBindingError(f"derived route report is invalid: {exc}") from exc

        # Move the completed staging directory into the final location only
        # after every frame, dense payload, and report has passed validation.
        staging.rename(output_dir)
        for item in derived_records:
            item["host_path"] = str(output_dir / Path(item["host_path"]).name)
        for frame, item in zip(report_frames, derived_records):
            frame["intermediate_record_ref"] = item
        derived_report["frames"] = report_frames
        _write_json(output_report, derived_report)
        audit = {
            "schema_version": BINDING_SCHEMA,
            "status": "bound",
            "expected_source": expected_source,
            "frame_count": len(derived_records),
            "source_intermediate_dir": str(source_dir),
            "source_route_report": str(source_report),
            "target_trace": str(target_trace),
            "target_trace_sha256": sha256_file(target_trace),
            "actor_manifest": {
                "path": str(actor_manifest_path),
                "manifest_sha256": manifest["manifest_sha256"],
                "manifest_file_sha256": manifest.get("manifest_file_sha256"),
                "summary": deepcopy(manifest.get("summary")),
            },
            "output_intermediate_dir": str(output_dir),
            "output_route_report": str(output_report),
            "output_route_report_sha256": sha256_file(output_report),
            "source_payload_hashes_verified": True,
            "source_frame_bindings_verified": True,
            "dynamic_object_hash_rebound": source_dynamic_hashes != target_dynamic_hashes,
            "source_dynamic_object_sha256_sequence": source_dynamic_hashes,
            "target_dynamic_object_sha256_sequence": target_dynamic_hashes,
            "dense_file_count": len(derived_records),
            "derived_records": derived_records,
        }
        _write_json(binding_audit, audit)
        return audit
    except Exception:
        if staging.exists():
            shutil.rmtree(staging)
        raise


def _rebind_record(
    source_record: Mapping[str, Any],
    observation: Mapping[str, Any],
    *,
    evidence_root: Path,
    final_output_dir: Path,
    dense_target: Path,
    source_record_path: Path,
    target_trace: Path,
    actor_manifest_path: Path,
    actor_manifest: Mapping[str, Any],
    expected_source: str,
    index: int,
) -> dict[str, Any]:
    result = deepcopy(dict(source_record))
    inputs = deepcopy(dict(result.get("inputs") or {}))
    trace_rgb = deepcopy(dict(((observation.get("rgb") or {}).get("camera_front")) or {}))
    trace_lidar = deepcopy(dict(observation.get("lidar") or {}))
    trace_calibration = deepcopy(dict(observation.get("calibration") or {}))
    inputs["camera_front"] = trace_rgb
    inputs["lidar_top"] = trace_lidar
    inputs["calibration"] = trace_calibration
    ego_state = observation.get("ego_state") or {}
    route = observation.get("route") or {}
    pose = ego_state.get("pose") or {}
    inputs.update(
        {
            "route_command": route.get("route_command"),
            "target_point_ego_m": deepcopy(route.get("target_point_ego_m")),
            "route_progress_index": route.get("progress_index"),
            "route_target_distance_along_m": route.get("target_distance_along_route_m"),
            "route_lookahead_m": route.get("lookahead_m"),
            "speed_mps": ego_state.get("speed_mps"),
            "ego_pose": deepcopy(pose),
            "nurec_frame_id": index,
            "dynamic_object_sha256": (observation.get("synchronization") or {}).get(
                "dynamic_object_sha256"
            ),
        }
    )
    result["inputs"] = inputs
    result["frame_id"] = index
    result["timestamp"] = float(observation["timestamp"])
    result["provenance"] = deepcopy(dict(result.get("provenance") or {}))
    trace_provenance = deepcopy(dict(observation.get("provenance") or {}))
    result["provenance"].update(
        {
            "input_source": trace_provenance.get("input_source", expected_source),
            "input_variant": trace_provenance.get("input_variant"),
            "source_frame_binding": deepcopy(trace_provenance.get("source_frame_binding")),
            "actor_manifest": deepcopy(trace_provenance.get("actor_manifest")),
            "derived_binding": {
                "schema_version": BINDING_SCHEMA,
                "source_record_path": str(source_record_path),
                "source_record_sha256": sha256_file(source_record_path),
                "target_trace_path": str(target_trace),
                "target_trace_sha256": sha256_file(target_trace),
                "actor_manifest_path": str(actor_manifest_path),
                "actor_manifest_sha256": actor_manifest["manifest_sha256"],
                "payload_hashes_verified": True,
                "frame_id": index,
            },
        }
    )
    result["synchronization"] = deepcopy(dict(observation.get("synchronization") or {}))
    dense = deepcopy(dict(result.get("dense_outputs") or {}))
    final_dense = final_output_dir / dense_target.name
    dense["path"] = _container_path(final_dense, evidence_root)
    dense["host_path"] = str(final_dense.resolve())
    dense["relative_path"] = _relative_path(final_dense, evidence_root)
    dense["sha256"] = sha256_file(dense_target)
    result["dense_outputs"] = dense
    return result


def _rebind_report(
    source_report: Mapping[str, Any],
    *,
    trace: Mapping[str, Any],
    target_trace: Path,
    actor_manifest_path: Path,
    actor_manifest: Mapping[str, Any],
    report_frames: list[dict[str, Any]],
    evidence_root: Path,
    run_id: str,
    expected_source: str,
    intermediate_count: int,
) -> dict[str, Any]:
    result = deepcopy(dict(source_report))
    source_route = deepcopy(dict(result.get("input_route") or {}))
    route_id = "reconstructed" if expected_source == "reconstructed_rgb_lidar" else "harmonized"
    if source_route.get("route_id") != route_id:
        raise DerivedBindingError("source route report route identity changed during binding")
    trace_binding = deepcopy(dict(trace.get("input_binding") or {}))
    old_binding = result.get("input_binding") or {}
    trace_binding.update(
        {
            "route": source_route,
            "rgb_source": old_binding.get("rgb_source"),
            "lidar_source": old_binding.get("lidar_source"),
            "trace_lidar_source": trace_binding.get("lidar_source"),
            "harmonizer_rgb_only": source_route.get("harmonizer_rgb_only") is True,
            "same_frame_rgb_lidar_required": True,
            "same_frame_rgb_lidar_verified": True,
        }
    )
    result["run_id"] = run_id
    result["input_binding"] = trace_binding
    result["observation_trace_path"] = _container_path(target_trace, evidence_root)
    result["observation_trace_sha256"] = sha256_file(target_trace)
    result["actor_manifest"] = {
        "path": _container_path(actor_manifest_path, evidence_root),
        "sha256": actor_manifest["manifest_sha256"],
        "file_sha256": actor_manifest.get("manifest_file_sha256"),
        "summary": deepcopy(actor_manifest.get("summary")),
    }
    result["frames"] = report_frames
    result["tfpp"] = deepcopy(dict(result.get("tfpp") or {}))
    result["tfpp"].update(
        {
            "intermediate_count": intermediate_count,
            "fallback_count": 0,
            "derived_binding": {
                "schema_version": BINDING_SCHEMA,
                "target_trace_path": str(target_trace),
                "target_trace_sha256": sha256_file(target_trace),
                "actor_manifest_sha256": actor_manifest["manifest_sha256"],
                "source_payload_hashes_verified": True,
                "source_frame_bindings_verified": True,
            },
        }
    )
    return result


def _validate_frame_binding(
    source_record: Mapping[str, Any],
    observation: Mapping[str, Any],
    source_report_frame: Mapping[str, Any],
    *,
    index: int,
    expected_source: str,
    manifest: Mapping[str, Any],
    evidence_root: Path,
) -> None:
    if source_record.get("frame_id") != index or observation.get("frame_id") != index:
        raise DerivedBindingError(f"frame identity mismatch at {index}")
    if source_report_frame.get("frame_id") != index:
        raise DerivedBindingError(f"source report frame identity mismatch at {index}")
    if abs(float(source_record.get("timestamp")) - float(observation.get("timestamp"))) > 1e-6:
        raise DerivedBindingError(f"timestamp mismatch at {index}")
    source_provenance = source_record.get("provenance") or {}
    trace_provenance = observation.get("provenance") or {}
    if source_provenance.get("input_source") != expected_source:
        raise DerivedBindingError(f"source intermediate input source mismatch at {index}")
    if trace_provenance.get("input_source") != expected_source:
        raise DerivedBindingError(f"target trace input source mismatch at {index}")
    _validate_source_frame_binding(
        source_provenance.get("source_frame_binding"),
        trace_provenance.get("source_frame_binding"),
        index=index,
    )

    rgb = ((observation.get("rgb") or {}).get("camera_front"))
    lidar = observation.get("lidar")
    if not isinstance(rgb, Mapping) or not isinstance(lidar, Mapping):
        raise DerivedBindingError(f"target sensor payloads are incomplete at {index}")
    rgb_path = _resolve_reference(rgb, evidence_root=evidence_root, label=f"target frame {index} RGB")
    lidar_path = _resolve_reference(lidar, evidence_root=evidence_root, label=f"target frame {index} LiDAR")
    _assert_hash(rgb_path, str(rgb.get("sha256") or ""), f"target frame {index} RGB")
    _assert_hash(lidar_path, str(lidar.get("sha256") or ""), f"target frame {index} LiDAR")
    if rgb_path.parent != lidar_path.parent or rgb_path.parent.name != f"frame_{index:08d}":
        raise DerivedBindingError(f"target RGB/LiDAR are not same-frame payloads at {index}")
    source_inputs = source_record.get("inputs") or {}
    for name, target_ref in (("camera_front", rgb), ("lidar_top", lidar)):
        source_ref = source_inputs.get(name) or {}
        if source_ref.get("sha256") != target_ref.get("sha256"):
            raise DerivedBindingError(f"source/target {name} payload hash mismatch at {index}")
        source_path = _resolve_reference(
            source_ref, evidence_root=evidence_root, label=f"source frame {index} {name}"
        )
        _assert_hash(source_path, str(source_ref.get("sha256") or ""), f"source frame {index} {name}")

    actor_frame = frame_binding(manifest, index)
    actor_provenance = trace_provenance.get("actor_manifest")
    if not isinstance(actor_provenance, Mapping):
        raise DerivedBindingError(f"target actor manifest provenance is missing at {index}")
    expected_actor_fields = {
        "actor_manifest_sha256": manifest.get("manifest_sha256"),
        "actor_manifest_file_sha256": manifest.get("manifest_file_sha256"),
        "frame_id": index,
        "active_actor_ids": actor_frame["active_actor_ids"],
        "active_actor_set_sha256": actor_frame["active_actor_set_sha256"],
        "pose_digest": actor_frame["pose_digest"],
        "manifest_dynamic_object_sha256": actor_frame["dynamic_object_sha256"],
    }
    for key, expected in expected_actor_fields.items():
        if actor_provenance.get(key) != expected:
            raise DerivedBindingError(f"target actor manifest {key} mismatch at {index}")
    target_sync = observation.get("synchronization") or {}
    if target_sync.get("dynamic_object_sha256") != actor_frame["dynamic_object_sha256"]:
        raise DerivedBindingError(f"target dynamic object digest mismatch at {index}")


def _load_records(source_dir: Path) -> list[dict[str, Any]]:
    paths = sorted(source_dir.glob("*.intermediate.json"))
    if not paths:
        raise DerivedBindingError(f"no intermediate records in {source_dir}")
    records: list[dict[str, Any]] = []
    for path in paths:
        value = _load_object(path, f"source intermediate {path.name}")
        if not isinstance(value, dict):
            raise DerivedBindingError(f"source intermediate is not an object: {path}")
        records.append(value)
    records.sort(key=lambda item: int(item.get("frame_id", -1)))
    if [item.get("frame_id") for item in records] != list(range(len(records))):
        raise DerivedBindingError("source intermediate frame IDs are not contiguous")
    return records


def _load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = strict_json_loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise DerivedBindingError(f"cannot load {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise DerivedBindingError(f"{label} must be a JSON object")
    return value


def _resolve_reference(
    reference: Any, *, evidence_root: Path, label: str
) -> Path:
    if not isinstance(reference, Mapping):
        raise DerivedBindingError(f"{label} reference is missing")
    candidates = [str(reference.get("host_path") or ""), str(reference.get("path") or "")]
    relative = str(reference.get("relative_path") or "")
    if relative:
        candidates.append(str(evidence_root / Path(relative)))
    for value in candidates:
        if not value:
            continue
        path = Path(value)
        if path.is_file():
            return path.resolve()
        if value.startswith("/sim-data/"):
            mapped = (evidence_root / Path(value.removeprefix("/sim-data/"))).resolve()
            if mapped.is_file():
                return mapped
    raise DerivedBindingError(f"{label} payload is unavailable")


def _assert_hash(path: Path, expected: str, label: str) -> None:
    if len(expected) != 64 or sha256_file(path) != expected:
        raise DerivedBindingError(f"{label} SHA-256 mismatch: {path}")


def _validate_source_frame_binding(
    source_binding: Any,
    target_binding: Any,
    *,
    index: int,
) -> None:
    """Compare physical frame identity while allowing corrected mode labels.

    The first reconstructed TF++ run labeled the NuRec original-replay branch
    as ``original``.  The corrected r2 trace calls the same materialized
    payload ``reconstructed``.  The payload/source hashes and frame timing are
    the authoritative identity; the target trace's mode labels are copied into
    the derived record below.
    """

    if not isinstance(source_binding, Mapping) or not isinstance(target_binding, Mapping):
        raise DerivedBindingError(f"source-frame binding is missing at {index}")
    critical_fields = (
        "source_kind",
        "source_frame_index",
        "source_timestamp_us",
        "source_time_sec",
        "delta_us",
        "ir_frame_id",
        "ir_timestamp_sec",
        "rgb_source_sha256",
        "lidar_source_sha256",
        "rgb_materialized_sha256",
        "lidar_materialized_sha256",
    )
    for field in critical_fields:
        if source_binding.get(field) != target_binding.get(field):
            raise DerivedBindingError(f"source-frame binding mismatch at {index}: {field}")


def _relative_path(path: Path, evidence_root: Path) -> str:
    try:
        return path.resolve().relative_to(evidence_root.resolve()).as_posix()
    except ValueError:
        return path.name


def _container_path(path: Path, evidence_root: Path) -> str:
    relative = _relative_path(path, evidence_root)
    return f"/sim-data/{relative}"


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--source-report", type=Path, required=True)
    parser.add_argument("--target-trace", type=Path, required=True)
    parser.add_argument("--actor-manifest", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--output-report", type=Path, required=True)
    parser.add_argument("--binding-audit", type=Path, required=True)
    parser.add_argument("--expected-source", choices=sorted(EXPECTED_SOURCES), required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args(argv)
    try:
        audit = bind_derived_artifacts(
            source_dir=args.source_dir,
            source_report=args.source_report,
            target_trace=args.target_trace,
            actor_manifest_path=args.actor_manifest,
            evidence_root=args.evidence_root,
            output_dir=args.output_dir,
            output_report=args.output_report,
            binding_audit=args.binding_audit,
            expected_source=args.expected_source,
            run_id=args.run_id,
        )
    except (OSError, ValueError, TypeError, json.JSONDecodeError, DerivedBindingError) as exc:
        print(json.dumps({"status": "failed", "error": f"{type(exc).__name__}: {exc}"}))
        return 2
    print(json.dumps({"status": audit["status"], "frame_count": audit["frame_count"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
