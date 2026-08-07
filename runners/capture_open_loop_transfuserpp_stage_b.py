"""Capture NuRec RGB/LiDAR observations for TransFuser++ M6 Stage B.

Stage B is an open-loop replay. Scenario IR owns every ego pose, NuRec owns
the static scene render, and no CARLA dynamic actor is created. The resulting
trace keeps all six RGB responses and the raw/normalized LiDAR evidence while
the observation contract exposes camera_front and lidar_top to TF++.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_260_client import build_nurec_260_client
from adapters.nurec_multimodal import (
    NuRecMultimodalError,
    build_open_loop_nurec_multimodal_frame,
    validate_nurec_multimodal_evidence,
)
from adapters.open_loop_bbox_binding import (
    frame_binding,
    frame_dynamic_objects,
    load_actor_manifest,
)
from agents.plugin_contract import canonical_sha256
from agents.transfuserpp_contract import camera_adaptation_contract, validate_observation
from runners.capture_open_loop_transfuserpp_stage_a import _route_for_frame
from runners.run_open_loop_gt_replay import (
    EXPECTED_OPENDRIVE_SHA256,
    EXPECTED_SCENARIO_IR_SHA256,
    load_pinned_inputs,
)


TRACE_SCHEMA = "transfuserpp_stage_b_nurec_observation_trace.v1"
SOURCE = "nurec_stage_b_6cam_rgb_lidar"
FIXED_DELTA_SECONDS = 0.05
EXPECTED_CAMERA_IDS = (
    "camera_front",
    "camera_front_left",
    "camera_front_right",
    "camera_back",
    "camera_back_left",
    "camera_back_right",
)


class StageBCaptureError(RuntimeError):
    """Raised when M6 NuRec evidence cannot be made complete."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_new(path: Path, value: Any) -> None:
    if path.exists():
        raise StageBCaptureError(f"refusing to overwrite existing output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _load_json(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise StageBCaptureError(f"{label} is unavailable: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise StageBCaptureError(f"{label} is not valid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise StageBCaptureError(f"{label} must contain a JSON object: {path}")
    return value


def _normalise_track(value: Any) -> list[dict[str, float]]:
    if not isinstance(value, list) or len(value) < 2:
        raise StageBCaptureError("Scenario IR ego reference trajectory has fewer than two frames")
    result = []
    previous_t = -math.inf
    for index, row in enumerate(value):
        if not isinstance(row, Mapping):
            raise StageBCaptureError(f"ego reference frame {index} is not an object")
        state = {
            "t_sec": float(row["t_sec"]),
            "x": float(row["x"]),
            "y": float(row["y"]),
            "z": float(row.get("z", 0.0)),
            "yaw": float(row.get("yaw", 0.0)),
            "speed_mps": float(row.get("speed_mps", 0.0)),
        }
        if not all(math.isfinite(item) for item in state.values()):
            raise StageBCaptureError(f"ego reference frame {index} contains non-finite values")
        if state["t_sec"] < previous_t:
            raise StageBCaptureError("Scenario IR ego timestamps are not monotonic")
        previous_t = state["t_sec"]
        result.append(state)
    return result


def _state_at_time(track: list[dict[str, float]], t_sec: float) -> dict[str, float]:
    if t_sec <= track[0]["t_sec"]:
        return deepcopy(track[0])
    if t_sec >= track[-1]["t_sec"]:
        return deepcopy(track[-1])
    for left, right in zip(track, track[1:]):
        if left["t_sec"] <= t_sec <= right["t_sec"]:
            duration = right["t_sec"] - left["t_sec"]
            ratio = 0.0 if duration <= 0.0 else (t_sec - left["t_sec"]) / duration
            return {
                name: left[name] + ratio * (right[name] - left[name])
                for name in ("t_sec", "x", "y", "z", "yaw", "speed_mps")
            } | {"t_sec": float(t_sec)}
    return deepcopy(track[-1])


def _pose(state: Mapping[str, Any]) -> dict[str, float]:
    return {
        "x": float(state["x"]),
        "y": float(state["y"]),
        "z": float(state.get("z", 0.0)),
        "yaw": float(state.get("yaw", 0.0)),
    }


def _actor_frame_provenance(
    manifest: Mapping[str, Any],
    frame: Mapping[str, Any],
    manifest_path: Path,
) -> dict[str, Any]:
    """Create the external audit record kept beside the NuRec v1 payload."""

    return {
        "schema_version": "open_loop_bbox_dynamic_provenance.v1",
        "actor_manifest_path": str(manifest_path.resolve()),
        "actor_manifest_sha256": str(manifest["manifest_sha256"]),
        "actor_manifest_file_sha256": str(manifest.get("manifest_file_sha256") or _sha256(manifest_path)),
        "frame_id": int(frame["frame_id"]),
        "active_actor_ids": list(frame["active_actor_ids"]),
        "active_actor_set_sha256": str(frame["active_actor_set_sha256"]),
        "pose_digest": str(frame["pose_digest"]),
        "manifest_dynamic_object_sha256": str(frame["dynamic_object_sha256"]),
        "pose_source": "scenario_ir_actor_reference_trajectory",
        "track_binding_source": "formal_usdz_sequence_tracks.json",
        "coordinate_frame": "scene_local_ego_start_x_forward_y_left_z_up",
        "usdz_track_bindings": deepcopy(frame["usdz_track_bindings"]),
    }


def _container_payload_ref(
    materialized: Mapping[str, Any], output_dir: Path, container_root: str
) -> dict[str, Any]:
    path = Path(str(materialized.get("path") or "")).expanduser().resolve()
    if not path.is_file():
        raise StageBCaptureError(f"NuRec materialized payload is missing: {path}")
    relative = str(materialized.get("relative_path") or "")
    if not relative:
        try:
            relative = path.relative_to(output_dir.resolve()).as_posix()
        except ValueError as exc:
            raise StageBCaptureError(
                f"NuRec materialized payload is outside capture output: {path}"
            ) from exc
    result = dict(materialized)
    capture_name = output_dir.name
    if relative == capture_name or relative.startswith(f"{capture_name}/"):
        container_relative = relative
    else:
        container_relative = f"{capture_name}/{relative}"
    result["path"] = f"{container_root.rstrip('/')}/{container_relative}"
    result["relative_path"] = relative
    return result


def _record_by_sensor(evidence: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    records = evidence.get("records")
    if not isinstance(records, list):
        raise StageBCaptureError("NuRec evidence records are missing")
    result: dict[str, Mapping[str, Any]] = {}
    for record in records:
        if not isinstance(record, Mapping):
            raise StageBCaptureError("NuRec evidence contains a non-object record")
        sensor_id = str(record.get("sensor_id") or "")
        if not sensor_id or sensor_id in result:
            raise StageBCaptureError("NuRec evidence contains duplicate or empty sensor IDs")
        result[sensor_id] = record
    return result


def _payload_from_record(
    record: Mapping[str, Any],
    *,
    output_dir: Path,
    container_root: str,
    label: str,
) -> dict[str, Any]:
    if record.get("status") != "passed":
        raise StageBCaptureError(
            f"NuRec {label} did not pass: {record.get('issues') or 'unknown error'}"
        )
    metadata = record.get("response_metadata")
    materialized = metadata.get("materialized_payload") if isinstance(metadata, Mapping) else None
    if not isinstance(materialized, Mapping):
        raise StageBCaptureError(f"NuRec {label} has no materialized payload")
    return _container_payload_ref(materialized, output_dir, container_root)


def _containerize_evidence_payloads(
    evidence: Mapping[str, Any], *, output_dir: Path, container_root: str
) -> dict[str, Any]:
    """Rewrite evidence metadata paths to the path visible in the algorithm container."""

    result = deepcopy(dict(evidence))
    records = result.get("records")
    if not isinstance(records, list):
        raise StageBCaptureError("NuRec evidence records are missing")
    for record in records:
        if not isinstance(record, dict):
            raise StageBCaptureError("NuRec evidence contains a non-object record")
        metadata = record.get("response_metadata")
        if not isinstance(metadata, dict):
            continue
        for name in ("materialized_payload", "raw_response_payload"):
            payload = metadata.get(name)
            if isinstance(payload, Mapping):
                metadata[name] = _container_payload_ref(
                    payload,
                    output_dir=output_dir,
                    container_root=container_root,
                )
    return result


def _calibration(
    camera_specs: list[Mapping[str, Any]], lidar_specs: list[Mapping[str, Any]]
) -> dict[str, Any]:
    cameras = {str(item.get("sensor_id")): item for item in camera_specs}
    lidars = {str(item.get("sensor_id")): item for item in lidar_specs}
    front = cameras.get("camera_front")
    lidar = lidars.get("lidar_top")
    if front is None or lidar is None:
        raise StageBCaptureError("M6 sensor specs must contain camera_front and lidar_top")
    return {
        "camera_sensor_id": "camera_front",
        "camera_sensor_to_ego": list(front["sensor_to_ego"]),
        "camera_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
        "camera_intrinsic": {
            "source": "nurec_26_04_available_camera_resolution",
            "resolution_w": int(front.get("width", 1600)),
            "resolution_h": int(front.get("height", 900)),
            "focal_length": "not_advertised_by_runtime_inventory",
        },
        "lidar_sensor_id": "lidar_top",
        "lidar_sensor_to_ego": list(lidar["sensor_to_ego"]),
        "lidar_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
        "camera_adaptation": camera_adaptation_contract(),
    }


def _runtime_context(runtime_config: Mapping[str, Any]) -> dict[str, Any]:
    experiment = runtime_config.get("experiment")
    if not isinstance(experiment, Mapping):
        raise StageBCaptureError("runtime config has no experiment identity")
    required = (
        "artifact_sha256",
        "scene_package_sha256",
        "scenario_ir_sha256",
        "immutable_matrix_sha256",
        "source_run_config_sha256",
        "variant_config_sha256",
        "run_config_sha256",
    )
    identity = {name: experiment.get(name) for name in required}
    if any(not isinstance(value, str) or len(value) != 64 for value in identity.values()):
        raise StageBCaptureError("runtime config experiment identity is incomplete")
    return {
        "run_id": str(
            runtime_config.get("run_id")
            or f"scene0061-m6-stage-b-seed-{experiment.get('seed')}"
        ),
        "scene_id": experiment.get("scene_id"),
        "case_id": experiment.get("case_id"),
        "seed": experiment.get("seed"),
        "identity": identity,
    }


def _validate_runtime_inputs(
    runtime_config: Mapping[str, Any],
    scene_package: Mapping[str, Any],
    camera_specs: list[Mapping[str, Any]],
    lidar_specs: list[Mapping[str, Any]],
) -> None:
    nurec = runtime_config.get("nurec_runtime")
    if not isinstance(nurec, Mapping):
        raise StageBCaptureError("M6 runtime config requires nurec_runtime")
    if str(nurec.get("runtime_scene_id") or "") != "scene-0061":
        raise StageBCaptureError("M6 runtime_scene_id must be scene-0061")
    if scene_package.get("alignment", {}).get("status") != "runtime_validated":
        raise StageBCaptureError("M6 requires a runtime_validated Scene Package")
    camera_ids = [str(item.get("sensor_id") or "") for item in camera_specs]
    if tuple(sorted(camera_ids)) != tuple(sorted(EXPECTED_CAMERA_IDS)):
        raise StageBCaptureError(
            "M6 requires exactly the six formal NuRec cameras: "
            + ", ".join(EXPECTED_CAMERA_IDS)
        )
    if len(lidar_specs) != 1 or str(lidar_specs[0].get("sensor_id") or "") != "lidar_top":
        raise StageBCaptureError("M6 requires exactly one lidar_top spec")
    for item in camera_specs:
        if int(item.get("width", 0)) != 1600 or int(item.get("height", 0)) != 900:
            raise StageBCaptureError(f"M6 camera {item.get('sensor_id')} is not 1600x900")


def capture_stage_b(
    *,
    scenario_ir_path: Path,
    opendrive_path: Path,
    runtime_config_path: Path,
    actor_manifest_path: Path | None = None,
    output_dir: Path,
    expected_scenario_ir_sha256: str = EXPECTED_SCENARIO_IR_SHA256,
    expected_opendrive_sha256: str = EXPECTED_OPENDRIVE_SHA256,
    max_frames: int | None = None,
    nurec_concurrency: int = 7,
    nurec_max_attempts: int = 2,
    container_root: str = "/sim-data",
) -> dict[str, Any]:
    if output_dir.exists():
        raise StageBCaptureError(f"refusing to overwrite capture directory: {output_dir}")
    runtime_config = _load_json(runtime_config_path, "runtime config")
    inputs = load_pinned_inputs(
        scenario_ir_path,
        opendrive_path,
        expected_scenario_ir_sha256=expected_scenario_ir_sha256,
        expected_opendrive_sha256=expected_opendrive_sha256,
    )
    if actor_manifest_path is None:
        raise StageBCaptureError(
            "actor_manifest_path is required for actor-aware open-loop capture"
        )
    actor_manifest = load_actor_manifest(
        actor_manifest_path,
        expected_scenario_ir_sha256=inputs.scenario_ir_sha256,
        expected_scene_id=str(inputs.scenario_ir["scenario_id"]),
    )
    scene_package_path = Path(str((runtime_config.get("nurec_runtime") or {}).get("scene_package") or ""))
    scene_package = _load_json(scene_package_path, "NuRec Scene Package")
    nurec_runtime = runtime_config.get("nurec_runtime")
    if not isinstance(nurec_runtime, Mapping):
        raise StageBCaptureError("runtime config has no nurec_runtime")
    camera_specs = [dict(item) for item in nurec_runtime.get("camera_specs") or []]
    lidar_specs = [dict(item) for item in nurec_runtime.get("lidar_specs") or []]
    if any(not isinstance(item, dict) for item in camera_specs + lidar_specs):
        raise StageBCaptureError("M6 sensor specs must be objects")
    _validate_runtime_inputs(runtime_config, scene_package, camera_specs, lidar_specs)
    track = _normalise_track((inputs.scenario_ir.get("ego") or {}).get("reference_trajectory"))
    if max_frames is not None:
        if isinstance(max_frames, bool) or max_frames < 2:
            raise StageBCaptureError("max_frames must be at least two")
        track = track[:max_frames]
    if len(track) < 2:
        raise StageBCaptureError("M6 needs at least two GT frames")
    if len(actor_manifest["frames"]) < len(track):
        raise StageBCaptureError(
            "actor manifest has fewer frames than the Scenario IR ego replay"
        )

    run_context = _runtime_context(runtime_config)
    calibration = _calibration(camera_specs, lidar_specs)
    output_dir.mkdir(parents=True)
    payload_root = output_dir / "payloads"
    payload_root.mkdir()
    client = None
    observations: list[dict[str, Any]] = []
    evidence_rows: list[dict[str, Any]] = []

    try:
        client = build_nurec_260_client(runtime_config, payload_output_dir=payload_root)
        client._payload_reference_root = output_dir.resolve()  # type: ignore[attr-defined]
        client.concurrency = max(1, int(nurec_concurrency))
        client.max_attempts = max(1, int(nurec_max_attempts))
        client.native_scan_alignment_required = False
        inventory = client.query_runtime_inventory()
        advertised = {
            str(row.get("logical_id")): row for row in inventory.get("cameras") or []
        }
        for sensor_id in EXPECTED_CAMERA_IDS:
            row = advertised.get(sensor_id)
            if row is None or int(row.get("resolution_w", 0)) != 1600 or int(row.get("resolution_h", 0)) != 900:
                raise StageBCaptureError(f"NuRec inventory does not advertise formal camera {sensor_id}")
        _write_new(output_dir / "nurec_runtime_inventory.json", inventory)

        for frame_id, state in enumerate(track):
            interval_start = max(0.0, float(state["t_sec"]) - FIXED_DELTA_SECONDS)
            start_state = _state_at_time(track, interval_start)
            ego_pose_pair = {
                "start": _pose(start_state),
                "end": _pose(state),
            }
            actor_frame = frame_binding(actor_manifest, frame_id)
            actor_provenance = _actor_frame_provenance(
                actor_manifest,
                actor_frame,
                actor_manifest_path,
            )
            frame = build_open_loop_nurec_multimodal_frame(
                scene_package,
                frame_id=frame_id,
                simulation_time_sec=float(state["t_sec"]),
                interval_start_sec=interval_start,
                ego_pose_pair=ego_pose_pair,
                camera_specs=camera_specs,
                lidar_specs=lidar_specs,
                dynamic_objects=frame_dynamic_objects(actor_manifest, frame_id),
                dynamic_object_provenance=actor_provenance,
            )
            live_evidence = client.dispatch_frame(frame)
            validate_nurec_multimodal_evidence(live_evidence)
            if live_evidence.get("status") != "passed":
                raise StageBCaptureError(
                    f"NuRec frame {frame_id} failed: {live_evidence.get('issues') or 'unknown error'}"
                )
            # Extract files while the client still exposes host paths, then
            # persist the evidence with the container-visible paths used by
            # the later TF++ runner.
            records = _record_by_sensor(live_evidence)
            payloads = {
                sensor_id: _payload_from_record(
                    records[sensor_id],
                    output_dir=output_dir,
                    container_root=container_root,
                    label=f"frame {frame_id} {sensor_id}",
                )
                for sensor_id in (*EXPECTED_CAMERA_IDS, "lidar_top")
            }
            # The normalized LiDAR payload is consumed by the TF++ observation
            # contract, which requires the exact extrinsic beside the bytes so
            # sensor-local coordinates cannot be interpreted ambiguously.
            payloads["lidar_top"]["sensor_to_ego"] = list(
                calibration["lidar_sensor_to_ego"]
            )
            raw_lidar = records["lidar_top"].get("response_metadata", {}).get(
                "raw_response_payload"
            )
            if not isinstance(raw_lidar, Mapping):
                raise StageBCaptureError(f"NuRec frame {frame_id} has no raw LiDAR payload")
            raw_lidar_ref = _container_payload_ref(raw_lidar, output_dir, container_root)
            evidence = _containerize_evidence_payloads(
                live_evidence,
                output_dir=output_dir,
                container_root=container_root,
            )
            validate_nurec_multimodal_evidence(evidence)
            observation = {
                "schema_version": "transfuserpp_observation.v1",
                "observation_id": f"{run_context['run_id']}-frame-{frame_id:08d}",
                "source": SOURCE,
                "frame_id": frame_id,
                "timestamp": float(state["t_sec"]),
                "rgb": {"camera_front": payloads["camera_front"]},
                "lidar": payloads["lidar_top"],
                "sensor_validity": {
                    **{sensor_id: True for sensor_id in EXPECTED_CAMERA_IDS},
                    "lidar": True,
                },
                "calibration": deepcopy(calibration),
                "ego_state": {
                    "pose": _pose(state),
                    "speed_mps": float(state["speed_mps"]),
                    "speed_source": "scenario_ir_reference_trajectory",
                    "compass_source": "scenario_ir_reference_trajectory",
                },
                "route": _route_for_frame(track, frame_id),
                "synchronization": {
                    "frame_id": frame_id,
                    "clock": "scenario_ir_reference_trajectory",
                    "error_ms": 0.0,
                    "dynamic_object_sha256": frame["shared_dynamic_object_sha256"],
                    "sensor_age_ticks": 0,
                },
                "run_context": deepcopy(run_context),
                "provenance": {
                    "gt_pose_replay": True,
                    "pose_source": "scenario_ir_reference_trajectory",
                    "control_applied": False,
                    "control_affects_next_ego_pose": False,
                    "carla_dynamic_actor_creation": False,
                    "carla_dynamic_actor_count": 0,
                    "nurec_dynamic_object_injection": True,
                    "nurec_dynamic_object_count": len(frame["shared_dynamic_objects"]),
                    "actor_manifest": deepcopy(actor_provenance),
                    "nurec_runtime_scene_id": client.runtime_scene_id,
                    "nurec_api": "SensorsimService/26.04",
                    "nurec_frame_sha256": canonical_sha256(frame),
                    "nurec_evidence_sha256": canonical_sha256(evidence),
                    "ir_pose_sha256": canonical_sha256(state),
                },
                "nurec_sensor_payloads": {
                    sensor_id: payloads[sensor_id] for sensor_id in EXPECTED_CAMERA_IDS
                }
                | {
                    "lidar_top_raw_response": raw_lidar_ref,
                    "lidar_top_normalized": payloads["lidar_top"],
                },
                "nurec_frame": frame,
                "nurec_evidence": evidence,
            }
            validate_observation(observation)
            observations.append(observation)
            evidence_rows.append(
                {
                    "frame_id": frame_id,
                    "simulation_time_sec": state["t_sec"],
                    "frame_sha256": canonical_sha256(frame),
                    "evidence_sha256": canonical_sha256(evidence),
                    "dynamic_object_count": evidence["dynamic_object_count"],
                    "records": deepcopy(evidence["records"]),
                }
            )
    except (OSError, ValueError, TypeError, NuRecMultimodalError) as exc:
        raise StageBCaptureError(str(exc)) from exc
    finally:
        if client is not None:
            client.close()

    inventory_path = output_dir / "nurec_runtime_inventory.json"
    trace = {
        "schema_version": TRACE_SCHEMA,
        "source": SOURCE,
        "scenario_ir": {
            "path": str(inputs.scenario_ir_path),
            "sha256": inputs.scenario_ir_sha256,
        },
        "opendrive": {
            "path": str(inputs.opendrive_path),
            "sha256": inputs.opendrive_sha256,
        },
        "runtime_config": {
            "path": str(runtime_config_path.resolve()),
            "sha256": _sha256(runtime_config_path),
        },
        "scene_package": {
            "path": str(scene_package_path.resolve()),
            "sha256": _sha256(scene_package_path),
            "alignment_status": scene_package["alignment"]["status"],
        },
        "nurec": {
            "runtime_scene_id": str(nurec_runtime["runtime_scene_id"]),
            "target": str(nurec_runtime.get("target") or ""),
            "api": "SensorsimService/26.04",
            "artifact_sha256": run_context["identity"]["artifact_sha256"],
            "camera_ids": list(EXPECTED_CAMERA_IDS),
            "lidar_ids": ["lidar_top"],
            "dynamic_actor_creation": False,
            "dynamic_object_injection": True,
            "dynamic_object_count_max": max(
                len(item["nurec_frame"]["shared_dynamic_objects"])
                for item in observations
            ),
            "actor_manifest": {
                "path": str(actor_manifest_path.resolve()),
                "sha256": actor_manifest["manifest_sha256"],
                "file_sha256": actor_manifest["manifest_file_sha256"],
            },
            "inventory": {
                "path": str(inventory_path),
                "sha256": _sha256(inventory_path),
            },
            "payload_root": str(payload_root),
        },
        "capture": {
            "ego_pose_owner": "scenario_ir_reference_trajectory",
            "control_affects_next_ego_pose": False,
            "carla_dynamic_actor_creation": False,
            "nurec_dynamic_object_injection": True,
            "fixed_logical_interval_seconds": FIXED_DELTA_SECONDS,
            "camera_resolution": {"width": 1600, "height": 900},
            "camera_count": 6,
            "lidar_sensor": "lidar_top",
            "lidar_axis_target": "sensor_local/carla_sensor",
        },
        "frames": observations,
        "evidence_rows": evidence_rows,
        "actor_manifest": {
            "path": str(actor_manifest_path.resolve()),
            "sha256": actor_manifest["manifest_sha256"],
            "file_sha256": actor_manifest["manifest_file_sha256"],
            "summary": deepcopy(actor_manifest["summary"]),
        },
    }
    trace_path = output_dir / "nurec_stage_b_observations.json"
    _write_new(trace_path, trace)
    nurec_trace_path = output_dir / "nurec_multimodal_trace.jsonl"
    if nurec_trace_path.exists():
        raise StageBCaptureError(f"refusing to overwrite existing output: {nurec_trace_path}")
    with nurec_trace_path.open("w", encoding="utf-8") as stream:
        for observation in observations:
            stream.write(
                json.dumps(
                    {
                        "frame_id": observation["frame_id"],
                        "simulation_time_sec": observation["timestamp"],
                        "frame": observation["nurec_frame"],
                        "evidence": observation["nurec_evidence"],
                    },
                    ensure_ascii=False,
                    allow_nan=False,
                )
                + "\n"
            )
    return {
        "status": "captured",
        "trace": str(trace_path),
        "nurec_trace": str(nurec_trace_path),
        "frame_count": len(observations),
        "camera_count": len(EXPECTED_CAMERA_IDS),
        "lidar_count": 1,
        "dynamic_actor_creation": False,
        "dynamic_object_injection": True,
        "dynamic_object_count_max": max(
            len(item["nurec_frame"]["shared_dynamic_objects"])
            for item in observations
        ),
        "payload_root": str(payload_root),
        "trace_sha256": _sha256(trace_path),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-ir", type=Path, required=True)
    parser.add_argument("--opendrive", type=Path, required=True)
    parser.add_argument("--runtime-config", type=Path, required=True)
    parser.add_argument("--actor-manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-scenario-ir-sha256", default=EXPECTED_SCENARIO_IR_SHA256)
    parser.add_argument("--expected-opendrive-sha256", default=EXPECTED_OPENDRIVE_SHA256)
    parser.add_argument("--max-frames", type=int, default=3)
    parser.add_argument("--nurec-concurrency", type=int, default=7)
    parser.add_argument("--nurec-max-attempts", type=int, default=2)
    parser.add_argument("--container-root", default="/sim-data")
    args = parser.parse_args(argv)
    try:
        result = capture_stage_b(
            scenario_ir_path=args.scenario_ir,
            opendrive_path=args.opendrive,
            runtime_config_path=args.runtime_config,
            actor_manifest_path=args.actor_manifest,
            output_dir=args.output_dir,
            expected_scenario_ir_sha256=args.expected_scenario_ir_sha256,
            expected_opendrive_sha256=args.expected_opendrive_sha256,
            max_frames=args.max_frames,
            nurec_concurrency=args.nurec_concurrency,
            nurec_max_attempts=args.nurec_max_attempts,
            container_root=args.container_root,
        )
    except (OSError, ValueError, RuntimeError, StageBCaptureError) as exc:
        print(json.dumps({"status": "failed", "error": f"{type(exc).__name__}: {exc}"}))
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
