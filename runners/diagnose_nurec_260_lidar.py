from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback
from typing import Any, Mapping
import zipfile


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_260_client import (  # noqa: E402
    NuRec260Client,
    _protobuf_wire_fields,
    build_nurec_260_client,
)
from adapters.nurec_multimodal import (  # noqa: E402
    NuRecMultimodalError,
    materialize_nurec_rpc_requests,
    validate_nurec_multimodal_frame,
)


SUPPORTED_DEVICE_TYPES = ("PANDAR128", "AT128")


def diagnose_lidar(
    config_path: Path,
    artifact_path: Path,
    baseline_frame_path: Path,
    moved_frame_path: Path,
    *,
    targets: list[str],
    device_types: list[str],
    client_factory=build_nurec_260_client,
) -> dict[str, Any]:
    config = _load_object(config_path)
    baseline_bytes = baseline_frame_path.read_bytes()
    moved_bytes = moved_frame_path.read_bytes()
    baseline = json.loads(baseline_bytes)
    moved = json.loads(moved_bytes)
    if not isinstance(baseline, dict) or not isinstance(moved, dict):
        raise ValueError("baseline and moved frames must contain JSON objects")
    validate_nurec_multimodal_frame(baseline)
    validate_nurec_multimodal_frame(moved)

    runtime = config.get("nurec_runtime")
    if not isinstance(runtime, Mapping):
        raise ValueError("config requires nurec_runtime")
    scene_start_us = int(runtime["scene_start_us"])
    artifact = _load_artifact_runtime(artifact_path)
    selected_scan = _select_native_scan(
        artifact,
        scene_start_us
        + int(round(float(baseline["simulation_time_sec"]) * 1_000_000)),
    )
    coordinate_check = _coordinate_check(baseline, moved, artifact, selected_scan)

    target_rows = []
    for target in targets:
        target_config = deepcopy(config)
        target_config.setdefault("nurec_runtime", {})["target"] = target
        row: dict[str, Any] = {"target": target, "cases": []}
        client: NuRec260Client | None = None
        try:
            client = client_factory(target_config)
            row["protobuf_boundary"] = _protobuf_boundary(client)
            row["cases"].append(
                _run_capability_case(client, artifact, selected_scan)
            )
            row["cases"].append(
                _run_rgb_case(client, baseline, artifact, selected_scan)
            )
            for device_type in device_types:
                row["cases"].extend(
                    _run_lidar_cases(
                        client,
                        baseline,
                        moved,
                        artifact,
                        selected_scan,
                        device_type,
                    )
                )
        except Exception as exc:
            row["setup_exception"] = _exception_record(exc)
        finally:
            if client is not None:
                client.close()
        row["status"] = (
            "passed"
            if row.get("cases")
            and not row.get("setup_exception")
            and all(case.get("status") == "passed" for case in row["cases"])
            else "failed"
        )
        target_rows.append(row)

    return {
        "schema_version": "closed_loopbench.nurec_260_lidar_diagnostic.v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "inputs": {
            "config": str(config_path.resolve()),
            "artifact": str(artifact_path.resolve()),
            "artifact_sha256": _sha256_file(artifact_path),
            "baseline_frame": str(baseline_frame_path.resolve()),
            "baseline_frame_sha256": hashlib.sha256(baseline_bytes).hexdigest(),
            "moved_frame": str(moved_frame_path.resolve()),
            "moved_frame_sha256": hashlib.sha256(moved_bytes).hexdigest(),
            "targets": targets,
            "device_types": device_types,
        },
        "artifact": {
            "sequence_id": artifact["sequence_id"],
            "timestamp_range_us": artifact["timestamp_range_us"],
            "lidar_id": artifact["lidar_id"],
            "source_lidar_model": artifact["source_lidar_model"],
            "source_lidar_model_parameters": artifact[
                "source_lidar_model_parameters"
            ],
            "selected_scan": selected_scan["summary"],
        },
        "coordinate_and_input_checks": coordinate_check,
        "targets": target_rows,
        "status": (
            "passed"
            if target_rows and all(row["status"] == "passed" for row in target_rows)
            else "failed"
        ),
    }


def _run_capability_case(
    client: NuRec260Client,
    artifact: Mapping[str, Any],
    selected_scan: Mapping[str, Any],
) -> dict[str, Any]:
    started = time.monotonic()
    case = {
        "case": "capability_only",
        "sequence": 1,
        "request": {
            "runtime_scene_id": client.runtime_scene_id,
            "selected_native_scan": selected_scan["summary"],
        },
    }
    try:
        inventory = client.query_runtime_inventory()
        trajectories = client.stub.get_available_trajectories(
            client._pb.AvailableTrajectoriesRequest(scene_id=client.runtime_scene_id),
            timeout=client.timeout_sec,
        )
        trajectory_ranges = []
        for item in trajectories.available_trajectories:
            poses = list(item.trajectory.poses)
            timestamps = [int(pose.timestamp_us) for pose in poses]
            trajectory_ranges.append(
                {
                    "trajectory_idx": int(item.trajectory_idx),
                    "pose_count": len(poses),
                    "first_timestamp_us": min(timestamps) if timestamps else None,
                    "last_timestamp_us": max(timestamps) if timestamps else None,
                }
            )
        case["response"] = {
            "inventory": inventory,
            "trajectory_ranges": trajectory_ranges,
            "artifact_range_matches_selected_scan": _timestamp_inside(
                selected_scan["start_us"],
                artifact["timestamp_range_us"],
            )
            and _timestamp_inside(
                selected_scan["end_us"],
                artifact["timestamp_range_us"],
                include_stop=True,
            ),
        }
        case["status"] = "passed"
    except Exception as exc:
        case["exception"] = _exception_record(exc)
        case["status"] = "failed"
    case["latency_ms"] = (time.monotonic() - started) * 1000.0
    return case


def _run_rgb_case(
    client: NuRec260Client,
    baseline: Mapping[str, Any],
    artifact: Mapping[str, Any],
    selected_scan: Mapping[str, Any],
) -> dict[str, Any]:
    rgb_payload = next(
        payload
        for payload in materialize_nurec_rpc_requests(baseline)
        if payload["modality"] == "rgb"
    )
    camera_id = str(rgb_payload["sensor"]["sensor_id"])
    camera_pair = _native_sensor_pose_pair(
        artifact, selected_scan, "camera", camera_id
    )
    payload = deepcopy(rgb_payload)
    payload["pose_interval_sec"] = _relative_interval(client, selected_scan)
    payload["sensor"]["pose_pair"] = camera_pair
    payload["dynamic_objects"] = []
    payload["sensor"]["parameters"] = {"width": 800, "height": 450}
    return _run_rpc_case(
        client,
        payload,
        case_name="rgb_only",
        sequence=2,
        device_type=None,
        pose_source="artifact_native_rig_times_camera_extrinsic",
    )


def _run_lidar_cases(
    client: NuRec260Client,
    baseline: Mapping[str, Any],
    moved: Mapping[str, Any],
    artifact: Mapping[str, Any],
    selected_scan: Mapping[str, Any],
    device_type: str,
) -> list[dict[str, Any]]:
    lidar_payload = next(
        payload
        for payload in materialize_nurec_rpc_requests(baseline)
        if payload["modality"] == "lidar"
    )
    moved_payload = next(
        payload
        for payload in materialize_nurec_rpc_requests(moved)
        if payload["modality"] == "lidar"
    )
    sensor_id = str(lidar_payload["sensor"]["sensor_id"])
    native_pair = _native_sensor_pose_pair(
        artifact, selected_scan, "lidar", sensor_id
    )
    relative_interval = _relative_interval(client, selected_scan)

    no_actors = deepcopy(lidar_payload)
    no_actors["pose_interval_sec"] = relative_interval
    no_actors["sensor"]["pose_pair"] = _identity_pose_pair()
    no_actors["sensor"]["parameters"] = {"device_type": device_type}
    no_actors["dynamic_objects"] = []

    native_sensor = deepcopy(no_actors)
    native_sensor["sensor"]["pose_pair"] = native_pair

    replay_actors = deepcopy(native_sensor)
    replay_actors["dynamic_objects"] = _transform_dynamic_objects_to_nre(
        lidar_payload["dynamic_objects"], artifact["nre_from_log"]
    )

    moved_actor = deepcopy(native_sensor)
    moved_actor["dynamic_objects"] = _transform_dynamic_objects_to_nre(
        moved_payload["dynamic_objects"], artifact["nre_from_log"]
    )

    base_sequence = 3 if device_type == "PANDAR128" else 7
    return [
        _run_rpc_case(
            client,
            no_actors,
            case_name="lidar_only_without_dynamic_actors",
            sequence=base_sequence,
            device_type=device_type,
            pose_source="identity_pose_control",
        ),
        _run_rpc_case(
            client,
            native_sensor,
            case_name="lidar_with_ego_sensor_pose",
            sequence=base_sequence + 1,
            device_type=device_type,
            pose_source="artifact_native_rig_times_lidar_extrinsic",
        ),
        _run_rpc_case(
            client,
            replay_actors,
            case_name="lidar_with_replay_actors",
            sequence=base_sequence + 2,
            device_type=device_type,
            pose_source="artifact_native_sensor_plus_log_to_nre_actor_poses",
        ),
        _run_rpc_case(
            client,
            moved_actor,
            case_name="lidar_with_moved_target_actor",
            sequence=base_sequence + 3,
            device_type=device_type,
            pose_source="artifact_native_sensor_plus_log_to_nre_moved_actor_pose",
        ),
    ]


def _run_rpc_case(
    client: NuRec260Client,
    payload: Mapping[str, Any],
    *,
    case_name: str,
    sequence: int,
    device_type: str | None,
    pose_source: str,
) -> dict[str, Any]:
    case: dict[str, Any] = {
        "case": case_name,
        "sequence": sequence,
        "modality": payload["modality"],
        "device_type": device_type,
        "pose_source": pose_source,
    }
    started = time.monotonic()
    try:
        encoder = client.encode_rgb if payload["modality"] == "rgb" else client.encode_lidar
        rpc = client.render_rgb if payload["modality"] == "rgb" else client.render_lidar
        encoded = encoder(payload)
        request = encoded["wire_request"]
        request_body = request.SerializeToString()
        case["request"] = _request_summary(client, payload, request, request_body)
        response = rpc(request)
        body = client.response_bytes(response)
        metadata = client.inspect_response(payload, response, body)
        case["response"] = {
            "serialized_bytes": len(body),
            "payload_sha256": hashlib.sha256(body).hexdigest(),
            "metadata": dict(metadata),
            "wire_layout": _response_wire_layout(response, body),
        }
        case["status"] = "passed"
    except Exception as exc:
        case["exception"] = _exception_record(exc)
        case["status"] = "failed"
    case["latency_ms"] = (time.monotonic() - started) * 1000.0
    return case


def _request_summary(
    client: NuRec260Client,
    payload: Mapping[str, Any],
    request: Any,
    request_body: bytes,
) -> dict[str, Any]:
    interval = payload["pose_interval_sec"]
    frame_start_us = client.scene_start_us + int(
        round(float(interval["start"]) * 1_000_000)
    )
    frame_end_us = client.scene_start_us + int(
        round(float(interval["end"]) * 1_000_000)
    )
    frame_end_us = max(frame_start_us + 1, frame_end_us)
    return {
        "scene_id": client.runtime_scene_id,
        "frame_start_us": frame_start_us,
        "frame_end_us": frame_end_us,
        "duration_us": frame_end_us - frame_start_us,
        "sensor_id": payload["sensor"]["sensor_id"],
        "sensor_model": payload["sensor"]["model"],
        "sensor_parameters": deepcopy(payload["sensor"].get("parameters") or {}),
        "sensor_pose_pair": deepcopy(payload["sensor"]["pose_pair"]),
        "dynamic_object_count": len(payload["dynamic_objects"]),
        "dynamic_track_ids": [
            str(item["track_id"]) for item in payload["dynamic_objects"]
        ],
        "dynamic_objects": deepcopy(payload["dynamic_objects"]),
        "serialized_bytes": len(request_body),
        "request_sha256": hashlib.sha256(request_body).hexdigest(),
        "protobuf_type": request.DESCRIPTOR.full_name,
    }


def _response_wire_layout(response: Any, body: bytes) -> dict[str, Any]:
    legacy_xyz = getattr(response, "point_xyzs", ())
    legacy_intensity = getattr(response, "point_intensities", ())
    result: dict[str, Any] = {
        "client_known_point_xyz_float_count": len(legacy_xyz),
        "client_known_intensity_float_count": len(legacy_intensity),
    }
    if not body:
        result["top_level_fields"] = []
        return result
    fields = _protobuf_wire_fields(body)
    rows = []
    for field_number in sorted(fields):
        for value in fields[field_number]:
            row: dict[str, Any] = {"field_number": field_number}
            if isinstance(value, int):
                row.update(wire_type="varint", value=value)
            else:
                row.update(
                    wire_type="bytes_or_fixed",
                    byte_count=len(value),
                    sha256=hashlib.sha256(value).hexdigest(),
                    prefix_hex=value[:32].hex(),
                )
            rows.append(row)
    result["top_level_fields"] = rows
    if fields.get(3) and isinstance(fields[3][0], int):
        result["buffered_num_points"] = fields[3][0]
    if fields.get(4) and isinstance(fields[4][0], bytes):
        result["point_xyzs_buffer_bytes"] = len(fields[4][0])
    if fields.get(5) and isinstance(fields[5][0], bytes):
        result["point_intensities_buffer_bytes"] = len(fields[5][0])
    return result


def _protobuf_boundary(client: NuRec260Client) -> dict[str, Any]:
    # The generated unary-unary callable does not expose a stable
    # ``_response_deserializer`` attribute across grpcio versions.  Inspect
    # the generated message descriptor directly instead; this is also the
    # descriptor the client actually uses to parse the response boundary.
    message_type = getattr(client._pb, "LidarRenderReturn", None)
    message_descriptor = getattr(message_type, "DESCRIPTOR", None)
    if message_descriptor is None:
        raise RuntimeError("client protobuf module has no LidarRenderReturn descriptor")
    return {
        "client_lidar_return": message_descriptor.full_name,
        "client_lidar_return_fields": [
            {"name": field.name, "number": field.number, "type": field.type}
            for field in message_descriptor.fields
        ],
        "expected_server_26_04_buffer_fields": [
            {"name": "num_points", "number": 3, "wire": "varint"},
            {"name": "point_xyzs_buffer", "number": 4, "wire": "bytes"},
            {"name": "point_intensities_buffer", "number": 5, "wire": "bytes"},
        ],
        "compatibility_decoder": "adapters.nurec_260_client._inspect_lidar_response",
    }


def _load_artifact_runtime(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"artifact does not exist: {path}")
    with zipfile.ZipFile(path) as archive:
        data_info = json.loads(archive.read("data_info.json"))
        rig = json.loads(archive.read("rig_trajectories.json"))
    interval = data_info["sequence_timestamp_interval_us"]
    trajectory = rig["rig_trajectories"][0]
    lidar_key, lidar_calibration = _calibration(rig, "lidar", "lidar_top")
    source_parameters = lidar_calibration["lidar_model"]["parameters"]
    return {
        "sequence_id": str(data_info["sequence_id"]),
        "timestamp_range_us": {
            "start": int(interval["start"]),
            "stop": int(interval.get("stop", interval.get("end"))),
        },
        "rig": rig,
        "trajectory": trajectory,
        "lidar_id": str(lidar_calibration["logical_sensor_name"]),
        "lidar_key": lidar_key,
        "source_lidar_model": str(lidar_calibration["lidar_model"]["type"]),
        "source_lidar_model_parameters": {
            "spinning_frequency_hz": float(source_parameters["spinning_frequency_hz"]),
            "spinning_direction": str(source_parameters["spinning_direction"]),
            "n_rows": int(source_parameters["n_rows"]),
            "n_columns": int(source_parameters["n_columns"]),
        },
        "nre_from_log": _invert_rigid(rig["T_world_base"]),
    }


def _select_native_scan(
    artifact: Mapping[str, Any], reference_us: int
) -> dict[str, Any]:
    trajectory = artifact["trajectory"]
    intervals = trajectory["lidars_frame_timestamps_us"][artifact["lidar_key"]]
    index = next(
        (
            index
            for index, interval in enumerate(intervals)
            if int(interval[0]) <= reference_us < int(interval[1])
        ),
        None,
    )
    if index is None:
        index = min(
            range(len(intervals)),
            key=lambda candidate: abs(
                (int(intervals[candidate][0]) + int(intervals[candidate][1])) // 2
                - reference_us
            ),
        )
    start_us, end_us = (int(value) for value in intervals[index])
    timestamps = [int(value) for value in trajectory["T_rig_world_timestamps_us"]]
    timestamp_index = {value: position for position, value in enumerate(timestamps)}
    if start_us not in timestamp_index or end_us not in timestamp_index:
        raise ValueError("native LiDAR scan endpoints lack exact rig poses")
    start_rig = trajectory["T_rig_worlds"][timestamp_index[start_us]]
    end_rig = trajectory["T_rig_worlds"][timestamp_index[end_us]]
    return {
        "index": index,
        "start_us": start_us,
        "end_us": end_us,
        "start_rig": start_rig,
        "end_rig": end_rig,
        "summary": {
            "index": index,
            "reference_us": reference_us,
            "start_us": start_us,
            "end_us": end_us,
            "duration_us": end_us - start_us,
            "reference_offset_from_start_us": reference_us - start_us,
        },
    }


def _native_sensor_pose_pair(
    artifact: Mapping[str, Any],
    selected_scan: Mapping[str, Any],
    modality: str,
    sensor_id: str,
) -> dict[str, Any]:
    _, calibration = _calibration(artifact["rig"], modality, sensor_id)
    sensor_to_rig = calibration["T_sensor_rig"]
    return {
        "start": _matrix_to_pose(
            _matmul(selected_scan["start_rig"], sensor_to_rig)
        ),
        "end": _matrix_to_pose(_matmul(selected_scan["end_rig"], sensor_to_rig)),
    }


def _calibration(
    rig: Mapping[str, Any], modality: str, sensor_id: str
) -> tuple[str, Mapping[str, Any]]:
    table_name = "camera_calibrations" if modality == "camera" else "lidar_calibrations"
    matches = [
        (key, value)
        for key, value in rig[table_name].items()
        if str(value.get("logical_sensor_name")) == sensor_id
    ]
    if len(matches) != 1:
        raise ValueError(
            f"expected one {modality} calibration for {sensor_id}, found {len(matches)}"
        )
    return matches[0]


def _relative_interval(
    client: NuRec260Client, selected_scan: Mapping[str, Any]
) -> dict[str, float]:
    return {
        "start": (int(selected_scan["start_us"]) - client.scene_start_us) / 1_000_000.0,
        "end": (int(selected_scan["end_us"]) - client.scene_start_us) / 1_000_000.0,
    }


def _transform_dynamic_objects_to_nre(
    objects: list[Mapping[str, Any]], nre_from_log: list[list[float]]
) -> list[dict[str, Any]]:
    result = []
    for item in objects:
        transformed = deepcopy(dict(item))
        transformed["pose_pair"] = {
            endpoint: _matrix_to_pose(
                _matmul(
                    nre_from_log,
                    _pose_to_matrix(item["pose_pair"][endpoint]),
                )
            )
            for endpoint in ("start", "end")
        }
        result.append(transformed)
    return result


def _coordinate_check(
    baseline: Mapping[str, Any],
    moved: Mapping[str, Any],
    artifact: Mapping[str, Any],
    selected_scan: Mapping[str, Any],
) -> dict[str, Any]:
    baseline_objects = baseline["shared_dynamic_objects"]
    moved_objects = moved["shared_dynamic_objects"]
    baseline_by_track = {str(item["track_id"]): item for item in baseline_objects}
    moved_by_track = {str(item["track_id"]): item for item in moved_objects}
    changed_tracks = []
    for track_id in sorted(set(baseline_by_track) | set(moved_by_track)):
        if baseline_by_track.get(track_id) != moved_by_track.get(track_id):
            changed_tracks.append(track_id)
    baseline_lidar = next(
        request
        for request in baseline["modalities"]["lidar"]["requests"]
    )
    native_pair = _native_sensor_pose_pair(
        artifact,
        selected_scan,
        "lidar",
        str(baseline_lidar["sensor"]["sensor_id"]),
    )
    original_position = baseline_lidar["sensor"]["pose_pair"]["start"]["position_m"]
    native_position = native_pair["start"]["position_m"]
    return {
        "input_coordinate_frame": deepcopy(baseline["coordinate_frame"]),
        "scene_start_us": artifact["timestamp_range_us"]["start"],
        "baseline_simulation_time_sec": baseline["simulation_time_sec"],
        "baseline_absolute_time_us": artifact["timestamp_range_us"]["start"]
        + int(round(float(baseline["simulation_time_sec"]) * 1_000_000)),
        "selected_scan_inside_artifact_range": _timestamp_inside(
            selected_scan["start_us"], artifact["timestamp_range_us"]
        )
        and _timestamp_inside(
            selected_scan["end_us"],
            artifact["timestamp_range_us"],
            include_stop=True,
        ),
        "original_probe_lidar_global_position_m": deepcopy(original_position),
        "artifact_native_lidar_position_m": deepcopy(native_position),
        "original_probe_to_native_position_distance_m": math.sqrt(
            sum(
                (float(original_position[axis]) - float(native_position[axis])) ** 2
                for axis in ("x", "y", "z")
            )
        ),
        "nre_from_log_transform": deepcopy(artifact["nre_from_log"]),
        "changed_dynamic_tracks": changed_tracks,
        "moved_frame_changes_exactly_one_track": len(changed_tracks) == 1,
        "sensor_to_ego_source": "artifact rig_trajectories.json T_sensor_rig",
    }


def _timestamp_inside(
    value: int, interval: Mapping[str, int], *, include_stop: bool = False
) -> bool:
    return int(interval["start"]) <= int(value) <= int(interval["stop"]) if include_stop else int(interval["start"]) <= int(value) < int(interval["stop"])


def _identity_pose_pair() -> dict[str, Any]:
    pose = {
        "position_m": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation_xyzw": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }
    return {"start": deepcopy(pose), "end": deepcopy(pose)}


def _pose_to_matrix(pose: Mapping[str, Any]) -> list[list[float]]:
    position = pose["position_m"]
    quaternion = pose["orientation_xyzw"]
    x = float(quaternion["x"])
    y = float(quaternion["y"])
    z = float(quaternion["z"])
    w = float(quaternion["w"])
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0 or not math.isfinite(norm):
        raise ValueError("pose quaternion must be finite and non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), float(position["x"])],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), float(position["y"])],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), float(position["z"])],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matrix_to_pose(matrix: list[list[float]]) -> dict[str, Any]:
    m00, m11, m22 = matrix[0][0], matrix[1][1], matrix[2][2]
    trace = m00 + m11 + m22
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return {
        "position_m": {
            "x": float(matrix[0][3]),
            "y": float(matrix[1][3]),
            "z": float(matrix[2][3]),
        },
        "orientation_xyzw": {
            "x": x / norm,
            "y": y / norm,
            "z": z / norm,
            "w": w / norm,
        },
    }


def _matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [
            sum(float(left[row][index]) * float(right[index][column]) for index in range(4))
            for column in range(4)
        ]
        for row in range(4)
    ]


def _invert_rigid(matrix: list[list[float]]) -> list[list[float]]:
    rotation = [[float(matrix[row][column]) for column in range(3)] for row in range(3)]
    transpose = [[rotation[column][row] for column in range(3)] for row in range(3)]
    translation = [float(matrix[row][3]) for row in range(3)]
    inverse_translation = [
        -sum(transpose[row][column] * translation[column] for column in range(3))
        for row in range(3)
    ]
    return [
        [*transpose[0], inverse_translation[0]],
        [*transpose[1], inverse_translation[1]],
        [*transpose[2], inverse_translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _exception_record(exc: Exception) -> dict[str, Any]:
    result = {
        "type": type(exc).__name__,
        "detail": str(exc),
        "traceback": traceback.format_exc(),
    }
    code = getattr(exc, "code", None)
    details = getattr(exc, "details", None)
    debug = getattr(exc, "debug_error_string", None)
    if callable(code):
        result["grpc_code"] = str(code())
    if callable(details):
        result["grpc_details"] = details()
    if callable(debug):
        result["grpc_debug_error_string"] = debug()
    return result


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run an ordered, fail-closed NRE 26.04 capability/RGB/LiDAR "
            "diagnostic across ports, actor states, poses, and device types."
        )
    )
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--baseline-frame", required=True, type=Path)
    parser.add_argument("--moved-frame", required=True, type=Path)
    parser.add_argument("--target", action="append", default=[])
    parser.add_argument("--device-type", action="append", default=[])
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    device_types = [str(value).upper() for value in args.device_type]
    if not device_types:
        device_types = list(SUPPORTED_DEVICE_TYPES)
    unsupported = sorted(set(device_types) - set(SUPPORTED_DEVICE_TYPES))
    if unsupported:
        parser.error("unsupported device types: " + ", ".join(unsupported))
    try:
        config = _load_object(args.config)
        runtime = config.get("nurec_runtime") or {}
        targets = args.target or [str(runtime.get("target") or "")]
        if not all(targets):
            raise ValueError("at least one non-empty NRE target is required")
        report = diagnose_lidar(
            args.config,
            args.artifact,
            args.baseline_frame,
            args.moved_frame,
            targets=targets,
            device_types=device_types,
        )
    except (OSError, ValueError, KeyError, zipfile.BadZipFile, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}))
        return 2
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "output": str(args.output)}))
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
