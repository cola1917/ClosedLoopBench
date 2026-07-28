from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def advertised_camera_calibrations(
    run_config: Mapping[str, Any],
    camera_specs: Mapping[str, Any],
) -> dict[str, Any]:
    """Freeze the NRE camera intrinsics advertised by the live service."""

    nurec = run_config.get("nurec_runtime")
    if not isinstance(nurec, Mapping):
        raise ValueError("run config requires nurec_runtime")
    requested = nurec.get("camera_specs")
    if not isinstance(requested, list) or not requested:
        raise ValueError("run config requires non-empty nurec_runtime.camera_specs")
    requested_by_id = {
        str(item.get("sensor_id") or ""): item
        for item in requested
        if isinstance(item, Mapping) and str(item.get("sensor_id") or "")
    }
    if len(requested_by_id) != len(requested):
        raise ValueError("run config camera sensor IDs must be unique and non-empty")
    missing = sorted(set(requested_by_id) - set(camera_specs))
    if missing:
        raise ValueError("NRE did not advertise requested cameras: " + ", ".join(missing))

    records = []
    for sensor_id in sorted(requested_by_id):
        raw = _protobuf_to_json(camera_specs[sensor_id])
        if not isinstance(raw, Mapping):
            raise ValueError(f"NRE camera {sensor_id} intrinsics are not a protobuf message")
        records.append(
            {
                "sensor_id": sensor_id,
                "channel": requested_by_id[sensor_id].get("channel"),
                "requested_resolution": {
                    "width": requested_by_id[sensor_id].get("width"),
                    "height": requested_by_id[sensor_id].get("height"),
                },
                "sensor_to_ego": requested_by_id[sensor_id].get("sensor_to_ego"),
                "calibrated_sensor_token": requested_by_id[sensor_id].get("calibrated_sensor_token"),
                "advertised_intrinsics": raw,
            }
        )
    return {
        "schema_version": "nurec_camera_calibration_capture.v1",
        "scene_id": str(run_config.get("scenario_id") or ""),
        "runtime_scene_id": str(nurec.get("runtime_scene_id") or ""),
        "source": "NRE SensorsimService/26.04 get_available_cameras",
        "camera_records": records,
    }


def attach_nuscenes_intrinsics(
    capture: Mapping[str, Any],
    calibrated_sensor_rows: list[Mapping[str, Any]],
    *,
    source_sha256: str,
) -> dict[str, Any]:
    """Bind run-config calibration tokens to authoritative source 3x3 intrinsics."""

    records = capture.get("camera_records")
    if not isinstance(records, list) or not records:
        raise ValueError("camera capture requires camera_records")
    by_token = {
        str(row.get("token") or ""): row
        for row in calibrated_sensor_rows
        if isinstance(row, Mapping) and str(row.get("token") or "")
    }
    result = json.loads(json.dumps(capture))
    missing: list[str] = []
    for record in result["camera_records"]:
        token = str(record.get("calibrated_sensor_token") or "")
        source = by_token.get(token)
        intrinsic = source.get("camera_intrinsic") if isinstance(source, Mapping) else None
        if not _valid_intrinsic_matrix(intrinsic):
            missing.append(str(record.get("sensor_id") or token or "unknown"))
            continue
        record["intrinsic_matrix_3x3"] = [[float(item) for item in row] for row in intrinsic]
        record["intrinsics_source"] = {
            "kind": "nuScenes calibrated_sensor",
            "token": token,
            "table_sha256": source_sha256,
        }
    if missing:
        raise ValueError("missing usable nuScenes camera intrinsics: " + ", ".join(sorted(missing)))
    result["intrinsics_status"] = "passed"
    return result


def _valid_intrinsic_matrix(value: Any) -> bool:
    if not isinstance(value, list) or len(value) != 3:
        return False
    try:
        matrix = [[float(item) for item in row] for row in value]
    except (TypeError, ValueError):
        return False
    return (
        all(len(row) == 3 for row in matrix)
        and matrix[0][0] > 0.0
        and matrix[1][1] > 0.0
        and matrix[2] == [0.0, 0.0, 1.0]
    )


def _protobuf_to_json(value: Any) -> dict[str, Any]:
    try:
        from google.protobuf.json_format import MessageToDict
    except ImportError as exc:  # pragma: no cover - depends on NRE runtime installation.
        raise RuntimeError("google.protobuf is required to serialize NRE camera intrinsics") from exc
    result = MessageToDict(value, preserving_proto_field_name=True)
    if not isinstance(result, dict):
        raise ValueError("protobuf JSON conversion did not produce an object")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Capture live NRE camera intrinsics for the M6 visibility evidence."
    )
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--nuscenes-calibrated-sensor-table", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite calibration capture: {args.output}")
        config_bytes = args.run_config.read_bytes()
        config = json.loads(config_bytes)
        from adapters.nurec_260_client import build_nurec_260_client

        client = build_nurec_260_client(config)
        try:
            result = advertised_camera_calibrations(config, client._camera_specs)
        finally:
            client.close()
        calibration_bytes = args.nuscenes_calibrated_sensor_table.read_bytes()
        calibrated_sensors = json.loads(calibration_bytes)
        if not isinstance(calibrated_sensors, list):
            raise ValueError("calibrated_sensor table must be a JSON array")
        result = attach_nuscenes_intrinsics(
            result,
            calibrated_sensors,
            source_sha256=hashlib.sha256(calibration_bytes).hexdigest(),
        )
        result["run_config_sha256"] = hashlib.sha256(config_bytes).hexdigest()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output": str(args.output), "camera_count": len(result["camera_records"])}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
