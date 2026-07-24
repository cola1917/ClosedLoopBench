from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping
import zipfile


SCHEMA_VERSION = "nurec_native_lidar_scan_manifest.v1"


def build_manifest(
    artifact_path: Path,
    *,
    expected_artifact_sha256: str,
    runtime_scene_id: str,
    sensor_id: str,
) -> dict[str, Any]:
    artifact_path = Path(artifact_path)
    if not artifact_path.is_file():
        raise FileNotFoundError(f"NuRec artifact does not exist: {artifact_path}")
    artifact_sha256 = _sha256_file(artifact_path)
    if artifact_sha256 != expected_artifact_sha256:
        raise ValueError("NuRec artifact SHA-256 mismatch")
    with zipfile.ZipFile(artifact_path) as archive:
        data_info_bytes = archive.read("data_info.json")
        rig_bytes = archive.read("rig_trajectories.json")
    data_info = json.loads(data_info_bytes)
    rig = json.loads(rig_bytes)
    if str(data_info.get("sequence_id") or "") != runtime_scene_id:
        raise ValueError("NuRec artifact sequence_id does not match runtime scene")
    interval = data_info.get("sequence_timestamp_interval_us") or {}
    scene_start_us = int(interval["start"])
    scene_stop_us = int(interval.get("stop", interval.get("end")))
    trajectories = rig.get("rig_trajectories") or []
    if len(trajectories) != 1:
        raise ValueError("NuRec artifact must contain exactly one rig trajectory")
    calibration_matches = [
        (key, value)
        for key, value in (rig.get("lidar_calibrations") or {}).items()
        if str(value.get("logical_sensor_name") or "") == sensor_id
    ]
    if len(calibration_matches) != 1:
        raise ValueError(f"expected exactly one LiDAR calibration for {sensor_id}")
    lidar_key = calibration_matches[0][0]
    raw_windows = (trajectories[0].get("lidars_frame_timestamps_us") or {}).get(
        lidar_key
    )
    if not isinstance(raw_windows, list) or not raw_windows:
        raise ValueError(f"NuRec artifact has no native scan windows for {sensor_id}")
    windows = []
    previous_start = -1
    for raw in raw_windows:
        if not isinstance(raw, list) or len(raw) != 2:
            raise ValueError("native LiDAR scan window must contain start/end")
        start, end = int(raw[0]), int(raw[1])
        if (
            start < scene_start_us
            or end > scene_stop_us
            or start >= end
            or start <= previous_start
        ):
            raise ValueError("native LiDAR scan windows are invalid or unsorted")
        windows.append([start, end])
        previous_start = start
    return {
        "schema_version": SCHEMA_VERSION,
        "runtime_scene_id": runtime_scene_id,
        "sensor_id": sensor_id,
        "scene_start_us": scene_start_us,
        "scene_stop_us": scene_stop_us,
        "artifact_sha256": artifact_sha256,
        "source_members": {
            "data_info.json": hashlib.sha256(data_info_bytes).hexdigest(),
            "rig_trajectories.json": hashlib.sha256(rig_bytes).hexdigest(),
        },
        "scan_windows_us": windows,
    }


def write_manifest(manifest: Mapping[str, Any], output: Path) -> None:
    output = Path(output)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite native scan manifest: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(dict(manifest), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build a hashed native LiDAR scan timeline from one NuRec USDZ."
    )
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--artifact-sha256", required=True)
    parser.add_argument("--runtime-scene-id", required=True)
    parser.add_argument("--sensor-id", default="lidar_top")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        manifest = build_manifest(
            args.artifact,
            expected_artifact_sha256=args.artifact_sha256,
            runtime_scene_id=args.runtime_scene_id,
            sensor_id=args.sensor_id,
        )
        write_manifest(manifest, args.output)
    except (OSError, ValueError, KeyError, zipfile.BadZipFile, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2
    print(
        json.dumps(
            {
                "status": "passed",
                "output": str(args.output.resolve()),
                "scan_count": len(manifest["scan_windows_us"]),
                "artifact_sha256": manifest["artifact_sha256"],
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
