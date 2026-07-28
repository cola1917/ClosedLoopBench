from __future__ import annotations

import json
import struct
from pathlib import Path


def _q(z=0.0):
    return [1.0, 0.0, 0.0, z]


def _write_fixture(root: Path) -> dict:
    meta = root / "v1.0-mini"
    (root / "samples/LIDAR_TOP").mkdir(parents=True)
    meta.mkdir(parents=True)
    sample_token = "sample"
    ann_token = "ann"
    sensor_token = "sensor"
    calibrated_token = "calibrated"
    ego_token = "ego"
    filename = "samples/LIDAR_TOP/sample.bin"
    tables = {
        "sample": [{"token": sample_token, "timestamp": 1}],
        "sensor": [{"token": sensor_token, "channel": "LIDAR_TOP"}],
        "calibrated_sensor": [{"token": calibrated_token, "sensor_token": sensor_token, "translation": [0, 0, 0], "rotation": _q()}],
        "ego_pose": [{"token": ego_token, "translation": [0, 0, 0], "rotation": _q()}],
        "sample_data": [{"token": "sd", "sample_token": sample_token, "ego_pose_token": ego_token, "calibrated_sensor_token": calibrated_token, "is_key_frame": True, "filename": filename}],
        "sample_annotation": [{"token": ann_token, "sample_token": sample_token, "translation": [1, 0, 0], "size": [2, 2, 2], "rotation": _q(), "num_lidar_pts": 1}],
    }
    for name, rows in tables.items():
        (meta / f"{name}.json").write_text(json.dumps(rows), encoding="utf-8")
    # One point inside x=[0,2], one outside.
    payload = struct.pack("<10f", 1, 0, 0, 1, 0, 5, 0, 0, 1, 0)
    (root / filename).write_bytes(payload)
    return {"scene_id": "scene", "records": [{"object_id": "track", "role": "background_replay", "semantic_class": "vehicle", "category": "vehicle.car", "source": {"source_track_id": "track", "annotation_tokens": [ann_token]}, "nurec": {"track_id": "track"}}]}


def test_audit_counts_oriented_box_hits(tmp_path: Path):
    from adapters.nuscenes_lidar_source_audit import audit_nuscenes_lidar_source

    registry = _write_fixture(tmp_path)
    result = audit_nuscenes_lidar_source(tmp_path, registry)
    assert result["status"] == "passed"
    row = result["tracks"][0]
    assert row["status"] == "raw_lidar_supported"
    assert row["sum_computed_box_hit_points"] == 1


def test_audit_distinguishes_zero_source_points(tmp_path: Path):
    from adapters.nuscenes_lidar_source_audit import audit_nuscenes_lidar_source

    registry = _write_fixture(tmp_path)
    (tmp_path / "samples/LIDAR_TOP/sample.bin").write_bytes(struct.pack("<5f", 5, 0, 0, 1, 0))
    result = audit_nuscenes_lidar_source(tmp_path, registry)
    assert result["status"] == "failed"
    assert result["tracks"][0]["status"] == "raw_lidar_absent"
