from __future__ import annotations

import struct
from pathlib import Path

from runners.diagnose_m8_lidar_geometry import diagnose


def test_lidar_geometry_diagnostic_keeps_axis_candidates_diagnostic_only(tmp_path: Path):
    payload = tmp_path / "lidar.bin"
    payload.write_bytes(struct.pack("<ffff", 2.0, 0.0, 0.0, 1.0))
    state = {
        "object_id": "obj",
        "pose": {"x": 2.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
    }
    runtime = [{
        "frame_id": 1,
        "simulation_time_sec": 0.1,
        "ego_state": {"pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}},
        "object_states": [state],
    }]
    occupancy = [{
        "frame_id": 1,
        "sensor_to_ego": [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "lidar_payload": {"path": str(payload)},
    }]
    expected = [{
        "frame_id": 1,
        "expected_world_objects": [{"object_id": "obj", "expected_lidar_support": True}],
    }]

    result = diagnose(runtime, occupancy, expected)

    assert result["status"] == "diagnostic_only"
    tick = result["ticks"][0]
    assert tick["identity_expected_supported_object_count"] == 1
    assert tick["axis_candidate_count"] == 48
