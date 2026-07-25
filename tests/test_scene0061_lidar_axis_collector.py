import hashlib
import json
import struct
import tempfile
import unittest
from pathlib import Path


IDENTITY = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
]


def _ref(path: Path, frame_id: int) -> dict:
    return {
        "path": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "byte_count": path.stat().st_size,
        "encoding": "float32_xyzi_little_endian",
        "carla_frame_id": frame_id,
    }


class Scene0061LiDARAxisCollectorTests(unittest.TestCase):
    def _inputs(self, root: Path):
        frame_id = 81
        points = [
            (1.0, 0.0, 0.0, 0.5),
            (0.0, 2.0, 0.0, 0.5),
            (0.0, 0.0, 3.0, 0.5),
            (2.0, 2.0, 2.0, 0.5),
        ]
        raw = b"".join(struct.pack("<4f", *point) for point in points)
        nurec_path = root / "nurec.xyzi"
        capture_points_path = root / "carla-native.xyzi"
        nurec_path.write_bytes(raw)
        capture_points_path.write_bytes(raw)
        capture = {
            "schema_version": "scene0061_carla_native_lidar_capture.v1",
            "status": "passed",
            "carla_frame_id": frame_id,
            "coordinate_frame": "carla_sensor",
            "sensor_to_ego": IDENTITY,
            "raw_xyzi_ref": _ref(capture_points_path, frame_id),
        }
        capture_path = root / "carla-native.json"
        capture_path.write_text(json.dumps(capture), encoding="utf-8")
        native_scan = root / "native-scan.json"
        native_scan.write_text("{\"scan\": 0}", encoding="utf-8")
        run_config = {
            "experiment": {"scene_id": "a" * 32, "artifact_sha256": "b" * 64},
            "nurec_runtime": {
                "runtime_scene_id": "scene-0061",
                "lidar_specs": [{"sensor_id": "lidar_top", "model": "AT128", "sensor_to_ego": IDENTITY}],
            },
        }
        nurec_evidence = {
            "frame_id": frame_id,
            "records": [{
                "modality": "lidar", "sensor_id": "lidar_top", "status": "passed",
                "response_metadata": {
                    "point_count": len(points),
                    "materialized_payload": {**_ref(nurec_path, frame_id), "coordinate_frame": "unverified"},
                },
            }],
            "dispatch": {"temporal_alignment": {"native_scan_index": 0}},
        }
        return run_config, nurec_evidence, {"world_tick_frame": frame_id}, capture_path, native_scan

    def test_collects_only_hash_bound_same_frame_native_carla_anchors(self):
        from runtime.scene0061_lidar_axis_collector import collect_lidar_axis_evidence
        from runtime.scene0061_lidar_axis_gate import validate_lidar_axis_evidence

        with tempfile.TemporaryDirectory() as directory:
            values = self._inputs(Path(directory))
            evidence = collect_lidar_axis_evidence(
                run_config=values[0], nurec_evidence=values[1], frame_trace=values[2],
                native_capture_path=values[3], native_scan_manifest_path=values[4],
            )
            self.assertEqual(evidence["axis_validation"]["evidence_source"], "scene0061_carla_native_lidar_capture.v1")
            self.assertGreaterEqual(len(evidence["axis_validation"]["anchors"]), 4)
            result = validate_lidar_axis_evidence(
                evidence, sensor_to_ego=IDENTITY, live_render_lidar=evidence["live_render_lidar"]
            )
            self.assertEqual(result["status"], "passed")

    def test_rejects_capture_from_another_frame_before_writing_a_claim(self):
        from runtime.scene0061_lidar_axis_collector import (
            LiDARAxisCollectionError,
            collect_lidar_axis_evidence,
        )

        with tempfile.TemporaryDirectory() as directory:
            values = self._inputs(Path(directory))
            capture = json.loads(values[3].read_text(encoding="utf-8"))
            capture["carla_frame_id"] = 82
            values[3].write_text(json.dumps(capture), encoding="utf-8")
            with self.assertRaisesRegex(LiDARAxisCollectionError, "same-frame"):
                collect_lidar_axis_evidence(
                    run_config=values[0], nurec_evidence=values[1], frame_trace=values[2],
                    native_capture_path=values[3], native_scan_manifest_path=values[4],
                )

    def test_gate_rejects_anchor_not_present_in_independent_capture(self):
        from runtime.scene0061_lidar_axis_collector import collect_lidar_axis_evidence
        from runtime.scene0061_lidar_axis_gate import LiDARAxisEvidenceError, validate_lidar_axis_evidence

        with tempfile.TemporaryDirectory() as directory:
            values = self._inputs(Path(directory))
            evidence = collect_lidar_axis_evidence(
                run_config=values[0], nurec_evidence=values[1], frame_trace=values[2],
                native_capture_path=values[3], native_scan_manifest_path=values[4],
            )
            evidence["axis_validation"]["anchors"][0]["carla_ego_point_m"][0] += 0.01
            with self.assertRaisesRegex(LiDARAxisEvidenceError, "independent CARLA LiDAR capture"):
                validate_lidar_axis_evidence(
                    evidence, sensor_to_ego=IDENTITY, live_render_lidar=evidence["live_render_lidar"]
                )


if __name__ == "__main__":
    unittest.main()
