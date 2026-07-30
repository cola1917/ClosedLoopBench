import copy
import hashlib
import struct
import tempfile
import unittest
from pathlib import Path


IDENTITY = [
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
]

# Camera sensor z is aligned with the ego x axis so the test object is in
# front of the camera while remaining in front of the LiDAR in ego x.
CAMERA_TO_EGO = [
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    -1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
]


class M8SensorEvidenceTests(unittest.TestCase):
    def _inputs(self, root: Path) -> tuple[dict, dict, list[dict], list[dict], dict]:
        rgb_path = root / "camera_front.bin"
        lidar_path = root / "lidar_top.bin"
        rgb_bytes = b"materialized-rgb-payload"
        lidar_bytes = struct.pack("<4f", 5.0, 0.0, 0.0, 0.75)
        rgb_path.write_bytes(rgb_bytes)
        lidar_path.write_bytes(lidar_bytes)

        runtime_row = {
            "frame_id": 42,
            "simulation_time_sec": 2.1,
            "ego_state": {
                "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                "extent_m": {"x": 2.0, "y": 1.0, "z": 1.0},
            },
            "object_states": [
                {
                    "object_id": "truck",
                    "carla_runtime_actor_id": 17,
                    "pose": {"x": 5.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                    "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
                }
            ],
        }
        camera_specs = [
            {
                "sensor_id": "camera_front",
                "model": "pinhole",
                "sensor_to_ego": CAMERA_TO_EGO,
                "calibrated_sensor_token": "calibrated-camera-front",
                "intrinsic_matrix_3x3": [
                    [50.0, 0.0, 50.0],
                    [0.0, 50.0, 50.0],
                    [0.0, 0.0, 1.0],
                ],
                "intrinsics_table_sha256": "a" * 64,
                "width": 100,
                "height": 100,
            }
        ]
        lidar_specs = [
            {
                "sensor_id": "lidar_top",
                "model": "verified",
                "sensor_to_ego": IDENTITY,
            }
        ]
        calibration = {
            "schema_version": "nurec_camera_calibration_capture.v1",
            "intrinsics_status": "passed",
            "camera_records": [
                {
                    "sensor_id": "camera_front",
                    "calibrated_sensor_token": "calibrated-camera-front",
                    "intrinsic_matrix_3x3": [
                        [50.0, 0.0, 50.0],
                        [0.0, 50.0, 50.0],
                        [0.0, 0.0, 1.0],
                    ],
                    "intrinsics_source": {"table_sha256": "a" * 64},
                    "requested_resolution": {"width": 100, "height": 100},
                }
            ],
        }
        evidence = {
            "frame_id": 42,
            "status": "passed",
            "records": [
                {
                    "modality": "rgb",
                    "sensor_id": "camera_front",
                    "status": "passed",
                    "response_metadata": {
                        "materialized_payload": {
                            "path": str(rgb_path),
                            "relative_path": "camera_front.bin",
                            "sha256": hashlib.sha256(rgb_bytes).hexdigest(),
                        }
                    },
                },
                {
                    "modality": "lidar",
                    "sensor_id": "lidar_top",
                    "status": "passed",
                    "response_metadata": {
                        "coordinate_frame": "sensor_local",
                        "axis_convention": "carla_sensor",
                        "materialized_payload": {
                            "path": str(lidar_path),
                            "relative_path": "lidar_top.bin",
                            "sha256": hashlib.sha256(lidar_bytes).hexdigest(),
                        },
                    },
                },
            ],
        }
        return runtime_row, evidence, camera_specs, lidar_specs, calibration

    def test_binds_rgb_projection_and_lidar_occupancy_to_one_frame(self):
        from adapters.m8_sensor_evidence import build_m8_sensor_evidence
        from adapters.scene_safety_audit import (
            audit_lidar_world_tick,
            audit_visibility_tick,
        )

        with tempfile.TemporaryDirectory() as directory:
            inputs = self._inputs(Path(directory))
            runtime_row, evidence, cameras, lidars, calibration = inputs
            result = build_m8_sensor_evidence(
                runtime_row,
                evidence,
                camera_specs=cameras,
                lidar_specs=lidars,
                camera_calibration_capture=calibration,
            )

            self.assertEqual(result["visibility"]["frame_id"], 42)
            self.assertEqual(len(result["visibility"]["projections"]), 1)
            self.assertEqual(result["lidar_world"]["frame_id"], 42)
            self.assertTrue(
                result["lidar_world"]["expected_world_objects"][0][
                    "expected_lidar_support"
                ]
            )
            self.assertEqual(result["lidar_world"]["lidar_occupancy"][0]["point_count"], 1)

            visibility = audit_visibility_tick(
                {"frame_id": 42, "simulation_time_sec": 2.1, **result["visibility"]}
            )
            lidar = audit_lidar_world_tick(
                {"frame_id": 42, "simulation_time_sec": 2.1, **result["lidar_world"]}
            )
            self.assertEqual(visibility["status"], "passed")
            self.assertEqual(lidar["status"], "passed")

    def test_requires_source_bound_camera_calibration(self):
        from adapters.m8_sensor_evidence import (
            M8SensorEvidenceError,
            build_m8_sensor_evidence,
        )

        with tempfile.TemporaryDirectory() as directory:
            runtime_row, evidence, cameras, lidars, _ = self._inputs(Path(directory))
            with self.assertRaisesRegex(M8SensorEvidenceError, "source-bound"):
                build_m8_sensor_evidence(
                    runtime_row,
                    evidence,
                    camera_specs=cameras,
                    lidar_specs=lidars,
                    camera_calibration_capture=None,
                )

    def test_materialized_rgb_hash_mismatch_fails_closed(self):
        from adapters.m8_sensor_evidence import (
            M8SensorEvidenceError,
            build_m8_sensor_evidence,
        )

        with tempfile.TemporaryDirectory() as directory:
            runtime_row, evidence, cameras, lidars, calibration = self._inputs(Path(directory))
            tampered = copy.deepcopy(evidence)
            tampered["records"][0]["response_metadata"]["materialized_payload"][
                "sha256"
            ] = "0" * 64
            with self.assertRaisesRegex(M8SensorEvidenceError, "hash mismatch"):
                build_m8_sensor_evidence(
                    runtime_row,
                    tampered,
                    camera_specs=cameras,
                    lidar_specs=lidars,
                    camera_calibration_capture=calibration,
                )

    def test_lidar_frame_mismatch_is_rejected_before_physical_binding(self):
        from adapters.m8_sensor_evidence import (
            M8SensorEvidenceError,
            build_m8_sensor_evidence,
        )

        with tempfile.TemporaryDirectory() as directory:
            runtime_row, evidence, cameras, lidars, calibration = self._inputs(Path(directory))
            mismatched = copy.deepcopy(evidence)
            mismatched["frame_id"] = 43
            with self.assertRaisesRegex(M8SensorEvidenceError, "frame"):
                build_m8_sensor_evidence(
                    runtime_row,
                    mismatched,
                    camera_specs=cameras,
                    lidar_specs=lidars,
                    camera_calibration_capture=calibration,
                )


if __name__ == "__main__":
    unittest.main()
