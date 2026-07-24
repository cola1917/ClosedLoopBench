import hashlib
import json
import tempfile
import unittest
from pathlib import Path
import zipfile


class BuildNuRecNativeScanManifestTests(unittest.TestCase):
    def test_builds_deterministic_manifest_and_refuses_bad_identity(self):
        from runners.build_nurec_native_scan_manifest import build_manifest

        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "last.usdz"
            data_info = {
                "sequence_id": "scene-0061",
                "sequence_timestamp_interval_us": {
                    "start": 1_000_000,
                    "stop": 1_200_000,
                },
            }
            rig = {
                "lidar_calibrations": {
                    "lidar-key": {"logical_sensor_name": "lidar_top"}
                },
                "rig_trajectories": [
                    {
                        "lidars_frame_timestamps_us": {
                            "lidar-key": [
                                [1_000_000, 1_050_000],
                                [1_050_100, 1_100_100],
                            ]
                        }
                    }
                ],
            }
            with zipfile.ZipFile(artifact, "w") as archive:
                archive.writestr("data_info.json", json.dumps(data_info))
                archive.writestr("rig_trajectories.json", json.dumps(rig))
            digest = hashlib.sha256(artifact.read_bytes()).hexdigest()

            manifest = build_manifest(
                artifact,
                expected_artifact_sha256=digest,
                runtime_scene_id="scene-0061",
                sensor_id="lidar_top",
            )

            self.assertEqual(manifest["artifact_sha256"], digest)
            self.assertEqual(manifest["scene_start_us"], 1_000_000)
            self.assertEqual(len(manifest["scan_windows_us"]), 2)
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                build_manifest(
                    artifact,
                    expected_artifact_sha256="0" * 64,
                    runtime_scene_id="scene-0061",
                    sensor_id="lidar_top",
                )


if __name__ == "__main__":
    unittest.main()
