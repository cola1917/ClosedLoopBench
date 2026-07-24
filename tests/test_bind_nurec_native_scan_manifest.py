import unittest
from pathlib import Path


class BindNuRecNativeScanManifestTests(unittest.TestCase):
    def test_binds_matching_manifest_and_rejects_scene_mismatch(self):
        from runners.bind_nurec_native_scan_manifest import bind_manifest

        artifact_sha256 = "a" * 64
        manifest_sha256 = "b" * 64
        run_config = {
            "experiment": {"identity": {}},
            "nurec_runtime": {
                "runtime_scene_id": "scene-0061",
                "scene_start_us": 1_000_000,
                "lidar_specs": [{"sensor_id": "lidar_top"}],
            },
        }
        manifest = {
            "schema_version": "nurec_native_lidar_scan_manifest.v1",
            "runtime_scene_id": "scene-0061",
            "sensor_id": "lidar_top",
            "scene_start_us": 1_000_000,
            "artifact_sha256": artifact_sha256,
            "scan_windows_us": [[1_000_000, 1_050_000]],
        }

        result = bind_manifest(
            run_config,
            manifest,
            manifest_path=Path("manifest.json"),
            manifest_sha256=manifest_sha256,
            artifact_sha256=artifact_sha256,
            max_midpoint_error_us=30_000,
        )

        reference = result["nurec_runtime"]["native_scan_manifest"]
        self.assertEqual(reference["sha256"], manifest_sha256)
        self.assertEqual(reference["max_midpoint_error_us"], 30_000)
        self.assertEqual(
            result["experiment"]["identity"]["artifact_sha256"], artifact_sha256
        )
        changed = dict(manifest)
        changed["runtime_scene_id"] = "scene-0062"
        with self.assertRaisesRegex(ValueError, "runtime_scene_id mismatch"):
            bind_manifest(
                run_config,
                changed,
                manifest_path=Path("manifest.json"),
                manifest_sha256=manifest_sha256,
                artifact_sha256=artifact_sha256,
                max_midpoint_error_us=30_000,
            )


if __name__ == "__main__":
    unittest.main()
