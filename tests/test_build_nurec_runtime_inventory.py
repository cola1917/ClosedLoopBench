import json
from pathlib import Path
import tempfile
import unittest

from runners.build_nurec_runtime_inventory import build_inventory_from_files
from tests.test_actor_binding import VEHICLE_TRACK
from tests.test_nurec_inventory import _probe


class BuildNuRecRuntimeInventoryTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.root = Path(self.tempdir.name)
        self.artifact = (self.root / "last.usdz").resolve()
        self.artifact.write_bytes(b"usdz")
        self.mapping = self.root / "mapping.json"
        self.mapping.write_text(
            json.dumps(
                {
                    "schema_version": "nurec_actor_mapping_observation.v1",
                    "source": "loaded_nurec_scenario.actor_mapping",
                    "usdz_path": str(self.artifact),
                    "track_count": 1,
                    "tracks": [
                        {
                            "track_id": VEHICLE_TRACK,
                            "runtime_actor_id": 101,
                            "runtime_type_id": "vehicle.tesla.model3",
                            "first_frame": 1,
                            "last_frame": 2,
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        self.probe = self.root / "probe.json"
        self.probe.write_text(
            json.dumps(
                {
                    "schema_version": "nurec_dynamic_pose_ab_probe.v1",
                    "track_id": VEHICLE_TRACK,
                    "status": "passed",
                    "probe": _probe(),
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self):
        self.tempdir.cleanup()

    def test_builds_canonical_inventory_from_loaded_mapping_and_probe(self):
        result = build_inventory_from_files(
            self.mapping,
            self.artifact,
            [self.probe],
            renderer_version="26.4.146",
        )
        self.assertEqual(result["summary"]["runtime_track_count"], 1)
        self.assertEqual(result["summary"]["pose_verified_track_count"], 1)
        self.assertTrue(result["tracks"][0]["dynamic_object_pose_verified"])

    def test_rejects_mapping_captured_from_a_different_artifact(self):
        payload = json.loads(self.mapping.read_text(encoding="utf-8"))
        payload["usdz_path"] = str(self.root / "other.usdz")
        self.mapping.write_text(json.dumps(payload), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "does not match artifact"):
            build_inventory_from_files(
                self.mapping,
                self.artifact,
                [self.probe],
                renderer_version="26.4.146",
            )

    def test_rejects_failed_probe(self):
        payload = json.loads(self.probe.read_text(encoding="utf-8"))
        payload["status"] = "failed"
        self.probe.write_text(json.dumps(payload), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "did not pass"):
            build_inventory_from_files(
                self.mapping,
                self.artifact,
                [self.probe],
                renderer_version="26.4.146",
            )


if __name__ == "__main__":
    unittest.main()
