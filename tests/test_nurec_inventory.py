import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from tests.test_actor_binding import VEHICLE_TRACK


def _probe(digest="a" * 64):
    return {
        "frame_id": 10,
        "pose_delta_m": 0.5,
        "baseline_dynamic_object_sha256": "d" * 64,
        "dynamic_object_sha256": digest,
        "modalities": {
            "rgb": {
                "status": "passed",
                "dynamic_object_sha256": digest,
                "baseline_payload_sha256": "1" * 64,
                "baseline_repeat_payload_sha256": "1" * 64,
                "moved_payload_sha256": "2" * 64,
                "baseline_repeatable": True,
                "content_changed": True,
            },
            "lidar": {
                "status": "passed",
                "dynamic_object_sha256": digest,
                "baseline_payload_sha256": "3" * 64,
                "baseline_repeat_payload_sha256": "3" * 64,
                "moved_payload_sha256": "4" * 64,
                "baseline_repeatable": True,
                "content_changed": True,
            },
        },
    }


class NuRecInventoryTests(unittest.TestCase):
    def test_promotes_only_runtime_tracks_with_same_digest_rgb_lidar_probe(self):
        from adapters.nurec_inventory import build_nurec_runtime_track_inventory

        second = "b" * 32
        mapping = {
            VEHICLE_TRACK: SimpleNamespace(actor_inst=SimpleNamespace(id=101, type_id="vehicle.car")),
            second: SimpleNamespace(actor_inst=SimpleNamespace(id=102, type_id="walker.pedestrian.0001")),
            "ego": SimpleNamespace(actor_inst=SimpleNamespace(id=1, type_id="vehicle.ego")),
        }
        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "last.usdz"
            artifact.write_bytes(b"usdz")
            inventory = build_nurec_runtime_track_inventory(
                mapping,
                artifact_path=artifact,
                renderer_version="26.04",
                probe_results={VEHICLE_TRACK: _probe()},
            )

        records = {item["track_id"]: item for item in inventory["tracks"]}
        self.assertTrue(records[VEHICLE_TRACK]["dynamic_object_pose_verified"])
        self.assertFalse(records[second]["dynamic_object_pose_verified"])
        self.assertEqual(inventory["summary"]["runtime_track_count"], 2)
        self.assertEqual(inventory["summary"]["pose_verified_track_count"], 1)

    def test_cross_modality_digest_mismatch_keeps_track_unverified(self):
        from adapters.nurec_inventory import build_nurec_runtime_track_inventory

        probe = _probe()
        probe["modalities"]["lidar"]["dynamic_object_sha256"] = "c" * 64
        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "last.usdz"
            artifact.write_bytes(b"usdz")
            inventory = build_nurec_runtime_track_inventory(
                {VEHICLE_TRACK: SimpleNamespace(actor_inst=SimpleNamespace(id=101, type_id="vehicle.car"))},
                artifact_path=artifact,
                renderer_version="26.04",
                probe_results={VEHICLE_TRACK: probe},
            )

        record = inventory["tracks"][0]
        self.assertFalse(record["dynamic_object_pose_verified"])
        self.assertIn("lidar_dynamic_object_digest_mismatch", record["issues"])

    def test_unchanged_render_keeps_track_unverified(self):
        from adapters.nurec_inventory import build_nurec_runtime_track_inventory

        probe = _probe()
        probe["modalities"]["rgb"]["moved_payload_sha256"] = "1" * 64
        probe["modalities"]["rgb"]["content_changed"] = False
        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "last.usdz"
            artifact.write_bytes(b"usdz")
            inventory = build_nurec_runtime_track_inventory(
                {VEHICLE_TRACK: SimpleNamespace(actor_inst=SimpleNamespace(id=101, type_id="vehicle.car"))},
                artifact_path=artifact,
                renderer_version="26.04",
                probe_results={VEHICLE_TRACK: probe},
            )

        self.assertFalse(inventory["tracks"][0]["dynamic_object_pose_verified"])
        self.assertIn("rgb_render_unchanged", inventory["tracks"][0]["issues"])

    def test_unrepeatable_baseline_keeps_track_unverified(self):
        from adapters.nurec_inventory import build_nurec_runtime_track_inventory

        probe = _probe()
        probe["modalities"]["lidar"]["baseline_repeat_payload_sha256"] = "5" * 64
        probe["modalities"]["lidar"]["baseline_repeatable"] = False
        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "last.usdz"
            artifact.write_bytes(b"usdz")
            inventory = build_nurec_runtime_track_inventory(
                {VEHICLE_TRACK: SimpleNamespace(actor_inst=SimpleNamespace(id=101, type_id="vehicle.car"))},
                artifact_path=artifact,
                renderer_version="26.04",
                probe_results={VEHICLE_TRACK: probe},
            )

        self.assertFalse(inventory["tracks"][0]["dynamic_object_pose_verified"])
        self.assertIn("lidar_baseline_unrepeatable", inventory["tracks"][0]["issues"])

    def test_failed_rpc_with_null_render_hashes_remains_valid_unverified_evidence(self):
        from adapters.nurec_inventory import build_nurec_runtime_track_inventory

        probe = _probe()
        rgb = probe["modalities"]["rgb"]
        rgb.update(
            status="failed",
            baseline_payload_sha256=None,
            baseline_repeat_payload_sha256=None,
            moved_payload_sha256=None,
            baseline_repeatable=False,
            content_changed=False,
        )
        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "last.usdz"
            artifact.write_bytes(b"usdz")
            inventory = build_nurec_runtime_track_inventory(
                {VEHICLE_TRACK: SimpleNamespace(actor_inst=SimpleNamespace(id=101, type_id="vehicle.car"))},
                artifact_path=artifact,
                renderer_version="26.04",
                probe_results={VEHICLE_TRACK: probe},
            )

        record = inventory["tracks"][0]
        self.assertFalse(record["dynamic_object_pose_verified"])
        self.assertIn("rgb_probe_failed", record["issues"])
        self.assertIn("rgb_render_digest_invalid", record["issues"])

    def test_rejects_probe_for_track_not_loaded_by_runtime(self):
        from adapters.nurec_inventory import NuRecInventoryError, build_nurec_runtime_track_inventory

        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "last.usdz"
            artifact.write_bytes(b"usdz")
            with self.assertRaisesRegex(NuRecInventoryError, "absent"):
                build_nurec_runtime_track_inventory(
                    {},
                    artifact_path=artifact,
                    renderer_version="26.04",
                    probe_results={VEHICLE_TRACK: _probe()},
                )


if __name__ == "__main__":
    unittest.main()
