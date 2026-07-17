import json
import tempfile
import unittest
from pathlib import Path

from tests.test_actor_binding import SCENE_TOKEN, VEHICLE_TRACK, _scenario_ir


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
                "moved_payload_sha256": "2" * 64,
                "content_changed": True,
            },
            "lidar": {
                "status": "passed",
                "dynamic_object_sha256": digest,
                "baseline_payload_sha256": "3" * 64,
                "moved_payload_sha256": "4" * 64,
                "content_changed": True,
            },
        },
    }


def _inventory(records):
    verified = sum(record["dynamic_object_pose_verified"] for record in records)
    return {
        "schema_version": "nurec_runtime_track_inventory.v1",
        "renderer": {"name": "nurec", "version": "test"},
        "artifact": {"name": "last.usdz", "sha256": "b" * 64, "size_bytes": 1},
        "extraction_source": "loaded_nurec_scenario.actor_mapping_plus_dynamic_pose_probe",
        "tracks": records,
        "summary": {
            "runtime_track_count": len(records),
            "pose_verified_track_count": verified,
            "unverified_track_count": len(records) - verified,
        },
    }


def _track(track_id=VEHICLE_TRACK, verified=True):
    return {
        "track_id": track_id,
        "runtime_actor_id": 101,
        "runtime_type_id": "vehicle.car",
        "dynamic_object_pose_verified": verified,
        "probe": _probe() if verified else None,
        "issues": [] if verified else ["dynamic_pose_probe_missing"],
    }


class BuildActorBindingsTests(unittest.TestCase):
    def test_writes_ready_artifact_from_explicit_nurec_inventory(self):
        from runners.build_actor_bindings import write_actor_bindings

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario = root / "scenario_ir.json"
            inventory = root / "tracks.json"
            output = root / "actor_bindings.json"
            scenario.write_text(json.dumps(_scenario_ir()), encoding="utf-8")
            inventory.write_text(
                json.dumps(_inventory([_track()])),
                encoding="utf-8",
            )

            result = write_actor_bindings(
                scenario,
                output,
                actor_ids=[VEHICLE_TRACK],
                nurec_inventory_path=inventory,
                control_modes={VEHICLE_TRACK: "scripted"},
                require_ready=True,
            )

            self.assertTrue(output.is_file())
            self.assertEqual(result["scene_id"], SCENE_TOKEN)
            self.assertEqual(json.loads(output.read_text())["readiness"]["status"], "ready")

    def test_require_ready_does_not_write_unverified_claim(self):
        from adapters.actor_binding import ActorBindingError
        from runners.build_actor_bindings import write_actor_bindings

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario = root / "scenario_ir.json"
            output = root / "actor_bindings.json"
            scenario.write_text(json.dumps(_scenario_ir()), encoding="utf-8")

            with self.assertRaises(ActorBindingError):
                write_actor_bindings(
                    scenario,
                    output,
                    actor_ids=[VEHICLE_TRACK],
                    require_ready=True,
                )
            self.assertFalse(output.exists())

    def test_inventory_loader_rejects_duplicate_tracks(self):
        from runners.build_actor_bindings import load_nurec_track_inventory

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tracks.json"
            path.write_text(
                json.dumps(_inventory([_track(), _track()])),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "duplicate"):
                load_nurec_track_inventory(path)

    def test_inventory_loader_does_not_promote_asset_presence_without_pose_probe(self):
        from runners.build_actor_bindings import load_nurec_track_inventory

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tracks.json"
            path.write_text(
                json.dumps(_inventory([_track(verified=False)])),
                encoding="utf-8",
            )
            self.assertEqual(load_nurec_track_inventory(path), [])


if __name__ == "__main__":
    unittest.main()
