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
    def test_ncore_dynamic_closure_requires_exact_track_and_class_match(self):
        from adapters.nurec_inventory import audit_registry_ncore_dynamic_closure

        vehicle = "a" * 32
        pedestrian = "b" * 32
        unexpected = "c" * 32
        registry = {
            "schema_version": "scene_object_registry.v1",
            "scene_id": "scene",
            "records": [
                {
                    "object_id": vehicle,
                    "semantic_class": "vehicle",
                    "role": "background_replay",
                    "carla": {"representation": "physical_actor", "collision_policy": "required"},
                    "nurec": {"representation": "dynamic_track", "track_id": vehicle},
                },
                {
                    "object_id": pedestrian,
                    "semantic_class": "pedestrian",
                    "role": "controlled_pedestrian",
                    "carla": {"representation": "physical_actor", "collision_policy": "required"},
                    "nurec": {"representation": "dynamic_track", "track_id": pedestrian},
                },
                {
                    "object_id": "road_boundary:carla_map",
                    "semantic_class": "road_boundary",
                    "role": "road_boundary",
                    "carla": {"representation": "road_topology", "collision_policy": "not_applicable"},
                    "nurec": {"representation": "projection_target", "track_id": None},
                },
            ],
        }
        ncore = {
            "schema_version": 1,
            "pass": True,
            "contract": {"accepted_sources": ["EXTERNAL"]},
            "eligible_tracks": [
                {"track_id": vehicle, "class_id": "automobile", "source": "EXTERNAL"},
                {"track_id": unexpected, "class_id": "pedestrian", "source": "EXTERNAL"},
            ],
        }

        audit = audit_registry_ncore_dynamic_closure(registry, ncore)

        self.assertEqual(audit["status"], "failed")
        self.assertEqual(audit["missing_from_ncore"], [pedestrian])
        self.assertEqual(audit["unexpected_from_ncore"], [unexpected])
        self.assertEqual(audit["summary"]["matched_track_count"], 1)

    def test_ncore_dynamic_closure_accepts_full_vehicle_pedestrian_and_two_wheeler_match(self):
        from adapters.nurec_inventory import audit_registry_ncore_dynamic_closure

        vehicle, pedestrian, motorcycle = "a" * 32, "b" * 32, "c" * 32
        def record(track_id, semantic_class):
            return {
                "object_id": track_id,
                "semantic_class": semantic_class,
                "role": "background_replay",
                "carla": {"representation": "physical_actor", "collision_policy": "required"},
                "nurec": {"representation": "dynamic_track", "track_id": track_id},
            }
        registry = {
            "schema_version": "scene_object_registry.v1",
            "scene_id": "scene",
            "records": [record(vehicle, "vehicle"), record(pedestrian, "pedestrian"), record(motorcycle, "two_wheeler")],
        }
        ncore = {
            "schema_version": 1,
            "pass": True,
            "contract": {"accepted_sources": ["EXTERNAL"]},
            "eligible_tracks": [
                {"track_id": vehicle, "class_id": "heavy_truck", "source": "EXTERNAL"},
                {"track_id": pedestrian, "class_id": "pedestrian", "source": "EXTERNAL"},
                {"track_id": motorcycle, "class_id": "motorcycle", "source": "EXTERNAL"},
            ],
        }

        audit = audit_registry_ncore_dynamic_closure(registry, ncore)

        self.assertEqual(audit["status"], "passed")
        self.assertEqual(audit["summary"]["matched_track_count"], 3)

    def test_ncore_dynamic_closure_uses_explicit_selected_tracks_when_present(self):
        from adapters.nurec_inventory import audit_registry_ncore_dynamic_closure

        required, ignored = "a" * 32, "b" * 32
        registry = {
            "schema_version": "scene_object_registry.v1",
            "scene_id": "scene",
            "records": [{
                "object_id": required,
                "semantic_class": "vehicle",
                "role": "background_replay",
                "carla": {"representation": "physical_actor", "collision_policy": "required"},
                "nurec": {"representation": "dynamic_track", "track_id": required},
            }],
        }
        ncore = {
            "schema_version": 1,
            "pass": True,
            "eligible_tracks": [
                {"track_id": required, "class_id": "automobile", "source": "EXTERNAL"},
                {"track_id": ignored, "class_id": "automobile", "source": "EXTERNAL"},
            ],
            "selected_tracks": [{"track_id": required, "class_id": "automobile", "source": "EXTERNAL"}],
        }

        audit = audit_registry_ncore_dynamic_closure(registry, ncore)

        self.assertEqual(audit["status"], "passed")
        self.assertEqual(audit["ncore_track_collection"], "selected_tracks")

    def test_ncore_dynamic_closure_rejects_unresolved_explicit_selection(self):
        from adapters.nurec_inventory import audit_registry_ncore_dynamic_closure

        track_id = "a" * 32
        registry = {
            "schema_version": "scene_object_registry.v1",
            "scene_id": "scene",
            "records": [{
                "object_id": track_id,
                "semantic_class": "vehicle",
                "role": "background_replay",
                "carla": {"representation": "physical_actor", "collision_policy": "required"},
                "nurec": {"representation": "dynamic_track", "track_id": track_id},
            }],
        }
        ncore = {
            "schema_version": 1,
            "pass": False,
            "selected_tracks": [{"track_id": track_id, "class_id": "automobile", "source": "EXTERNAL"}],
            "selected_track_ids_missing_from_eligible": ["b" * 32],
        }

        audit = audit_registry_ncore_dynamic_closure(registry, ncore)

        self.assertEqual(audit["status"], "failed")
        self.assertIn("ncore_selected_track_ids_missing_from_eligible", audit["issues"])
    def test_registry_source_content_audit_is_fail_closed_for_missing_and_static_objects(self):
        from adapters.nurec_inventory import audit_registry_source_content

        registry = {
            "schema_version": "scene_object_registry.v1",
            "scene_id": "scene",
            "records": [
                {
                    "object_id": VEHICLE_TRACK,
                    "role": "background_replay",
                    "nurec": {"track_id": VEHICLE_TRACK},
                },
                {
                    "object_id": "static:parked",
                    "role": "static_obstacle",
                    "nurec": {"track_id": None},
                },
                {
                    "object_id": "road_boundary:carla_map",
                    "role": "road_boundary",
                    "nurec": {"track_id": None},
                },
            ],
        }
        inventory = {
            "schema_version": "nurec_runtime_track_inventory.v1",
            "tracks": [
                {
                    "track_id": VEHICLE_TRACK,
                    "dynamic_object_pose_verified": True,
                    "issues": [],
                }
            ],
        }

        audit = audit_registry_source_content(registry, inventory)

        self.assertEqual(audit["status"], "failed")
        self.assertEqual(audit["summary"]["verified_count"], 1)
        self.assertEqual(audit["summary"]["missing_from_artifact_count"], 0)
        self.assertIn(
            "static_source_content_evidence_missing",
            {issue for row in audit["issues"] for issue in row["issues"]},
        )

    def test_registry_source_content_audit_distinguishes_track_absent_from_probe_failure(self):
        from adapters.nurec_inventory import audit_registry_source_content

        missing = "f" * 32
        failed = "e" * 32
        registry = {
            "schema_version": "scene_object_registry.v1",
            "scene_id": "scene",
            "records": [
                {"object_id": missing, "role": "background_replay", "nurec": {"track_id": missing}},
                {"object_id": failed, "role": "background_replay", "nurec": {"track_id": failed}},
            ],
        }
        inventory = {
            "schema_version": "nurec_runtime_track_inventory.v1",
            "tracks": [
                {
                    "track_id": failed,
                    "dynamic_object_pose_verified": False,
                    "issues": ["lidar_render_unchanged"],
                }
            ],
        }

        audit = audit_registry_source_content(registry, inventory)
        rows = {row["object_id"]: row for row in audit["records"]}

        self.assertEqual(rows[missing]["status"], "missing_from_artifact")
        self.assertEqual(rows[failed]["status"], "unverified")
        self.assertEqual(audit["summary"]["missing_from_artifact_count"], 1)
        self.assertEqual(audit["summary"]["unverified_count"], 1)

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
