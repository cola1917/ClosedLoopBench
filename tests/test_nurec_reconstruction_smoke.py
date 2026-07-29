import unittest


def _registry(*, static=False):
    records = [
        {
            "object_id": "dynamic-a",
            "role": "background_replay",
            "safety_relevant": True,
            "nurec": {"track_id": "track-a"},
        }
    ]
    if static:
        records.append(
            {
                "object_id": "static-parked",
                "role": "static_obstacle",
                "safety_relevant": True,
                "nurec": {"track_id": None},
            }
        )
    return {
        "schema_version": "scene_object_registry.v1",
        "scene_id": "scene-0061",
        "records": records,
    }


def _config(*, samples=1000, static=False):
    config = {
        "checkpoint": {
            "artifact": {"sequence_tracks": {"enabled": True}}
        },
        "dataset": {
            "camera_ids": ["camera_front", "camera_back"],
            "n_samples_per_epoch": samples,
        },
        "trainer": {"max_epochs": 1},
    }
    config["dataset"]["generate_static_rigid_cuboid_tracks"] = {"enabled": static}
    return config


def _config_with_track_ids(*track_ids):
    config = _config()
    config["model"] = {
        "layers": {
            "dynamic_rigids": {"tracks": {"ids": list(track_ids)}},
            "dynamic_deformables": {"tracks": {"ids": []}},
        }
    }
    return config


class NuRecReconstructionSmokeTests(unittest.TestCase):
    def test_passes_small_dynamic_replay_preflight(self):
        from adapters.nurec_reconstruction_smoke import audit_reconstruction_smoke

        report = audit_reconstruction_smoke(
            _config(),
            _registry(),
            source_track_ids={"track-a", "extra-track"},
            expected_camera_ids=("camera_front", "camera_back"),
        )

        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["summary"]["source_track_count"], 2)

    def test_rejects_formal_budget_before_smoke(self):
        from adapters.nurec_reconstruction_smoke import audit_reconstruction_smoke

        report = audit_reconstruction_smoke(
            _config(samples=40000),
            _registry(),
            source_track_ids={"track-a"},
            expected_camera_ids=("camera_front", "camera_back"),
        )

        self.assertEqual(report["status"], "failed")
        self.assertIn("formal_training_budget_would_run_before_smoke", report["issues"])

    def test_rejects_missing_dynamic_track(self):
        from adapters.nurec_reconstruction_smoke import audit_reconstruction_smoke

        report = audit_reconstruction_smoke(
            _config(),
            _registry(),
            source_track_ids={"other-track"},
            expected_camera_ids=("camera_front", "camera_back"),
        )

        self.assertEqual(report["status"], "failed")
        self.assertEqual(
            report["checks"]["dynamic_track_coverage"]["missing_track_ids"],
            ["track-a"],
        )

    def test_rejects_static_registry_without_explicit_static_generation(self):
        from adapters.nurec_reconstruction_smoke import audit_reconstruction_smoke

        report = audit_reconstruction_smoke(
            _config(static=False),
            _registry(static=True),
            source_track_ids={"track-a"},
            expected_camera_ids=("camera_front", "camera_back"),
        )

        self.assertEqual(report["status"], "failed")
        self.assertIn("static_object_generation_disabled", report["issues"])

    def test_accepts_explicit_static_generation(self):
        from adapters.nurec_reconstruction_smoke import audit_reconstruction_smoke

        report = audit_reconstruction_smoke(
            _config(static=True),
            _registry(static=True),
            source_track_ids={"track-a"},
            expected_camera_ids=("camera_front", "camera_back"),
        )

        self.assertEqual(report["status"], "passed")

    def test_accepts_explicit_static_track_representation(self):
        from adapters.nurec_reconstruction_smoke import audit_reconstruction_smoke

        registry = _registry(static=True)
        registry["records"][1]["nurec"]["track_id"] = "static-track"
        report = audit_reconstruction_smoke(
            _config_with_track_ids("track-a", "static-track"),
            registry,
            source_track_ids={"track-a", "static-track"},
            expected_camera_ids=("camera_front", "camera_back"),
        )

        self.assertEqual(report["status"], "passed")


if __name__ == "__main__":
    unittest.main()
