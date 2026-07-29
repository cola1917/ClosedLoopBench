import unittest


def _registry():
    return {
        "schema_version": "scene_object_registry.v1",
        "records": [
            {"object_id": "lead", "role": "controlled_lead_vehicle", "semantic_class": "vehicle", "nurec": {"track_id": "lead-track"}},
            {"object_id": "ped", "role": "controlled_pedestrian", "semantic_class": "pedestrian", "nurec": {"track_id": "ped-track"}},
            {"object_id": "road", "role": "road_boundary", "semantic_class": "road_boundary", "nurec": {"track_id": None}},
        ],
    }


def _config():
    return {
        "checkpoint": {"artifact": {"sequence_tracks": {"enabled": False}}},
        "dataset": {"n_samples_per_epoch": 40000, "camera_ids": ["old"], "lidar_ids": ["old"], "cuboid_tracks_params": {"track_label_sources": ["AUTOLABEL"]}, "generate_static_rigid_cuboid_tracks": {"enabled": True}},
        "trainer": {"max_epochs": 40},
        "model": {"layers": {"dynamic_rigids": {"tracks": {}}, "dynamic_deformables": {"tracks": {}}}},
    }


def _selection():
    return {"schema_version": "nurec_render_selection.v1", "status": "passed", "selected_object_ids": ["lead", "ped", "road"]}


def _quality():
    return {"schema_version": "lidar_quality_window_manifest.v1", "status": "passed", "candidate_object_ids": ["lead", "ped"], "required_object_ids": ["lead", "ped"]}


class NuRecCandidateConfigTests(unittest.TestCase):
    def test_derives_bounded_layer_correct_config(self):
        from adapters.nurec_candidate_config import derive_candidate_config

        result = derive_candidate_config(_config(), _registry(), _selection(), _quality())
        self.assertEqual(result["dataset"]["n_samples_per_epoch"], 1000)
        self.assertEqual(result["trainer"]["max_epochs"], 1)
        self.assertEqual(result["dataset"]["camera_ids"], ["camera_front", "camera_front_left", "camera_front_right", "camera_back", "camera_back_left", "camera_back_right"])
        self.assertEqual(result["dataset"]["lidar_ids"], ["lidar_top"])
        self.assertEqual(result["dataset"]["cuboid_tracks_params"]["track_label_sources"], ["EXTERNAL"])
        self.assertEqual(result["model"]["layers"]["dynamic_rigids"]["tracks"]["ids"], ["lead-track"])
        self.assertEqual(result["model"]["layers"]["dynamic_deformables"]["tracks"]["ids"], ["ped-track"])
        self.assertFalse(result["dataset"]["generate_static_rigid_cuboid_tracks"]["enabled"])

    def test_rejects_unrepresented_static_candidate(self):
        from adapters.nurec_candidate_config import derive_candidate_config, NuRecCandidateConfigError

        registry = _registry()
        registry["records"].append({"object_id": "static", "role": "static_obstacle", "semantic_class": "object", "nurec": {"track_id": None}})
        selection = _selection()
        selection["selected_object_ids"].append("static")
        with self.assertRaisesRegex(NuRecCandidateConfigError, "static NuRec geometry"):
            derive_candidate_config(_config(), registry, selection, _quality())

    def test_rejects_failed_quality_window(self):
        from adapters.nurec_candidate_config import derive_candidate_config, NuRecCandidateConfigError

        quality = _quality()
        quality["status"] = "failed"
        with self.assertRaisesRegex(NuRecCandidateConfigError, "not passed"):
            derive_candidate_config(_config(), _registry(), _selection(), quality)


if __name__ == "__main__":
    unittest.main()
