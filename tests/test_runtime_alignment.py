import copy
import unittest


TOKEN = "cc8c0bf57f984915a77078b10eb33198"


def _package():
    from tests.test_scene_package import _coordinate_frame

    return {
        "schema_version": "closed_loop_scene_package.v1",
        "scene_id": TOKEN,
        "source": {"dataset": "nuscenes", "scene_id": TOKEN, "scene_token": TOKEN},
        "coordinate_frame": _coordinate_frame(),
        "motion": {"scene_ir": "scene_ir.json"},
        "map": {"location": "test", "opendrive": "road.xodr", "source": "fixture"},
        "scenario": {"openscenario": "scenario.xosc"},
        "visual": {"nurec_usdz": "last.usdz", "nurec_checkpoint": None, "reconstruction_package": "reconstruction_package.json"},
        "alignment": {
            "sim_from_log_transform": [1.0, 0.0, 0.0, -10.0, 0.0, 1.0, 0.0, -20.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0],
            "matrix_layout": "row_major_4x4",
            "source_frame": "nuscenes_global",
            "target_frame": "scene_local_ego_start",
            "status": "log_to_sim_defined",
        },
    }


def _observations():
    return {
        "schema_version": "runtime_alignment_observations.v1",
        "scene_id": TOKEN,
        "captured_at": "2026-07-15T12:00:00Z",
        "runtime": {
            "simulator": "CARLA 0.9.16",
            "renderer": "NuRec",
            "capture_method": "surveyed-runtime-landmarks",
            "nurec_artifact_sha256": "a" * 64,
        },
        "landmarks": [
            {"landmark_id": "origin", "log_global": {"x": 10, "y": 20, "z": 1, "yaw_deg": 0}, "sim_measured": {"x": 0, "y": 0, "z": 0, "yaw_deg": 0}},
            {"landmark_id": "east", "log_global": {"x": 20, "y": 20, "z": 1}, "sim_measured": {"x": 10, "y": 0, "z": 0}},
            {"landmark_id": "north", "log_global": {"x": 10, "y": 30, "z": 1}, "sim_measured": {"x": 0, "y": 10, "z": 0}},
        ],
    }


class RuntimeAlignmentTests(unittest.TestCase):
    def test_passed_landmarks_can_promote_a_new_scene_package(self):
        from adapters.runtime_alignment import promote_runtime_validated_package, validate_runtime_alignment

        evidence = validate_runtime_alignment(_package(), _observations())
        self.assertEqual(evidence["status"], "passed")
        promoted = promote_runtime_validated_package(
            _package(), evidence, evidence_path="runtime_alignment_evidence.json"
        )
        self.assertEqual(promoted["alignment"]["status"], "runtime_validated")

    def test_threshold_failure_cannot_promote(self):
        from adapters.runtime_alignment import RuntimeAlignmentError, promote_runtime_validated_package, validate_runtime_alignment

        observations = _observations()
        observations["landmarks"][1]["sim_measured"]["x"] = 11.0
        evidence = validate_runtime_alignment(_package(), observations)
        self.assertEqual(evidence["status"], "failed")
        with self.assertRaisesRegex(RuntimeAlignmentError, "failed alignment"):
            promote_runtime_validated_package(
                _package(), evidence, evidence_path="runtime_alignment_evidence.json"
            )

    def test_rejects_collinear_or_wrong_scene_observations(self):
        from adapters.runtime_alignment import RuntimeAlignmentError, validate_runtime_alignment

        observations = _observations()
        observations["landmarks"][2]["log_global"].update(x=30, y=20)
        with self.assertRaisesRegex(RuntimeAlignmentError, "non-collinear"):
            validate_runtime_alignment(_package(), observations)
        observations = _observations()
        observations["scene_id"] = "d" * 32
        with self.assertRaisesRegex(RuntimeAlignmentError, "scene_id"):
            validate_runtime_alignment(_package(), observations)


if __name__ == "__main__":
    unittest.main()
