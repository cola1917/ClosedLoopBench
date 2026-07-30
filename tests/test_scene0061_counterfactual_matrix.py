import copy
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path


class Scene0061CounterfactualMatrixTests(unittest.TestCase):
    def test_builds_deterministic_frozen_s0_s7_contract(self):
        from runtime.scene0061_counterfactual import (
            CASE_IDS,
            PEDESTRIAN_TRACK,
            VEHICLE_TRACK,
            build_scene0061_counterfactual_matrix,
        )

        first = build_scene0061_counterfactual_matrix()
        second = build_scene0061_counterfactual_matrix()

        self.assertEqual(first, second)
        self.assertEqual(tuple(case["case_id"] for case in first["cases"]), CASE_IDS)
        self.assertEqual(first["actors"]["lead_vehicle"]["track_id"], VEHICLE_TRACK)
        self.assertEqual(first["actors"]["pedestrian"]["track_id"], PEDESTRIAN_TRACK)
        self.assertEqual(first["seeds"], [41, 42, 43])
        self.assertTrue(all(case["remote_validation_required"] for case in first["cases"]))
        removal = first["cases"][-1]
        self.assertTrue(removal["quality_stress_only"])
        self.assertFalse(removal["perception_ranking_allowed"])
        self.assertTrue(all(algorithm["checkpoint_sha256"] == "not_applicable" for algorithm in first["algorithms"]))
        self.assertEqual(first["cases"][4]["required_actor_outcomes"], ["crossing"])
        self.assertEqual(first["cases"][5]["required_actor_outcomes"], ["yield"])
        self.assertEqual(first["cases"][6]["required_actor_outcomes"], ["crossing"])

    def test_formal_matrix_binds_multiroad_canonical_opendrive(self):
        from runtime.scene0061_counterfactual import (
            CounterfactualMatrixError,
            build_scene0061_counterfactual_matrix,
            validate_scene0061_counterfactual_matrix,
        )

        matrix = build_scene0061_counterfactual_matrix()
        opendrive = next(
            item for item in matrix["immutable_inputs"] if item["role"] == "opendrive"
        )
        self.assertEqual(
            opendrive["logical_ref"],
            "evidence://scene0061/scene0061_exchange_v2/road.xodr",
        )
        self.assertEqual(opendrive["road_count"], 229)
        self.assertEqual(opendrive["junction_count"], 17)
        self.assertEqual(opendrive["ego_corridor_road_count"], 0)

        opendrive["logical_ref"] = "evidence://scene0061/road.nurec-route-extended-both-v7.xodr"
        with self.assertRaisesRegex(CounterfactualMatrixError, "canonical scene0061"):
            validate_scene0061_counterfactual_matrix(matrix)

        matrix = build_scene0061_counterfactual_matrix()
        opendrive = next(
            item for item in matrix["immutable_inputs"] if item["role"] == "opendrive"
        )
        opendrive["ego_corridor_road_count"] = 1
        with self.assertRaisesRegex(CounterfactualMatrixError, "must not include"):
            validate_scene0061_counterfactual_matrix(matrix)

    def test_checked_in_formal_matrix_matches_generator(self):
        from runtime.scene0061_counterfactual import (
            build_scene0061_counterfactual_matrix,
            validate_scene0061_counterfactual_matrix,
        )

        path = Path(__file__).resolve().parents[1] / "configs" / "scene0061_counterfactual_matrix.v1.json"
        checked_in = json.loads(path.read_text(encoding="utf-8"))
        validate_scene0061_counterfactual_matrix(checked_in)
        self.assertEqual(checked_in, build_scene0061_counterfactual_matrix())

    def test_rejects_pedestrian_free_space_vehicle_large_shift_and_removal_promotion(self):
        from runtime.scene0061_counterfactual import (
            CounterfactualMatrixError,
            build_scene0061_counterfactual_matrix,
            validate_scene0061_counterfactual_matrix,
        )

        matrix = build_scene0061_counterfactual_matrix()
        matrix["actors"]["pedestrian"]["free_space_path_allowed"] = True
        with self.assertRaisesRegex(CounterfactualMatrixError, "free-space"):
            validate_scene0061_counterfactual_matrix(matrix)

        matrix = build_scene0061_counterfactual_matrix()
        matrix["cases"][3]["edit"]["parameters"]["longitudinal_shift_m"] = 20.0
        with self.assertRaisesRegex(CounterfactualMatrixError, "light-edit"):
            validate_scene0061_counterfactual_matrix(matrix)

        matrix = build_scene0061_counterfactual_matrix()
        matrix["cases"][-1]["perception_ranking_allowed"] = True
        with self.assertRaisesRegex(CounterfactualMatrixError, "quality-stress-only"):
            validate_scene0061_counterfactual_matrix(matrix)

    def test_rejects_tampered_hash_and_algorithm_config(self):
        from runtime.scene0061_counterfactual import (
            CounterfactualMatrixError,
            build_scene0061_counterfactual_matrix,
            validate_scene0061_counterfactual_matrix,
        )

        matrix = build_scene0061_counterfactual_matrix()
        matrix["scene_identity"]["artifact_sha256"] = "0" * 64
        with self.assertRaisesRegex(CounterfactualMatrixError, "immutable_matrix_sha256"):
            validate_scene0061_counterfactual_matrix(matrix)

        matrix = build_scene0061_counterfactual_matrix()
        matrix["algorithms"][0]["parameters"]["lookahead_m"] = 99.0
        with self.assertRaisesRegex(CounterfactualMatrixError, "config hash"):
            validate_scene0061_counterfactual_matrix(matrix)

    def test_cli_build_and_validate_round_trip(self):
        from runners.build_scene0061_counterfactual_matrix import main

        with tempfile.TemporaryDirectory() as tmpdir:
            output = Path(tmpdir) / "matrix.json"
            validation_output = Path(tmpdir) / "validation.json"
            self.assertEqual(main(["--output", str(output)]), 0)
            payload = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema_version"], "scene_counterfactual_matrix.v1")
            self.assertEqual(
                main(["--validate", str(output), "--validation-output", str(validation_output)]),
                0,
            )
            validation = json.loads(validation_output.read_text(encoding="utf-8"))
            self.assertEqual(validation["status"], "passed")
            self.assertEqual(validation["validation_class"], "offline_conformance")
            self.assertEqual(validation["expected_remote_run_count"], 48)
            self.assertTrue(validation["remote_validation_required"])

            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main(["--output", str(output)])


if __name__ == "__main__":
    unittest.main()
