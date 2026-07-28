import json
import tempfile
import unittest
import zipfile
from pathlib import Path


class DeriveNuRecArtifactCoordinateConfigTests(unittest.TestCase):
    def test_inverts_artifact_world_transform_for_wire_requests(self):
        from runners.derive_nurec_artifact_coordinate_config import (
            derive_artifact_coordinate_config,
        )

        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "scene.usdz"
            with zipfile.ZipFile(artifact, "w") as archive:
                archive.writestr(
                    "rig_trajectories.json",
                    json.dumps(
                        {
                            "T_world_base": [
                                [0.0, -1.0, 0.0, 10.0],
                                [1.0, 0.0, 0.0, -2.0],
                                [0.0, 0.0, 1.0, 5.0],
                                [0.0, 0.0, 0.0, 1.0],
                            ]
                        }
                    ),
                )
            result = derive_artifact_coordinate_config(
                {"nurec_runtime": {"runtime_scene_id": "scene-0061"}},
                artifact_path=artifact,
            )

        self.assertEqual(
            result["nurec_runtime"]["nre_from_log_transform"],
            [0.0, 1.0, 0.0, 2.0, -1.0, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, -5.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.assertEqual(
            result["nurec_runtime"]["nre_from_log_transform_identity"]["direction"],
            "nre_render_from_log_world",
        )

    def test_rejects_double_coordinate_transformation(self):
        from runners.derive_nurec_artifact_coordinate_config import (
            derive_artifact_coordinate_config,
        )

        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "scene.usdz"
            with zipfile.ZipFile(artifact, "w") as archive:
                archive.writestr("rig_trajectories.json", json.dumps({"T_world_base": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}))
            with self.assertRaisesRegex(ValueError, "already has"):
                derive_artifact_coordinate_config(
                    {"nurec_runtime": {"nre_from_log_transform": [1] * 16}},
                    artifact_path=artifact,
                )


if __name__ == "__main__":
    unittest.main()
