import hashlib
import json
import pickle
import tempfile
import unittest
import zipfile
from pathlib import Path


TOKEN = "cc8c0bf57f984915a77078b10eb33198"


class ReconstructionPlanningTests(unittest.TestCase):
    def _fixture(self, root: Path, *, step: int = 1000):
        from adapters.reconstruction_package import load_reconstruction_package

        artifact_dir = root / "reconstruction"
        artifact_dir.mkdir()
        usdz = artifact_dir / "last.usdz"
        checkpoint = artifact_dir / "last.ckpt"
        config = artifact_dir / "parsed.yaml"
        usdz.write_bytes(b"usdz")
        with zipfile.ZipFile(checkpoint, "w") as archive:
            archive.writestr("archive/data.pkl", pickle.dumps({"global_step": step}, protocol=2))
        config.write_text(
            json.dumps({
                "dataset": {
                    "camera_ids": ["camera_front", "camera_front_left", "camera_front_right"],
                    "n_samples_per_epoch": 1000,
                },
                "trainer": {"max_epochs": 1},
            }),
            encoding="utf-8",
        )
        items = []
        for role, path in (
            ("nurec_usdz", usdz),
            ("nurec_checkpoint", checkpoint),
            ("nurec_config", config),
        ):
            items.append({
                "role": role,
                "path": path.relative_to(root).as_posix(),
                "media_type": "application/octet-stream",
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "size_bytes": path.stat().st_size,
            })
        package_path = root / "reconstruction_package.json"
        package_path.write_text(json.dumps({
            "schema_version": "reconstruction_package.v1",
            "scene_id": TOKEN,
            "source": {"dataset": "nuscenes", "scene_token": TOKEN},
            "backend": {"name": "nurec", "version": "26.04"},
            "artifacts": items,
            "alignment": {"status": "pending_runtime_alignment"},
        }), encoding="utf-8")
        return load_reconstruction_package(package_path, expected_scene_id=TOKEN)

    def _scenario_ir(self):
        return {
            "scenario_id": TOKEN,
            "source": {"dataset": "nuscenes", "scene_name": "scene-0061", "scene_token": TOKEN},
        }

    def test_accepts_strict_1000_step_result_and_records_runtime_boundary(self):
        from adapters.reconstruction_planning import build_reconstruction_integration_plan

        with tempfile.TemporaryDirectory() as directory:
            plan = build_reconstruction_integration_plan(
                self._scenario_ir(), self._fixture(Path(directory))
            )
        self.assertEqual(plan["validation"]["status"], "passed")
        self.assertEqual(plan["validation"]["training_gate"]["global_step"], 1000)
        self.assertEqual(plan["closed_loop_plan"]["carla_visual_binding"], "not_implemented")
        self.assertEqual(len(plan["reconstruction"]["nurec_usdz"]["sha256"]), 64)

    def test_rejects_non_1000_step_checkpoint(self):
        from adapters.reconstruction_planning import (
            ReconstructionPlanningError,
            build_reconstruction_integration_plan,
        )

        with tempfile.TemporaryDirectory() as directory:
            package = self._fixture(Path(directory), step=999)
            with self.assertRaisesRegex(ReconstructionPlanningError, "expected global_step=1000"):
                build_reconstruction_integration_plan(self._scenario_ir(), package)

    def test_materializing_package_in_place_does_not_copy_file_onto_itself(self):
        from adapters.reconstruction_package import materialize_reconstruction_package

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = self._fixture(root)
            copied = materialize_reconstruction_package(package, root)
        self.assertEqual(copied["nurec_usdz"], "reconstruction/last.usdz")
        self.assertEqual(copied["reconstruction_package"], "reconstruction_package.json")


if __name__ == "__main__":
    unittest.main()
