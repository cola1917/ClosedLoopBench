from __future__ import annotations

import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixture(root: Path):
    from runners.derive_scene0061_lidar_axis_config import lidar_axis_normalization_contract
    scene_id = "cc8c0bf57f984915a77078b10eb33198"
    roles = ("nurec_usdz", "runtime_validated_scene_package", "actor_ready_scenario_ir", "actor_selection", "opendrive")
    paths = {}
    rows = []
    for role in roles:
        path = root / f"{role}.blob"
        path.write_bytes(role.encode())
        paths[role] = path
        rows.append({"role": role, "sha256": _sha(path)})
    execution = {}
    for role in ("runtime_scene_package", "runtime_actor_bindings", "runtime_native_scan_manifest", "runtime_opendrive"):
        path = root / f"{role}.json"
        path.write_bytes(role.encode())
        execution[role] = path
    matrix = {
        "scene_identity": {"scene_id": scene_id, "scene_version": "formal40k-v1", "artifact_sha256": "1" * 64, "scene_package_sha256": _sha(paths["runtime_validated_scene_package"]), "scenario_ir_sha256": _sha(paths["actor_ready_scenario_ir"])},
        "immutable_inputs": rows, "immutable_matrix_sha256": "2" * 64,
    }
    source = {
        "run_id": "r19", "scenario_id": scene_id, "carla": {"fixed_delta_seconds": 0.05},
        "experiment": {"scene_version": "smoke", "identity": {"artifact_sha256": "1" * 64, "runtime_opendrive_sha256": _sha(execution["runtime_opendrive"]), "runtime_opendrive_source_sha256": _sha(paths["opendrive"])}},
        "config_derivation": {"schema_version": "axis"},
        "actor_binding": {"selected_actor_ids": ["c1958768d48640948f6053d04cffd35b", "71603dd1a2ba4e9daf095535e38310ac"]},
        "actors": [{"source_track_id": "c1958768d48640948f6053d04cffd35b"}, {"source_track_id": "71603dd1a2ba4e9daf095535e38310ac"}],
        "nurec_runtime": {
            "runtime_scene_id": "scene-0061", "scene_package": str(execution["runtime_scene_package"].resolve()),
            "actor_bindings": str(execution["runtime_actor_bindings"].resolve()), "actor_bindings_sha256": _sha(execution["runtime_actor_bindings"]),
            "native_scan_manifest": {"path": str(execution["runtime_native_scan_manifest"].resolve()), "sha256": _sha(execution["runtime_native_scan_manifest"])},
            "lidar_axis_normalization": lidar_axis_normalization_contract(),
            "camera_specs": [{"sensor_id": name, "width": 1600, "height": 900} for name in ("camera_front", "camera_front_left", "camera_front_right", "camera_back", "camera_back_left", "camera_back_right")],
            "lidar_specs": [{"sensor_id": "lidar_top", "model": "PANDAR128"}],
        },
    }
    return matrix, paths, execution, source


class FormalBaseDerivationTests(unittest.TestCase):
    def test_derives_hashed_formal_base_and_refuses_old_lidar_evidence(self):
        from runners.derive_scene0061_formal_base_config import derive_formal_base_file
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            matrix, paths, execution, source = _fixture(root)
            source_path, matrix_path, output = root / "source.json", root / "matrix.json", root / "formal.json"
            source_path.write_text(json.dumps(source), encoding="utf-8")
            matrix_path.write_text(json.dumps(matrix), encoding="utf-8")
            with patch("runners.derive_scene0061_formal_base_config.validate_scene0061_counterfactual_matrix"):
                result = derive_formal_base_file(source_config=source_path, matrix=matrix_path, immutable_paths=paths, execution_paths=execution, formal_run_id="formal-live", output=output)
            actual = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(result["status"], "passed")
            self.assertEqual(actual["experiment"]["scene_version"], "formal40k-v1")
            self.assertEqual(actual["nurec_runtime"]["lidar_axis_convention"], "carla_sensor")
            self.assertNotIn("lidar_coordinate_validation", actual["nurec_runtime"])
            self.assertEqual(actual["formal_base_derivation"]["source_run_config"]["sha256"], _sha(source_path))
            self.assertEqual({row["role"] for row in actual["formal_base_derivation"]["runtime_execution_inputs"]}, set(execution))

    def test_refuses_input_hash_drift_without_output(self):
        from runners.derive_scene0061_formal_base_config import Scene0061FormalBaseError, derive_formal_base_file
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            matrix, paths, execution, source = _fixture(root)
            source_path, matrix_path, output = root / "source.json", root / "matrix.json", root / "formal.json"
            source_path.write_text(json.dumps(source), encoding="utf-8")
            matrix_path.write_text(json.dumps(matrix), encoding="utf-8")
            paths["opendrive"].write_text("drift", encoding="utf-8")
            with patch("runners.derive_scene0061_formal_base_config.validate_scene0061_counterfactual_matrix"):
                with self.assertRaisesRegex(Scene0061FormalBaseError, "opendrive SHA-256"):
                    derive_formal_base_file(source_config=source_path, matrix=matrix_path, immutable_paths=paths, execution_paths=execution, formal_run_id="formal-live", output=output)
            self.assertFalse(output.exists())

    def test_refuses_runtime_input_drift_without_output(self):
        from runners.derive_scene0061_formal_base_config import Scene0061FormalBaseError, derive_formal_base_file
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            matrix, paths, execution, source = _fixture(root)
            source_path, matrix_path, output = root / "source.json", root / "matrix.json", root / "formal.json"
            source_path.write_text(json.dumps(source), encoding="utf-8")
            matrix_path.write_text(json.dumps(matrix), encoding="utf-8")
            execution["runtime_opendrive"].write_text("drift", encoding="utf-8")
            with patch("runners.derive_scene0061_formal_base_config.validate_scene0061_counterfactual_matrix"):
                with self.assertRaisesRegex(Scene0061FormalBaseError, "runtime_opendrive SHA-256"):
                    derive_formal_base_file(source_config=source_path, matrix=matrix_path, immutable_paths=paths, execution_paths=execution, formal_run_id="formal-live", output=output)
            self.assertFalse(output.exists())

    def test_evidence_binding_requires_fresh_formal_identity(self):
        from runners.derive_scene0061_formal_base_config import (
            Scene0061FormalBaseError,
            bind_lidar_evidence,
        )

        matrix, _, _, source = _fixture(Path(tempfile.mkdtemp()))
        source["experiment"].update({"scene_id": matrix["scene_identity"]["scene_id"], "scene_version": "formal40k-v1", "artifact_sha256": "1" * 64})
        source["formal_base_derivation"] = {}
        source["nurec_runtime"].update({"lidar_response_coordinate_frame": "sensor_local", "lidar_axis_convention": "carla_sensor", "lidar_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up"})
        with self.assertRaisesRegex(Scene0061FormalBaseError, "LiDAR evidence scene_id"):
            bind_lidar_evidence(source, {"path": "base", "sha256": "a" * 64}, {"schema_version": "scene0061_lidar_coordinate_validation.v1", "status": "passed", "axis_validation": {"status": "passed"}}, {"path": "evidence", "sha256": "b" * 64}, "bound")


if __name__ == "__main__":
    unittest.main()
