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


def _identity(path: Path) -> dict:
    return {
        "path": str(path.resolve()),
        "sha256": _sha(path),
        "byte_count": path.stat().st_size,
    }


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
    def _fresh_evidence_fixture(self, root: Path):
        from runners.derive_scene0061_formal_base_config import derive_formal_base_file

        matrix, paths, execution, source = _fixture(root)
        source_path = root / "source.json"
        matrix_path = root / "matrix.json"
        base_path = root / "formal.json"
        source_path.write_text(json.dumps(source), encoding="utf-8")
        matrix_path.write_text(json.dumps(matrix), encoding="utf-8")
        with patch(
            "runners.derive_scene0061_formal_base_config.validate_scene0061_counterfactual_matrix"
        ):
            derive_formal_base_file(
                source_config=source_path,
                matrix=matrix_path,
                immutable_paths=paths,
                execution_paths=execution,
                formal_run_id="formal-live",
                output=base_path,
            )
        base = json.loads(base_path.read_text(encoding="utf-8"))
        output_root = root / "live-tick"
        output_root.mkdir()
        snapshot = output_root / "runtime_run_config.json"
        snapshot.write_bytes(base_path.read_bytes())
        validation = output_root / "live_tick_validation.json"
        validation.write_text(json.dumps({"status": "passed"}), encoding="utf-8")
        evidence_path = output_root / "lidar_axis_evidence.json"
        evidence = {
            "schema_version": "scene0061_lidar_coordinate_validation.v1",
            "status": "passed",
            "scene_id": base["experiment"]["scene_id"],
            "runtime_scene_id": base["nurec_runtime"]["runtime_scene_id"],
            "artifact_sha256": base["experiment"]["artifact_sha256"],
            "sensor_id": "lidar_top",
            "response_coordinate_frame": "sensor_local",
            "axis_convention": "carla_sensor",
            "sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
            "axis_validation": {"status": "passed"},
            "gate_replay": {"status": "passed"},
            "live_tick_provenance": {
                "schema_version": "scene0061_lidar_live_tick_provenance.v1",
                "run_id": "formal-live",
                "selected_config": _identity(base_path),
                "runtime_config": _identity(snapshot),
                "opendrive": _identity(execution["runtime_opendrive"]),
                "live_tick_validation": {**_identity(validation), "status": "passed"},
            },
        }
        evidence_path.write_text(json.dumps(evidence), encoding="utf-8")
        return base_path, evidence_path, evidence, execution

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

    def test_evidence_binding_accepts_same_run_config_opendrive_and_output(self):
        from runners.derive_scene0061_formal_base_config import bind_lidar_evidence_file

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base_path, evidence_path, _, _ = self._fresh_evidence_fixture(root)
            output = root / "formal-bound.json"
            result = bind_lidar_evidence_file(
                base_config=base_path,
                lidar_evidence=evidence_path,
                bound_run_id="formal-bound",
                output=output,
            )

            bound = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(result["status"], "passed")
            self.assertEqual(bound["run_id"], "formal-bound")
            self.assertEqual(bound["formal_lidar_evidence_binding"]["evidence_status"], "passed")
            self.assertEqual(
                bound["nurec_runtime"]["lidar_coordinate_validation"]["evidence_sha256"],
                _sha(evidence_path),
            )

    def test_evidence_binding_rejects_validation_from_another_output_directory(self):
        from runners.derive_scene0061_formal_base_config import (
            Scene0061FormalBaseError,
            bind_lidar_evidence_file,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base_path, evidence_path, evidence, _ = self._fresh_evidence_fixture(root)
            other = root / "other"
            other.mkdir()
            validation = other / "live_tick_validation.json"
            validation.write_text(json.dumps({"status": "passed"}), encoding="utf-8")
            evidence["live_tick_provenance"]["live_tick_validation"] = {
                **_identity(validation),
                "status": "passed",
            }
            evidence_path.write_text(json.dumps(evidence), encoding="utf-8")

            with self.assertRaisesRegex(Scene0061FormalBaseError, "same output directory"):
                bind_lidar_evidence_file(
                    base_config=base_path,
                    lidar_evidence=evidence_path,
                    bound_run_id="formal-bound",
                    output=root / "must-not-exist.json",
                )
            self.assertFalse((root / "must-not-exist.json").exists())

    def test_evidence_binding_rejects_a_different_formal_run_id(self):
        from runners.derive_scene0061_formal_base_config import (
            Scene0061FormalBaseError,
            bind_lidar_evidence_file,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base_path, evidence_path, evidence, _ = self._fresh_evidence_fixture(root)
            evidence["live_tick_provenance"]["run_id"] = "another-formal-run"
            evidence_path.write_text(json.dumps(evidence), encoding="utf-8")

            with self.assertRaisesRegex(Scene0061FormalBaseError, "run_id does not match"):
                bind_lidar_evidence_file(
                    base_config=base_path,
                    lidar_evidence=evidence_path,
                    bound_run_id="formal-bound",
                    output=root / "must-not-exist.json",
                )
            self.assertFalse((root / "must-not-exist.json").exists())


if __name__ == "__main__":
    unittest.main()
