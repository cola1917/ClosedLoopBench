import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


def _source_config() -> dict:
    return {
        "schema_version": "carla_run_config.mvp.v0",
        "run_id": "r18-source",
        "scenario_id": "scene-0061",
        "carla": {"map": "Town04"},
        "ego": {
            "initial_state": {"x": 0.0, "y": 0.0},
            "reference_trajectory": [{"x": 1.0, "y": 0.0}],
        },
        "experiment": {"identity": {"code_commit": "a" * 40}},
        "nurec_runtime": {
            "runtime_scene_id": "scene-0061",
            "lidar_specs": [{"sensor_id": "lidar_top", "sensor_to_ego": [1] * 16}],
        },
    }


class DeriveScene0061LiDARAxisConfigTests(unittest.TestCase):
    def test_derives_validated_axis_contract_and_source_identity(self):
        from runners.derive_scene0061_lidar_axis_config import (
            R18_RESPONSE_TO_SENSOR,
            derive_lidar_axis_config_file,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "r18.json"
            source.write_text(json.dumps(_source_config(), indent=2) + "\n", encoding="utf-8")
            original = source.read_bytes()
            output = root / "r19-axis-bound.json"

            result = derive_lidar_axis_config_file(source, output)
            derived = json.loads(output.read_text(encoding="utf-8"))

            self.assertEqual(derived["run_id"], "r18-source")
            self.assertNotIn("lidar_axis_normalization", _source_config()["nurec_runtime"])
            contract = derived["nurec_runtime"]["lidar_axis_normalization"]
            self.assertEqual(contract["response_to_sensor"], R18_RESPONSE_TO_SENSOR)
            self.assertEqual(contract["source_coordinate_frame"], "nre_26_04_lidar_sensor")
            self.assertEqual(contract["target_axis_convention"], "carla_sensor")
            self.assertEqual(len(contract["response_to_sensor_sha256"]), 64)
            provenance = derived["config_derivation"]
            self.assertEqual(provenance["schema_version"], "scene0061_lidar_axis_config_derivation.v1")
            self.assertEqual(provenance["source_config"]["path"], str(source.resolve()))
            self.assertEqual(provenance["source_config"]["sha256"], hashlib.sha256(original).hexdigest())
            self.assertEqual(provenance["source_config"]["byte_count"], len(original))
            self.assertEqual(provenance["lidar_axis_normalization_sha256"], contract["response_to_sensor_sha256"])
            self.assertEqual(result["output_sha256"], hashlib.sha256(output.read_bytes()).hexdigest())

    def test_rejects_already_bound_or_ambiguous_lidar_source(self):
        from runners.derive_scene0061_lidar_axis_config import (
            Scene0061LiDARConfigDerivationError,
            derive_lidar_axis_config,
            lidar_axis_normalization_contract,
        )

        source = _source_config()
        source["nurec_runtime"]["lidar_axis_normalization"] = lidar_axis_normalization_contract()
        with self.assertRaisesRegex(Scene0061LiDARConfigDerivationError, "already declares"):
            derive_lidar_axis_config(
                source, source_path=Path("r18.json"), source_sha256="a" * 64, source_byte_count=1
            )

        source = _source_config()
        source["nurec_runtime"]["lidar_specs"] = [{"sensor_id": "lidar_other"}]
        with self.assertRaisesRegex(Scene0061LiDARConfigDerivationError, "exactly one lidar_top"):
            derive_lidar_axis_config(
                source, source_path=Path("r18.json"), source_sha256="a" * 64, source_byte_count=1
            )

    def test_cli_writes_once_and_refuses_an_existing_output(self):
        from runners.derive_scene0061_lidar_axis_config import main

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "r18.json"
            source.write_text(json.dumps(_source_config()), encoding="utf-8")
            output = root / "new" / "r19.json"
            args = ["--source-config", str(source), "--output", str(output)]
            self.assertEqual(main(args), 0)
            original_output = output.read_bytes()
            self.assertEqual(main(args), 2)
            self.assertEqual(output.read_bytes(), original_output)

    def test_cli_runs_by_absolute_script_path_outside_the_repository(self):
        """The remote runbook executes this tool by absolute script path."""
        with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryDirectory() as cwd:
            root = Path(directory)
            source = root / "r18.json"
            source.write_text(json.dumps(_source_config()), encoding="utf-8")
            output = root / "r19.json"
            script = (
                Path(__file__).resolve().parents[1]
                / "runners"
                / "derive_scene0061_lidar_axis_config.py"
            )
            completed = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--source-config",
                    str(source),
                    "--output",
                    str(output),
                ],
                cwd=cwd,
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual(json.loads(completed.stdout)["status"], "passed")
            self.assertTrue(output.is_file())

    def test_derived_config_is_accepted_as_an_explicit_live_tick_input(self):
        from runners.derive_scene0061_lidar_axis_config import derive_lidar_axis_config_file
        from runners.scene0061_live_tick import prepare_live_tick

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "r18.json"
            source.write_text(json.dumps(_source_config()), encoding="utf-8")
            derived_path = root / "r19-axis-bound.json"
            derive_lidar_axis_config_file(source, derived_path)
            xodr = root / "scene.xodr"
            xodr.write_text("<OpenDRIVE/>\n", encoding="utf-8")

            environment = prepare_live_tick(
                config_path=derived_path,
                output_dir=root / "diagnostic",
                run_id="scene0061-live-tick-r19",
                opendrive_path=xodr,
                capture_native_lidar=True,
            )

            self.assertEqual(environment["status"], "prepared")
            self.assertEqual(environment["config"]["path"], str(derived_path.resolve()))
            snapshot = json.loads(
                (root / "diagnostic" / "runtime_run_config.json").read_text(encoding="utf-8")
            )
            self.assertEqual(
                snapshot["config_derivation"]["source_config"]["sha256"],
                hashlib.sha256(source.read_bytes()).hexdigest(),
            )
            self.assertEqual(
                snapshot["nurec_runtime"]["lidar_axis_normalization"]["response_to_sensor"],
                [-0.0, 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            )


if __name__ == "__main__":
    unittest.main()
