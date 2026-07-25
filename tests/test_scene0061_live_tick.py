import json
import tempfile
import unittest
from pathlib import Path


def _config(sidecar: Path) -> dict:
    return {
        "scenario_id": "scene-0061",
        "run_id": "source-run",
        "ego": {
            "initial_state": {"x": 0.0, "y": 0.0},
            "reference_trajectory": [{"x": 1.0, "y": 0.0}],
        },
        "carla": {"map": "Town04"},
        "nurec_runtime": {
            "actor_bindings": str(sidecar),
            "actor_bindings_sha256": __import__("hashlib").sha256(sidecar.read_bytes()).hexdigest(),
        },
    }


class Scene0061LiveTickTests(unittest.TestCase):
    def _inputs(self, root: Path):
        sidecar = root / "bindings.json"
        sidecar.write_text("{}\n", encoding="utf-8")
        config = root / "smoke.json"
        config.write_text(json.dumps(_config(sidecar)), encoding="utf-8")
        xodr = root / "scene.xodr"
        xodr.write_text("<OpenDRIVE/>\n", encoding="utf-8")
        return config, xodr, sidecar

    def test_prepare_snapshots_explicit_inputs_and_records_hashes(self):
        from runners.scene0061_live_tick import prepare_live_tick

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, xodr, sidecar = self._inputs(root)
            output = root / "diagnostic"
            environment = prepare_live_tick(
                config_path=config,
                output_dir=output,
                run_id="r11-fixed",
                opendrive_path=xodr,
            )

            self.assertEqual(environment["status"], "prepared")
            self.assertEqual(environment["run_id"], "r11-fixed")
            self.assertEqual(environment["config"]["path"], str(config.resolve()))
            self.assertEqual(environment["actor_bindings"]["path"], str(sidecar.resolve()))
            self.assertEqual(environment["opendrive"]["path"], str(xodr.resolve()))
            self.assertEqual(
                environment["config"]["sha256"],
                environment["runtime_config"]["sha256"],
            )
            plan = json.loads((output / "basic_agent_plan.json").read_text(encoding="utf-8"))
            self.assertEqual(plan["limits"]["max_ticks"], 1)
            self.assertEqual(plan["run_id"], "r11-fixed")
            self.assertEqual(plan["world"]["opendrive_path"], str(xodr.resolve()))
            self.assertTrue(plan["runtime"]["multimodal_sensor_required"])
            provenance = plan["runtime"]["provenance"]
            self.assertEqual(provenance["selected_config_path"], str(config.resolve()))
            self.assertEqual(provenance["selected_config_sha256"], environment["config"]["sha256"])
            self.assertEqual(environment["config"]["byte_count"], config.stat().st_size)
            self.assertEqual(environment["python_runtime"]["executable"], str(Path(__import__("sys").executable).resolve()))
            self.assertIn("artifact_manifest", environment["artifacts"])

    def test_execute_rejects_a_different_interpreter_before_callback(self):
        from runners.scene0061_live_tick import (
            Scene0061LiveTickError,
            execute_live_tick,
            prepare_live_tick,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, xodr, _ = self._inputs(root)
            output = root / "diagnostic"
            prepare_live_tick(
                config_path=config, output_dir=output, run_id="r11-fixed", opendrive_path=xodr
            )
            environment_path = output / "runtime_environment.json"
            environment = json.loads(environment_path.read_text(encoding="utf-8"))
            environment["python_runtime"]["executable"] = "/wrong/python"
            environment_path.write_text(json.dumps(environment), encoding="utf-8")
            with self.assertRaisesRegex(Scene0061LiveTickError, "interpreter/protobuf identity"):
                execute_live_tick(output, sensor_handler_factory=lambda _config, _output: lambda _context: {})

    def test_execute_fails_before_callback_when_recorded_runtime_config_drifts(self):
        from runners.scene0061_live_tick import (
            Scene0061LiveTickError,
            execute_live_tick,
            prepare_live_tick,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, xodr, _ = self._inputs(root)
            output = root / "diagnostic"
            prepare_live_tick(
                config_path=config, output_dir=output, run_id="r11-fixed", opendrive_path=xodr
            )
            (output / "runtime_run_config.json").write_text("{}\n", encoding="utf-8")
            called = False

            def factory(_config, _output):
                nonlocal called
                called = True
                return lambda _context: {}

            with self.assertRaisesRegex(Scene0061LiveTickError, "runtime_config path/SHA-256"):
                execute_live_tick(output, sensor_handler_factory=factory)
            self.assertFalse(called)

    def test_execute_requires_the_preflighted_sensor_handler_factory(self):
        from runners.scene0061_live_tick import (
            Scene0061LiveTickError,
            execute_live_tick,
            prepare_live_tick,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, xodr, _ = self._inputs(root)
            output = root / "diagnostic"
            environment = prepare_live_tick(
                config_path=config, output_dir=output, run_id="r11-fixed", opendrive_path=xodr
            )
            environment["sensor_handler_preflight"] = {
                "status": "passed",
                "factory": "expected.module:factory",
            }
            (output / "runtime_environment.json").write_text(
                json.dumps(environment), encoding="utf-8"
            )
            with self.assertRaisesRegex(Scene0061LiveTickError, "factory does not match"):
                execute_live_tick(
                    output, sensor_handler_factory=lambda _config, _output: lambda _context: {}
                )

    def test_verify_rejects_a_drifted_recorded_basic_agent_file(self):
        from runners.scene0061_live_tick import (
            Scene0061LiveTickError,
            _carla_basic_agent_identity,
            prepare_live_tick,
            verify_prepared_live_tick,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, xodr, _ = self._inputs(root)
            output = root / "diagnostic"
            prepare_live_tick(
                config_path=config, output_dir=output, run_id="r11-fixed", opendrive_path=xodr
            )
            python_api = root / "PythonAPI" / "carla"
            basic_agent = python_api / "agents" / "navigation" / "basic_agent.py"
            basic_agent.parent.mkdir(parents=True)
            basic_agent.write_text("class BasicAgent: pass\n", encoding="utf-8")
            environment_path = output / "runtime_environment.json"
            environment = json.loads(environment_path.read_text(encoding="utf-8"))
            environment["carla_basic_agent"] = {
                "status": "passed",
                **_carla_basic_agent_identity(python_api),
            }
            environment_path.write_text(json.dumps(environment), encoding="utf-8")
            basic_agent.write_text("class BasicAgent: changed = True\n", encoding="utf-8")
            with self.assertRaisesRegex(Scene0061LiveTickError, "CARLA BasicAgent path/SHA-256"):
                verify_prepared_live_tick(output)

    def test_sidecar_hash_mismatch_is_rejected_before_output_creation(self):
        from runners.scene0061_live_tick import Scene0061LiveTickError, prepare_live_tick

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, xodr, _ = self._inputs(root)
            payload = json.loads(config.read_text(encoding="utf-8"))
            payload["nurec_runtime"]["actor_bindings_sha256"] = "0" * 64
            config.write_text(json.dumps(payload), encoding="utf-8")
            output = root / "diagnostic"
            with self.assertRaisesRegex(Scene0061LiveTickError, "actor_bindings_sha256"):
                prepare_live_tick(
                    config_path=config, output_dir=output, run_id="r11-fixed", opendrive_path=xodr
                )
            self.assertFalse(output.exists())

    def test_result_validation_rejects_empty_or_failed_persisted_evidence(self):
        from runners.scene0061_live_tick import validate_live_tick_result

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "frame_trace.jsonl").write_text("{}\n", encoding="utf-8")
            (root / "nurec_multimodal_trace.jsonl").write_text("{}\n", encoding="utf-8")
            (root / "cleanup_audit.json").write_text(
                json.dumps({"succeeded": False}), encoding="utf-8"
            )
            validation = validate_live_tick_result(
                {
                    "status": "failed",
                    "cleanup_succeeded": False,
                    "report": {"runtime": {"frame_trace_count": 1}},
                    "nurec_multimodal_trace": [{}],
                },
                root,
            )

            self.assertEqual(validation["status"], "failed")
            self.assertIn("persisted cleanup audit did not succeed", validation["problems"])
            self.assertIn("persisted cleanup audit has no actions", validation["problems"])
