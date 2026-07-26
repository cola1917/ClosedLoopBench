import json
import tempfile
import unittest
from unittest import mock
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
            self.assertFalse(environment["physical_lidar_probe"]["requested"])
            self.assertFalse(plan["runtime"]["provenance"]["capture_native_lidar_requested"])

    def test_prepare_records_explicit_native_lidar_probe_request(self):
        from runners.scene0061_live_tick import prepare_live_tick, verify_prepared_live_tick

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, xodr, _ = self._inputs(root)
            output = root / "diagnostic"
            environment = prepare_live_tick(
                config_path=config,
                output_dir=output,
                run_id="native-lidar",
                opendrive_path=xodr,
                capture_native_lidar=True,
            )
            self.assertTrue(environment["physical_lidar_probe"]["requested"])
            self.assertTrue(
                json.loads((output / "basic_agent_plan.json").read_text(encoding="utf-8"))["runtime"]["provenance"]["capture_native_lidar_requested"]
            )
            environment["sensor_handler_preflight"] = {
                "status": "passed", "factory": "builtins:print"
            }
            (output / "runtime_environment.json").write_text(json.dumps(environment), encoding="utf-8")
            self.assertTrue(verify_prepared_live_tick(output)["physical_lidar_probe"]["requested"])

    def test_verify_rejects_native_lidar_request_that_drifts_from_plan(self):
        from runners.scene0061_live_tick import (
            Scene0061LiveTickError,
            prepare_live_tick,
            verify_prepared_live_tick,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, xodr, _ = self._inputs(root)
            output = root / "diagnostic"
            environment = prepare_live_tick(
                config_path=config,
                output_dir=output,
                run_id="native-lidar",
                opendrive_path=xodr,
            )
            environment["physical_lidar_probe"]["requested"] = True
            (output / "runtime_environment.json").write_text(json.dumps(environment), encoding="utf-8")
            with self.assertRaisesRegex(Scene0061LiveTickError, "native LiDAR probe request"):
                verify_prepared_live_tick(output)

    def test_physical_axis_collection_requires_production_gate_replay(self):
        """A collector candidate is not a passed physical axis proof by itself."""
        from runners.scene0061_live_tick import _collect_lidar_axis_evidence

        candidate = {
            "schema_version": "scene0061_lidar_coordinate_validation.v1",
            "status": "passed",
            "sensor_to_ego": [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
            "live_render_lidar": {"carla_frame_id": 42, "point_count": 4},
            "axis_validation": {"status": "passed"},
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = root / "native-capture.json"
            capture.write_text("{}\n", encoding="utf-8")
            native_scan = root / "native-scan.json"
            native_scan.write_text("{}\n", encoding="utf-8")
            (root / "frame_trace.jsonl").write_text(
                json.dumps({"native_lidar_capture": {"capture_path": str(capture)}}) + "\n",
                encoding="utf-8",
            )
            (root / "nurec_multimodal_trace.jsonl").write_text(
                json.dumps({"frame_id": 42}) + "\n", encoding="utf-8"
            )
            with mock.patch(
                "runners.scene0061_live_tick.collect_lidar_axis_evidence",
                return_value=candidate,
            ) as collect, mock.patch(
                "runners.scene0061_live_tick.validate_lidar_axis_evidence",
                return_value={"status": "passed"},
            ) as replay:
                evidence = _collect_lidar_axis_evidence(
                    output_dir=root,
                    config={},
                    environment={"native_scan_manifest": {"path": str(native_scan)}},
                    capture_native_lidar=True,
                )

            self.assertEqual(evidence["status"], "passed")
            self.assertEqual(evidence["gate_replay"]["status"], "passed")
            collect.assert_called_once()
            replay.assert_called_once_with(
                candidate,
                sensor_to_ego=candidate["sensor_to_ego"],
                live_render_lidar=candidate["live_render_lidar"],
            )

    def test_physical_axis_gate_replay_failure_cannot_remain_passed(self):
        from runners.scene0061_live_tick import _collect_lidar_axis_evidence
        from runtime.scene0061_lidar_axis_gate import LiDARAxisEvidenceError

        candidate = {
            "status": "passed",
            "sensor_to_ego": [1.0] * 16,
            "live_render_lidar": {"carla_frame_id": 42, "point_count": 4},
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = root / "native-capture.json"
            capture.write_text("{}\n", encoding="utf-8")
            native_scan = root / "native-scan.json"
            native_scan.write_text("{}\n", encoding="utf-8")
            (root / "frame_trace.jsonl").write_text(
                json.dumps({"native_lidar_capture": {"capture_path": str(capture)}}) + "\n",
                encoding="utf-8",
            )
            (root / "nurec_multimodal_trace.jsonl").write_text(
                json.dumps({"frame_id": 42}) + "\n", encoding="utf-8"
            )
            with mock.patch(
                "runners.scene0061_live_tick.collect_lidar_axis_evidence",
                return_value=candidate,
            ), mock.patch(
                "runners.scene0061_live_tick.validate_lidar_axis_evidence",
                side_effect=LiDARAxisEvidenceError("tampered anchor"),
            ):
                evidence = _collect_lidar_axis_evidence(
                    output_dir=root,
                    config={},
                    environment={"native_scan_manifest": {"path": str(native_scan)}},
                    capture_native_lidar=True,
                )

            self.assertEqual(evidence["status"], "failed")
            self.assertEqual(evidence["reason"], "physical_axis_gate_replay_failed")
            self.assertIn("tampered anchor", evidence["detail"])
            self.assertEqual(evidence["unverified_collection"], candidate)

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

    def test_verify_rejects_a_rehashed_runtime_snapshot_that_differs_from_source(self):
        from runners.scene0061_live_tick import (
            Scene0061LiveTickError,
            _file_identity,
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
            snapshot = output / "runtime_run_config.json"
            snapshot.write_text(json.dumps({"scenario_id": "different"}), encoding="utf-8")
            snapshot_identity = _file_identity(snapshot, required=True)
            assert snapshot_identity is not None
            environment_path = output / "runtime_environment.json"
            environment = json.loads(environment_path.read_text(encoding="utf-8"))
            environment["runtime_config"] = snapshot_identity
            environment_path.write_text(json.dumps(environment), encoding="utf-8")

            with self.assertRaisesRegex(
                Scene0061LiveTickError, "snapshot does not match the selected config bytes"
            ):
                verify_prepared_live_tick(output)

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

    def test_formal_execution_input_drift_is_rejected_before_output_creation(self):
        from runners.scene0061_live_tick import Scene0061LiveTickError, prepare_live_tick

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, xodr, sidecar = self._inputs(root)
            package = root / "nre-wrapper.json"
            package.write_text("{}\n", encoding="utf-8")
            native = root / "native-scan.json"
            native.write_text("{}\n", encoding="utf-8")
            payload = json.loads(config.read_text(encoding="utf-8"))
            identity = lambda path: {
                "path": str(path.resolve()),
                "sha256": __import__("hashlib").sha256(path.read_bytes()).hexdigest(),
                "byte_count": path.stat().st_size,
            }
            payload["nurec_runtime"].update(
                {
                    "scene_package": str(package.resolve()),
                    "native_scan_manifest": {
                        "path": str(native.resolve()),
                        "sha256": identity(native)["sha256"],
                    },
                }
            )
            payload["formal_base_derivation"] = {
                "schema_version": "scene0061_formal_base_derivation.v1",
                "runtime_execution_inputs": [
                    {"role": "runtime_scene_package", **identity(package)},
                    {"role": "runtime_actor_bindings", **identity(sidecar)},
                    {"role": "runtime_native_scan_manifest", **identity(native)},
                    {"role": "runtime_opendrive", **identity(xodr)},
                ],
            }
            config.write_text(json.dumps(payload), encoding="utf-8")
            package.write_text('{"drift":true}\n', encoding="utf-8")
            with self.assertRaisesRegex(Scene0061LiveTickError, "runtime_scene_package"):
                prepare_live_tick(
                    config_path=config,
                    output_dir=root / "diagnostic",
                    run_id="formal-drift",
                    opendrive_path=xodr,
                )

    def test_formal_run_id_mismatch_is_rejected_before_output_creation(self):
        from runners.scene0061_live_tick import Scene0061LiveTickError, prepare_live_tick

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, xodr, sidecar = self._inputs(root)
            package = root / "nre-wrapper.json"
            package.write_text("{}\n", encoding="utf-8")
            native = root / "native-scan.json"
            native.write_text("{}\n", encoding="utf-8")
            payload = json.loads(config.read_text(encoding="utf-8"))

            def identity(path: Path) -> dict:
                return {
                    "path": str(path.resolve()),
                    "sha256": __import__("hashlib").sha256(path.read_bytes()).hexdigest(),
                    "byte_count": path.stat().st_size,
                }

            payload["run_id"] = "formal-exact-run"
            payload["experiment"] = {"run_id": "formal-exact-run"}
            payload["nurec_runtime"].update(
                {
                    "scene_package": str(package.resolve()),
                    "native_scan_manifest": {
                        "path": str(native.resolve()),
                        "sha256": identity(native)["sha256"],
                    },
                }
            )
            payload["formal_base_derivation"] = {
                "schema_version": "scene0061_formal_base_derivation.v1",
                "runtime_execution_inputs": [
                    {"role": "runtime_scene_package", **identity(package)},
                    {"role": "runtime_actor_bindings", **identity(sidecar)},
                    {"role": "runtime_native_scan_manifest", **identity(native)},
                    {"role": "runtime_opendrive", **identity(xodr)},
                ],
            }
            config.write_text(json.dumps(payload), encoding="utf-8")
            output = root / "diagnostic"
            with self.assertRaisesRegex(Scene0061LiveTickError, "run_id must match"):
                prepare_live_tick(
                    config_path=config,
                    output_dir=output,
                    run_id="different-run",
                    opendrive_path=xodr,
                )
            self.assertFalse(output.exists())

    def test_passing_axis_evidence_is_bound_to_exact_live_tick_inputs(self):
        from runners.scene0061_live_tick import _bind_axis_evidence_to_live_tick

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "formal.json"
            config.write_text("{}\n", encoding="utf-8")
            snapshot = root / "runtime_run_config.json"
            snapshot.write_bytes(config.read_bytes())
            opendrive = root / "road.xodr"
            opendrive.write_text("<OpenDRIVE/>\n", encoding="utf-8")
            validation_path = root / "live_tick_validation.json"
            validation = {"status": "passed", "frame_trace_count": 1}
            validation_path.write_text(json.dumps(validation), encoding="utf-8")

            def identity(path: Path) -> dict:
                return {
                    "path": str(path.resolve()),
                    "sha256": __import__("hashlib").sha256(path.read_bytes()).hexdigest(),
                    "byte_count": path.stat().st_size,
                }

            result = _bind_axis_evidence_to_live_tick(
                {"status": "passed"},
                environment={
                    "run_id": "formal-live",
                    "config": identity(config),
                    "runtime_config": identity(snapshot),
                    "opendrive": identity(opendrive),
                },
                validation=validation,
                validation_path=validation_path,
            )

            provenance = result["live_tick_provenance"]
            self.assertEqual(provenance["run_id"], "formal-live")
            self.assertEqual(provenance["selected_config"], identity(config))
            self.assertEqual(provenance["runtime_config"], identity(snapshot))
            self.assertEqual(provenance["opendrive"], identity(opendrive))
            self.assertEqual(provenance["live_tick_validation"]["status"], "passed")
            self.assertEqual(
                provenance["live_tick_validation"]["sha256"], identity(validation_path)["sha256"]
            )

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

    def _write_g0_evidence(self, root: Path) -> tuple[dict, Path]:
        import hashlib

        payload_root = root / "algorithm_sensor_payloads" / "frame_00000042"
        payload_root.mkdir(parents=True)
        records = []
        for modality, sensor_ids in (
            ("rgb", [f"camera_{index}" for index in range(6)]),
            ("lidar", ["lidar_top"]),
        ):
            for sensor_id in sensor_ids:
                suffix = ".jpg" if modality == "rgb" else ".bin"
                data = f"{modality}:{sensor_id}".encode("ascii")
                payload = payload_root / f"{sensor_id}{suffix}"
                payload.write_bytes(data)
                materialized = {
                    "path": str(payload.resolve()),
                    "sha256": hashlib.sha256(data).hexdigest(),
                    "byte_count": len(data),
                    "encoding": (
                        "jpeg" if modality == "rgb" else "float32_xyzi_little_endian"
                    ),
                    "coordinate_frame": (
                        "camera_optical" if modality == "rgb" else "carla_sensor"
                    ),
                }
                if modality == "lidar":
                    materialized["axis_convention"] = "x_forward_y_right_z_up"
                metadata = (
                    {"width": 1600, "height": 900, "encoding": "jpeg"}
                    if modality == "rgb"
                    else {"point_count": 1, "encoding": "float_xyz_intensity"}
                )
                metadata["materialized_payload"] = materialized
                records.append(
                    {
                        "request_id": f"{modality}-{sensor_id}",
                        "modality": modality,
                        "sensor_id": sensor_id,
                        "status": "passed",
                        "latency_ms": 1.0,
                        "payload_sha256": "a" * 64,
                        "response_metadata": metadata,
                        "issues": [],
                    }
                )
        evidence = {
            "schema_version": "nurec_multimodal_evidence.v1",
            "scene_id": "0" * 32,
            "frame_id": 42,
            "simulation_time_sec": 2.1,
            "dynamic_object_sha256": "b" * 64,
            "dynamic_object_count": 1,
            "records": records,
            "modalities": {
                "rgb": {"requested_count": 6, "passed_count": 6},
                "lidar": {"requested_count": 1, "passed_count": 1},
            },
            "max_latency_ms": None,
            "issues": [],
            "status": "passed",
            "dispatch": {
                "sdk_boundary": "injected_version_specific_encoder",
                "dynamic_object_verification": "encoder_echo_checked_before_rpc",
                "response_digest": "sha256_of_serialized_rpc_response",
                "response_validation": "injected_modality_specific_inspector",
                "runtime_scene_id": "scene-0061",
                "canonical_scene_id": "0" * 32,
                "nre_api": "SensorsimService/26.04",
            },
        }
        (root / "frame_trace.jsonl").write_text(
            json.dumps(
                {
                    "world_tick_frame": 42,
                    "ego_control": {"throttle": 0.2, "steer": 0.0, "brake": 0.0},
                }
            )
            + "\n",
            encoding="utf-8",
        )
        (root / "nurec_multimodal_trace.jsonl").write_text(
            json.dumps(evidence) + "\n", encoding="utf-8"
        )
        (root / "cleanup_audit.json").write_text(
            json.dumps({"succeeded": True, "actions": [{"action": "ego.destroy", "status": "succeeded"}]}),
            encoding="utf-8",
        )
        result = {
            "status": "ego_closed_loop",
            "cleanup_succeeded": True,
            "report": {"runtime": {"frame_trace_count": 1}},
            "nurec_multimodal_trace": [evidence],
        }
        return result, payload_root / "camera_0.jpg"

    def test_result_validation_requires_rehashable_6rgb_1lidar_nre_evidence(self):
        from runners.scene0061_live_tick import validate_live_tick_result

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            result, first_payload = self._write_g0_evidence(root)
            self.assertEqual(validate_live_tick_result(result, root)["status"], "passed")

            first_payload.write_bytes(b"tampered")
            validation = validate_live_tick_result(result, root)
            self.assertEqual(validation["status"], "failed")
            self.assertIn(
                "rgb:camera_0 materialized payload SHA-256 does not match",
                validation["problems"],
            )

    def test_result_validation_rejects_non_physical_nurec_coverage(self):
        from runners.scene0061_live_tick import validate_live_tick_result

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            result, _ = self._write_g0_evidence(root)
            evidence = result["nurec_multimodal_trace"][0]
            evidence["records"].pop(0)
            evidence["modalities"]["rgb"] = {"requested_count": 5, "passed_count": 5}
            (root / "nurec_multimodal_trace.jsonl").write_text(
                json.dumps(evidence) + "\n", encoding="utf-8"
            )
            validation = validate_live_tick_result(result, root)
            self.assertEqual(validation["status"], "failed")
            self.assertIn(
                "persisted NuRec frame does not contain exactly six RGB responses",
                validation["problems"],
            )
