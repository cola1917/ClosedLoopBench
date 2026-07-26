from __future__ import annotations


"""Regression coverage for the strict r22-to-TF++ warmup bridge.

The fixture deliberately creates the complete, hash-bound evidence graph in a
temporary directory.  It must not depend on the untracked r22 evidence mirror:
that mirror is useful for an integration check, but would make CI results rely
on a prior remote run.
"""

import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path


SCENE_ID = "cc8c0bf57f984915a77078b10eb33198"
CAMERAS = (
    "camera_front",
    "camera_front_left",
    "camera_front_right",
    "camera_back",
    "camera_back_left",
    "camera_back_right",
)
SENSOR_TO_EGO = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 2.5,
    0.0, 0.0, 0.0, 1.0,
]


def _sha_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _identity(path: Path) -> dict[str, object]:
    return {
        "path": str(path.resolve()),
        "sha256": _sha_bytes(path.read_bytes()),
        "byte_count": path.stat().st_size,
    }


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _write_jsonl(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")


def _fixture(root: Path) -> dict[str, object]:
    """Build one self-contained formal r22-like trace plus an S0 bundle.

    The real r22 trace has a separate RPC payload digest and materialized
    payload digest.  Preserve that distinction here: accepting only equal
    digests would reject the valid real evidence.
    """

    from agents.plugin_contract import canonical_sha256
    from agents.transfuserpp_contract import camera_adaptation_contract

    diagnostics = root / "diagnostics"
    diagnostics.mkdir()
    formal_base = root / "scene0061_formal.base.json"
    _write_json(formal_base, {"schema_version": "formal-base-test.v1"})
    runtime_config = diagnostics / "runtime_run_config.json"
    runtime_config.write_bytes(formal_base.read_bytes())

    axis_evidence = diagnostics / "lidar_axis_evidence.json"
    _write_json(
        axis_evidence,
        {
            "schema_version": "scene0061_lidar_coordinate_validation.v1",
            "status": "passed",
            "sensor_id": "lidar_top",
            "response_coordinate_frame": "sensor_local",
            "axis_convention": "carla_sensor",
            "sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
            "sensor_to_ego": list(SENSOR_TO_EGO),
            "sensor_to_ego_sha256": canonical_sha256(SENSOR_TO_EGO),
        },
    )

    nurec_runtime = {
        "runtime_scene_id": "scene-0061",
        "camera_specs": [
            {
                "sensor_id": sensor_id,
                "width": 1600,
                "height": 900,
                "sensor_to_ego": list(SENSOR_TO_EGO),
            }
            for sensor_id in CAMERAS
        ],
        "lidar_specs": [
            {
                "sensor_id": "lidar_top",
                "model": "AT128",
                "sensor_to_ego": list(SENSOR_TO_EGO),
            }
        ],
        "lidar_response_coordinate_frame": "sensor_local",
        "lidar_axis_convention": "carla_sensor",
        "lidar_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
        "lidar_coordinate_validation": {
            "evidence_path": str(axis_evidence.resolve()),
            "evidence_sha256": _identity(axis_evidence)["sha256"],
        },
    }
    acceptance = {
        "scenario_id": SCENE_ID,
        "experiment": {
            "scene_id": SCENE_ID,
            "scene_version": "formal40k-v1",
            "identity": {
                "artifact_sha256": "1" * 64,
                "scene_package_sha256": "2" * 64,
                "scenario_ir_sha256": "3" * 64,
                "immutable_matrix_sha256": "4" * 64,
            },
        },
        "nurec_runtime": nurec_runtime,
        "formal_lidar_evidence_binding": {
            "schema_version": "scene0061_formal_lidar_evidence_binding.v1",
            "evidence_status": "passed",
            "base_run_config": _identity(formal_base),
            "lidar_coordinate_evidence": _identity(axis_evidence),
        },
    }
    acceptance_path = root / "scene0061_formal.acceptance.json"
    _write_json(acceptance_path, acceptance)

    experiment = {
        "scene_id": SCENE_ID,
        "scene_version": "formal40k-v1",
        "case_id": "S0_original_replay",
        "seed": 41,
        "artifact_sha256": "1" * 64,
        "scene_package_sha256": "2" * 64,
        "scenario_ir_sha256": "3" * 64,
        "immutable_matrix_sha256": "4" * 64,
        "source_run_config_sha256": canonical_sha256(acceptance),
        "variant_config_sha256": "5" * 64,
    }
    binding = {
        "camera_sensor_id": "camera_front",
        "camera_source_width": 1600,
        "camera_source_height": 900,
        "camera_sensor_to_ego": list(SENSOR_TO_EGO),
        "camera_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
        "camera_adaptation": camera_adaptation_contract(),
        "lidar_sensor_id": "lidar_top",
        "lidar_sensor_to_ego": list(SENSOR_TO_EGO),
        "lidar_axis_convention": "carla_sensor",
        "lidar_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
        "container_payload_root": "/sim-data",
        "route_lookahead_m": 7.5,
    }
    run = {
        "run_id": "scene0061-tfpp-S0_original_replay-seed-41",
        "experiment": experiment,
        "ego": {
            "algorithm_sensor_binding": binding,
            "reference_trajectory": [
                {"x": 0.0, "y": 0.0, "route_command": "LANE_FOLLOW"},
                {"x": 10.0, "y": 0.0, "route_command": "LANE_FOLLOW"},
                {"x": 20.0, "y": 0.0, "route_command": "LANE_FOLLOW"},
            ],
        },
        "nurec_runtime": copy.deepcopy(nurec_runtime),
    }
    run_canonical = canonical_sha256(run)
    run["config_identity"] = {
        "schema_version": "closedloopbench_run_config_identity.v1",
        "canonical_sha256": run_canonical,
        "hash_scope": "whole_run_config_excluding_config_identity_and_algorithm_gpu_validation",
    }
    runtime = {
        "schema_version": "transfuserpp_runtime_config.v1",
        "case_id": "S0_original_replay",
        "seed": 41,
        "experiment": {
            name: experiment[name]
            for name in (
                "scene_id",
                "scene_version",
                "case_id",
                "seed",
                "artifact_sha256",
                "scene_package_sha256",
                "scenario_ir_sha256",
                "immutable_matrix_sha256",
                "source_run_config_sha256",
                "variant_config_sha256",
            )
        }
        | {"run_config_sha256": run_canonical},
    }
    bundle = {
        "schema_version": "scene0061_transfuserpp_remote_run_bundle.v1",
        "status": "remote_validation_required",
        "run_id": run["run_id"],
        "case_id": "S0_original_replay",
        "seed": 41,
        "run_config_sha256": run_canonical,
        "runtime_config_sha256": canonical_sha256(runtime),
    }
    s0 = root / "s0"
    (s0 / "runtime").mkdir(parents=True)
    _write_json(s0 / "carla_run_config.json", run)
    _write_json(s0 / "runtime" / "transfuserpp.runtime.json", runtime)
    _write_json(s0 / "remote_run_bundle.json", bundle)

    frame_id = 42
    payload_dir = diagnostics / "algorithm_sensor_payloads" / f"frame_{frame_id:08d}"
    payload_dir.mkdir(parents=True)
    records: list[dict[str, object]] = []
    for sensor_id in CAMERAS:
        payload = payload_dir / f"{sensor_id}.jpg"
        bytes_value = f"synthetic-jpeg:{sensor_id}".encode("ascii")
        payload.write_bytes(bytes_value)
        materialized = {
            **_identity(payload),
            "relative_path": f"algorithm_sensor_payloads/frame_{frame_id:08d}/{sensor_id}.jpg",
            "encoding": "jpeg",
            "coordinate_frame": "camera_optical",
        }
        records.append(
            {
                "sensor_id": sensor_id,
                "modality": "rgb",
                "status": "passed",
                # Deliberately distinct from materialized.sha256, as in r22.
                "payload_sha256": _sha_bytes(b"rpc-response:" + bytes_value),
                "response_metadata": {
                    "encoding": "jpeg",
                    "width": 1600,
                    "height": 900,
                    "materialized_payload": materialized,
                },
            }
        )
    lidar_payload = payload_dir / "lidar_top.bin"
    lidar_bytes = b"\x00" * 16
    lidar_payload.write_bytes(lidar_bytes)
    lidar_materialized = {
        **_identity(lidar_payload),
        "relative_path": f"algorithm_sensor_payloads/frame_{frame_id:08d}/lidar_top.bin",
        "encoding": "float32_xyzi_little_endian",
        "coordinate_frame": "sensor_local",
        "axis_convention": "carla_sensor",
    }
    records.append(
        {
            "sensor_id": "lidar_top",
            "modality": "lidar",
            "status": "passed",
            "payload_sha256": _sha_bytes(b"rpc-response:" + lidar_bytes),
            "response_metadata": {
                "encoding": "float_xyz_intensity",
                "materialized_payload": lidar_materialized,
            },
        }
    )
    validation = {
        "schema_version": "scene0061_live_tick_validation.v1",
        "status": "passed",
        "completion_class": "one_tick_physical_multimodal_smoke",
        "expected_one_tick_termination": True,
        "frame_trace_count": 1,
        "cleanup_succeeded": True,
        "problems": [],
    }
    _write_json(diagnostics / "live_tick_validation.json", validation)
    _write_json(
        diagnostics / "runtime_environment.json",
        {
            "schema_version": "scene0061_live_tick_environment.v2",
            "status": "passed",
            "run_id": "scene0061-formal-live-tick-test",
            "git_commit": "a" * 40,
            "execution_code_commit": "a" * 40,
            "config": _identity(formal_base),
            "runtime_config": _identity(runtime_config),
            "native_scan_manifest": {"sha256": "b" * 64},
            "validation": validation,
        },
    )
    _write_json(
        diagnostics / "runtime_result.json",
        {
            "status": "failed",
            "reason": "basic_agent_runtime_failed",
            "detail": "route_incomplete: termination=max_ticks max_ticks=1",
        },
    )
    _write_jsonl(
        diagnostics / "frame_trace.jsonl",
        {
            "world_tick_frame": frame_id,
            "snapshot_frame": frame_id,
            "simulation_time_sec": 0.05,
            "ego_pose": {"x": 0.5, "y": 0.0, "yaw": 0.0},
            "ego_speed_mps": 9.0,
            "multimodal_sensor": {
                "status": "passed",
                "frame_id": frame_id,
                "modalities": {
                    "rgb": {"requested_count": 6, "passed_count": 6},
                    "lidar": {"requested_count": 1, "passed_count": 1},
                },
            },
        },
    )
    _write_jsonl(
        diagnostics / "nurec_multimodal_trace.jsonl",
        {
            "schema_version": "nurec_multimodal_evidence.v1",
            "status": "passed",
            "scene_id": SCENE_ID,
            "frame_id": frame_id,
            "simulation_time_sec": 0.05,
            "dynamic_object_sha256": "c" * 64,
            "records": records,
            "dispatch": {
                "nre_api": "SensorsimService/26.04",
                "canonical_scene_id": SCENE_ID,
                "runtime_scene_id": "scene-0061",
                "temporal_alignment": {
                    "source": "hashed_native_scan_manifest",
                    "manifest_sha256": "b" * 64,
                    "midpoint_error_us": 100,
                },
            },
        },
    )
    for name in (
        "basic_agent_plan.json",
        "metrics_trace.jsonl",
        "cleanup_audit.json",
        "closed_loop_report.json",
    ):
        _write_json(diagnostics / name, {"test_artifact": name})

    required = (
        "basic_agent_plan.json",
        "runtime_result.json",
        "frame_trace.jsonl",
        "nurec_multimodal_trace.jsonl",
        "metrics_trace.jsonl",
        "cleanup_audit.json",
        "closed_loop_report.json",
        "live_tick_validation.json",
        "runtime_environment.json",
        "lidar_axis_evidence.json",
    )

    def rewrite_manifest() -> None:
        _write_json(
            diagnostics / "artifact_manifest.json",
            {
                "schema_version": "scene0061_live_tick_artifact_manifest.v1",
                "status": "complete",
                "missing_artifacts": [],
                "artifacts": [
                    {"name": name, **_identity(diagnostics / name)} for name in required
                ],
            },
        )

    rewrite_manifest()
    return {
        "diagnostics": diagnostics,
        "formal_base": formal_base,
        "acceptance": acceptance_path,
        "s0": s0,
        "rewrite_manifest": rewrite_manifest,
        "frame_id": frame_id,
    }


class Scene0061TransFuserPPWarmupObservationTests(unittest.TestCase):
    def _build(self, fixture: dict[str, object]) -> dict[str, object]:
        from runners.build_scene0061_transfuserpp_warmup_observation import (
            build_scene0061_transfuserpp_warmup_observation,
        )

        s0 = fixture["s0"]
        assert isinstance(s0, Path)
        return build_scene0061_transfuserpp_warmup_observation(
            diagnostics_dir=fixture["diagnostics"],
            formal_acceptance_config=fixture["acceptance"],
            formal_base_config=fixture["formal_base"],
            s0_bundle_dir=s0,
            payload_output_dir=s0 / "warmup_payloads",
            observation_output=s0 / "runtime" / "formal_warmup_observation.json",
            provenance_output=s0 / "runtime" / "formal_warmup_provenance.json",
        )

    def test_builds_container_safe_hash_bound_observation_from_one_physical_frame(self):
        from agents.plugin_contract import canonical_sha256

        with tempfile.TemporaryDirectory() as directory:
            fixture = _fixture(Path(directory))
            result = self._build(fixture)
            s0 = fixture["s0"]
            diagnostics = fixture["diagnostics"]
            assert isinstance(s0, Path) and isinstance(diagnostics, Path)
            self.assertEqual(result["status"], "passed")
            self.assertEqual(result["frame_id"], 42)

            observation = json.loads(
                (s0 / "runtime" / "formal_warmup_observation.json").read_text(
                    encoding="utf-8"
                )
            )
            provenance = json.loads(
                (s0 / "runtime" / "formal_warmup_provenance.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(
                observation["rgb"]["camera_front"]["path"],
                "/sim-data/warmup_payloads/frame_00000042/camera_front.jpg",
            )
            self.assertEqual(
                observation["lidar"]["path"],
                "/sim-data/warmup_payloads/frame_00000042/lidar_top.bin",
            )
            self.assertEqual(
                set(provenance["payload_coverage"]), set(CAMERAS) | {"lidar_top"}
            )
            self.assertEqual(
                provenance["observation"]["canonical_sha256"],
                canonical_sha256(observation),
            )
            self.assertEqual(provenance["route_derivation"]["target_index"], 1)
            self.assertEqual(provenance["route_derivation"]["next_target_index"], 2)
            source = diagnostics / "algorithm_sensor_payloads" / "frame_00000042"
            copied = s0 / "warmup_payloads" / "frame_00000042"
            self.assertEqual(
                (copied / "camera_front.jpg").read_bytes(),
                (source / "camera_front.jpg").read_bytes(),
            )
            self.assertEqual(
                (copied / "lidar_top.bin").read_bytes(),
                (source / "lidar_top.bin").read_bytes(),
            )
            self.assertNotEqual(
                json.loads((diagnostics / "nurec_multimodal_trace.jsonl").read_text())
                ["records"][0]["payload_sha256"],
                observation["rgb"]["camera_front"]["sha256"],
            )

    def test_refuses_materialized_payload_drift_before_any_output_is_written(self):
        from runners.build_scene0061_transfuserpp_warmup_observation import (
            Scene0061TransFuserPPWarmupError,
        )

        with tempfile.TemporaryDirectory() as directory:
            fixture = _fixture(Path(directory))
            diagnostics = fixture["diagnostics"]
            s0 = fixture["s0"]
            assert isinstance(diagnostics, Path) and isinstance(s0, Path)
            (
                diagnostics
                / "algorithm_sensor_payloads"
                / "frame_00000042"
                / "camera_front.jpg"
            ).write_bytes(b"tampered")
            with self.assertRaisesRegex(Scene0061TransFuserPPWarmupError, "materialized payload SHA-256"):
                self._build(fixture)
            self.assertFalse((s0 / "warmup_payloads").exists())
            self.assertFalse((s0 / "runtime" / "formal_warmup_observation.json").exists())
            self.assertFalse((s0 / "runtime" / "formal_warmup_provenance.json").exists())

    def test_refuses_frame_mismatch_and_payload_path_escape(self):
        from runners.build_scene0061_transfuserpp_warmup_observation import (
            Scene0061TransFuserPPWarmupError,
        )

        with tempfile.TemporaryDirectory() as directory:
            fixture = _fixture(Path(directory))
            diagnostics = fixture["diagnostics"]
            s0 = fixture["s0"]
            assert isinstance(diagnostics, Path) and isinstance(s0, Path)
            trace_path = diagnostics / "nurec_multimodal_trace.jsonl"
            trace = json.loads(trace_path.read_text(encoding="utf-8"))
            trace["frame_id"] = 43
            _write_jsonl(trace_path, trace)
            fixture["rewrite_manifest"]()
            with self.assertRaisesRegex(Scene0061TransFuserPPWarmupError, "frame identities"):
                self._build(fixture)
            self.assertFalse((s0 / "warmup_payloads").exists())

        with tempfile.TemporaryDirectory() as directory:
            fixture = _fixture(Path(directory))
            diagnostics = fixture["diagnostics"]
            s0 = fixture["s0"]
            assert isinstance(diagnostics, Path) and isinstance(s0, Path)
            trace_path = diagnostics / "nurec_multimodal_trace.jsonl"
            trace = json.loads(trace_path.read_text(encoding="utf-8"))
            trace["records"][0]["response_metadata"]["materialized_payload"][
                "relative_path"
            ] = "algorithm_sensor_payloads/../frame_00000042/camera_front.jpg"
            _write_jsonl(trace_path, trace)
            fixture["rewrite_manifest"]()
            with self.assertRaisesRegex(Scene0061TransFuserPPWarmupError, "escapes diagnostics"):
                self._build(fixture)
            self.assertFalse((s0 / "warmup_payloads").exists())

        with tempfile.TemporaryDirectory() as directory:
            fixture = _fixture(Path(directory))
            diagnostics = fixture["diagnostics"]
            s0 = fixture["s0"]
            assert isinstance(diagnostics, Path) and isinstance(s0, Path)
            trace_path = diagnostics / "nurec_multimodal_trace.jsonl"
            trace = json.loads(trace_path.read_text(encoding="utf-8"))
            replacement = (
                diagnostics
                / "algorithm_sensor_payloads"
                / "frame_00000042"
                / "camera_back.jpg"
            )
            materialized = trace["records"][0]["response_metadata"]["materialized_payload"]
            materialized.update(_identity(replacement))
            materialized["relative_path"] = (
                "algorithm_sensor_payloads/frame_00000042/camera_back.jpg"
            )
            _write_jsonl(trace_path, trace)
            fixture["rewrite_manifest"]()
            with self.assertRaisesRegex(
                Scene0061TransFuserPPWarmupError, "does not bind the NuRec trace frame and sensor"
            ):
                self._build(fixture)
            self.assertFalse((s0 / "warmup_payloads").exists())

    def test_refuses_validation_or_s0_identity_drift_and_overwrite(self):
        from runners.build_scene0061_transfuserpp_warmup_observation import (
            Scene0061TransFuserPPWarmupError,
        )

        with tempfile.TemporaryDirectory() as directory:
            fixture = _fixture(Path(directory))
            diagnostics = fixture["diagnostics"]
            s0 = fixture["s0"]
            assert isinstance(diagnostics, Path) and isinstance(s0, Path)
            validation_path = diagnostics / "live_tick_validation.json"
            validation = json.loads(validation_path.read_text(encoding="utf-8"))
            validation["status"] = "failed"
            _write_json(validation_path, validation)
            fixture["rewrite_manifest"]()
            with self.assertRaisesRegex(Scene0061TransFuserPPWarmupError, "validation"):
                self._build(fixture)
            self.assertFalse((s0 / "warmup_payloads").exists())

        with tempfile.TemporaryDirectory() as directory:
            fixture = _fixture(Path(directory))
            s0 = fixture["s0"]
            assert isinstance(s0, Path)
            run_path = s0 / "carla_run_config.json"
            run = json.loads(run_path.read_text(encoding="utf-8"))
            run["ego"]["algorithm_sensor_binding"]["camera_source_width"] = 800
            _write_json(run_path, run)
            with self.assertRaisesRegex(Scene0061TransFuserPPWarmupError, "run config identity"):
                self._build(fixture)

        with tempfile.TemporaryDirectory() as directory:
            fixture = _fixture(Path(directory))
            self._build(fixture)
            with self.assertRaisesRegex(Scene0061TransFuserPPWarmupError, "refusing to overwrite"):
                self._build(fixture)

        with tempfile.TemporaryDirectory() as directory:
            fixture = _fixture(Path(directory))
            from runners.build_scene0061_transfuserpp_warmup_observation import (
                build_scene0061_transfuserpp_warmup_observation,
            )

            s0 = fixture["s0"]
            assert isinstance(s0, Path)
            with self.assertRaisesRegex(
                Scene0061TransFuserPPWarmupError,
                "observation output must not be inside the payload output directory",
            ):
                build_scene0061_transfuserpp_warmup_observation(
                    diagnostics_dir=fixture["diagnostics"],
                    formal_acceptance_config=fixture["acceptance"],
                    formal_base_config=fixture["formal_base"],
                    s0_bundle_dir=s0,
                    payload_output_dir=s0 / "warmup_payloads",
                    observation_output=(
                        s0
                        / "warmup_payloads"
                        / "frame_00000042"
                        / "camera_front.jpg"
                    ),
                    provenance_output=s0 / "runtime" / "formal_warmup_provenance.json",
                )
            self.assertFalse((s0 / "warmup_payloads").exists())

        with tempfile.TemporaryDirectory() as directory:
            fixture = _fixture(Path(directory))
            s0 = fixture["s0"]
            assert isinstance(s0, Path)
            observation_output = s0 / "runtime" / "formal_warmup_observation.json"
            with self.assertRaisesRegex(
                Scene0061TransFuserPPWarmupError,
                "payload output directory must not be inside observation output",
            ):
                build_scene0061_transfuserpp_warmup_observation(
                    diagnostics_dir=fixture["diagnostics"],
                    formal_acceptance_config=fixture["acceptance"],
                    formal_base_config=fixture["formal_base"],
                    s0_bundle_dir=s0,
                    payload_output_dir=observation_output / "payloads",
                    observation_output=observation_output,
                    provenance_output=s0 / "runtime" / "formal_warmup_provenance.json",
                )
            self.assertFalse(observation_output.exists())


if __name__ == "__main__":
    unittest.main()
