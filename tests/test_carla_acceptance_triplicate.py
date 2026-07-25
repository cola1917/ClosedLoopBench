import copy
import hashlib
import importlib
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


def _result(status="interactive_closed_loop"):
    return {
        "status": status,
        "cleanup_succeeded": True,
        "report": {
            "summary": {"route_progress": 0.99, "collision_count": 0},
            "runtime": {
                "collision_sensor_available": True,
                "frame_trace_count": 600,
                "actor_physical_response": {
                    "trigger": {"displacement_m": 3.0, "speed_mps": 2.0}
                },
            },
        },
    }


class CarlaAcceptanceTriplicateTests(unittest.TestCase):
    def test_import_basic_agent_from_explicit_carla_python_api(self):
        from runners.run_carla_basic_agent import _import_basic_agent_cls

        agents_package = importlib.import_module("agents")
        original_navigation = sys.modules.get("agents.navigation")
        original_basic_agent = sys.modules.get("agents.navigation.basic_agent")
        with tempfile.TemporaryDirectory() as directory:
            python_api = Path(directory) / "CARLA" / "PythonAPI" / "carla"
            navigation = python_api / "agents" / "navigation"
            navigation.mkdir(parents=True)
            (navigation / "__init__.py").write_text("", encoding="utf-8")
            (navigation / "basic_agent.py").write_text(
                "class BasicAgent:\n    pass\n",
                encoding="utf-8",
            )
            appended_path = str((python_api / "agents").resolve())
            try:
                basic_agent_cls = _import_basic_agent_cls(python_api)
                self.assertEqual(basic_agent_cls.__name__, "BasicAgent")
                module_path = Path(sys.modules[basic_agent_cls.__module__].__file__).resolve()
                self.assertIn((python_api / "agents").resolve(), module_path.parents)
            finally:
                sys.modules.pop("agents.navigation.basic_agent", None)
                sys.modules.pop("agents.navigation", None)
                while appended_path in agents_package.__path__:
                    agents_package.__path__.remove(appended_path)
                if original_navigation is not None:
                    sys.modules["agents.navigation"] = original_navigation
                if original_basic_agent is not None:
                    sys.modules["agents.navigation.basic_agent"] = original_basic_agent
                importlib.invalidate_caches()

    def test_import_basic_agent_rejects_missing_explicit_path(self):
        from runners.run_carla_basic_agent import _import_basic_agent_cls

        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ImportError, "agents.navigation.basic_agent.py"):
                _import_basic_agent_cls(Path(directory))

    def test_basic_agent_binds_explicit_route_without_global_route_search(self):
        import types

        from runners import run_carla_basic_agent as runner

        class Waypoint:
            def __init__(self, waypoint_id):
                self.id = waypoint_id

        class Map:
            def get_waypoint(self, location, *, project_to_road):
                self.assert_project_to_road = project_to_road
                return Waypoint(int(location.x))

        class Agent:
            def __init__(self):
                self._map = Map()
                self.plan = None

            def set_global_plan(self, plan, **options):
                self.plan = plan
                self.options = options

            def set_destination(self, _destination):
                raise AssertionError("global route search must not be used")

        fake_local_planner = types.SimpleNamespace(
            RoadOption=types.SimpleNamespace(LANEFOLLOW="lane-follow")
        )
        carla = types.SimpleNamespace(
            Location=lambda x, y, z: types.SimpleNamespace(x=x, y=y, z=z)
        )
        agent = Agent()
        route = [
            {"x": 0.0, "y": 0.0, "z": 0.0},
            {"x": 0.0, "y": 0.0, "z": 0.0},
            {"x": 10.0, "y": 0.0, "z": 0.0},
        ]
        with mock.patch(
            "runners.run_carla_basic_agent.importlib.import_module",
            return_value=fake_local_planner,
        ):
            evidence = runner._set_agent_global_plan(agent, carla, route)

        self.assertEqual([item[0].id for item in agent.plan], [0, 10])
        self.assertTrue(all(item[1] == "lane-follow" for item in agent.plan))
        self.assertEqual(
            agent.options,
            {"stop_waypoint_creation": True, "clean_queue": True},
        )
        self.assertEqual(evidence["source_route_point_count"], 3)
        self.assertEqual(evidence["projected_waypoint_count"], 2)

    def test_basic_agent_preflight_fails_before_output_creation(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            run_acceptance_triplicate,
        )

        with tempfile.TemporaryDirectory() as directory:
            output_root = Path(directory) / "new-output"
            with mock.patch(
                "runners.run_carla_acceptance_triplicate._import_basic_agent_cls",
                side_effect=ImportError("missing CARLA agents"),
            ):
                with self.assertRaisesRegex(CarlaAcceptanceError, "before attempt creation"):
                    run_acceptance_triplicate(
                        {"scenario_id": "scene-triplicate"},
                        output_root,
                    )
            self.assertFalse(output_root.exists())

    def test_custom_execute_skips_basic_agent_preflight(self):
        from runners.run_carla_acceptance_triplicate import run_acceptance_triplicate

        with tempfile.TemporaryDirectory() as directory:
            with mock.patch(
                "runners.run_carla_acceptance_triplicate._import_basic_agent_cls",
                side_effect=AssertionError("preflight should be skipped"),
            ):
                result = run_acceptance_triplicate(
                    {"scenario_id": "scene-triplicate"},
                    Path(directory),
                    execute=lambda _plan: _result(),
                )
        self.assertEqual(result["status"], "passed")

    def test_triplicate_carries_formal_map_driver_and_injected_handler(self):
        from runners.run_carla_acceptance_triplicate import run_acceptance_triplicate

        plans = []
        handlers = []

        class Handler:
            def __init__(self):
                self.closed = False

            def __call__(self, context):
                return {"frame": context.get("frame_id")}

            def close(self):
                self.closed = True

        def handler_factory(config, run_dir):
            handler = Handler()
            handlers.append((config["run_id"], run_dir, handler))
            return handler

        def execute(plan, *, sensor_frame_handler):
            plans.append((plan, sensor_frame_handler))
            return _result()

        config = {
            "scenario_id": "scene-triplicate",
            "run_id": "formal",
            "ego": {"initial_state": {"x": 0.0, "y": 0.0}},
            "actors": [],
            "metrics": ["collision", "route_progress"],
        }
        with tempfile.TemporaryDirectory() as directory:
            result = run_acceptance_triplicate(
                config,
                Path(directory),
                max_ticks=1200,
                opendrive_path="/runtime/scene0061-v7.xodr",
                ego_driver="topology_follower",
                sensor_frame_handler_factory=handler_factory,
                execute=execute,
            )

        self.assertEqual(result["run_count"], 3)
        self.assertEqual(len(handlers), 3)
        self.assertEqual(len(plans), 3)
        self.assertTrue(all(handler.closed for _, _, handler in handlers))
        self.assertTrue(
            all(plan["world"]["opendrive_path"].endswith("scene0061-v7.xodr") for plan, _ in plans)
        )
        self.assertTrue(all(plan["ego"]["driver"] == "topology_follower" for plan, _ in plans))
        self.assertTrue(all(plan["limits"]["max_ticks"] == 1200 for plan, _ in plans))
        self.assertTrue(all(plan["runtime"]["snap_to_map"] for plan, _ in plans))

    def test_real_multimodal_triplicate_requires_handler_factory(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            run_acceptance_triplicate,
        )

        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(CarlaAcceptanceError, "real sensor frame handler"):
                run_acceptance_triplicate(
                    {"scenario_id": "scene-triplicate"},
                    Path(directory),
                    require_multimodal=True,
                )

    def test_requires_three_complete_physical_runs(self):
        from runners.run_carla_acceptance_triplicate import validate_acceptance_runs

        validated = validate_acceptance_runs([_result(), _result(), _result()])
        self.assertEqual(len(validated), 3)
        self.assertTrue(all(item["cleanup_succeeded"] for item in validated))

    def test_transfuserpp_clean_algorithm_evidence_is_mandatory(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            validate_acceptance_runs,
        )

        valid = _result()
        valid["algorithm_evidence_validation"] = {"status": "passed"}
        accepted = validate_acceptance_runs(
            [copy.deepcopy(valid), copy.deepcopy(valid), copy.deepcopy(valid)],
            require_algorithm_clean=True,
        )
        self.assertEqual(
            accepted[0]["algorithm_evidence_validation"]["status"], "passed"
        )
        invalid = copy.deepcopy(valid)
        invalid["algorithm_evidence_validation"] = {
            "status": "failed",
            "problems": ["backend_failure_trace_nonempty"],
        }
        with self.assertRaisesRegex(CarlaAcceptanceError, "algorithm evidence"):
            validate_acceptance_runs(
                [copy.deepcopy(valid), invalid, copy.deepcopy(valid)],
                require_algorithm_clean=True,
            )

    def test_transfuserpp_lidar_coordinate_file_hash_is_verified(self):
        from agents.plugin_contract import canonical_sha256
        from agents.transfuserpp_contract import cuda_runtime_identity
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            _validate_transfuserpp_external_evidence,
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "lidar-coordinate.json"
            payload_path = Path(directory) / "live-lidar.xyzi"
            native_capture_points_path = Path(directory) / "carla-native-lidar.xyzi"
            native_capture_path = Path(directory) / "carla-native-lidar.json"
            native_scan_path = Path(directory) / "native-scan-manifest.json"
            points = [
                (1.0, 0.0, 0.0, 0.5),
                (0.0, 2.0, 0.0, 0.5),
                (0.0, 0.0, 3.0, 0.5),
                (2.0, 2.0, 2.0, 0.5),
            ]
            payload_path.write_bytes(
                b"".join(struct.pack("<4f", *point) for point in points)
            )
            # This is a separately materialized CARLA-native capture from the
            # same simulated sensor/frame, not a claim derived from the NRE
            # response metadata.  It supplies independent anchor ground truth.
            native_capture_points_path.write_bytes(
                b"".join(struct.pack("<4f", *point) for point in points)
            )
            native_scan_path.write_text("{\"scan\":0}", encoding="utf-8")
            matrix = [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 2.5,
                0.0, 0.0, 0.0, 1.0,
            ]
            artifact_sha = "a" * 64
            native_capture = {
                "schema_version": "scene0061_carla_native_lidar_capture.v1",
                "status": "passed",
                "carla_frame_id": 491,
                "coordinate_frame": "carla_sensor",
                "sensor_to_ego_observation": "carla_actor_get_transform",
                "sensor_to_ego": matrix,
                "observed_sensor_world_transform": matrix,
                "observed_ego_world_transform": [
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0,
                ],
                "raw_xyzi_ref": {
                    "path": str(native_capture_points_path),
                    "sha256": hashlib.sha256(native_capture_points_path.read_bytes()).hexdigest(),
                    "byte_count": native_capture_points_path.stat().st_size,
                    "encoding": "float32_xyzi_little_endian",
                    "carla_frame_id": 491,
                },
            }
            native_capture_path.write_text(json.dumps(native_capture), encoding="utf-8")
            evidence = {
                "schema_version": "scene0061_lidar_coordinate_validation.v1",
                "status": "passed",
                "scene_id": "cc8c0bf57f984915a77078b10eb33198",
                "runtime_scene_id": "scene-0061",
                "artifact_sha256": artifact_sha,
                "sensor_id": "lidar_top",
                "device_type": "AT128",
                "response_coordinate_frame": "sensor_local",
                "axis_convention": "carla_sensor",
                "sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
                "sensor_to_ego": matrix,
                "sensor_to_ego_sha256": canonical_sha256(matrix),
                "live_render_lidar": {
                    "status": "passed",
                    "rpc_status": "ok",
                    "payload_sha256_valid": True,
                    "point_count": len(points),
                    "timestamp_inside_artifact_range": True,
                    "scene_start_matches_artifact": True,
                    "carla_frame_id": 491,
                },
                "axis_validation": {
                    "schema_version": "scene0061_lidar_axis_alignment.v1",
                    "status": "passed",
                    "evidence_source": "scene0061_carla_native_lidar_capture.v1",
                    "carla_frame_id": 491,
                    "tolerance_m": 0.01,
                    "measured_max_abs_error_m": 0.0,
                    "measured_rms_error_m": 0.0,
                    "payload_ref": {
                        "path": str(payload_path),
                        "sha256": hashlib.sha256(payload_path.read_bytes()).hexdigest(),
                        "byte_count": payload_path.stat().st_size,
                        "encoding": "float32_xyzi_little_endian",
                        "carla_frame_id": 491,
                    },
                    "native_scan_manifest_ref": {
                        "path": str(native_scan_path),
                        "sha256": hashlib.sha256(native_scan_path.read_bytes()).hexdigest(),
                        "byte_count": native_scan_path.stat().st_size,
                        "carla_frame_id": 491,
                        "scan_index": 0,
                    },
                    "independent_carla_capture_ref": {
                        "path": str(native_capture_path),
                        "sha256": hashlib.sha256(native_capture_path.read_bytes()).hexdigest(),
                        "byte_count": native_capture_path.stat().st_size,
                        "carla_frame_id": 491,
                    },
                    "independent_carla_points_ref": native_capture["raw_xyzi_ref"],
                    "anchors": [
                        {
                            "source_point_index": index,
                            "sensor_local_point_m": list(point[:3]),
                            "carla_ego_point_m": [point[0], point[1], point[2] + 2.5],
                            "independent_capture_point_index": index,
                            "ground_truth_source": "same_frame_carla_native_lidar",
                        }
                        for index, point in enumerate(points)
                    ],
                },
            }
            path.write_text(json.dumps(evidence), encoding="utf-8")
            config = {
                "experiment": {
                    "scene_id": "cc8c0bf57f984915a77078b10eb33198",
                    "scene_version": "formal40k-v1",
                    "case_id": "S0_original_replay",
                    "seed": 41,
                    "artifact_sha256": artifact_sha,
                    "scene_package_sha256": "2" * 64,
                    "scenario_ir_sha256": "3" * 64,
                    "immutable_matrix_sha256": "4" * 64,
                    "source_run_config_sha256": "5" * 64,
                    "variant_config_sha256": "6" * 64,
                    "run_config_sha256": "7" * 64,
                },
                "nurec_runtime": {
                    "runtime_scene_id": "scene-0061",
                    "lidar_specs": [
                        {
                            "sensor_id": "lidar_top",
                            "model": "AT128",
                            "sensor_to_ego": matrix,
                        }
                    ],
                    "lidar_coordinate_validation": {
                        "evidence_path": str(path),
                        "evidence_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    }
                }
            }
            cuda_path = Path(directory) / "cuda-preflight.json"
            runtime_config = {
                "repo_revision": "b" * 40,
                "repo_sha256": "c" * 64,
                "upstream_reference": "refs/remotes/origin/leaderboard_2",
                "checkpoint_sha256": "d" * 64,
                "model_config_sha256": "e" * 64,
                "carla_agents_sha256": "f" * 64,
                "container_image_digest": "sha256:" + "1" * 64,
                "device": "cuda:0",
                "cuda_gate": {
                    "warmup_iterations": 2,
                    "measured_iterations": 3,
                    "max_peak_memory_bytes": 2048,
                    "max_p95_latency_ms": 100.0,
                    "max_p99_latency_ms": 120.0,
                },
            }
            runtime_identity = cuda_runtime_identity(runtime_config)
            cuda_evidence = {
                "schema_version": "transfuserpp_cuda_preflight.v1",
                "status": "passed",
                "real_checkpoint_loaded": True,
                "tensor_warmup_completed": True,
                "warmup_iterations": 2,
                "measured_iterations": 3,
                "latency_ms": {
                    "samples": [10.0, 11.0, 12.0],
                    "mean": 11.0,
                    "p50": 11.0,
                    "p95": 11.9,
                    "p99": 11.98,
                },
                "cuda_peak_memory_allocated_bytes": 1024,
                "gate": runtime_config["cuda_gate"],
                "runtime_identity": runtime_identity,
                "experiment": config["experiment"],
            }
            cuda_path.write_text(json.dumps(cuda_evidence), encoding="utf-8")
            config["algorithm_runtime_identity"] = runtime_identity
            config["algorithm_gpu_validation"] = {
                "status": "bound",
                "evidence_path": str(cuda_path),
                "evidence_sha256": hashlib.sha256(cuda_path.read_bytes()).hexdigest(),
            }
            _validate_transfuserpp_external_evidence(config)
            cuda_evidence["latency_ms"] = {
                "samples": [10.0, 11.0, 900.0],
                "mean": 11.0,
                "p50": 11.0,
                "p95": 11.9,
                "p99": 11.98,
            }
            cuda_path.write_text(json.dumps(cuda_evidence), encoding="utf-8")
            config["algorithm_gpu_validation"]["evidence_sha256"] = hashlib.sha256(
                cuda_path.read_bytes()
            ).hexdigest()
            with self.assertRaisesRegex(CarlaAcceptanceError, "CUDA warmup/VRAM/latency"):
                _validate_transfuserpp_external_evidence(config)
            cuda_evidence["latency_ms"] = {
                "samples": [10.0, "not-a-latency", 12.0],
                "mean": 11.0,
                "p50": 11.0,
                "p95": 11.9,
                "p99": 11.98,
            }
            cuda_path.write_text(json.dumps(cuda_evidence), encoding="utf-8")
            config["algorithm_gpu_validation"]["evidence_sha256"] = hashlib.sha256(
                cuda_path.read_bytes()
            ).hexdigest()
            with self.assertRaisesRegex(CarlaAcceptanceError, "CUDA warmup/VRAM/latency"):
                _validate_transfuserpp_external_evidence(config)
            config["nurec_runtime"]["runtime_scene_id"] = "different-runtime-scene"
            with self.assertRaisesRegex(CarlaAcceptanceError, "cannot be verified"):
                _validate_transfuserpp_external_evidence(config)
            config["nurec_runtime"]["runtime_scene_id"] = "scene-0061"
            config["nurec_runtime"]["lidar_coordinate_validation"][
                "evidence_sha256"
            ] = "0" * 64
            with self.assertRaisesRegex(CarlaAcceptanceError, "cannot be verified"):
                _validate_transfuserpp_external_evidence(config)

    def test_transfuserpp_lidar_evidence_rejects_runtime_scene_mismatch_and_duplicate_keys(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            _validate_transfuserpp_external_evidence,
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "lidar-coordinate.json"
            path.write_text(
                '{"schema_version":"scene0061_lidar_coordinate_validation.v1",'
                '"status":"passed","status":"passed"}',
                encoding="utf-8",
            )
            config = {
                "experiment": {"scene_id": "scene", "artifact_sha256": "a" * 64},
                "nurec_runtime": {
                    "runtime_scene_id": "runtime-A",
                    "lidar_specs": [
                        {
                            "sensor_id": "lidar_top",
                            "model": "AT128",
                            "sensor_to_ego": [1.0] * 16,
                        }
                    ],
                    "lidar_coordinate_validation": {
                        "evidence_path": str(path),
                        "evidence_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    },
                },
            }
            with self.assertRaisesRegex(CarlaAcceptanceError, "cannot be verified"):
                _validate_transfuserpp_external_evidence(config)

            path.write_text("{}", encoding="utf-8")
            config["nurec_runtime"]["lidar_coordinate_validation"][
                "evidence_sha256"
            ] = hashlib.sha256(path.read_bytes()).hexdigest()
            config["nurec_runtime"]["runtime_scene_id"] = ""
            with self.assertRaisesRegex(CarlaAcceptanceError, "cannot be verified"):
                _validate_transfuserpp_external_evidence(config)

    def test_rejects_control_only_actor_claim_and_unknown_collision_sensor(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            validate_acceptance_runs,
        )

        control_only = _result()
        control_only["report"]["runtime"]["actor_physical_response"] = {}
        with self.assertRaisesRegex(CarlaAcceptanceError, "physical Actor"):
            validate_acceptance_runs([_result(), control_only, _result()])

        no_sensor = _result()
        no_sensor["report"]["runtime"]["collision_sensor_available"] = False
        with self.assertRaisesRegex(CarlaAcceptanceError, "collision sensor"):
            validate_acceptance_runs([_result(), no_sensor, _result()])

    def test_rejects_route_cleanup_and_run_count_gaps(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            validate_acceptance_runs,
        )

        with self.assertRaisesRegex(CarlaAcceptanceError, "exactly three"):
            validate_acceptance_runs([_result(), _result()])
        route = _result()
        route["report"]["summary"]["route_progress"] = 0.94
        with self.assertRaisesRegex(CarlaAcceptanceError, "route progress"):
            validate_acceptance_runs([_result(), route, _result()])
        cleanup = _result()
        cleanup["cleanup_succeeded"] = False
        with self.assertRaisesRegex(CarlaAcceptanceError, "cleanup"):
            validate_acceptance_runs([_result(), cleanup, _result()])

    def test_optional_multimodal_gate_requires_complete_actor_sensor_evidence(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            validate_acceptance_runs,
        )
        from tests.test_multimodal_closed_loop_acceptance import _result as multimodal_result

        valid = multimodal_result()
        valid["cleanup_succeeded"] = True
        valid["sensor_handler_cleanup_succeeded"] = True
        valid["report"]["summary"] = {"route_progress": 0.99, "collision_count": 0}
        valid["report"]["runtime"].update(
            collision_sensor_available=True,
            frame_trace_count=2,
        )
        accepted = validate_acceptance_runs(
            [copy.deepcopy(valid), copy.deepcopy(valid), copy.deepcopy(valid)],
            require_multimodal=True,
        )
        self.assertEqual(accepted[0]["multimodal_closed_loop"]["status"], "passed")

        invalid = copy.deepcopy(valid)
        invalid["nurec_multimodal_trace"][0]["modalities"]["rgb"]["passed_count"] = 0
        with self.assertRaisesRegex(CarlaAcceptanceError, "multimodal"):
            validate_acceptance_runs(
                [copy.deepcopy(valid), invalid, copy.deepcopy(valid)],
                require_multimodal=True,
            )

    def test_multimodal_handler_cleanup_is_fail_closed(self):
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            run_acceptance_triplicate,
        )

        class Handler:
            def __call__(self, _context):
                return {}

            def close(self):
                raise RuntimeError("grpc close failed")

        def execute(_plan, *, sensor_frame_handler):
            self.assertIsInstance(sensor_frame_handler, Handler)
            return _result()

        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(CarlaAcceptanceError, "cleanup failed"):
                run_acceptance_triplicate(
                    {
                        "scenario_id": "scene-triplicate",
                        "run_id": "cleanup",
                        "ego": {"initial_state": {"x": 0.0, "y": 0.0}},
                        "actors": [],
                    },
                    Path(directory),
                    require_multimodal=True,
                    sensor_frame_handler_factory=lambda _config, _run_dir: Handler(),
                    execute=execute,
                )


if __name__ == "__main__":
    unittest.main()
