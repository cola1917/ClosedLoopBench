import copy
import unittest


IDENTITY = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 2.5,
    0.0, 0.0, 0.0, 1.0,
]


def _base():
    cameras = [
        "camera_front",
        "camera_front_left",
        "camera_front_right",
        "camera_back",
        "camera_back_left",
        "camera_back_right",
    ]
    vehicle = [
        {"t_sec": float(index), "x": float(index * 5), "y": 0.0, "speed_mps": 5.0}
        for index in range(8)
    ]
    walker = [
        {"t_sec": float(index + 2), "x": 5.0, "y": float(index), "speed_mps": 1.0}
        for index in range(8)
    ]
    return {
        "scenario_id": "scene-0061",
        "carla": {"fixed_delta_seconds": 0.05},
        "ego": {
            "initial_state": {"x": 0.0, "y": 0.0},
            "reference_trajectory": [{"x": 10.0, "y": 0.0}],
        },
        "actors": [
            {
                "actor_id": "lead",
                "binding": {"nurec_track_id": "c1958768d48640948f6053d04cffd35b"},
                "reference_trajectory": vehicle,
            },
            {
                "actor_id": "walker",
                "binding": {"nurec_track_id": "71603dd1a2ba4e9daf095535e38310ac"},
                "reference_trajectory": walker,
            },
        ],
        "nurec_runtime": {
            "runtime_scene_id": "scene-0061",
            "camera_specs": [
                {
                    "sensor_id": name,
                    "model": "recorded",
                    "width": 1600,
                    "height": 900,
                    "sensor_to_ego": IDENTITY,
                }
                for name in cameras
            ],
            "lidar_specs": [
                {"sensor_id": "lidar_top", "model": "AT128", "sensor_to_ego": IDENTITY}
            ],
            "lidar_response_coordinate_frame": "sensor_local",
            "lidar_axis_convention": "carla_sensor",
            "lidar_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
            "lidar_coordinate_validation": {
                "evidence_path": "/evidence/lidar-coordinate.json",
                "evidence_sha256": "f" * 64,
            },
        },
    }


class Scene0061TransFuserPPRemoteTests(unittest.TestCase):
    def test_prepares_unique_s2_runtime_and_multimodal_binding(self):
        from runtime.scene0061_counterfactual import build_scene0061_counterfactual_matrix
        from runtime.scene0061_transfuserpp_remote import (
            prepare_scene0061_transfuserpp_remote_run,
        )

        run, runtime, bundle = prepare_scene0061_transfuserpp_remote_run(
            _base(),
            {
                "repo_path": "/opt/algorithm/carla_garage",
                "cuda_gate": {
                    "warmup_iterations": 3,
                    "measured_iterations": 10,
                    "max_peak_memory_bytes": 100,
                    "max_p95_latency_ms": 500.0,
                    "max_p99_latency_ms": 750.0,
                },
            },
            build_scene0061_counterfactual_matrix(),
            case_id="S2_lead_hard_brake",
            seed=41,
            event_timestamp_sec=2.0,
        )
        self.assertEqual(run["ego"]["algorithm_sensor_binding"]["container_payload_root"], "/sim-data")
        self.assertEqual(
            run["ego"]["algorithm_sensor_binding"]["camera_sensor_to_ego"],
            IDENTITY,
        )
        self.assertEqual(
            (
                run["ego"]["algorithm_sensor_binding"]["camera_source_width"],
                run["ego"]["algorithm_sensor_binding"]["camera_source_height"],
            ),
            (1600, 900),
        )
        self.assertEqual(
            run["ego"]["algorithm_sensor_binding"]["camera_adaptation"][
                "target_width"
            ],
            800,
        )
        self.assertIn("S2_lead_hard_brake/seed_41", runtime["intermediate_output_dir"])
        self.assertEqual(runtime["experiment"]["variant_config_sha256"], run["experiment"]["variant_config_sha256"])
        self.assertEqual(bundle["status"], "remote_validation_required")
        self.assertEqual(run["algorithm_gpu_validation"]["status"], "pending")
        self.assertIn("cuda_preflight_evidence_bound", bundle["required_remote_gates"])
        from runners.run_carla_acceptance_triplicate import (
            CarlaAcceptanceError,
            _validate_transfuserpp_cuda_evidence,
            _validate_transfuserpp_run_config_identity,
        )
        _validate_transfuserpp_run_config_identity(run)
        with self.assertRaisesRegex(CarlaAcceptanceError, "CUDA warmup"):
            _validate_transfuserpp_cuda_evidence(run)
        rebound = copy.deepcopy(run)
        rebound["algorithm_gpu_validation"] = {
            "status": "bound",
            "evidence_path": "/new/evidence/cuda.json",
            "evidence_sha256": "a" * 64,
        }
        _validate_transfuserpp_run_config_identity(rebound)
        self.assertNotIn("real_checkpoint_loaded", runtime)
        self.assertEqual(
            bundle["counterfactual_event_evidence_sha256"],
            __import__("agents.plugin_contract", fromlist=["canonical_sha256"]).canonical_sha256(
                run["counterfactual_event_evidence"]
            ),
        )
        self.assertEqual(
            run["actor_control_contract"]["effective_modes_by_track"],
            {
                "c1958768d48640948f6053d04cffd35b": "scripted",
                "71603dd1a2ba4e9daf095535e38310ac": "replay",
            },
        )

    def test_rejects_unverified_lidar_coordinate_contract(self):
        from runtime.scene0061_counterfactual import build_scene0061_counterfactual_matrix
        from runtime.scene0061_transfuserpp_remote import (
            Scene0061TransFuserPPRemoteError,
            prepare_scene0061_transfuserpp_remote_run,
        )

        config = copy.deepcopy(_base())
        config["nurec_runtime"]["lidar_axis_convention"] = "unverified"
        with self.assertRaisesRegex(Scene0061TransFuserPPRemoteError, "axis convention"):
            prepare_scene0061_transfuserpp_remote_run(
                config,
                {},
                build_scene0061_counterfactual_matrix(),
                case_id="S0_original_replay",
                seed=41,
                event_timestamp_sec=None,
            )

    def test_rejects_noncanonical_formal_sensor_records(self):
        from runtime.scene0061_counterfactual import build_scene0061_counterfactual_matrix
        from runtime.scene0061_transfuserpp_remote import (
            Scene0061TransFuserPPRemoteError,
            prepare_scene0061_transfuserpp_remote_run,
        )

        matrix = build_scene0061_counterfactual_matrix()
        cases = []
        missing_runtime_scene = copy.deepcopy(_base())
        missing_runtime_scene["nurec_runtime"]["runtime_scene_id"] = ""
        cases.append((missing_runtime_scene, "runtime_scene_id"))
        duplicate = copy.deepcopy(_base())
        duplicate["nurec_runtime"]["camera_specs"][-1]["sensor_id"] = "camera_front"
        cases.append((duplicate, "unique"))
        wrong_size = copy.deepcopy(_base())
        wrong_size["nurec_runtime"]["camera_specs"][0]["width"] = 640
        cases.append((wrong_size, "1600x900"))
        invalid_pose = copy.deepcopy(_base())
        invalid_pose["nurec_runtime"]["camera_specs"][0]["sensor_to_ego"] = IDENTITY[:-1]
        cases.append((invalid_pose, "16"))
        extra_lidar = copy.deepcopy(_base())
        extra_lidar["nurec_runtime"]["lidar_specs"].append(
            copy.deepcopy(extra_lidar["nurec_runtime"]["lidar_specs"][0])
        )
        cases.append((extra_lidar, "exactly one LiDAR.*lidar_top"))
        unsupported_lidar = copy.deepcopy(_base())
        unsupported_lidar["nurec_runtime"]["lidar_specs"][0]["model"] = "VELODYNE64"
        cases.append((unsupported_lidar, "AT128 or PANDAR128"))
        reflected = copy.deepcopy(_base())
        reflected["nurec_runtime"]["lidar_specs"][0]["sensor_to_ego"] = [
            -1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 2.5,
            0.0, 0.0, 0.0, 1.0,
        ]
        cases.append((reflected, "determinant"))
        for config, message in cases:
            with self.subTest(message=message):
                with self.assertRaisesRegex(Scene0061TransFuserPPRemoteError, message):
                    prepare_scene0061_transfuserpp_remote_run(
                        config,
                        {},
                        matrix,
                        case_id="S0_original_replay",
                        seed=41,
                        event_timestamp_sec=None,
                    )


if __name__ == "__main__":
    unittest.main()
