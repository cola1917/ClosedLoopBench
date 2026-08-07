import unittest


HASH = "b" * 64
def _calibration():
    from agents.transfuserpp_contract import camera_adaptation_contract

    return {
    "camera_sensor_id": "camera_front",
    "camera_sensor_to_ego": [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 2.5,
        0.0, 0.0, 0.0, 1.0,
    ],
    "camera_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
    "camera_adaptation": camera_adaptation_contract(),
    "lidar_sensor_id": "lidar_top",
    "lidar_sensor_to_ego": [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 2.5,
        0.0, 0.0, 0.0, 1.0,
    ],
    }


CALIBRATION = _calibration()


def _record(frame_id=1):
    return {
        "schema_version": "transfuserpp_intermediate_frame.v1",
        "algorithm_id": "transfuserpp_v5",
        "algorithm_version": "carla_garage.leaderboard_2.transfuser_v5",
        "frame_id": frame_id,
        "timestamp": 0.05,
        "identity": {
            "repo_sha256": HASH,
            "checkpoint_sha256": HASH,
            "model_config_sha256": HASH,
            "repo_revision": "c" * 40,
            "runtime_config_sha256": HASH,
            "carla_agents_sha256": HASH,
            "adapter_source_sha256": HASH,
            "container_image_digest": "sha256:" + HASH,
        },
        "experiment": {
            "scene_id": "cc8c0bf57f984915a77078b10eb33198",
            "scene_version": "formal40k-v1",
            "case_id": "S0_original_replay",
            "seed": 41,
            "run_id": "scene0061-attempt-01",
            "artifact_sha256": HASH,
            "scene_package_sha256": HASH,
            "scenario_ir_sha256": HASH,
            "immutable_matrix_sha256": HASH,
            "source_run_config_sha256": HASH,
            "variant_config_sha256": HASH,
            "run_config_sha256": HASH,
        },
        "provenance": {
            "execution_mode": "remote_model_inference",
            "real_checkpoint_loaded": True,
        },
        "inputs": {
            "camera_front": {
                "path": "front.jpg",
                "sha256": HASH,
                "byte_count": 1,
                "encoding": "jpeg",
                "coordinate_frame": "camera_optical",
            },
            "lidar_top": {
                "path": "lidar.bin",
                "sha256": HASH,
                "byte_count": 16,
                "encoding": "float32_xyzi_little_endian",
                "coordinate_frame": "sensor_local",
                "axis_convention": "carla_sensor",
                "sensor_to_ego": list(CALIBRATION["lidar_sensor_to_ego"]),
            },
            "calibration": dict(CALIBRATION),
            "camera_adaptation": __import__(
                "agents.transfuserpp_contract", fromlist=["camera_adaptation_evidence"]
            ).camera_adaptation_evidence(
                contract=CALIBRATION["camera_adaptation"],
                source_payload={
                    "sha256": HASH,
                    "byte_count": 1,
                },
                model_sensor_width=1024,
                model_sensor_height=512,
                center_crop_xyxy=[0, 25, 800, 425],
                model_crop_applied_by_upstream=True,
            ),
            "model_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
            "ego_pose_coordinate_frame": "closedloopbench_scene_x_forward_y_left_z_up",
            "ego_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        },
        "outputs": {
            "waypoints_ego_m": [[1.0, 0.0]],
            "route_checkpoints_ego_m": [[2.0, 0.0]],
            "target_speed_mps": 5.0,
            "target_speed_probabilities": [0.2, 0.8],
            "target_speed_bins_mps": [0.0, 5.0],
            "target_speed_selection_mode": "argmax",
            "target_speed_selected_index": 1,
            "target_speed_brake_uncertainty_threshold": 0.5,
            "bounding_boxes_ego": [],
            "control": {
                "throttle": 0.2,
                "steer": 0.1,
                "brake": 0.0,
                "hand_brake": False,
                "reverse": False,
            },
        },
        "latency_ms": {"inference": 12.0},
        "synchronization": {
            "frame_id": frame_id,
            "error_ms": 0.0,
            "dynamic_object_sha256": HASH,
        },
        "dense_outputs": {
            "path": "not-read-by-plugin-contract.npz",
            "sha256": HASH,
            "encoding": "numpy_npz",
            "required_keys": [
                "bev_semantic_labels",
                "perspective_semantic_labels",
                "depth",
                "target_speed_probabilities",
            ],
        },
        "dynamic_bev_proxy": {
            "class_mapping": {"vehicle": 9, "pedestrian": 10},
            "grid": {"row_axis": "ego_x_forward", "column_axis": "ego_y_right"},
        },
        "semantics": {
            "occupancy_evaluation": "dynamic_bev_proxy_only",
            "full_3d_occupancy_ground_truth_available": False,
        },
    }


class FakeRuntime:
    def __init__(self, _config):
        self.reset_count = 0
        self.closed = False
        self.warmup_calls = []

    def health_check(self):
        return {
            "status": "ready",
            "real_checkpoint_loaded": False,
            "identity": {},
        }

    def reset(self):
        self.reset_count += 1

    def warmup(self, observation, *, iterations=1):
        self.warmup_calls.append((observation, iterations))
        return {
            "status": "completed",
            "iterations": iterations,
            "frame_id": observation["frame_id"],
            "formal_frame_excluded": True,
            "intermediate_count": 0,
            "real_checkpoint_loaded": True,
        }

    def predict(self, observation):
        return _record(observation["frame_id"])

    def close(self):
        self.closed = True


class TransFuserPPPluginTests(unittest.TestCase):
    def test_target_speed_selection_modes_match_official_uncertainty_logic(self):
        from agents.transfuserpp_contract import validate_intermediate_record

        weighted = _record(1)
        weighted["outputs"].update(
            target_speed_mps=4.0,
            target_speed_selection_mode="weighted_expectation",
            target_speed_selected_index=None,
        )
        validate_intermediate_record(weighted)

        override = _record(2)
        override["outputs"].update(
            target_speed_mps=0.0,
            target_speed_probabilities=[0.8, 0.2],
            target_speed_selection_mode="brake_uncertainty_override",
            target_speed_selected_index=0,
        )
        validate_intermediate_record(override)

    def test_test_runtime_is_explicit_and_never_claims_real_checkpoint(self):
        from agents.transfuserpp_plugin import TransFuserPPPlugin

        plugin = TransFuserPPPlugin(runtime_factory=FakeRuntime)
        plugin.initialize({"allow_test_runtime": True})
        plugin.reset({"scene_id": "scene"})
        control = plugin.predict_control({"frame_id": 9})
        self.assertEqual(control["source_frame_id"], 9)
        self.assertEqual(control["target_speed_mps"], 5.0)
        self.assertFalse(plugin.health_check()["real_checkpoint_loaded"])
        plugin.close()

    def test_formal_warmup_is_forwarded_without_a_scored_intermediate(self):
        from agents.transfuserpp_plugin import TransFuserPPPlugin

        plugin = TransFuserPPPlugin(runtime_factory=FakeRuntime)
        plugin.initialize({"allow_test_runtime": True})
        observation = {"frame_id": 0}
        result = plugin.warmup(observation, iterations=3)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["intermediate_count"], 0)
        self.assertEqual(plugin.runtime.warmup_calls, [(observation, 3)])
        plugin.close()

    def test_fake_runtime_cannot_be_presented_as_real_without_test_flag(self):
        from agents.transfuserpp_plugin import TransFuserPPPlugin

        plugin = TransFuserPPPlugin(runtime_factory=FakeRuntime)
        with self.assertRaisesRegex(RuntimeError, "real checkpoint"):
            plugin.initialize({})


if __name__ == "__main__":
    unittest.main()
