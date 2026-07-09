import json
import unittest


class SensorAdapterContractTests(unittest.TestCase):
    def test_nuscenes_cameras_are_dataset_sources_not_closed_loop_inputs(self):
        from agents.sensor_contract import (
            NUSCENES_CAMERA_NAMES,
            build_sensor_source_contract,
        )

        contract = build_sensor_source_contract()

        self.assertEqual(
            set(NUSCENES_CAMERA_NAMES),
            {
                "CAM_FRONT",
                "CAM_FRONT_LEFT",
                "CAM_FRONT_RIGHT",
                "CAM_BACK",
                "CAM_BACK_LEFT",
                "CAM_BACK_RIGHT",
            },
        )
        self.assertEqual(contract["dataset_source"], "nuscenes")
        self.assertEqual(contract["closed_loop_observation_source"], "carla_current_tick")
        self.assertEqual(contract["nuscenes_usage"], ["mining", "reconstruction_source", "scenario_seed"])
        self.assertNotIn("ego_runtime_input", contract["nuscenes_usage"])

    def test_tcp_front_camera_profile_uses_carla_current_tick_front_camera(self):
        from agents.sensor_contract import build_camera_profile

        profile = build_camera_profile("tcp_front", role_name="ego_vehicle")

        self.assertEqual(profile["name"], "tcp_front")
        self.assertEqual(profile["source"], "carla_current_tick")
        self.assertEqual(profile["required_cameras"], ["rgb_front"])
        self.assertEqual(
            profile["topics"]["rgb_front"],
            "/carla/ego_vehicle/rgb_front/image",
        )
        self.assertEqual(profile["fallback"], "fail_fast")

    def test_multi_view_profile_maps_six_carla_camera_roles(self):
        from agents.sensor_contract import build_camera_profile

        profile = build_camera_profile("multi_view", role_name="ego_vehicle")

        self.assertEqual(profile["name"], "multi_view")
        self.assertEqual(len(profile["required_cameras"]), 6)
        self.assertEqual(
            profile["required_cameras"],
            [
                "rgb_front",
                "rgb_front_left",
                "rgb_front_right",
                "rgb_back",
                "rgb_back_left",
                "rgb_back_right",
            ],
        )
        self.assertEqual(
            profile["topics"]["rgb_back_right"],
            "/carla/ego_vehicle/rgb_back_right/image",
        )

    def test_unknown_camera_profile_is_rejected(self):
        from agents.sensor_contract import build_camera_profile

        with self.assertRaises(ValueError):
            build_camera_profile("fisheye_ring")


class TcpAdapterContractTests(unittest.TestCase):
    def test_tcp_io_contract_declares_speed_route_command_and_control(self):
        from agents.tcp_adapter_contract import build_tcp_io_contract

        contract = build_tcp_io_contract(camera_profile="tcp_front")

        self.assertEqual(contract["adapter"], "tcp")
        self.assertEqual(contract["inputs"]["sensor_profile"]["name"], "tcp_front")
        self.assertIn("speed_mps", contract["inputs"]["ego_state"]["fields"])
        self.assertIn("route_waypoints", contract["inputs"]["route"]["fields"])
        self.assertIn("route_command", contract["inputs"]["route"]["fields"])
        self.assertEqual(
            contract["outputs"]["control"],
            {
                "type": "vehicle_control",
                "fields": ["throttle", "steer", "brake", "hand_brake", "reverse"],
                "topic": "/closed_loop/ego/control_cmd",
            },
        )

    def test_tcp_adapter_config_is_optional_and_model_free(self):
        from agents.tcp_adapter_contract import build_tcp_adapter_config

        config = build_tcp_adapter_config(
            camera_profile="tcp_front",
            runtime_path=None,
            checkpoint_path=None,
        )

        self.assertEqual(config["plugin"], "external_ros2_tcp")
        self.assertEqual(config["availability"], "optional_adapter")
        self.assertFalse(config["required_runtime"])
        self.assertFalse(config["vendors_model_code"])
        self.assertIsNone(config["runtime"]["repo_path"])
        self.assertIsNone(config["runtime"]["checkpoint_path"])
        json.dumps(config, sort_keys=True)

    def test_tcp_adapter_config_supports_multiview_profile_without_changing_boundary(self):
        from agents.tcp_adapter_contract import build_tcp_adapter_config

        config = build_tcp_adapter_config(camera_profile="multi_view", role_name="ego_vehicle")

        self.assertEqual(config["io"]["inputs"]["sensor_profile"]["name"], "multi_view")
        self.assertEqual(config["io"]["inputs"]["sensor_profile"]["source"], "carla_current_tick")
        self.assertEqual(config["runtime"]["execution_mode"], "external_process_or_ros2_node")


if __name__ == "__main__":
    unittest.main()
