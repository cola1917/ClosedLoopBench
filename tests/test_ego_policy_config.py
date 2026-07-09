import json
import unittest


class EgoPolicyConfigContractTests(unittest.TestCase):
    def test_basic_agent_config_has_type_and_settings(self):
        from agents.ego_policy import build_ego_policy_config

        config = build_ego_policy_config(kind="basic_agent")

        self.assertEqual(config["type"], "basic_agent")
        self.assertEqual(config["runtime"], "builtin")
        self.assertEqual(config["settings"]["agent"], "basic_agent")
        self.assertEqual(config["settings"]["control_mode"], "vehicle_control")

    def test_ros2_external_agent_transfuser_config_has_bridge_contract(self):
        from agents.ego_policy import build_ego_policy_config

        config = build_ego_policy_config(kind="ros2_external_agent", stack="transfuser")

        self.assertEqual(config["type"], "ros2_external_agent")
        self.assertEqual(config["stack"], "transfuser")
        self.assertEqual(config["runtime"], "ros2_bridge")
        self.assertIn("/closed_loop/ego/state", config["input_topics"])
        self.assertIn("/closed_loop/world/state", config["input_topics"])
        self.assertIn("/closed_loop/route", config["input_topics"])
        self.assertEqual(config["output_topic"], "/closed_loop/ego/control_cmd")
        self.assertGreater(config["timeout"]["tick_seconds"], 0)
        self.assertEqual(config["safety_fallback"]["policy"], "basic_agent")

    def test_ros2_external_agent_supports_known_stack_names(self):
        from agents.ego_policy import build_ego_policy_config

        supported_stacks = {"transfuser", "interfuser", "tcp", "uniad", "external"}

        for stack in supported_stacks:
            with self.subTest(stack=stack):
                config = build_ego_policy_config(kind="ros2_external_agent", stack=stack)
                self.assertEqual(config["stack"], stack)

    def test_tcp_stack_is_optional_adapter_with_runtime_metadata(self):
        from agents.ego_policy import build_ego_policy_config

        config = build_ego_policy_config(
            kind="ros2_external_agent",
            stack="tcp",
            runtime_path="E:/models/TCP",
            checkpoint_path="E:/models/TCP/checkpoints/tcp.pth",
        )

        self.assertEqual(config["availability"], "optional_adapter")
        self.assertFalse(config["required_runtime"])
        self.assertEqual(config["adapter"]["name"], "tcp")
        self.assertEqual(config["adapter"]["runtime_path"], "E:/models/TCP")
        self.assertEqual(config["adapter"]["checkpoint_path"], "E:/models/TCP/checkpoints/tcp.pth")
        self.assertIn("camera", config["adapter"]["expected_inputs"])
        self.assertIn("vehicle_control", config["adapter"]["expected_outputs"])

    def test_ros2_external_agent_has_carla_ros_bridge_topic_mapping(self):
        from agents.ego_policy import build_ego_policy_config

        config = build_ego_policy_config(kind="ros2_external_agent", stack="tcp", role_name="ego_vehicle")

        carla_topics = config["carla_ros_bridge_topics"]

        self.assertEqual(carla_topics["vehicle_status"], "/carla/ego_vehicle/vehicle_status")
        self.assertEqual(carla_topics["odometry"], "/carla/ego_vehicle/odometry")
        self.assertEqual(carla_topics["camera_front"], "/carla/ego_vehicle/rgb_front/image")
        self.assertEqual(carla_topics["control_cmd"], "/carla/ego_vehicle/vehicle_control_cmd")

    def test_uniad_is_optional_showcase_not_required_runtime(self):
        from agents.ego_policy import build_ego_policy_config

        config = build_ego_policy_config(kind="ros2_external_agent", stack="uniad")

        self.assertEqual(config["availability"], "optional_showcase")
        self.assertFalse(config["required_runtime"])
        self.assertIn("showcase", config["notes"])

    def test_configs_are_json_serializable(self):
        from agents.ego_policy import build_ego_policy_config

        configs = [
            build_ego_policy_config(kind="basic_agent"),
            build_ego_policy_config(kind="ros2_external_agent", stack="transfuser"),
            build_ego_policy_config(kind="ros2_external_agent", stack="uniad"),
            build_ego_policy_config(kind="ros2_external_agent", stack="tcp"),
        ]

        for config in configs:
            with self.subTest(config=config):
                json.dumps(config, sort_keys=True)


if __name__ == "__main__":
    unittest.main()
