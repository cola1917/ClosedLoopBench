import json
import unittest


class Ros2TcpEgoPluginContractTests(unittest.TestCase):
    def test_tcp_policy_config_declares_optional_external_adapter(self):
        from agents.ego_policy import build_ego_policy_config

        config = build_ego_policy_config(kind="ros2_external_agent", stack="tcp")

        self.assertEqual(config["type"], "ros2_external_agent")
        self.assertEqual(config["runtime"], "ros2_bridge")
        self.assertEqual(config["stack"], "tcp")
        self.assertEqual(config["plugin"], "external_ros2_tcp")
        self.assertEqual(config["availability"], "optional_adapter")
        self.assertFalse(config["required_runtime"])

    def test_tcp_policy_config_has_required_ros2_topics(self):
        from agents.ego_policy import build_ego_policy_config

        config = build_ego_policy_config(kind="ros2_external_agent", stack="tcp")

        self.assertIn("/closed_loop/ego/state", config["input_topics"])
        self.assertIn("/closed_loop/world/state", config["input_topics"])
        self.assertIn("/closed_loop/route", config["input_topics"])
        self.assertIn("/closed_loop/sensors", config["input_topics"])
        self.assertEqual(config["output_topic"], "/closed_loop/ego/control_cmd")

    def test_tcp_policy_config_maps_internal_topics_to_carla_ros_bridge(self):
        from agents.ego_policy import build_ego_policy_config

        config = build_ego_policy_config(kind="ros2_external_agent", stack="tcp")
        bridge_topics = config["ros2_bridge_topics"]

        self.assertEqual(
            bridge_topics["ego_state"],
            {
                "internal": "/closed_loop/ego/state",
                "carla_ros_bridge": "/carla/ego_vehicle/odometry",
            },
        )
        self.assertEqual(
            bridge_topics["control_cmd"],
            {
                "internal": "/closed_loop/ego/control_cmd",
                "carla_ros_bridge": "/carla/ego_vehicle/vehicle_control_cmd",
            },
        )

    def test_tcp_policy_config_keeps_runtime_path_optional(self):
        from agents.ego_policy import build_ego_policy_config

        config = build_ego_policy_config(kind="ros2_external_agent", stack="tcp")
        runtime_path = config["runtime_path"]

        self.assertEqual(runtime_path["adapter_kind"], "python_module")
        self.assertIsNone(runtime_path["repo_path"])
        self.assertEqual(runtime_path["entrypoint"], "tcp_ros2_adapter")
        self.assertIsNone(runtime_path["launch_package"])

    def test_tcp_policy_config_is_json_serializable(self):
        from agents.ego_policy import build_ego_policy_config

        config = build_ego_policy_config(kind="ros2_external_agent", stack="tcp")

        json.dumps(config, sort_keys=True)


if __name__ == "__main__":
    unittest.main()
