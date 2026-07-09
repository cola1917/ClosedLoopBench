import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.subscriptions = []
        self.publishers = {}

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append(
            {
                "msg_type": msg_type,
                "topic": topic,
                "callback": callback,
                "qos": qos,
            }
        )
        return callback

    def create_publisher(self, msg_type, topic, qos):
        publisher = FakePublisher(topic)
        self.publishers[topic] = publisher
        return publisher


class FakeBackend:
    def __init__(self):
        self.observations = []

    def predict_control(self, observation):
        self.observations.append(observation)
        return {
            "throttle": 0.3,
            "steer": 0.05,
            "brake": 0.0,
            "hand_brake": False,
            "reverse": False,
        }


class Ros2TcpBridgeTests(unittest.TestCase):
    def test_build_ros2_tcp_bridge_plan_wires_expected_topics(self):
        from agents.ros2_tcp_bridge import build_ros2_tcp_bridge_plan

        plan = build_ros2_tcp_bridge_plan(
            scenario_id="scene-bridge-001",
            role_name="ego_vehicle",
            camera_profile="tcp_front",
        )

        self.assertEqual(plan["schema_version"], "ros2_tcp_bridge_plan.mvp.v0")
        self.assertEqual(plan["plugin"], "external_ros2_tcp")
        self.assertEqual(plan["role_name"], "ego_vehicle")
        self.assertEqual(
            plan["topics"]["sensors"]["rgb_front"],
            "/carla/ego_vehicle/rgb_front/image",
        )
        self.assertEqual(plan["topics"]["ego_state"], "/closed_loop/ego/state")
        self.assertEqual(plan["topics"]["route"], "/closed_loop/route")
        self.assertEqual(
            plan["topics"]["control_cmd"],
            "/carla/ego_vehicle/vehicle_control_cmd",
        )
        self.assertFalse(plan["runtime_boundary"]["requires_rclpy_for_tests"])
        json.dumps(plan, sort_keys=True)

    def test_bridge_registers_subscribers_and_control_publisher(self):
        from agents.ros2_tcp_bridge import Ros2TcpBridge, build_ros2_tcp_bridge_plan

        node = FakeNode()
        plan = build_ros2_tcp_bridge_plan(scenario_id="scene-bridge-001")
        bridge = Ros2TcpBridge(node=node, plan=plan, backend=FakeBackend())

        self.assertIs(bridge.node, node)
        topics = [subscription["topic"] for subscription in node.subscriptions]
        self.assertIn("/carla/ego_vehicle/rgb_front/image", topics)
        self.assertIn("/closed_loop/ego/state", topics)
        self.assertIn("/closed_loop/route", topics)
        self.assertIn("/carla/ego_vehicle/vehicle_control_cmd", node.publishers)

    def test_fake_messages_tick_adapter_and_publish_control(self):
        from agents.ros2_tcp_bridge import Ros2TcpBridge, build_ros2_tcp_bridge_plan

        backend = FakeBackend()
        node = FakeNode()
        plan = build_ros2_tcp_bridge_plan(scenario_id="scene-bridge-001")
        bridge = Ros2TcpBridge(node=node, plan=plan, backend=backend)

        bridge.receive_sensor("rgb_front", {"frame": "image-001"}, t_sec=1.0)
        bridge.receive_ego_state({"speed_mps": 5.0}, t_sec=1.0)
        bridge.receive_route({"route_command": "LANE_FOLLOW"}, t_sec=1.0)
        result = bridge.tick(now_sec=1.0)

        self.assertEqual(result["status"], "control")
        self.assertEqual(backend.observations[0]["sensors"]["rgb_front"], {"frame": "image-001"})
        publisher = node.publishers["/carla/ego_vehicle/vehicle_control_cmd"]
        self.assertEqual(len(publisher.messages), 1)
        self.assertEqual(publisher.messages[0]["throttle"], 0.3)
        self.assertEqual(publisher.messages[0]["brake"], 0.0)

    def test_timeout_fallback_publishes_safe_stop_without_backend_call(self):
        from agents.ros2_tcp_bridge import Ros2TcpBridge, build_ros2_tcp_bridge_plan

        backend = FakeBackend()
        node = FakeNode()
        plan = build_ros2_tcp_bridge_plan(scenario_id="scene-bridge-001", timeout_sec=0.2)
        bridge = Ros2TcpBridge(node=node, plan=plan, backend=backend)

        bridge.receive_sensor("rgb_front", {"frame": "image-001"}, t_sec=1.0)
        bridge.receive_ego_state({"speed_mps": 5.0}, t_sec=1.0)
        bridge.receive_route({"route_command": "LANE_FOLLOW"}, t_sec=1.0)
        result = bridge.tick(now_sec=1.31)

        self.assertEqual(result["status"], "fallback")
        self.assertEqual(result["reason"], "stale_observation")
        self.assertEqual(backend.observations, [])
        publisher = node.publishers["/carla/ego_vehicle/vehicle_control_cmd"]
        self.assertEqual(publisher.messages[0]["brake"], 1.0)

    def test_plan_ros2_tcp_bridge_cli_writes_plan(self):
        from runners.plan_ros2_tcp_bridge import main

        with tempfile.TemporaryDirectory() as tmpdir:
            output = Path(tmpdir) / "ros2_tcp_bridge_plan.json"
            stdout = io.StringIO()

            with contextlib.redirect_stdout(stdout):
                exit_code = main(
                    [
                        "--scenario-id",
                        "scene-bridge-cli",
                        "--role-name",
                        "ego_vehicle",
                        "--timeout-sec",
                        "0.75",
                        "--output",
                        str(output),
                    ]
                )

            self.assertEqual(exit_code, 0)
            cli_result = json.loads(stdout.getvalue())
            self.assertEqual(cli_result["status"], "planned")
            self.assertEqual(cli_result["ros2_tcp_bridge_plan"], str(output))
            plan = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(plan["scenario_id"], "scene-bridge-cli")
            self.assertEqual(plan["timeout_sec"], 0.75)


if __name__ == "__main__":
    unittest.main()
