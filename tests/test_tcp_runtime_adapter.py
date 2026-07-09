import json
import unittest


class TcpRuntimeAdapterTests(unittest.TestCase):
    def test_build_tcp_runtime_plan_uses_optional_repo_and_checkpoint(self):
        from agents.tcp_runtime_adapter import build_tcp_runtime_plan

        plan = build_tcp_runtime_plan(
            scenario_id="scene-tcp-001",
            role_name="ego_vehicle",
            runtime_path="E:/models/TCP",
            checkpoint_path="E:/models/TCP/tcp.pth",
        )

        self.assertEqual(plan["schema_version"], "tcp_runtime_plan.mvp.v0")
        self.assertEqual(plan["scenario_id"], "scene-tcp-001")
        self.assertEqual(plan["plugin"], "external_ros2_tcp")
        self.assertEqual(plan["algorithm"], "TCP")
        self.assertEqual(plan["runtime"]["repo_path"], "E:/models/TCP")
        self.assertEqual(plan["runtime"]["checkpoint_path"], "E:/models/TCP/tcp.pth")
        self.assertEqual(plan["io"]["inputs"]["sensor_profile"]["name"], "tcp_front")
        self.assertFalse(plan["runtime_boundary"]["vendors_model_code"])
        json.dumps(plan, sort_keys=True)

    def test_adapter_tick_calls_backend_with_tcp_observation(self):
        from agents.tcp_runtime_adapter import TcpRuntimeAdapter

        class FakeBackend:
            def __init__(self):
                self.seen = None

            def predict_control(self, observation):
                self.seen = observation
                return {
                    "throttle": 0.2,
                    "steer": -0.1,
                    "brake": 0.0,
                    "hand_brake": False,
                    "reverse": False,
                }

        backend = FakeBackend()
        adapter = TcpRuntimeAdapter(backend=backend)
        result = adapter.tick(
            sensors={"rgb_front": "frame-001"},
            ego_state={"speed_mps": 4.5, "pose": {"x": 1.0, "y": 2.0}},
            route={"route_command": "LANE_FOLLOW", "target_point": [10.0, 0.0]},
        )

        self.assertEqual(result["status"], "control")
        self.assertEqual(result["control"]["throttle"], 0.2)
        self.assertEqual(backend.seen["sensors"]["rgb_front"], "frame-001")
        self.assertEqual(backend.seen["ego_state"]["speed_mps"], 4.5)
        self.assertEqual(backend.seen["route"]["route_command"], "LANE_FOLLOW")

    def test_adapter_falls_back_when_backend_missing(self):
        from agents.tcp_runtime_adapter import TcpRuntimeAdapter

        adapter = TcpRuntimeAdapter(backend=None)
        result = adapter.tick(
            sensors={"rgb_front": "frame-001"},
            ego_state={"speed_mps": 4.5},
            route={"route_command": "LANE_FOLLOW"},
        )

        self.assertEqual(result["status"], "fallback")
        self.assertEqual(result["reason"], "backend_unavailable")
        self.assertEqual(result["control"], {"throttle": 0.0, "steer": 0.0, "brake": 1.0, "hand_brake": False, "reverse": False})

    def test_adapter_rejects_invalid_control_and_falls_back(self):
        from agents.tcp_runtime_adapter import TcpRuntimeAdapter

        class InvalidBackend:
            def predict_control(self, observation):
                return {"throttle": 4.0, "steer": 0.0, "brake": 0.0}

        result = TcpRuntimeAdapter(backend=InvalidBackend()).tick(
            sensors={"rgb_front": "frame-001"},
            ego_state={"speed_mps": 4.5},
            route={"route_command": "LANE_FOLLOW"},
        )

        self.assertEqual(result["status"], "fallback")
        self.assertEqual(result["reason"], "invalid_control")
        self.assertEqual(result["control"]["brake"], 1.0)

    def test_adapter_requires_front_camera_for_tcp_front_profile(self):
        from agents.tcp_runtime_adapter import TcpRuntimeAdapter

        result = TcpRuntimeAdapter(backend=object()).tick(
            sensors={"rgb_back": "frame-001"},
            ego_state={"speed_mps": 4.5},
            route={"route_command": "LANE_FOLLOW"},
        )

        self.assertEqual(result["status"], "fallback")
        self.assertEqual(result["reason"], "missing_required_sensor")
        self.assertIn("rgb_front", result["missing"])


if __name__ == "__main__":
    unittest.main()
