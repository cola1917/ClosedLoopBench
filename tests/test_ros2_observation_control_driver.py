import json
import unittest


class Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Vehicle:
    def get_transform(self):
        return type("Transform", (), {
            "location": Vector(0.0, 0.0, 0.0),
            "rotation": type("Rotation", (), {"yaw": 0.0})(),
        })()

    def get_velocity(self):
        return Vector(2.0, 0.0, 0.0)

    def get_acceleration(self):
        return Vector()


class StringMessage:
    def __init__(self):
        self.data = ""


class Control:
    def __init__(self, frame_id=""):
        self.header = type("Header", (), {"frame_id": frame_id})()
        self.throttle = 0.4
        self.steer = 0.1
        self.brake = 0.0
        self.hand_brake = False
        self.reverse = False


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Node:
    def __init__(self):
        self.publisher = Publisher()
        self.callback = None

    def create_publisher(self, *_args):
        return self.publisher

    def create_subscription(self, _type, _topic, callback, _qos):
        self.callback = callback
        return callback


class Carla:
    class VehicleControl:
        def __init__(self, **kwargs):
            for key, value in kwargs.items():
                setattr(self, key, value)


class ObservationControlDriverTests(unittest.TestCase):
    def test_publishes_observation_and_accepts_only_matching_control(self):
        from agents.ros2_observation_control_driver import Ros2ObservationControlDriver

        node = Node()
        now = [10.0]

        def spin_once(_node, _timeout):
            observation = json.loads(node.publisher.messages[-1].data)
            now[0] += 0.004
            node.callback(Control(observation["observation_id"]))

        driver = Ros2ObservationControlDriver(
            node=node,
            carla_module=Carla,
            vehicle=Vehicle(),
            route=[{"x": 0.0, "y": 0.0}, {"x": 10.0, "y": 0.0}],
            control_message_type=Control,
            observation_message_type=StringMessage,
            control_topic="/control",
            observation_topic="/observation",
            spin_once=spin_once,
            clock=lambda: now[0],
        )
        control = driver.run_step()
        observation = json.loads(node.publisher.messages[0].data)

        self.assertEqual(observation["source"], "carla_current_tick")
        self.assertEqual(observation["ego_state"]["speed_mps"], 2.0)
        self.assertEqual(control.throttle, 0.4)
        self.assertEqual(driver.diagnostics()["matched_frame_ratio"], 1.0)
        self.assertAlmostEqual(driver.diagnostics()["latency_ms"]["mean"], 4.0)

    def test_pure_pursuit_steers_toward_route(self):
        from examples.reference_algorithm_plugins.reference_plugins import (
            ReferenceCruiseBackend,
        )

        backend = ReferenceCruiseBackend({
            "algorithm_id": "reference_pure_pursuit_short",
            "control_topic": "/control",
            "observation_topic": "/observation",
        })
        control = backend.predict_control({
            "ego_state": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}, "speed_mps": 1.0},
            "route": {
                "nearest_index": 0,
                "route_waypoints": [
                    {"x": 0.0, "y": 0.0},
                    {"x": 5.0, "y": 0.0},
                    {"x": 10.0, "y": 2.0},
                    {"x": 15.0, "y": 5.0},
                ],
            },
        })
        self.assertGreater(control["steer"], 0.0)
        self.assertGreater(control["throttle"], 0.0)


if __name__ == "__main__":
    unittest.main()
