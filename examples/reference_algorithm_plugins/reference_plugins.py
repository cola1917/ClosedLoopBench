from __future__ import annotations

import json
import math
import time
from typing import Any


_THROTTLE_BY_ALGORITHM = {
    "reference_cruise_035": 0.35,
    "reference_cruise_055": 0.55,
}

_ROUTE_FOLLOWERS = {
    "reference_pure_pursuit_short": {"lookahead_points": 3, "target_speed_mps": 4.0},
    "reference_pure_pursuit_long": {"lookahead_points": 7, "target_speed_mps": 5.0},
}


class ReferenceCruiseBackend:
    """Reference ROS2 algorithms for transport and route-closed-loop evidence."""

    def __init__(self, config: dict[str, Any]) -> None:
        algorithm_id = str(config["algorithm_id"])
        if algorithm_id not in _THROTTLE_BY_ALGORITHM and algorithm_id not in _ROUTE_FOLLOWERS:
            raise ValueError(f"unsupported reference algorithm_id: {algorithm_id}")
        self.algorithm_id = algorithm_id
        self.throttle = _THROTTLE_BY_ALGORITHM.get(algorithm_id, 0.0)
        self.control_topic = str(config["control_topic"])
        self.observation_topic = str(
            config.get("observation_topic", "/closed_loop/ego/observation")
        )
        self.route_config = _ROUTE_FOLLOWERS.get(algorithm_id)

    def health_check(self) -> dict[str, str]:
        return {"status": "ready", "algorithm_id": self.algorithm_id}

    def predict_control(self, observation: Any) -> dict[str, Any]:
        if self.route_config is not None:
            return _pure_pursuit_control(observation, self.route_config)
        return {
            "throttle": self.throttle,
            "steer": 0.0,
            "brake": 0.0,
            "hand_brake": False,
            "reverse": False,
        }

    def run(self) -> None:
        import rclpy
        from carla_msgs.msg import CarlaEgoVehicleControl
        from std_msgs.msg import String

        rclpy.init(args=None)
        node = rclpy.create_node(self.algorithm_id)
        publisher = node.create_publisher(CarlaEgoVehicleControl, self.control_topic, 10)

        def publish_control(
            control: dict[str, Any] | None = None, observation_id: str = ""
        ) -> None:
            values = control or self.predict_control(None)
            message = CarlaEgoVehicleControl()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = observation_id
            message.throttle = float(values["throttle"])
            message.steer = float(values["steer"])
            message.brake = float(values["brake"])
            message.hand_brake = bool(values.get("hand_brake", False))
            message.reverse = bool(values.get("reverse", False))
            message.gear = 1
            message.manual_gear_shift = False
            publisher.publish(message)

        subscription = None
        if self.route_config is not None:
            def on_observation(message: Any) -> None:
                observation = json.loads(message.data)
                publish_control(
                    self.predict_control(observation),
                    str(observation["observation_id"]),
                )

            subscription = node.create_subscription(
                String, self.observation_topic, on_observation, 10
            )

        try:
            while rclpy.ok():
                if self.route_config is None:
                    publish_control()
                rclpy.spin_once(node, timeout_sec=0.05)
                if self.route_config is None:
                    time.sleep(0.05)
        finally:
            del subscription
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def create_backend(config: dict[str, Any]) -> ReferenceCruiseBackend:
    return ReferenceCruiseBackend(config)


def _pure_pursuit_control(
    observation: dict[str, Any], config: dict[str, Any]
) -> dict[str, Any]:
    pose = observation["ego_state"]["pose"]
    speed = float(observation["ego_state"]["speed_mps"])
    route = observation["route"]
    waypoints = route["route_waypoints"]
    nearest = int(route.get("nearest_index", 0))
    target_index = min(len(waypoints) - 1, nearest + int(config["lookahead_points"]))
    target = waypoints[target_index]
    dx = float(target["x"]) - float(pose["x"])
    dy = float(target["y"]) - float(pose["y"])
    yaw = math.radians(float(pose["yaw"]))
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    lookahead = max(1.0, math.hypot(local_x, local_y))
    alpha = math.atan2(local_y, max(0.1, local_x))
    wheel_angle = math.atan2(2.0 * 2.8 * math.sin(alpha), lookahead)
    steer = max(-1.0, min(1.0, wheel_angle / math.radians(40.0)))
    speed_error = float(config["target_speed_mps"]) - speed
    throttle = max(0.0, min(0.65, 0.22 * speed_error))
    brake = max(0.0, min(0.7, -0.35 * speed_error))
    remaining = math.hypot(
        float(waypoints[-1]["x"]) - float(pose["x"]),
        float(waypoints[-1]["y"]) - float(pose["y"]),
    )
    if remaining < 3.0:
        throttle = min(throttle, max(0.0, remaining / 6.0))
        if remaining < 0.75:
            throttle, brake = 0.0, 1.0
    return {
        "throttle": throttle,
        "steer": steer,
        "brake": brake,
        "hand_brake": False,
        "reverse": False,
    }
