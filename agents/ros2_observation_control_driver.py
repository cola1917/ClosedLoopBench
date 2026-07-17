from __future__ import annotations

import json
import math
import statistics
import time
from typing import Any, Callable

from agents.ros2_control_driver import _carla_control, _control_dict, _valid_control


_SAFE_STOP = {
    "throttle": 0.0,
    "steer": 0.0,
    "brake": 1.0,
    "hand_brake": False,
    "reverse": False,
}


class Ros2ObservationControlDriver:
    """Publish current ego/route state and accept only frame-matched ROS2 control."""

    def __init__(
        self,
        *,
        node: Any,
        carla_module: Any,
        vehicle: Any,
        route: list[dict[str, Any]],
        control_message_type: Any,
        observation_message_type: Any,
        control_topic: str,
        observation_topic: str,
        timeout_sec: float = 0.5,
        spin_once: Callable[[Any, float], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
        close_runtime: Callable[[], None] | None = None,
    ) -> None:
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        if len(route) < 2:
            raise ValueError("observation control requires at least two route points")
        self.node = node
        self.carla_module = carla_module
        self.vehicle = vehicle
        self.route = [dict(point) for point in route]
        self.timeout_sec = float(timeout_sec)
        self._observation_message_type = observation_message_type
        self._spin_once = spin_once or (lambda _node, _timeout: None)
        self._clock = clock
        self._close_runtime = close_runtime
        self._latest_control: dict[str, Any] | None = None
        self._latest_control_frame: str | None = None
        self._latest_received_at: float | None = None
        self._sequence = 0
        self._observation_count = 0
        self._control_count = 0
        self._fallback_count = 0
        self._mismatched_control_count = 0
        self._latency_ms: list[float] = []
        self.publisher = self.node.create_publisher(
            observation_message_type, str(observation_topic), 10
        )
        self.subscription = self.node.create_subscription(
            control_message_type, str(control_topic), self.receive_control, 10
        )

    def receive_control(self, message: Any) -> None:
        control = _control_dict(message)
        if not _valid_control(control):
            return
        header = getattr(message, "header", None)
        frame_id = getattr(header, "frame_id", None)
        if isinstance(message, dict):
            frame_id = message.get("frame_id", frame_id)
        self._latest_control = control
        self._latest_control_frame = str(frame_id or "")
        self._latest_received_at = float(self._clock())

    def done(self) -> bool:
        pose = _vehicle_pose(self.vehicle)
        return _distance(pose, self.route[-1]) <= 0.75

    def run_step(self) -> Any:
        self._sequence += 1
        observation_id = f"obs-{self._sequence:06d}"
        observation = self._build_observation(observation_id)
        message = self._observation_message_type()
        message.data = json.dumps(observation, separators=(",", ":"), allow_nan=False)
        published_at = float(self._clock())
        self.publisher.publish(message)
        self._observation_count += 1

        deadline = published_at + self.timeout_sec
        while float(self._clock()) < deadline:
            remaining = max(0.0, deadline - float(self._clock()))
            self._spin_once(self.node, min(0.01, remaining))
            if self._latest_control_frame == observation_id and self._latest_control is not None:
                received_at = self._latest_received_at or float(self._clock())
                self._latency_ms.append(max(0.0, (received_at - published_at) * 1000.0))
                self._control_count += 1
                return _carla_control(self.carla_module, self._latest_control)
            if self._latest_control is not None and self._latest_control_frame:
                self._mismatched_control_count += 1
                self._latest_control = None
                self._latest_control_frame = None

        self._fallback_count += 1
        return _carla_control(self.carla_module, _SAFE_STOP)

    def diagnostics(self) -> dict[str, Any]:
        latency = self._latency_ms
        return {
            "driver": "ros2_observation_control",
            "observation_count": self._observation_count,
            "control_count": self._control_count,
            "fallback_count": self._fallback_count,
            "mismatched_control_count": self._mismatched_control_count,
            "matched_frame_ratio": (
                self._control_count / self._observation_count
                if self._observation_count
                else 0.0
            ),
            "latency_ms": {
                "count": len(latency),
                "mean": statistics.fmean(latency) if latency else None,
                "max": max(latency) if latency else None,
            },
        }

    def close(self) -> None:
        if hasattr(self.node, "destroy_node"):
            self.node.destroy_node()
        if self._close_runtime is not None:
            self._close_runtime()

    def _build_observation(self, observation_id: str) -> dict[str, Any]:
        pose = _vehicle_pose(self.vehicle)
        velocity = self.vehicle.get_velocity()
        acceleration = self.vehicle.get_acceleration() if hasattr(self.vehicle, "get_acceleration") else None
        nearest = min(range(len(self.route)), key=lambda index: _distance(pose, self.route[index]))
        target_index = min(len(self.route) - 1, nearest + 4)
        return {
            "schema_version": "carla_route_observation.v1",
            "observation_id": observation_id,
            "source": "carla_current_tick",
            "ego_state": {
                "pose": pose,
                "speed_mps": math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2),
                "velocity": {"x": velocity.x, "y": velocity.y, "z": velocity.z},
                "acceleration": (
                    {"x": acceleration.x, "y": acceleration.y, "z": acceleration.z}
                    if acceleration is not None
                    else {"x": 0.0, "y": 0.0, "z": 0.0}
                ),
            },
            "route": {
                "route_waypoints": self.route,
                "nearest_index": nearest,
                "target_index": target_index,
                "target_point": self.route[target_index],
                "route_command": "LANE_FOLLOW",
            },
        }


def create_ros2_observation_control_driver(
    *,
    carla_module: Any,
    vehicle: Any,
    route: list[dict[str, Any]],
    control_topic: str,
    observation_topic: str,
    timeout_sec: float = 0.5,
) -> Ros2ObservationControlDriver:
    try:
        import rclpy
        from carla_msgs.msg import CarlaEgoVehicleControl
        from std_msgs.msg import String
    except Exception as exc:
        raise RuntimeError(f"ROS2 observation control runtime is unavailable: {exc}") from exc

    owns_runtime = not rclpy.ok()
    if owns_runtime:
        rclpy.init(args=None)
    node = rclpy.create_node("closed_loop_bench_observation_control")

    def spin_once(active_node: Any, timeout: float) -> None:
        rclpy.spin_once(active_node, timeout_sec=timeout)

    def close_runtime() -> None:
        if owns_runtime and rclpy.ok():
            rclpy.shutdown()

    return Ros2ObservationControlDriver(
        node=node,
        carla_module=carla_module,
        vehicle=vehicle,
        route=route,
        control_message_type=CarlaEgoVehicleControl,
        observation_message_type=String,
        control_topic=control_topic,
        observation_topic=observation_topic,
        timeout_sec=timeout_sec,
        spin_once=spin_once,
        close_runtime=close_runtime,
    )


def _vehicle_pose(vehicle: Any) -> dict[str, float]:
    transform = vehicle.get_transform()
    return {
        "x": float(transform.location.x),
        "y": float(transform.location.y),
        "z": float(transform.location.z),
        "yaw": float(transform.rotation.yaw),
    }


def _distance(left: dict[str, Any], right: dict[str, Any]) -> float:
    return math.hypot(float(left["x"]) - float(right["x"]), float(left["y"]) - float(right["y"]))
