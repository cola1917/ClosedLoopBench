from __future__ import annotations

import time
from copy import deepcopy
from typing import Any, Callable


_SAFE_STOP = {
    "throttle": 0.0,
    "steer": 0.0,
    "brake": 1.0,
    "hand_brake": False,
    "reverse": False,
}


class Ros2ControlDriver:
    """Consume ROS2 vehicle-control commands as a CARLA ego driver."""

    def __init__(
        self,
        *,
        node: Any,
        carla_module: Any,
        message_type: Any,
        control_topic: str,
        timeout_sec: float = 0.5,
        spin_once: Callable[[Any], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
        close_runtime: Callable[[], None] | None = None,
    ) -> None:
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        self.node = node
        self.carla_module = carla_module
        self.timeout_sec = float(timeout_sec)
        self._spin_once = spin_once or (lambda _node: None)
        self._clock = clock
        self._close_runtime = close_runtime
        self._latest_control: dict[str, Any] | None = None
        self._latest_received_at: float | None = None
        self._fallback_count = 0
        self._control_count = 0
        self.subscription = self.node.create_subscription(
            message_type,
            str(control_topic),
            self.receive_control,
            10,
        )

    def receive_control(self, message: Any) -> None:
        control = _control_dict(message)
        if not _valid_control(control):
            return
        self._latest_control = control
        self._latest_received_at = float(self._clock())

    def done(self) -> bool:
        return False

    def run_step(self) -> Any:
        self._spin_once(self.node)
        now = float(self._clock())
        if self._latest_control is None or self._latest_received_at is None:
            self._fallback_count += 1
            return _carla_control(self.carla_module, _SAFE_STOP)
        if now - self._latest_received_at > self.timeout_sec:
            self._fallback_count += 1
            return _carla_control(self.carla_module, _SAFE_STOP)
        self._control_count += 1
        return _carla_control(self.carla_module, self._latest_control)

    def diagnostics(self) -> dict[str, Any]:
        return {
            "driver": "ros2_control",
            "control_count": self._control_count,
            "fallback_count": self._fallback_count,
            "has_received_control": self._latest_control is not None,
        }

    def close(self) -> None:
        if hasattr(self.node, "destroy_node"):
            self.node.destroy_node()
        if self._close_runtime is not None:
            self._close_runtime()


def create_ros2_control_driver(
    *,
    carla_module: Any,
    control_topic: str,
    timeout_sec: float = 0.5,
) -> Ros2ControlDriver:
    try:
        import rclpy
        from carla_msgs.msg import CarlaEgoVehicleControl
    except Exception as exc:
        raise RuntimeError(f"ROS2 control runtime is unavailable: {exc}") from exc

    owns_runtime = not rclpy.ok()
    if owns_runtime:
        rclpy.init(args=None)
    node = rclpy.create_node("closed_loop_bench_ego_control")

    def spin_once(active_node: Any) -> None:
        rclpy.spin_once(active_node, timeout_sec=timeout_sec)

    def close_runtime() -> None:
        if owns_runtime and rclpy.ok():
            rclpy.shutdown()

    return Ros2ControlDriver(
        node=node,
        carla_module=carla_module,
        message_type=CarlaEgoVehicleControl,
        control_topic=control_topic,
        timeout_sec=timeout_sec,
        spin_once=spin_once,
        close_runtime=close_runtime,
    )


def _control_dict(message: Any) -> dict[str, Any]:
    if isinstance(message, dict):
        source = deepcopy(message.get("control", message))
        return {
            "throttle": source.get("throttle"),
            "steer": source.get("steer"),
            "brake": source.get("brake"),
            "hand_brake": source.get("hand_brake", False),
            "reverse": source.get("reverse", False),
        }
    return {
        "throttle": getattr(message, "throttle", None),
        "steer": getattr(message, "steer", None),
        "brake": getattr(message, "brake", None),
        "hand_brake": getattr(message, "hand_brake", False),
        "reverse": getattr(message, "reverse", False),
    }


def _valid_control(control: dict[str, Any]) -> bool:
    return (
        _in_range(control.get("throttle"), 0.0, 1.0)
        and _in_range(control.get("steer"), -1.0, 1.0)
        and _in_range(control.get("brake"), 0.0, 1.0)
        and isinstance(control.get("hand_brake"), bool)
        and isinstance(control.get("reverse"), bool)
    )


def _in_range(value: Any, lower: float, upper: float) -> bool:
    return isinstance(value, (int, float)) and lower <= float(value) <= upper


def _carla_control(carla_module: Any, values: dict[str, Any]) -> Any:
    normalized = {
        "throttle": float(values["throttle"]),
        "steer": float(values["steer"]),
        "brake": float(values["brake"]),
        "hand_brake": bool(values["hand_brake"]),
        "reverse": bool(values["reverse"]),
    }
    control_type = getattr(carla_module, "VehicleControl", None)
    if control_type is None:
        return type("VehicleControl", (), normalized)()
    try:
        return control_type(**normalized)
    except TypeError:
        control = control_type()
        for key, value in normalized.items():
            setattr(control, key, value)
        return control
