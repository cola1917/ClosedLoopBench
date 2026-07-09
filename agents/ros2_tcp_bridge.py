from __future__ import annotations

from copy import deepcopy
from typing import Any

from agents.tcp_runtime_adapter import TcpRuntimeAdapter

_SAFE_STOP_CONTROL = {
    "throttle": 0.0,
    "steer": 0.0,
    "brake": 1.0,
    "hand_brake": False,
    "reverse": False,
}


def build_ros2_tcp_bridge_plan(
    *,
    scenario_id: str,
    role_name: str = "ego_vehicle",
    camera_profile: str = "tcp_front",
    timeout_sec: float = 0.5,
    qos: int = 10,
) -> dict[str, Any]:
    adapter = TcpRuntimeAdapter(camera_profile=camera_profile, role_name=role_name)
    sensor_topics = {
        camera: "/carla/{}/{}/image".format(role_name, camera)
        for camera in adapter.required_cameras
    }
    return {
        "schema_version": "ros2_tcp_bridge_plan.mvp.v0",
        "scenario_id": str(scenario_id),
        "plugin": "external_ros2_tcp",
        "algorithm": "TCP",
        "role_name": str(role_name),
        "camera_profile": str(camera_profile),
        "timeout_sec": float(timeout_sec),
        "qos": int(qos),
        "topics": {
            "sensors": sensor_topics,
            "ego_state": "/closed_loop/ego/state",
            "route": "/closed_loop/route",
            "control_cmd": "/carla/{}/vehicle_control_cmd".format(role_name),
        },
        "message_contract": {
            "input_messages": "adapter_owned_dict_or_ros2_decoded_payload",
            "output_message": "vehicle_control_dict_for_runtime_binding",
        },
        "runtime_boundary": {
            "requires_rclpy_for_tests": False,
            "requires_tcp_repo_for_tests": False,
            "loads_tcp_model": False,
            "owns_ros2_topic_wiring": True,
            "owns_model_inference": False,
            "uses_tcp_runtime_adapter_backend_interface": True,
        },
    }


class Ros2TcpBridge:
    """ROS2/TCP bridge shell with injectable node and backend for model-free tests."""

    def __init__(
        self,
        *,
        node: Any,
        plan: dict[str, Any],
        backend: Any = None,
        adapter: TcpRuntimeAdapter | None = None,
    ) -> None:
        self.node = node
        self.plan = deepcopy(plan)
        self.timeout_sec = float(self.plan.get("timeout_sec", 0.5))
        self.qos = int(self.plan.get("qos", 10))
        self.topics = deepcopy(self.plan["topics"])
        self.adapter = adapter or TcpRuntimeAdapter(
            backend=backend,
            camera_profile=self.plan.get("camera_profile", "tcp_front"),
            role_name=self.plan.get("role_name", "ego_vehicle"),
        )
        self._sensors: dict[str, Any] = {}
        self._sensor_times: dict[str, float] = {}
        self._ego_state: dict[str, Any] | None = None
        self._ego_state_time: float | None = None
        self._route: dict[str, Any] | None = None
        self._route_time: float | None = None
        self.publisher = self.node.create_publisher(
            dict,
            self.topics["control_cmd"],
            self.qos,
        )
        self._subscriptions = []
        self._register_subscriptions()

    def _register_subscriptions(self) -> None:
        for camera, topic in self.topics["sensors"].items():
            self._subscriptions.append(
                self.node.create_subscription(
                    dict,
                    topic,
                    lambda message, camera=camera: self.receive_sensor(camera, message),
                    self.qos,
                )
            )
        self._subscriptions.append(
            self.node.create_subscription(
                dict,
                self.topics["ego_state"],
                self.receive_ego_state,
                self.qos,
            )
        )
        self._subscriptions.append(
            self.node.create_subscription(
                dict,
                self.topics["route"],
                self.receive_route,
                self.qos,
            )
        )

    def receive_sensor(self, camera: str, message: Any, t_sec: float | None = None) -> None:
        self._sensors[str(camera)] = _decode_message(message)
        self._sensor_times[str(camera)] = _timestamp(message, t_sec)

    def receive_ego_state(self, message: Any, t_sec: float | None = None) -> None:
        self._ego_state = _decode_message(message)
        self._ego_state_time = _timestamp(message, t_sec)

    def receive_route(self, message: Any, t_sec: float | None = None) -> None:
        self._route = _decode_message(message)
        self._route_time = _timestamp(message, t_sec)

    def tick(self, now_sec: float) -> dict[str, Any]:
        readiness = self._readiness(now_sec)
        if readiness["status"] != "ready":
            result = _fallback(readiness["reason"], **readiness.get("detail", {}))
            self.publisher.publish(result["control"])
            return result

        result = self.adapter.tick(
            sensors=deepcopy(self._sensors),
            ego_state=deepcopy(self._ego_state or {}),
            route=deepcopy(self._route or {}),
        )
        self.publisher.publish(result["control"])
        return result

    def _readiness(self, now_sec: float) -> dict[str, Any]:
        missing = [camera for camera in self.adapter.required_cameras if camera not in self._sensors]
        if missing:
            return {"status": "blocked", "reason": "missing_sensor", "detail": {"missing": missing}}
        if self._ego_state is None:
            return {"status": "blocked", "reason": "missing_ego_state"}
        if self._route is None:
            return {"status": "blocked", "reason": "missing_route"}

        observed_times = [
            self._sensor_times[camera]
            for camera in self.adapter.required_cameras
        ]
        observed_times.extend([self._ego_state_time or 0.0, self._route_time or 0.0])
        oldest = min(observed_times)
        age = float(now_sec) - float(oldest)
        if age > self.timeout_sec:
            return {
                "status": "blocked",
                "reason": "stale_observation",
                "detail": {"age_sec": age, "timeout_sec": self.timeout_sec},
            }
        return {"status": "ready"}


def _decode_message(message: Any) -> Any:
    if isinstance(message, dict) and "data" in message and set(message.keys()).issubset({"data", "stamp", "t_sec"}):
        return message["data"]
    return message


def _timestamp(message: Any, t_sec: float | None) -> float:
    if t_sec is not None:
        return float(t_sec)
    if isinstance(message, dict):
        if "t_sec" in message:
            return float(message["t_sec"])
        if "stamp" in message:
            return float(message["stamp"])
    return 0.0


def _fallback(reason: str, **extra: Any) -> dict[str, Any]:
    result = {
        "status": "fallback",
        "reason": reason,
        "control": deepcopy(_SAFE_STOP_CONTROL),
    }
    result.update(extra)
    return result
