from __future__ import annotations

from copy import deepcopy
from typing import Any

from agents.ego_observation import TickObservationAggregator
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
    max_skew_sec: float = 0.001,
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
        "max_skew_sec": float(max_skew_sec),
        "qos": int(qos),
        "topics": {
            "sensors": sensor_topics,
            "ego_state": "/closed_loop/ego/state",
            "route": "/closed_loop/route",
            "control_cmd": "/carla/{}/vehicle_control_cmd".format(role_name),
        },
        "message_contract": {
            "input_messages": "decoded_payload_envelope_with_frame_id_timestamp_and_calibration",
            "output_message": "vehicle_control_dict_for_runtime_binding",
            "same_tick_required": True,
            "fail_closed": True,
        },
        "runtime_boundary": {
            "requires_rclpy_for_tests": False,
            "requires_tcp_repo_for_tests": False,
            "loads_tcp_model": False,
            "owns_ros2_topic_wiring": True,
            "owns_model_inference": False,
            "uses_tcp_runtime_adapter_backend_interface": True,
            "real_ros2_message_binding_implemented": False,
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
        self.aggregator = TickObservationAggregator(
            required_cameras=self.adapter.required_cameras,
            timeout_sec=self.timeout_sec,
            max_skew_sec=float(self.plan.get("max_skew_sec", 0.001)),
            require_calibration=True,
        )
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

    def receive_sensor(
        self,
        camera: str,
        message: Any,
        t_sec: float | None = None,
        frame_id: int | str | None = None,
        calibration: dict[str, Any] | None = None,
    ) -> None:
        self.aggregator.receive_camera(
            camera,
            _decode_message(message),
            frame_id=_frame_id(message, frame_id, t_sec),
            t_sec=_timestamp(message, t_sec),
            calibration=calibration or _metadata(message, "calibration"),
        )

    def receive_ego_state(
        self, message: Any, t_sec: float | None = None, frame_id: int | str | None = None
    ) -> None:
        self.aggregator.receive_ego_state(
            _decode_message(message),
            frame_id=_frame_id(message, frame_id, t_sec),
            t_sec=_timestamp(message, t_sec),
        )

    def receive_route(
        self, message: Any, t_sec: float | None = None, frame_id: int | str | None = None
    ) -> None:
        self.aggregator.receive_route(
            _decode_message(message),
            frame_id=_frame_id(message, frame_id, t_sec),
            t_sec=_timestamp(message, t_sec),
        )

    def tick(self, now_sec: float) -> dict[str, Any]:
        readiness = self.aggregator.build(now_sec=now_sec)
        if readiness["status"] != "ready":
            result = _fallback(readiness["reason"], detail=readiness.get("detail", {}))
            self.publisher.publish(result["control"])
            return result

        observation = readiness["observation"]
        result = self.adapter.tick(
            sensors=observation["sensors"],
            ego_state=observation["ego_state"],
            route=observation["route"],
            calibration=observation["calibration"],
            observation_metadata={
                "frame_id": observation["frame_id"],
                "t_sec": observation["t_sec"],
                "source": observation["source"],
            },
        )
        self.publisher.publish(result["control"])
        return result


def _decode_message(message: Any) -> Any:
    if isinstance(message, dict) and "data" in message:
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


def _frame_id(message: Any, frame_id: int | str | None, t_sec: float | None) -> int | str:
    if frame_id is not None:
        return frame_id
    if isinstance(message, dict) and message.get("frame_id") is not None:
        return message["frame_id"]
    if t_sec is not None:
        return f"timestamp:{float(t_sec):.9f}"
    raise ValueError("message requires frame_id for same-tick aggregation")


def _metadata(message: Any, key: str) -> Any:
    return message.get(key) if isinstance(message, dict) else None


def _fallback(reason: str, **extra: Any) -> dict[str, Any]:
    result = {
        "status": "fallback",
        "reason": reason,
        "control": deepcopy(_SAFE_STOP_CONTROL),
    }
    result.update(extra)
    return result
