from __future__ import annotations

import json
import math
import statistics
import time
from copy import deepcopy
from pathlib import PurePosixPath
from typing import Mapping
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
        algorithm_sensor_binding: Mapping[str, Any] | None = None,
        run_context: Mapping[str, Any] | None = None,
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
        self.algorithm_sensor_binding = deepcopy(dict(algorithm_sensor_binding or {}))
        self.run_context = deepcopy(dict(run_context or {}))
        self._sensor_input_required = bool(self.algorithm_sensor_binding)
        run_id = self.run_context.get("run_id")
        if self._sensor_input_required and (
            not isinstance(run_id, str)
            or not run_id
            or any(
                character
                not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-"
                for character in run_id
            )
        ):
            raise ValueError(
                "sensor-bound observation control requires a filesystem-safe run_context.run_id"
            )
        self._latest_sensor_packet: dict[str, Any] | None = None
        self._last_sensor_frame_received: int | None = None
        self._sensor_frame_count = 0
        self._sensor_frame_rejection_count = 0
        self._initialization_safe_stop_count = 0
        self._latest_control: dict[str, Any] | None = None
        self._latest_control_frame: str | None = None
        self._latest_received_at: float | None = None
        self._route_progress_index = 0
        self._route_lookahead_m = float(
            self.algorithm_sensor_binding.get("route_lookahead_m", 7.5)
        )
        if self._route_lookahead_m <= 0.0:
            raise ValueError("route_lookahead_m must be positive")
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
        if self._sensor_input_required and self._latest_sensor_packet is None:
            self._fallback_count += 1
            self._initialization_safe_stop_count += 1
            return _carla_control(self.carla_module, _SAFE_STOP)
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
                matched_control = self._latest_control
                self._latest_control = None
                self._latest_control_frame = None
                if self._sensor_input_required:
                    self._latest_sensor_packet = None
                return _carla_control(self.carla_module, matched_control)
            if self._latest_control is not None and self._latest_control_frame:
                self._mismatched_control_count += 1
                self._latest_control = None
                self._latest_control_frame = None

        self._fallback_count += 1
        if self._sensor_input_required:
            self._latest_sensor_packet = None
        return _carla_control(self.carla_module, _SAFE_STOP)

    def receive_multimodal_evidence(
        self,
        evidence: Mapping[str, Any],
        *,
        context: Mapping[str, Any] | None = None,
    ) -> None:
        """Bind one materialized NuRec frame to the next synchronous control tick."""

        try:
            if evidence.get("status") != "passed":
                raise ValueError("NuRec evidence did not pass")
            frame_id = evidence.get("frame_id")
            if isinstance(frame_id, bool) or not isinstance(frame_id, int):
                raise ValueError("NuRec evidence frame_id is invalid")
            if self._last_sensor_frame_received is not None and frame_id <= self._last_sensor_frame_received:
                raise ValueError("NuRec algorithm sensor frame is stale")
            records = evidence.get("records")
            if not isinstance(records, list):
                raise ValueError("NuRec evidence has no records")
            camera_id = str(
                self.algorithm_sensor_binding.get("camera_sensor_id", "camera_front")
            )
            lidar_id = str(
                self.algorithm_sensor_binding.get("lidar_sensor_id", "lidar_top")
            )
            camera = self._materialized_record(records, camera_id, "rgb")
            lidar = self._materialized_record(records, lidar_id, "lidar")
            camera = self._remap_materialized_path(camera)
            lidar = self._remap_materialized_path(lidar)
            if lidar["encoding"] != "float32_xyzi_little_endian":
                raise ValueError("materialized lidar encoding is incompatible")
            if lidar["coordinate_frame"] not in {"sensor_local", "carla_ego"}:
                raise ValueError("materialized LiDAR coordinate frame is unverified")
            sensor_to_ego = self.algorithm_sensor_binding.get("lidar_sensor_to_ego")
            if lidar["coordinate_frame"] == "sensor_local":
                if not isinstance(sensor_to_ego, list) or len(sensor_to_ego) != 16:
                    raise ValueError("sensor-local LiDAR requires lidar_sensor_to_ego")
                lidar["sensor_to_ego"] = [float(value) for value in sensor_to_ego]
                expected_axis = str(
                    self.algorithm_sensor_binding.get("lidar_axis_convention", "unverified")
                )
                if (
                    expected_axis != "carla_sensor"
                    or lidar.get("axis_convention") != expected_axis
                ):
                    raise ValueError("LiDAR axis convention is not verified as carla_sensor")
                if self.algorithm_sensor_binding.get(
                    "lidar_sensor_to_ego_coordinate_frame"
                ) != "carla_x_forward_y_right_z_up":
                    raise ValueError("LiDAR sensor_to_ego coordinate frame is unverified")
            packet_context = deepcopy(dict(context or {}))
            self._latest_sensor_packet = {
                "frame_id": frame_id,
                "timestamp": float(evidence.get("simulation_time_sec", 0.0)),
                "rgb": {"camera_front": camera},
                "lidar": lidar,
                "context": packet_context,
                "dynamic_object_sha256": evidence.get("dynamic_object_sha256"),
            }
            self._last_sensor_frame_received = frame_id
            self._sensor_frame_count += 1
        except (TypeError, ValueError, KeyError):
            self._sensor_frame_rejection_count += 1
            self._latest_sensor_packet = None
            raise

    @staticmethod
    def _materialized_record(
        records: list[Any], sensor_id: str, modality: str
    ) -> dict[str, Any]:
        matches = [
            row
            for row in records
            if isinstance(row, Mapping)
            and row.get("sensor_id") == sensor_id
            and row.get("modality") == modality
            and row.get("status") == "passed"
        ]
        if len(matches) != 1:
            raise ValueError(
                f"expected one passing materialized {modality} record for {sensor_id}"
            )
        payload = (matches[0].get("response_metadata") or {}).get(
            "materialized_payload"
        )
        if not isinstance(payload, Mapping):
            raise ValueError(f"{sensor_id} has no materialized payload")
        required = ("path", "sha256", "encoding", "coordinate_frame")
        if any(not payload.get(name) for name in required):
            raise ValueError(f"{sensor_id} materialized payload is incomplete")
        return deepcopy(dict(payload))

    def _remap_materialized_path(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        result = deepcopy(dict(payload))
        container_root = self.algorithm_sensor_binding.get("container_payload_root")
        if not container_root:
            return result
        relative = str(result.get("relative_path") or "")
        relative_path = PurePosixPath(relative.replace("\\", "/"))
        if (
            not relative
            or relative_path.is_absolute()
            or ".." in relative_path.parts
        ):
            raise ValueError(
                "materialized payload lacks a safe path relative to the shared attempt root"
            )
        result["host_path"] = str(result["path"])
        root_path = PurePosixPath(str(container_root).replace("\\", "/"))
        if not root_path.is_absolute():
            raise ValueError("container_payload_root must be an absolute POSIX path")
        result["path"] = str(root_path.joinpath(*relative_path.parts))
        result["path_mapping"] = "attempt_relative_to_container_payload_root"
        return result

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
            "algorithm_sensor_binding": {
                "required": self._sensor_input_required,
                "received_frame_count": self._sensor_frame_count,
                "rejected_frame_count": self._sensor_frame_rejection_count,
                "initialization_safe_stop_count": self._initialization_safe_stop_count,
                "last_frame_id": self._last_sensor_frame_received,
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
        nearest = _monotonic_nearest_route_index(
            self.route,
            pose,
            start_index=self._route_progress_index,
        )
        self._route_progress_index = max(self._route_progress_index, nearest)
        target_index, target_distance_m = _route_lookahead_index(
            self.route,
            start_index=self._route_progress_index,
            lookahead_m=self._route_lookahead_m,
        )
        result = {
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
                "progress_index": self._route_progress_index,
                "target_index": target_index,
                "target_distance_along_route_m": target_distance_m,
                "lookahead_m": self._route_lookahead_m,
                "target_point": self.route[target_index],
                "route_command": _normalize_route_command(
                    self.route[target_index].get(
                        "route_command",
                        self.route[target_index].get("command", "LANE_FOLLOW"),
                    )
                ),
            },
        }
        if self._sensor_input_required:
            packet = self._latest_sensor_packet
            if packet is None:
                raise RuntimeError("algorithm sensor packet is unavailable")
            target_ego = _world_to_ego(self.route[target_index], pose)
            next_index, _ = _route_lookahead_index(
                self.route,
                start_index=target_index,
                lookahead_m=self._route_lookahead_m,
            )
            result.update(
                {
                    "schema_version": "transfuserpp_observation.v1",
                    "frame_id": int(packet["frame_id"]),
                    "timestamp": float(packet["timestamp"]),
                    "rgb": deepcopy(packet["rgb"]),
                    "lidar": deepcopy(packet["lidar"]),
                    "sensor_validity": {
                        "camera_front": True,
                        "lidar": True,
                    },
                    "calibration": deepcopy(self.algorithm_sensor_binding),
                    "synchronization": {
                        "frame_id": int(packet["frame_id"]),
                        "clock": "carla_snapshot",
                        "error_ms": 0.0,
                        "dynamic_object_sha256": packet.get(
                            "dynamic_object_sha256"
                        ),
                        "sensor_age_ticks": 0,
                    },
                    "actor_proxies": _actor_proxies_from_context(
                        packet.get("context") or {}
                    ),
                    "run_context": deepcopy(self.run_context),
                }
            )
            result["route"].update(
                {
                    "target_point_ego_m": target_ego,
                    "target_point_next_ego_m": _world_to_ego(
                        self.route[next_index], pose
                    ),
                    "target_point_coordinate_frame": "carla_ego",
                    "gps_source": "bypassed",
                }
            )
            result["ego_state"].update(
                {
                    "compass_rad": math.radians(float(pose["yaw"])),
                    "speed_source": "carla_actor_velocity",
                    "compass_source": "carla_actor_transform_yaw",
                }
            )
        return result


def create_ros2_observation_control_driver(
    *,
    carla_module: Any,
    vehicle: Any,
    route: list[dict[str, Any]],
    control_topic: str,
    observation_topic: str,
    timeout_sec: float = 0.5,
    algorithm_sensor_binding: Mapping[str, Any] | None = None,
    run_context: Mapping[str, Any] | None = None,
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
        algorithm_sensor_binding=algorithm_sensor_binding,
        run_context=run_context,
    )


def _vehicle_pose(vehicle: Any) -> dict[str, float]:
    """Convert the native CARLA pose to the plan's canonical right-handed frame."""

    transform = vehicle.get_transform()
    return {
        "x": float(transform.location.x),
        "y": -float(transform.location.y),
        "z": float(transform.location.z),
        "yaw": -float(transform.rotation.yaw),
    }


def _distance(left: dict[str, Any], right: dict[str, Any]) -> float:
    return math.hypot(float(left["x"]) - float(right["x"]), float(left["y"]) - float(right["y"]))


def _monotonic_nearest_route_index(
    route: list[dict[str, Any]],
    pose: dict[str, Any],
    *,
    start_index: int,
    search_horizon_m: float = 30.0,
) -> int:
    end_index = max(0, min(len(route) - 1, int(start_index)))
    distance_along = 0.0
    while end_index < len(route) - 1 and distance_along < search_horizon_m:
        distance_along += _distance(route[end_index], route[end_index + 1])
        end_index += 1
    return min(
        range(start_index, end_index + 1),
        key=lambda index: _distance(pose, route[index]),
    )


def _route_lookahead_index(
    route: list[dict[str, Any]],
    *,
    start_index: int,
    lookahead_m: float,
) -> tuple[int, float]:
    index = max(0, min(len(route) - 1, int(start_index)))
    distance_along = 0.0
    while index < len(route) - 1 and distance_along < lookahead_m:
        distance_along += _distance(route[index], route[index + 1])
        index += 1
    return index, distance_along


def _normalize_route_command(value: Any) -> str:
    raw = str(value or "LANE_FOLLOW").upper().split(".")[-1]
    aliases = {
        "LANEFOLLOW": "LANE_FOLLOW",
        "CHANGELANELEFT": "CHANGE_LANE_LEFT",
        "CHANGELANERIGHT": "CHANGE_LANE_RIGHT",
    }
    return aliases.get(raw, raw)


def _world_to_ego(point: Mapping[str, Any], ego_pose: Mapping[str, Any]) -> list[float]:
    """Map canonical scene y-left coordinates to CARLA ego x-forward/y-right."""

    dx = float(point["x"]) - float(ego_pose["x"])
    dy = float(point["y"]) - float(ego_pose["y"])
    yaw = math.radians(float(ego_pose["yaw"]))
    return [
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        math.sin(yaw) * dx - math.cos(yaw) * dy,
    ]


def _actor_proxies_from_context(context: Mapping[str, Any]) -> list[dict[str, Any]]:
    ego_pair = context.get("ego_pose_pair") or {}
    ego = ego_pair.get("end") or {}
    if not all(name in ego for name in ("x", "y", "yaw")):
        return []
    result = []
    for actor_id, sample in sorted((context.get("actor_samples") or {}).items()):
        pair = sample.get("pose_pair") or {}
        pose = pair.get("end") or {}
        if not all(name in pose for name in ("x", "y")):
            continue
        result.append(
            {
                "actor_id": str(actor_id),
                "track_id": sample.get("nurec_track_id"),
                "actor_type": sample.get("actor_type"),
                "center_ego_m": _world_to_ego(pose, ego),
                "yaw_ego_deg": -(
                    (
                        float(pose.get("yaw", 0.0))
                        - float(ego["yaw"])
                        + 180.0
                    )
                    % 360.0
                    - 180.0
                ),
                "extent_m": deepcopy(sample.get("extent_m")),
                "source": "carla_actor_proxy_same_nurec_frame",
            }
        )
    return result
