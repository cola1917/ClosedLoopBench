from __future__ import annotations

import json
import re
import traceback
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

from agents.transfuserpp_plugin import TransFuserPPPlugin


_SAFE_STOP = {
    "throttle": 0.0,
    "steer": 0.0,
    "brake": 1.0,
    "hand_brake": False,
    "reverse": False,
}


class TransFuserPPRos2Backend:
    """ROS2 transport owned by the TF++ sidecar; CARLA stays host-owned."""

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.config = deepcopy(dict(config))
        self.plugin = TransFuserPPPlugin()
        self.plugin.initialize(self.config)
        self.plugin.reset(
            {
                "scene_id": self.config.get("scene_id"),
                "case_id": self.config.get("case_id"),
                "seed": self.config.get("seed"),
            }
        )
        self._node = None
        self._failure_count = 0
        self._last_failure: dict[str, Any] | None = None
        self._failure_root = Path(
            str(self.config["intermediate_output_dir"])
        ) / "backend_failures"

    def health_check(self) -> dict[str, Any]:
        health = self.plugin.health_check()
        if health.get("status") != "ready" or not health.get("real_checkpoint_loaded"):
            return {"status": "unhealthy", "detail": health}
        return {
            "status": "ready",
            "algorithm_id": "transfuserpp_v5",
            "real_checkpoint_loaded": True,
            "identity": deepcopy(health.get("identity")),
            "failure_count": self._failure_count,
            "last_failure": deepcopy(self._last_failure),
        }

    def predict_control(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        return self.plugin.predict_control(observation)

    def run(self) -> None:
        try:
            import rclpy
            from carla_msgs.msg import CarlaEgoVehicleControl
            from std_msgs.msg import String
        except Exception as exc:
            raise RuntimeError(f"TransFuser++ ROS2 runtime is unavailable: {exc}") from exc

        owns_runtime = not rclpy.ok()
        if owns_runtime:
            rclpy.init(args=None)
        node = rclpy.create_node("closed_loop_bench_transfuserpp")
        self._node = node
        publisher = node.create_publisher(
            CarlaEgoVehicleControl,
            str(self.config.get("control_topic", "/carla/ego_vehicle/vehicle_control_cmd")),
            10,
        )

        def receive(message: Any) -> None:
            observation_id = ""
            failed = False
            observation: dict[str, Any] = {}
            try:
                observation = json.loads(message.data)
                observation_id = str(observation.get("observation_id") or "")
                control = self.predict_control(observation)
            except Exception as exc:
                failed = True
                self._record_failure(observation, observation_id, exc)
                control = dict(_SAFE_STOP)
            outbound = CarlaEgoVehicleControl()
            outbound.header.frame_id = (
                f"error:{observation_id}" if failed else observation_id
            )
            outbound.throttle = float(control["throttle"])
            outbound.steer = float(control["steer"])
            outbound.brake = float(control["brake"])
            outbound.hand_brake = bool(control["hand_brake"])
            outbound.reverse = bool(control["reverse"])
            publisher.publish(outbound)

        node.create_subscription(
            String,
            str(self.config.get("observation_topic", "/closed_loop/ego/observation")),
            receive,
            10,
        )
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            self._node = None
            self.plugin.close()
            if owns_runtime and rclpy.ok():
                rclpy.shutdown()

    def _record_failure(
        self,
        observation: Mapping[str, Any],
        observation_id: str,
        exc: Exception,
    ) -> None:
        self._failure_count += 1
        run_context = observation.get("run_context") or {}
        run_id = str(run_context.get("run_id") or "unavailable")
        if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", run_id) is None:
            run_id = "unavailable"
        row = {
            "schema_version": "transfuserpp_backend_failure.v1",
            "failure_index": self._failure_count,
            "observation_id": observation_id or "unavailable",
            "frame_id": observation.get("frame_id"),
            "timestamp": observation.get("timestamp"),
            "run_id": run_id,
            "scene_id": run_context.get("scene_id"),
            "case_id": run_context.get("case_id"),
            "seed": run_context.get("seed"),
            "experiment_identity": deepcopy(dict(run_context.get("identity") or {})),
            "exception_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
            "safe_stop_published_as_valid_control": False,
        }
        self._last_failure = row
        try:
            self._failure_root.mkdir(parents=True, exist_ok=True)
            failure_trace = self._failure_root / f"{run_id}.jsonl"
            with failure_trace.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        except OSError:
            pass
        if self._node is not None and hasattr(self._node, "get_logger"):
            self._node.get_logger().error(json.dumps(row, ensure_ascii=False))


def create_backend(config: Mapping[str, Any]) -> TransFuserPPRos2Backend:
    return TransFuserPPRos2Backend(config)
