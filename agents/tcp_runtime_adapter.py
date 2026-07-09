from __future__ import annotations

from copy import deepcopy
from typing import Any

from agents.tcp_adapter_contract import build_tcp_adapter_config, build_tcp_io_contract

_SAFE_STOP_CONTROL = {
    "throttle": 0.0,
    "steer": 0.0,
    "brake": 1.0,
    "hand_brake": False,
    "reverse": False,
}

_REQUIRED_CONTROL_FIELDS = ("throttle", "steer", "brake", "hand_brake", "reverse")


def build_tcp_runtime_plan(
    *,
    scenario_id: str,
    role_name: str = "ego_vehicle",
    camera_profile: str = "tcp_front",
    runtime_path: str | None = None,
    checkpoint_path: str | None = None,
) -> dict[str, Any]:
    adapter_config = build_tcp_adapter_config(
        camera_profile=camera_profile,
        role_name=role_name,
        runtime_path=runtime_path,
        checkpoint_path=checkpoint_path,
    )
    return {
        "schema_version": "tcp_runtime_plan.mvp.v0",
        "scenario_id": str(scenario_id),
        "plugin": "external_ros2_tcp",
        "algorithm": "TCP",
        "role_name": str(role_name),
        "io": build_tcp_io_contract(camera_profile=camera_profile, role_name=role_name),
        "runtime": adapter_config["runtime"],
        "safety": adapter_config["safety"],
        "runtime_boundary": {
            "vendors_model_code": False,
            "requires_tcp_repo_for_execution": True,
            "requires_tcp_repo_for_tests": False,
            "owns_model_inference": False,
            "owns_observation_packaging": True,
            "owns_control_validation": True,
        },
    }


class TcpRuntimeAdapter:
    """Model-free TCP adapter shell for unit tests and later remote runtime binding."""

    def __init__(self, backend=None, camera_profile: str = "tcp_front", role_name: str = "ego_vehicle") -> None:
        self.backend = backend
        self.io_contract = build_tcp_io_contract(camera_profile=camera_profile, role_name=role_name)
        self.required_cameras = tuple(
            self.io_contract["inputs"]["sensor_profile"].get("required_cameras", [])
        )

    def tick(
        self,
        *,
        sensors: dict[str, Any],
        ego_state: dict[str, Any],
        route: dict[str, Any],
    ) -> dict[str, Any]:
        missing = [camera for camera in self.required_cameras if camera not in sensors]
        if missing:
            return _fallback("missing_required_sensor", missing=missing)

        if self.backend is None or not hasattr(self.backend, "predict_control"):
            return _fallback("backend_unavailable")

        observation = self.build_observation(sensors=sensors, ego_state=ego_state, route=route)
        try:
            control = self.backend.predict_control(observation)
        except Exception as exc:
            return _fallback("backend_exception", detail=str(exc))

        if not is_valid_vehicle_control(control):
            return _fallback("invalid_control")

        return {
            "status": "control",
            "control": normalize_vehicle_control(control),
            "observation_summary": {
                "sensor_count": len(observation["sensors"]),
                "route_command": observation["route"].get("route_command"),
            },
        }

    def build_observation(
        self,
        *,
        sensors: dict[str, Any],
        ego_state: dict[str, Any],
        route: dict[str, Any],
    ) -> dict[str, Any]:
        return {
            "sensors": {camera: sensors[camera] for camera in self.required_cameras},
            "ego_state": deepcopy(ego_state),
            "route": deepcopy(route),
            "io_contract": deepcopy(self.io_contract),
        }


def is_valid_vehicle_control(control: Any) -> bool:
    if not isinstance(control, dict):
        return False
    for field in _REQUIRED_CONTROL_FIELDS:
        if field not in control:
            return False
    return (
        _in_range(control["throttle"], 0.0, 1.0)
        and _in_range(control["steer"], -1.0, 1.0)
        and _in_range(control["brake"], 0.0, 1.0)
        and isinstance(control["hand_brake"], bool)
        and isinstance(control["reverse"], bool)
    )


def normalize_vehicle_control(control: dict[str, Any]) -> dict[str, Any]:
    return {
        "throttle": float(control["throttle"]),
        "steer": float(control["steer"]),
        "brake": float(control["brake"]),
        "hand_brake": bool(control["hand_brake"]),
        "reverse": bool(control["reverse"]),
    }


def _in_range(value: Any, lower: float, upper: float) -> bool:
    return isinstance(value, (int, float)) and lower <= float(value) <= upper


def _fallback(reason: str, **extra: Any) -> dict[str, Any]:
    result = {
        "status": "fallback",
        "reason": reason,
        "control": deepcopy(_SAFE_STOP_CONTROL),
    }
    result.update(extra)
    return result
