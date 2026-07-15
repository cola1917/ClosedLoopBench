from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Callable


class HostRuntimeError(RuntimeError):
    """Raised when the native CARLA runtime is not ready for execution."""


def validate_host_runtime(
    plan: dict[str, Any],
    *,
    probe: Callable[[dict[str, Any]], dict[str, Any]],
    clock: Callable[[], float] = time.time,
    algorithm_heartbeat_timeout_sec: float = 30.0,
) -> dict[str, Any]:
    """Validate native CARLA and, when selected, the external algorithm."""

    connection = plan["carla"]
    result = probe(
        {
            "host": connection["host"],
            "port": connection["port"],
            "timeout_sec": connection.get("timeout_sec", 3.0),
            "map_name": None,
        }
    )
    if result.get("status") != "available":
        raise HostRuntimeError(
            "CARLA is unavailable at {}:{}: {}".format(
                connection["host"],
                connection["port"],
                result.get("reason", "unknown"),
            )
        )

    expected = str(connection.get("expected_version") or "").lstrip("v")
    actual = str(result.get("carla_version") or "").lstrip("v")
    if expected and actual and actual != expected:
        raise HostRuntimeError(
            f"CARLA version mismatch: expected {expected}, server reported {actual}"
        )

    algorithm = plan["algorithm"]
    if algorithm["driver"] == "ros2_control":
        ready_path = Path(algorithm["ready_file"])
        if not ready_path.is_file():
            raise HostRuntimeError(f"external algorithm is not ready: {ready_path}")
        try:
            ready = json.loads(ready_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise HostRuntimeError(f"invalid algorithm ready file: {exc}") from exc
        if ready.get("status") != "ready":
            raise HostRuntimeError(f"external algorithm reported {ready.get('status')!r}")
        if str(ready.get("algorithm_id")) != str(algorithm["id"]):
            raise HostRuntimeError(
                "algorithm id mismatch: expected {!r}, ready file has {!r}".format(
                    algorithm["id"], ready.get("algorithm_id")
                )
            )
        heartbeat = ready.get("heartbeat_unix")
        if not isinstance(heartbeat, (int, float)):
            raise HostRuntimeError("algorithm ready file has no heartbeat")
        heartbeat_age = float(clock()) - float(heartbeat)
        if heartbeat_age < -5.0 or heartbeat_age > algorithm_heartbeat_timeout_sec:
            raise HostRuntimeError(
                f"external algorithm heartbeat is stale: age={heartbeat_age:.3f}s"
            )

    return {
        "status": "ready",
        "carla": result,
        "algorithm_id": algorithm["id"],
        "ego_driver": algorithm["driver"],
    }
