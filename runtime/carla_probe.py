from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class CarlaProbeConfig:
    host: str = "127.0.0.1"
    port: int = 2000
    timeout_sec: float = 2.0
    map_name: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "host": self.host,
            "port": self.port,
            "timeout_sec": self.timeout_sec,
            "map_name": self.map_name,
        }


def build_probe_config(
    host: str = "127.0.0.1",
    port: int = 2000,
    timeout_sec: float = 2.0,
    map_name: str | None = None,
) -> dict[str, Any]:
    return CarlaProbeConfig(
        host=str(host),
        port=int(port),
        timeout_sec=float(timeout_sec),
        map_name=map_name,
    ).to_dict()


def probe_carla(config: dict[str, Any], carla_module=None) -> dict[str, Any]:
    host = str(config.get("host", "127.0.0.1"))
    port = int(config.get("port", 2000))
    timeout_sec = float(config.get("timeout_sec", 2.0))
    expected_map = config.get("map_name")
    warnings: list[str] = []

    if carla_module is None:
        try:
            import carla as carla_module  # type: ignore[no-redef]
        except Exception as exc:  # pragma: no cover - exercised by import guard tests indirectly
            return _unavailable(host, port, "missing_python_api", str(exc), warnings)

    try:
        client = carla_module.Client(host, port)
        if hasattr(client, "set_timeout"):
            client.set_timeout(timeout_sec)
        world = client.get_world()
        world_map = world.get_map() if hasattr(world, "get_map") else None
        map_name = _map_name(world_map)
        version = client.get_server_version() if hasattr(client, "get_server_version") else None
    except Exception as exc:
        return _unavailable(host, port, "connection_failed", str(exc), warnings)

    if expected_map and map_name and expected_map != map_name:
        return {
            "status": "unavailable",
            "reason": "map_mismatch",
            "host": host,
            "port": port,
            "map": map_name,
            "expected_map": expected_map,
            "carla_version": version,
            "warnings": warnings,
        }

    if expected_map and not map_name:
        warnings.append("map_name_unavailable")

    return {
        "status": "available",
        "host": host,
        "port": port,
        "map": map_name or expected_map,
        "carla_version": version,
        "warnings": warnings,
    }


def _map_name(world_map: Any) -> str | None:
    if world_map is None:
        return None
    name = getattr(world_map, "name", None)
    return str(name) if name else None


def _unavailable(host: str, port: int, reason: str, detail: str, warnings: list[str]) -> dict[str, Any]:
    return {
        "status": "unavailable",
        "reason": reason,
        "detail": detail,
        "host": host,
        "port": port,
        "warnings": warnings,
    }
