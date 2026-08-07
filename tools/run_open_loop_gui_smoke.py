"""Run a short visual-only CARLA smoke for the open-loop scene.

This is observability tooling, not open-loop evidence.  It follows the pinned
Scenario IR ego trajectory with a CARLA spectator and never feeds controls into
the M5 scorer.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any


def _load_track(path: Path) -> list[dict[str, float]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    track = (payload.get("ego") or {}).get("reference_trajectory")
    if not isinstance(track, list) or len(track) < 2:
        raise ValueError("Scenario IR needs at least two ego reference samples")
    result = []
    for sample in track:
        if not isinstance(sample, dict):
            raise ValueError("ego reference sample must be an object")
        result.append(
            {
                "t_sec": float(sample["t_sec"]),
                "x": float(sample["x"]),
                "y": float(sample["y"]),
                "z": float(sample.get("z", 0.0)),
                "yaw": float(sample["yaw"]),
            }
        )
    result.sort(key=lambda item: item["t_sec"])
    return result


def _interpolate(track: list[dict[str, float]], t_sec: float) -> dict[str, float]:
    if t_sec <= track[0]["t_sec"]:
        return dict(track[0])
    if t_sec >= track[-1]["t_sec"]:
        return dict(track[-1])
    for before, after in zip(track, track[1:]):
        if t_sec <= after["t_sec"]:
            span = after["t_sec"] - before["t_sec"]
            ratio = 0.0 if span <= 0 else (t_sec - before["t_sec"]) / span
            return {
                name: before[name] + (after[name] - before[name]) * ratio
                for name in ("t_sec", "x", "y", "z", "yaw")
            }
    return dict(track[-1])


def _transform(carla: Any, state: dict[str, float]) -> Any:
    return carla.Transform(
        carla.Location(
            x=state["x"],
            y=-state["y"],
            z=state["z"] + 0.5,
        ),
        carla.Rotation(yaw=-state["yaw"]),
    )


def _follow_spectator(carla: Any, spectator: Any, transform: Any) -> None:
    yaw_rad = math.radians(float(transform.rotation.yaw))
    spectator.set_transform(
        carla.Transform(
            carla.Location(
                x=transform.location.x - 10.0 * math.cos(yaw_rad),
                y=transform.location.y - 10.0 * math.sin(yaw_rad),
                z=transform.location.z + 5.0,
            ),
            carla.Rotation(pitch=-18.0, yaw=transform.rotation.yaw),
        )
    )


def run_smoke(
    *,
    scenario_ir: Path,
    opendrive: Path,
    host: str,
    port: int,
    duration_sec: float,
    output: Path | None,
) -> dict[str, Any]:
    if duration_sec <= 0:
        raise ValueError("duration_sec must be positive")
    track = _load_track(scenario_ir)
    try:
        import carla
    except ImportError as exc:
        raise RuntimeError("CARLA 0.9.16 Python API is unavailable") from exc

    client = carla.Client(host, port)
    client.set_timeout(10.0)
    if client.get_server_version() != "0.9.16":
        raise RuntimeError(f"expected CARLA 0.9.16, got {client.get_server_version()}")
    original_world = client.get_world()
    original_settings = original_world.get_settings()
    world = client.generate_opendrive_world(
        opendrive.read_text(encoding="utf-8"),
        carla.OpendriveGenerationParameters(),
    )
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    ego = None
    role_name = "m5_gui_smoke_ego"
    started = time.monotonic()
    ticks = 0
    try:
        blueprint_library = world.get_blueprint_library()
        candidates = blueprint_library.filter("vehicle.tesla.model3") or blueprint_library.filter("vehicle.*")
        if not candidates:
            raise RuntimeError("CARLA has no vehicle blueprint")
        for actor in world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == role_name:
                actor.destroy()
        blueprint = candidates[0]
        blueprint.set_attribute("role_name", role_name)
        ego = world.try_spawn_actor(blueprint, _transform(carla, track[0]))
        if ego is None:
            raise RuntimeError("could not spawn GUI smoke ego")
        ego.set_simulate_physics(False)
        spectator = world.get_spectator()
        while time.monotonic() - started < duration_sec:
            elapsed = time.monotonic() - started
            state = _interpolate(track, elapsed)
            transform = _transform(carla, state)
            ego.set_transform(transform)
            world.tick()
            _follow_spectator(carla, spectator, transform)
            ticks += 1
            time.sleep(0.02)
    finally:
        if ego is not None:
            ego.destroy()
        world.apply_settings(original_settings)

    result = {
        "schema_version": "open_loop_gui_smoke.v1",
        "status": "completed",
        "formal_evidence": False,
        "server_version": "0.9.16",
        "scenario_ir": str(scenario_ir.resolve()),
        "opendrive": str(opendrive.resolve()),
        "duration_sec": round(time.monotonic() - started, 3),
        "ticks": ticks,
        "role_name": role_name,
    }
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-ir", type=Path, required=True)
    parser.add_argument("--opendrive", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--duration-sec", type=float, default=15.0)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    try:
        print(
            json.dumps(
                run_smoke(
                    scenario_ir=args.scenario_ir,
                    opendrive=args.opendrive,
                    host=args.host,
                    port=args.port,
                    duration_sec=args.duration_sec,
                    output=args.output,
                ),
                indent=2,
            )
        )
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}))
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
