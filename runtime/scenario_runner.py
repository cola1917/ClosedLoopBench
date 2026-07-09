from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Any


def build_scenario_runner_command(config: dict[str, Any]) -> list[str]:
    python_executable = str(config.get("python") or "python")
    scenario_runner_root = Path(str(config["scenario_runner_root"]))
    openscenario = str(config["openscenario"])
    host = str(config.get("host", "127.0.0.1"))
    port = str(int(config.get("port", 2000)))

    command = [
        python_executable,
        str(scenario_runner_root / "scenario_runner.py"),
        "--openscenario",
        openscenario,
        "--host",
        host,
        "--port",
        port,
    ]

    if config.get("output", False):
        command.append("--output")

    output_dir = config.get("output_dir")
    if output_dir:
        command.extend(["--outputDir", str(output_dir)])

    timeout = config.get("timeout")
    if timeout is not None:
        command.extend(["--timeout", str(timeout)])

    return command


def validate_scenario_runner_config(config: dict[str, Any]) -> None:
    if not config.get("scenario_runner_root"):
        raise ValueError("scenario_runner_root is required")
    if not config.get("openscenario"):
        raise ValueError("openscenario is required")
    xosc_path = Path(str(config["openscenario"]))
    if not xosc_path.exists():
        raise FileNotFoundError("OpenSCENARIO file does not exist: {}".format(xosc_path))


def run_scenario_runner(config: dict[str, Any], *, dry_run: bool = True) -> dict[str, Any]:
    command = build_scenario_runner_command(config)
    if dry_run:
        return {"status": "planned", "command": command}
    validate_scenario_runner_config(config)
    completed = subprocess.run(command, check=False)
    return {"status": "completed" if completed.returncode == 0 else "failed", "returncode": completed.returncode, "command": command}
