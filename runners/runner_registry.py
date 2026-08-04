"""Canonical command surface for the legacy runner modules.

The individual runner files remain import-compatible for existing scripts and
tests. New workflows should enter through ``python -m runners`` so the active
surface stays small even while diagnostic tools are retained.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class RunnerSpec:
    command: str
    module: str
    description: str


CANONICAL_RUNNERS = (
    RunnerSpec(
        "build-scene",
        "runners.build_nuscenes_exchange",
        "Build the portable Scene IR exchange bundle.",
    ),
    RunnerSpec(
        "build-carla-config",
        "runners.build_carla_config",
        "Build a CARLA run configuration from Scenario IR.",
    ),
    RunnerSpec(
        "validate-exchange",
        "runners.esmini_smoke",
        "Validate an OpenSCENARIO/OpenDRIVE exchange artifact.",
    ),
    RunnerSpec(
        "offline-acceptance",
        "runners.run_offline_acceptance",
        "Run the core offline MVP acceptance gates.",
    ),
    RunnerSpec(
        "run-basic-agent",
        "runners.run_carla_basic_agent",
        "Run the CARLA BasicAgent closed-loop path.",
    ),
    RunnerSpec(
        "open-loop-gt-replay",
        "runners.run_open_loop_gt_replay",
        "Replay pinned Scenario IR poses without control-owned ego motion.",
    ),
    RunnerSpec(
        "evaluate-open-loop",
        "runners.evaluate_open_loop",
        "Score open-loop predictions against pinned Scenario IR ground truth.",
    ),
    RunnerSpec(
        "open-loop-ros-smoke",
        "runners.run_open_loop_ros_smoke",
        "Run Pure Pursuit through the frame-matched local ROS open-loop boundary.",
    ),
    RunnerSpec(
        "open-loop-tfpp-stage-a",
        "runners.run_open_loop_transfuserpp_stage_a",
        "Run or preflight TransFuser++ on native CARLA Stage A observations at IR poses.",
    ),
    RunnerSpec(
        "acceptance-triplicate",
        "runners.run_carla_acceptance_triplicate",
        "Run the strict three-run CARLA acceptance gate.",
    ),
    RunnerSpec(
        "run-host-closed-loop",
        "runners.run_host_closed_loop",
        "Run the host-owned closed-loop orchestration path.",
    ),
    RunnerSpec(
        "probe-carla",
        "runners.probe_carla",
        "Probe CARLA availability without starting a scenario.",
    ),
    RunnerSpec(
        "build-report",
        "runners.build_evaluation_result",
        "Build a named evaluation result artifact.",
    ),
    RunnerSpec(
        "compare-reports",
        "runners.compare_reports",
        "Compare completed closed-loop reports.",
    ),
)


_CATEGORY_BY_PREFIX = {
    "add": "support",
    "attach": "support",
    "audit": "diagnostic",
    "bind": "preparation",
    "build": "build",
    "capture": "preparation",
    "compare": "reporting",
    "consume": "exchange",
    "derive": "preparation",
    "diagnose": "diagnostic",
    "esmini": "validation",
    "evaluate": "evaluation",
    "mirror": "preparation",
    "plan": "planning",
    "prepare": "preparation",
    "probe": "diagnostic",
    "publish": "exchange",
    "render": "observability",
    "replay": "diagnostic",
    "run": "runtime",
    "scene0061": "diagnostic",
    "shared": "exchange",
    "validate": "validation",
}

_INTERNAL_MODULES = {"__init__", "__main__", "runner_registry"}


def _runner_directory(root: Path | None = None) -> Path:
    base = Path(root) if root is not None else Path(__file__).resolve().parents[1]
    return base if base.name == "runners" else base / "runners"


def discover_runner_names(root: Path | None = None) -> tuple[str, ...]:
    """Return top-level compatibility runner names in stable order."""

    directory = _runner_directory(root)
    return tuple(
        sorted(
            path.stem
            for path in directory.glob("*.py")
            if path.stem not in _INTERNAL_MODULES
        )
    )


def classify_runner(name: str) -> str:
    prefix = name.partition("_")[0]
    return _CATEGORY_BY_PREFIX.get(prefix, "unclassified")


def runner_inventory(root: Path | None = None) -> dict[str, Any]:
    names = discover_runner_names(root)
    groups: dict[str, list[str]] = {}
    for name in names:
        groups.setdefault(classify_runner(name), []).append(name)
    return {
        "schema_version": "runner_surface.v1",
        "top_level_count": len(names),
        "canonical_commands": [spec.command for spec in CANONICAL_RUNNERS],
        "canonical_modules": [spec.module for spec in CANONICAL_RUNNERS],
        "groups": {key: groups[key] for key in sorted(groups)},
        "unclassified": groups.get("unclassified", []),
    }


def canonical_runner(command: str) -> RunnerSpec:
    for spec in CANONICAL_RUNNERS:
        if spec.command == command:
            return spec
    raise KeyError(command)
