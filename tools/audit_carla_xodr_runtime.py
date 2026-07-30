#!/usr/bin/env python3
"""Audit a canonical scene XODR against a live CARLA waypoint graph."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.carla_xodr_runtime_audit import (  # noqa: E402
    audit_waypoint_samples,
    canonical_to_carla_point,
    canonical_to_carla_yaw,
    load_route_topology_contract,
    normalize_waypoint,
)
from adapters.opendrive_contract import validate_topology_artifact  # noqa: E402


def audit_live_carla(
    *,
    xodr_path: Path,
    scenario_ir_path: Path,
    expected_sha256: str | None = None,
    host: str,
    port: int,
    timeout_sec: float,
    carla_python_api: Path | None = None,
    generate_world: bool = False,
) -> dict[str, Any]:
    """Connect to CARLA and audit every Scenario IR Ego reference sample."""

    contract = validate_topology_artifact(
        xodr_path,
        expected_sha256=expected_sha256,
        expected_ego_corridor_count=0,
        require_map_topology=True,
        require_junction_topology=True,
        require_route_chain=True,
        require_route_map_integration=True,
        require_route_source_audit=True,
        scenario_ir_path=scenario_ir_path,
        require_ego_route_coverage=True,
        require_boundary_audit=True,
        require_connector_evidence=True,
    )
    scenario_ir = json.loads(scenario_ir_path.read_text(encoding="utf-8"))
    trajectory = (scenario_ir.get("ego") or {}).get("reference_trajectory") or []
    if len(trajectory) < 2:
        raise ValueError("Scenario IR must contain at least two Ego route samples")
    route_contract = load_route_topology_contract(xodr_path)

    if carla_python_api is not None:
        sys.path.insert(0, str(carla_python_api.expanduser().resolve()))
    try:
        import carla
    except ImportError as exc:
        raise RuntimeError(
            "CARLA Python API is unavailable; run this audit in the CARLA environment"
        ) from exc

    client = carla.Client(host, int(port))
    client.set_timeout(float(timeout_sec))
    generated = False
    if generate_world:
        xodr = xodr_path.read_text(encoding="utf-8")
        parameters_type = getattr(carla, "OpendriveGenerationParameters", None)
        if parameters_type is None:
            world = client.generate_opendrive_world(xodr)
        else:
            world = client.generate_opendrive_world(xodr, parameters_type())
        generated = True
    else:
        world = client.get_world()
    carla_map = world.get_map()

    samples = []
    for index, state in enumerate(trajectory):
        expected = canonical_to_carla_point(state)
        expected["yaw_deg"] = canonical_to_carla_yaw(state)
        location = carla.Location(
            x=expected["x"], y=expected["y"], z=expected["z"]
        )
        waypoint = carla_map.get_waypoint(
            location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        next_waypoints = []
        step_distance = 0.0
        if index + 1 < len(trajectory):
            next_state = trajectory[index + 1]
            step_distance = math.hypot(
                float(next_state["x"]) - float(state["x"]),
                float(next_state["y"]) - float(state["y"]),
            )
        if waypoint is not None:
            query_distance = max(0.5, step_distance)
            next_waypoints = [
                normalized
                for candidate in waypoint.next(query_distance)
                if (normalized := normalize_waypoint(candidate, carla_module=carla))
                is not None
            ]
        samples.append(
            {
                "index": index,
                "expected": expected,
                "waypoint": normalize_waypoint(waypoint, carla_module=carla),
                "next_waypoints": next_waypoints,
                "step_distance_m": step_distance,
            }
        )

    audit = audit_waypoint_samples(samples, route_contract=route_contract)
    report = {
        **audit,
        "schema_version": "carla_xodr_runtime_audit.v1",
        "xodr": {
            "path": str(xodr_path.resolve()),
            "sha256": _sha256(xodr_path),
            "contract": contract,
        },
        "scenario_ir": str(scenario_ir_path.resolve()),
        "route_topology": route_contract,
        "runtime": {
            "carla_host": host,
            "carla_port": int(port),
            "map_name": str(getattr(carla_map, "name", "")),
            "world_generated_from_xodr": generated,
        },
        "samples": samples,
    }
    return report


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Validate a canonical OpenDRIVE map against CARLA waypoint "
            "continuity, lane membership, and junction branch behavior."
        )
    )
    parser.add_argument("--xodr", required=True, type=Path)
    parser.add_argument("--scenario-ir", required=True, type=Path)
    parser.add_argument(
        "--expected-sha256",
        help="require the exact OpenDRIVE artifact SHA-256 before connecting to CARLA",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--carla-python-api", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument(
        "--generate-world",
        action="store_true",
        help="generate a fresh CARLA world from --xodr before querying waypoints",
    )
    args = parser.parse_args(argv)
    xodr = args.xodr.expanduser().resolve()
    scenario_ir = args.scenario_ir.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not xodr.is_file():
        parser.error(f"XODR does not exist: {xodr}")
    if not scenario_ir.is_file():
        parser.error(f"Scenario IR does not exist: {scenario_ir}")
    if output.exists():
        parser.error(f"output already exists: {output}")
    try:
        report = audit_live_carla(
            xodr_path=xodr,
            scenario_ir_path=scenario_ir,
            expected_sha256=args.expected_sha256,
            host=args.host,
            port=args.port,
            timeout_sec=args.timeout_sec,
            carla_python_api=args.carla_python_api,
            generate_world=args.generate_world,
        )
    except Exception as exc:
        print(f"CARLA XODR runtime audit failed to execute: {exc}", file=sys.stderr)
        return 2
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("status", "sample_count", "lane_membership", "waypoint_continuity", "route_branch")}, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
