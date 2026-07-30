"""Build a Scene Package whose OpenDRIVE is the topology-aware export.

The historical ``build_nuscenes_exchange.py`` is retained for compatibility,
but it imports the limited lane-strip writer. This entry point makes the map
choice explicit and keeps a route corridor optional rather than implicit.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.actor_binding import validate_actor_binding_set
from adapters.opendrive_contract import validate_topology_artifact
from adapters.reconstruction_package import (
    load_reconstruction_package,
    load_reconstruction_result,
    materialize_reconstruction_package,
)
from adapters.scene_package import build_scene_package
from runners.build_nuscenes_topology_opendrive import (
    write_nuscenes_topology_opendrive,
)
from runners.build_openscenario import write_openscenario
from runners.build_scene_ir_from_nuscenes import write_scene_ir


def build_nuscenes_topology_exchange(
    dataroot: Path,
    version: str,
    scene: str,
    output_dir: Path,
    *,
    radius_m: float = 50.0,
    connection_tolerance_m: float = 8.0,
    boundary_connection_tolerance_m: float | None = 20.0,
    boundary_region_tolerance_m: float = 5.0,
    junction_tolerance_m: float = 20.0,
    max_turn_deg: float = 135.0,
    include_ego_corridor: bool = False,
    include_route_inference: bool = True,
    route_alignment_distance_m: float = 1.0,
    route_alignment_heading_deg: float = 5.0,
    route_region_tolerance_m: float = 3.0,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = _paths(output_dir)
    write_scene_ir(dataroot, version, scene, paths["scene_ir"])
    return _build_from_ir(
        dataroot,
        paths,
        radius_m=radius_m,
        connection_tolerance_m=connection_tolerance_m,
        boundary_connection_tolerance_m=boundary_connection_tolerance_m,
        boundary_region_tolerance_m=boundary_region_tolerance_m,
        junction_tolerance_m=junction_tolerance_m,
        max_turn_deg=max_turn_deg,
        include_ego_corridor=include_ego_corridor,
        include_route_inference=include_route_inference,
        route_alignment_distance_m=route_alignment_distance_m,
        route_alignment_heading_deg=route_alignment_heading_deg,
        route_region_tolerance_m=route_region_tolerance_m,
        reconstruction_package_path=None,
        actor_binding_set_path=None,
    )


def build_topology_exchange_from_scenario_ir(
    dataroot: Path,
    scenario_ir_path: Path,
    output_dir: Path,
    *,
    reconstruction_package_path: Path | None = None,
    reconstruction_result_path: Path | None = None,
    actor_binding_set_path: Path | None = None,
    exchange_root: Path | None = None,
    radius_m: float = 50.0,
    connection_tolerance_m: float = 8.0,
    boundary_connection_tolerance_m: float | None = 20.0,
    boundary_region_tolerance_m: float = 5.0,
    junction_tolerance_m: float = 20.0,
    max_turn_deg: float = 135.0,
    include_ego_corridor: bool = False,
    include_route_inference: bool = True,
    route_alignment_distance_m: float = 1.0,
    route_alignment_heading_deg: float = 5.0,
    route_region_tolerance_m: float = 3.0,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = _paths(output_dir)
    source_ir = Path(scenario_ir_path).resolve()
    if source_ir != paths["scene_ir"].resolve():
        shutil.copy2(source_ir, paths["scene_ir"])
    return _build_from_ir(
        dataroot,
        paths,
        radius_m=radius_m,
        connection_tolerance_m=connection_tolerance_m,
        boundary_connection_tolerance_m=boundary_connection_tolerance_m,
        boundary_region_tolerance_m=boundary_region_tolerance_m,
        junction_tolerance_m=junction_tolerance_m,
        max_turn_deg=max_turn_deg,
        include_ego_corridor=include_ego_corridor,
        include_route_inference=include_route_inference,
        route_alignment_distance_m=route_alignment_distance_m,
        route_alignment_heading_deg=route_alignment_heading_deg,
        route_region_tolerance_m=route_region_tolerance_m,
        reconstruction_package_path=reconstruction_package_path,
        reconstruction_result_path=reconstruction_result_path,
        actor_binding_set_path=actor_binding_set_path,
        exchange_root=exchange_root,
    )


def _paths(output_dir: Path) -> dict[str, Path]:
    return {
        "scene_ir": output_dir / "scene_ir.json",
        "opendrive": output_dir / "road.xodr",
        "openscenario": output_dir / "scenario.xosc",
        "scene_package": output_dir / "scene_package.json",
    }


def _build_from_ir(
    dataroot: Path,
    paths: dict[str, Path],
    *,
    radius_m: float,
    connection_tolerance_m: float,
    boundary_connection_tolerance_m: float | None,
    boundary_region_tolerance_m: float,
    junction_tolerance_m: float,
    max_turn_deg: float,
    include_ego_corridor: bool,
    include_route_inference: bool,
    route_alignment_distance_m: float,
    route_alignment_heading_deg: float,
    route_region_tolerance_m: float,
    reconstruction_package_path: Path | None,
    reconstruction_result_path: Path | None = None,
    actor_binding_set_path: Path | None = None,
    exchange_root: Path | None = None,
) -> dict[str, Path]:
    write_nuscenes_topology_opendrive(
        dataroot,
        paths["opendrive"],
        scenario_ir_path=paths["scene_ir"],
        radius_m=radius_m,
        connection_tolerance_m=connection_tolerance_m,
        boundary_connection_tolerance_m=boundary_connection_tolerance_m,
        boundary_region_tolerance_m=boundary_region_tolerance_m,
        junction_tolerance_m=junction_tolerance_m,
        max_turn_deg=max_turn_deg,
        include_ego_corridor=include_ego_corridor,
        include_route_inference=include_route_inference,
        route_alignment_distance_m=route_alignment_distance_m,
        route_alignment_heading_deg=route_alignment_heading_deg,
        route_region_tolerance_m=route_region_tolerance_m,
    )
    _validate_topology_artifact(
        paths["opendrive"],
        include_ego_corridor=include_ego_corridor,
        require_route_chain=include_route_inference,
        scenario_ir_path=paths["scene_ir"],
    )
    write_openscenario(
        paths["scene_ir"],
        paths["openscenario"],
        road_file=paths["opendrive"].name,
    )

    scene_ir = json.loads(paths["scene_ir"].read_text(encoding="utf-8"))
    actor_bindings_name = None
    if actor_binding_set_path is not None:
        binding_source = Path(actor_binding_set_path).resolve()
        actor_bindings = json.loads(binding_source.read_text(encoding="utf-8"))
        validate_actor_binding_set(actor_bindings)
        if actor_bindings["scene_id"] != str(scene_ir["scenario_id"]):
            raise ValueError("Actor Binding Set scene_id does not match Scenario IR")
        binding_target = paths["scene_package"].parent / "actor_bindings.json"
        if binding_source != binding_target.resolve():
            shutil.copy2(binding_source, binding_target)
        paths["actor_bindings"] = binding_target
        actor_bindings_name = binding_target.name

    if reconstruction_package_path is not None and reconstruction_result_path is not None:
        raise ValueError("provide only one reconstruction package or result")
    reconstruction_paths: dict[str, str] = {}
    if reconstruction_result_path is not None:
        if exchange_root is None:
            raise ValueError("exchange_root is required with reconstruction result")
        reconstruction = load_reconstruction_result(
            reconstruction_result_path,
            exchange_root=exchange_root,
            expected_scene_id=str(scene_ir["scenario_id"]),
        )
        reconstruction_paths = materialize_reconstruction_package(
            reconstruction,
            paths["scene_package"].parent,
        )
    elif reconstruction_package_path is not None:
        reconstruction = load_reconstruction_package(
            reconstruction_package_path,
            expected_scene_id=str(scene_ir["scenario_id"]),
        )
        reconstruction_paths = materialize_reconstruction_package(
            reconstruction,
            paths["scene_package"].parent,
        )

    map_source_parts = ["nuscenes_topology_map_expansion"]
    if include_route_inference:
        map_source_parts.append("with_mixed_route_path")
    if include_ego_corridor:
        map_source_parts.append("and_ego_corridor")
    package = build_scene_package(
        scene_ir,
        scene_ir_path=paths["scene_ir"].name,
        openscenario_path=paths["openscenario"].name,
        opendrive_path=paths["opendrive"].name,
        map_source="_".join(map_source_parts),
        actor_bindings_path=actor_bindings_name,
        nurec_usdz=reconstruction_paths.get("nurec_usdz"),
        nurec_checkpoint=reconstruction_paths.get("nurec_checkpoint"),
        reconstruction_package_path=reconstruction_paths.get("reconstruction_package"),
    )
    paths["scene_package"].write_text(
        json.dumps(package, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return paths


def _validate_topology_artifact(
    path: Path,
    *,
    include_ego_corridor: bool,
    require_route_chain: bool = False,
    scenario_ir_path: Path | None = None,
) -> dict:
    summary = validate_topology_artifact(
        path,
        expected_ego_corridor_count=1 if include_ego_corridor else 0,
        require_map_topology=True,
        require_junction_topology=True,
        require_route_chain=require_route_chain,
        require_route_map_integration=require_route_chain,
        require_route_source_audit=require_route_chain,
        scenario_ir_path=scenario_ir_path,
        require_ego_route_coverage=require_route_chain,
        require_boundary_audit=True,
        require_connector_evidence=True,
    )
    report_path = Path(path).with_name("xodr_topology_validation.json")
    report_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Build Scene IR, topology-aware OpenDRIVE, OpenSCENARIO, "
            "and a portable Scene Package from nuScenes."
        )
    )
    parser.add_argument("--dataroot", required=True)
    parser.add_argument("--version", default="v1.0-mini")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--scene")
    source.add_argument("--scenario-ir")
    reconstruction = parser.add_mutually_exclusive_group()
    reconstruction.add_argument("--reconstruction-package")
    reconstruction.add_argument("--reconstruction-result")
    parser.add_argument("--actor-bindings")
    parser.add_argument("--exchange-root")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--radius-m", type=float, default=50.0)
    parser.add_argument("--connection-tolerance-m", type=float, default=8.0)
    parser.add_argument(
        "--boundary-connection-tolerance-m",
        type=float,
        default=20.0,
        help=(
            "allow larger endpoint gaps only inside or near source intersections "
            "(default: 12 m)"
        ),
    )
    parser.add_argument("--boundary-region-tolerance-m", type=float, default=5.0)
    parser.add_argument("--junction-tolerance-m", type=float, default=20.0)
    parser.add_argument("--max-turn-deg", type=float, default=135.0)
    route_inference = parser.add_mutually_exclusive_group()
    route_inference.add_argument(
        "--include-route-inference",
        dest="include_route_inference",
        action="store_true",
        help="add separately named route roads for source-map geometry gaps (default)",
    )
    route_inference.add_argument(
        "--no-route-inference",
        dest="include_route_inference",
        action="store_false",
        help="disable inferred route roads for raw map diagnostics",
    )
    parser.set_defaults(include_route_inference=True)
    parser.add_argument("--route-alignment-distance-m", type=float, default=1.0)
    parser.add_argument("--route-alignment-heading-deg", type=float, default=5.0)
    parser.add_argument("--route-region-tolerance-m", type=float, default=3.0)
    corridor = parser.add_mutually_exclusive_group()
    corridor.add_argument(
        "--include-ego-corridor",
        dest="include_ego_corridor",
        action="store_true",
        help="include the separate Ego corridor for diagnostics (opt-in)",
    )
    corridor.add_argument(
        "--map-only",
        dest="include_ego_corridor",
        action="store_false",
        help="emit the route-chain map without the diagnostic corridor (default)",
    )
    parser.set_defaults(include_ego_corridor=False)
    args = parser.parse_args(argv)

    if args.scenario_ir:
        paths = build_topology_exchange_from_scenario_ir(
            Path(args.dataroot),
            Path(args.scenario_ir),
            Path(args.output_dir),
            reconstruction_package_path=(
                Path(args.reconstruction_package)
                if args.reconstruction_package
                else None
            ),
            reconstruction_result_path=(
                Path(args.reconstruction_result)
                if args.reconstruction_result
                else None
            ),
            actor_binding_set_path=(
                Path(args.actor_bindings) if args.actor_bindings else None
            ),
            exchange_root=Path(args.exchange_root) if args.exchange_root else None,
            radius_m=args.radius_m,
            connection_tolerance_m=args.connection_tolerance_m,
            boundary_connection_tolerance_m=args.boundary_connection_tolerance_m,
            boundary_region_tolerance_m=args.boundary_region_tolerance_m,
            junction_tolerance_m=args.junction_tolerance_m,
            max_turn_deg=args.max_turn_deg,
            include_ego_corridor=args.include_ego_corridor,
            include_route_inference=args.include_route_inference,
            route_alignment_distance_m=args.route_alignment_distance_m,
            route_alignment_heading_deg=args.route_alignment_heading_deg,
            route_region_tolerance_m=args.route_region_tolerance_m,
        )
    else:
        if args.reconstruction_package or args.reconstruction_result or args.actor_bindings:
            parser.error("reconstruction or actor binding input requires --scenario-ir")
        paths = build_nuscenes_topology_exchange(
            Path(args.dataroot),
            args.version,
            args.scene,
            Path(args.output_dir),
            radius_m=args.radius_m,
            connection_tolerance_m=args.connection_tolerance_m,
            boundary_connection_tolerance_m=args.boundary_connection_tolerance_m,
            boundary_region_tolerance_m=args.boundary_region_tolerance_m,
            junction_tolerance_m=args.junction_tolerance_m,
            max_turn_deg=args.max_turn_deg,
            include_ego_corridor=args.include_ego_corridor,
            include_route_inference=args.include_route_inference,
            route_alignment_distance_m=args.route_alignment_distance_m,
            route_alignment_heading_deg=args.route_alignment_heading_deg,
            route_region_tolerance_m=args.route_region_tolerance_m,
        )

    root = ET.parse(paths["opendrive"]).getroot()
    print(
        json.dumps(
            {
                "paths": {key: str(path) for key, path in paths.items()},
                "scope": "nuscenes_topology_exchange",
                "road_count": len(root.findall("road")),
                "map_lane_road_count": sum(
                    road.attrib.get("name", "").startswith("nuscenes_lane_")
                    for road in root.findall("road")
                ),
                "connector_road_count": sum(
                    road.attrib.get("name", "").startswith("inferred_connector_")
                    for road in root.findall("road")
                ),
                "map_connector_road_count": sum(
                    road.attrib.get("name", "").startswith("inferred_connector_")
                    and not road.attrib.get("name", "").startswith(
                        "inferred_connector_route_"
                    )
                    for road in root.findall("road")
                ),
                "route_connector_road_count": sum(
                    road.attrib.get("name", "").startswith(
                        "inferred_connector_route_"
                    )
                    for road in root.findall("road")
                ),
                "route_inference_road_count": sum(
                    road.attrib.get("name", "").startswith("inferred_route_")
                    for road in root.findall("road")
                ),
                "ego_corridor_road_count": sum(
                    road.attrib.get("name") == "ego_route_corridor"
                    for road in root.findall("road")
                ),
                "junction_count": len(root.findall("junction")),
                "ego_corridor_included": args.include_ego_corridor,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
