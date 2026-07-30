from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nuscenes_map_to_opendrive import load_nuscenes_map
from adapters.opendrive_contract import validate_topology_artifact
from adapters.nuscenes_scene import build_scene_ir
from adapters.nuscenes_topology_opendrive import build_topology_opendrive_xml


def write_nuscenes_topology_opendrive(
    dataroot: Path,
    output: Path,
    *,
    scene: str | None = None,
    scenario_ir_path: Path | None = None,
    version: str = "v1.0-mini",
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
    ego_lane_width_m: float = 3.7,
    ego_extension_m: float = 10.0,
    ego_sample_spacing_m: float = 2.0,
) -> Path:
    if (scene is None) == (scenario_ir_path is None):
        raise ValueError("provide exactly one of scene or scenario_ir_path")
    scenario_ir = (
        build_scene_ir(dataroot, scene, version=version)
        if scene is not None
        else json.loads(Path(scenario_ir_path).read_text(encoding="utf-8"))
    )
    location = (scenario_ir.get("map_context") or {}).get("location")
    if not location:
        raise ValueError("Scenario IR does not identify a nuScenes map location")
    map_data = load_nuscenes_map(dataroot, str(location))
    xml_text = build_topology_opendrive_xml(
        scenario_ir,
        map_data,
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
        ego_lane_width_m=ego_lane_width_m,
        ego_extension_m=ego_extension_m,
        ego_sample_spacing_m=ego_sample_spacing_m,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xml_text, encoding="utf-8")
    validate_topology_artifact(
        output,
        expected_ego_corridor_count=1 if include_ego_corridor else 0,
        require_map_topology=True,
        require_junction_topology=True,
        require_route_chain=include_route_inference,
        require_route_map_integration=include_route_inference,
        require_route_source_audit=include_route_inference,
        require_boundary_audit=True,
        require_connector_evidence=True,
    )
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Build a multi-road, junction-aware OpenDRIVE graph from nuScenes lanes."
    )
    parser.add_argument("--dataroot", required=True)
    parser.add_argument("--version", default="v1.0-mini")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--scene")
    source.add_argument("--scenario-ir")
    parser.add_argument("--radius-m", type=float, default=50.0)
    parser.add_argument("--connection-tolerance-m", type=float, default=8.0)
    parser.add_argument(
        "--boundary-connection-tolerance-m",
        type=float,
        default=20.0,
        help=(
            "allow larger endpoint gaps only when the transition midpoint is "
            "inside or near a source intersection (default: 12 m)"
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
    parser.add_argument(
        "--include-ego-corridor",
        action="store_true",
        help="add a route-aligned Ego control road while retaining the inferred multi-road graph",
    )
    parser.add_argument("--ego-lane-width-m", type=float, default=3.7)
    parser.add_argument("--ego-extension-m", type=float, default=10.0)
    parser.add_argument("--ego-sample-spacing-m", type=float, default=2.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)

    output = write_nuscenes_topology_opendrive(
        Path(args.dataroot),
        Path(args.output),
        scene=args.scene,
        scenario_ir_path=Path(args.scenario_ir) if args.scenario_ir else None,
        version=args.version,
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
        ego_lane_width_m=args.ego_lane_width_m,
        ego_extension_m=args.ego_extension_m,
        ego_sample_spacing_m=args.ego_sample_spacing_m,
    )
    root = ET.parse(output).getroot()
    roads = root.findall("road")
    print(
        json.dumps(
            {
                "opendrive": str(output.resolve()),
                "scope": (
                    "nuscenes_topology_with_ego_corridor"
                    if args.include_ego_corridor
                    else "nuscenes_topology_inferred"
                ),
                "road_count": len(roads),
                "map_lane_road_count": sum(
                    road.attrib.get("name", "").startswith("nuscenes_lane_")
                    for road in roads
                ),
                "connector_road_count": sum(
                    road.attrib.get("name", "").startswith("inferred_connector_")
                    for road in roads
                ),
                "map_connector_road_count": sum(
                    road.attrib.get("name", "").startswith("inferred_connector_")
                    and not road.attrib.get("name", "").startswith(
                        "inferred_connector_route_"
                    )
                    for road in roads
                ),
                "route_connector_road_count": sum(
                    road.attrib.get("name", "").startswith(
                        "inferred_connector_route_"
                    )
                    for road in roads
                ),
                "route_inference_road_count": sum(
                    road.attrib.get("name", "").startswith("inferred_route_")
                    for road in roads
                ),
                "ego_corridor_road_count": sum(
                    road.attrib.get("name") == "ego_route_corridor" for road in roads
                ),
                "junction_count": len(root.findall("junction")),
                "junction_connection_count": len(root.findall("./junction/connection")),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
