from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
from typing import Any

from adapters.nuscenes_map_to_opendrive import (
    NuScenesMapError,
    _index,
    _lane_centerline,
    _point_segment_distance,
    _polyline_distance_to_points,
    _to_local,
)
from adapters.route_aligned_opendrive import (
    _densify as _densify_route,
    _ego_trajectory,
    _extend_path as _extend_route,
    _fallback_path as _fallback_route,
)


# The nuScenes mini export has no explicit lane predecessor/successor table.
# Geometry-only links beyond this distance need source-map evidence; otherwise
# increasing a CLI tolerance would manufacture road connectivity.
_MAX_UNVERIFIED_CONNECTION_TOLERANCE_M = 8.0


def build_topology_opendrive_xml(
    scenario_ir: dict[str, Any],
    map_data: dict[str, Any],
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
    route_junction_tolerance_m: float = 25.0,
    ego_lane_width_m: float = 3.7,
    ego_extension_m: float = 10.0,
    ego_sample_spacing_m: float = 2.0,
) -> str:
    """Build a multi-road OpenDRIVE graph around the source trajectories.

    nuScenes mini does not expose explicit lane predecessor/successor records
    in its JSON map export. Connections are therefore inferred from lane
    endpoint proximity and directed centerline headings. Unambiguous straight
    continuations become reciprocal road/lane links; ambiguous or turning
    transitions are represented by junction-local connector roads. A connector
    is real geometry between the incoming and outgoing lane endpoints; the
    outgoing map road is never reused as the junction's connecting road.
    Incoming and outgoing map roads point at their junction; each connector
    has its own road predecessor and successor. This preserves branching
    movements without putting multiple successors on a single map road.
    """

    if radius_m <= 0.0:
        raise ValueError("radius_m must be positive")
    if connection_tolerance_m <= 0.0:
        raise ValueError("connection_tolerance_m must be positive")
    if boundary_connection_tolerance_m is not None:
        if boundary_connection_tolerance_m < connection_tolerance_m:
            raise ValueError(
                "boundary_connection_tolerance_m must be >= connection_tolerance_m"
            )
        if boundary_connection_tolerance_m <= 0.0:
            raise ValueError("boundary_connection_tolerance_m must be positive")
    if boundary_region_tolerance_m <= 0.0:
        raise ValueError("boundary_region_tolerance_m must be positive")
    if junction_tolerance_m < connection_tolerance_m:
        raise ValueError("junction_tolerance_m must be >= connection_tolerance_m")
    if not 0.0 < max_turn_deg < 180.0:
        raise ValueError("max_turn_deg must be between 0 and 180")
    if route_alignment_distance_m <= 0.0:
        raise ValueError("route_alignment_distance_m must be positive")
    if not 0.0 < route_alignment_heading_deg < 180.0:
        raise ValueError("route_alignment_heading_deg must be between 0 and 180")
    if route_region_tolerance_m <= 0.0:
        raise ValueError("route_region_tolerance_m must be positive")
    if route_junction_tolerance_m <= 0.0:
        raise ValueError("route_junction_tolerance_m must be positive")
    if ego_lane_width_m <= 0.0:
        raise ValueError("ego_lane_width_m must be positive")
    if ego_extension_m < 0.0:
        raise ValueError("ego_extension_m must be non-negative")
    if ego_sample_spacing_m <= 0.0:
        raise ValueError("ego_sample_spacing_m must be positive")

    frame = scenario_ir.get("coordinate_frame") or {}
    origin = frame.get("origin_global_translation")
    yaw_deg = frame.get("origin_global_yaw_deg")
    if not isinstance(origin, list) or len(origin) < 2 or yaw_deg is None:
        raise NuScenesMapError("Scenario IR lacks the global-to-local coordinate transform")

    nodes = _index(map_data.get("node", []))
    lines = _index(map_data.get("line", []))
    polygons = _index(map_data.get("polygon", []))
    source_road_blocks = _source_road_blocks(
        map_data,
        nodes,
        polygons,
        origin,
        float(yaw_deg),
    )
    track_points = _track_points(scenario_ir)
    if not track_points:
        raise NuScenesMapError("Scenario IR has no Ego or Actor trajectory points")

    lanes = _select_lanes(
        map_data,
        nodes,
        lines,
        polygons,
        origin,
        float(yaw_deg),
        track_points,
        radius_m,
        source_road_blocks,
        connection_tolerance_m=connection_tolerance_m,
        max_turn_deg=max_turn_deg,
    )
    if not lanes:
        raise NuScenesMapError(
            f"no usable nuScenes lanes found within {radius_m:.1f} m of scene trajectories"
        )

    lanes.sort(key=lambda lane: lane["token"])
    route_specs = (
        _infer_route_alignment_specs(
            scenario_ir,
            map_data,
            nodes,
            polygons,
            lanes,
            origin,
            float(yaw_deg),
            distance_threshold_m=route_alignment_distance_m,
            heading_threshold_deg=route_alignment_heading_deg,
            region_tolerance_m=route_region_tolerance_m,
            sample_spacing_m=min(1.0, ego_sample_spacing_m),
        )
        if include_route_inference
        else []
    )
    road_ids = {lane["token"]: index for index, lane in enumerate(lanes, start=1)}
    intersection_regions = _intersection_transition_regions(
        map_data, nodes, polygons, origin, float(yaw_deg)
    )
    candidate_transitions = _infer_transitions(
        lanes,
        connection_tolerance_m=connection_tolerance_m,
        boundary_connection_tolerance_m=boundary_connection_tolerance_m,
        boundary_region_tolerance_m=boundary_region_tolerance_m,
        boundary_regions=intersection_regions,
        max_turn_deg=max_turn_deg,
    )
    transitions = _prune_block_pair_transitions(candidate_transitions)
    raw_incoming_counts: dict[str, int] = {}
    raw_outgoing_counts: dict[str, int] = {}
    for transition in candidate_transitions:
        raw_incoming_counts[transition["incoming"]] = (
            raw_incoming_counts.get(transition["incoming"], 0) + 1
        )
        raw_outgoing_counts[transition["outgoing"]] = (
            raw_outgoing_counts.get(transition["outgoing"], 0) + 1
        )
    matched_incoming = {transition["incoming"] for transition in transitions}
    matched_outgoing = {transition["outgoing"] for transition in transitions}
    for lane in lanes:
        token = lane["token"]
        if token in matched_incoming or token in matched_outgoing:
            lane["topology_boundary"] = False
            lane["topology_boundary_reason"] = "connected_transition"
        elif raw_incoming_counts.get(token) or raw_outgoing_counts.get(token):
            lane["topology_boundary"] = True
            lane["topology_boundary_reason"] = "one_to_one_matching_boundary"
        else:
            lane["topology_boundary"] = True
            lane["topology_boundary_reason"] = "no_legal_source_transition"
    transition_groups = _cluster_transitions(transitions, junction_tolerance_m)

    direct_candidates: list[dict[str, Any]] = []
    for group in transition_groups:
        if (
            len(group) == 1
            and group[0]["turn_deg"] <= 45.0
            and group[0]["distance_m"] <= 0.5
        ):
            direct_candidates.append(group[0])

    # A road has one successor and one predecessor slot.  Clustering usually
    # catches branches at the same endpoint, but retain this explicit
    # one-to-one guard for transitions that fall into separate spatial groups.
    direct_incoming_counts: dict[str, int] = {}
    direct_outgoing_counts: dict[str, int] = {}
    for transition in direct_candidates:
        direct_incoming_counts[transition["incoming"]] = (
            direct_incoming_counts.get(transition["incoming"], 0) + 1
        )
        direct_outgoing_counts[transition["outgoing"]] = (
            direct_outgoing_counts.get(transition["outgoing"], 0) + 1
        )

    direct_keys = {
        (transition["incoming"], transition["outgoing"])
        for transition in direct_candidates
        if direct_incoming_counts[transition["incoming"]] == 1
        and direct_outgoing_counts[transition["outgoing"]] == 1
    }
    direct_links: list[dict[str, Any]] = []
    junction_groups: list[list[dict[str, Any]]] = []
    for group in transition_groups:
        transition = group[0] if len(group) == 1 else None
        if transition is not None and (
            transition["incoming"], transition["outgoing"]
        ) in direct_keys:
            direct_links.append(group[0])
        else:
            junction_groups.append(group)

    road_links: dict[str, dict[str, dict[str, str]]] = {}
    for transition in direct_links:
        incoming = transition["incoming"]
        outgoing = transition["outgoing"]
        _set_road_link(
            road_links,
            incoming,
            "successor",
            _road_link_ref("road", str(road_ids[outgoing]), "start"),
        )
        _set_road_link(
            road_links,
            outgoing,
            "predecessor",
            _road_link_ref("road", str(road_ids[incoming]), "end"),
        )

    junction_specs: list[dict[str, Any]] = []
    connector_specs: list[dict[str, Any]] = []
    next_connector_id = 1001
    for junction_id, group in enumerate(junction_groups, start=1):
        connections = []
        for transition in group:
            connector_id = next_connector_id
            next_connector_id += 1
            connector = dict(transition)
            connector["connector_id"] = connector_id
            connector_specs.append(
                {
                    "id": connector_id,
                    "junction_id": junction_id,
                    "incoming": connector["incoming"],
                    "outgoing": connector["outgoing"],
                    "points": _connector_points(connector),
                    "width": min(
                        max(
                            (connector["incoming_width"] + connector["outgoing_width"])
                            / 2.0,
                            2.0,
                        ),
                        8.0,
                    ),
                    "source_intersection_index": connector.get(
                        "source_intersection_index"
                    ),
                    "source_edge_line_continuity": connector.get(
                        "source_edge_line_continuity", False
                    ),
                    "source_evidence": connector.get(
                        "source_evidence", "endpoint_heading"
                    ),
                }
            )
            connections.append(connector)
        junction_specs.append({"id": junction_id, "connections": connections})

    # OpenDRIVE junction connections select one connector for every allowed
    # incoming -> outgoing movement.  The incoming and outgoing map roads
    # therefore point at the junction, while each connector carries its own
    # road predecessor/successor references.  This preserves every branch
    # without trying to put multiple successors on one map road.
    for junction in junction_specs:
        junction_id = str(junction["id"])
        for transition in junction["connections"]:
            _set_road_link(
                road_links,
                transition["incoming"],
                "successor",
                _road_link_ref("junction", junction_id, "end"),
            )
            _set_road_link(
                road_links,
                transition["outgoing"],
                "predecessor",
                _road_link_ref("junction", junction_id, "start"),
            )

    for connector in connector_specs:
        connector["links"] = {
            "predecessor": _road_link_ref(
                "road", str(road_ids[connector["incoming"]]), "end"
            ),
            "successor": _road_link_ref(
                "road", str(road_ids[connector["outgoing"]]), "start"
            ),
        }

    mixed_route_path: list[dict[str, Any]] = []
    if include_route_inference:
        mixed_route_path, mixed_route_specs = _build_mixed_route_path(
            scenario_ir,
            lanes,
            road_ids,
            connector_specs,
            road_links,
            sample_spacing_m=min(1.0, ego_sample_spacing_m),
        )
        if mixed_route_path:
            route_specs = mixed_route_specs

    # Allocate IDs after route matching. Map lane and map connector IDs stay
    # stable; only explicit source-gap roads consume the route ID range.
    next_route_id = max(2001, next_connector_id + len(route_specs) + 2)
    for index, route in enumerate(route_specs):
        route["id"] = next_route_id + index
    for entry in mixed_route_path:
        if entry.get("route_spec") is not None:
            entry["road_id"] = entry["route_spec"]["id"]
    if mixed_route_path:
        _integrate_mixed_route_topology(
            mixed_route_path,
            junction_specs,
            connector_specs,
            road_ids=road_ids,
            road_links=road_links,
            next_connector_id=next_connector_id,
        )
    else:
        next_connector_id = _integrate_route_topology(
            route_specs,
            junction_specs,
            connector_specs,
            road_ids=road_ids,
            next_connector_id=next_connector_id,
            junction_tolerance_m=route_junction_tolerance_m,
        )

    route_annotations = _route_path_annotations(mixed_route_path)

    all_points = [point for lane in lanes for point in lane["points"]]
    all_points.extend(
        point for connector in connector_specs for point in connector["points"]
    )
    all_points.extend(point for route in route_specs for point in route["points"])
    ego_points = _ego_corridor_points(
        scenario_ir,
        extension_m=ego_extension_m,
        sample_spacing_m=ego_sample_spacing_m,
    ) if include_ego_corridor else []
    all_points.extend(ego_points)
    xs = [point[0] for point in all_points]
    ys = [point[1] for point in all_points]

    root = ET.Element("OpenDRIVE")
    ET.SubElement(
        root,
        "header",
        {
            "revMajor": "1",
            "revMinor": "4",
            "name": f"nuscenes_topology_{scenario_ir.get('scenario_id', 'scene')}",
            "version": "1.00",
            "date": "2026-07-29T00:00:00",
            "north": _fmt(max(ys)),
            "south": _fmt(min(ys)),
            "east": _fmt(max(xs)),
            "west": _fmt(min(xs)),
        },
    )

    for lane in lanes:
        _add_lane_road(
            root,
            road_id=road_ids[lane["token"]],
            name=f"nuscenes_lane_{lane['token']}",
            points=lane["points"],
            width=lane["width"],
            links=road_links.get(lane["token"], {}),
            junction_id=-1,
            user_data={
                "source_lane_token": lane["token"],
                "source_road_block_token": lane.get(
                    "source_road_block_token", ""
                ),
                "source_road_segment_token": lane.get(
                    "source_road_segment_token", ""
                ),
                "topology_boundary": str(
                    bool(lane.get("topology_boundary", False))
                ).lower(),
                "topology_boundary_reason": lane.get(
                    "topology_boundary_reason", ""
                ),
                **route_annotations.get(str(road_ids[lane["token"]]), {}),
            },
        )

    for connector in connector_specs:
        connector_name = connector.get("name")
        if not connector_name:
            connector_name = (
                f"inferred_connector_{connector['junction_id']}_"
                f"{connector['incoming']}_to_{connector['outgoing']}"
            )
        _add_lane_road(
            root,
            road_id=connector["id"],
            name=connector_name,
            points=connector["points"],
            width=connector["width"],
            links=connector["links"],
            junction_id=connector["junction_id"],
            user_data={
                "topology_evidence": connector.get(
                    "source_evidence", "endpoint_heading"
                ),
                "source_edge_line_continuity": str(
                    bool(connector.get("source_edge_line_continuity", False))
                ).lower(),
                "source_intersection_index": connector.get(
                    "source_intersection_index"
                ),
                **route_annotations.get(str(connector["id"]), {}),
            },
        )

    for route in route_specs:
        _add_lane_road(
            root,
            road_id=route["id"],
            name=route["name"],
            points=route["points"],
            width=route["width"],
            links=route.get("links", {}),
            junction_id=-1,
            user_data={
                "route_geometry_authority": "synthetic_reference_trajectory",
                "route_inference_reason": "source_map_lane_alignment_gap",
                "route_source_kind": route.get("source_kind", ""),
                "route_source_token": route.get("source_token", ""),
                "route_start_sample_index": route.get("start_index"),
                "route_end_sample_index": route.get("end_index"),
                **route_annotations.get(str(route["id"]), {}),
            },
        )

    if ego_points:
        _add_lane_road(
            root,
            road_id=1000,
            name="ego_route_corridor",
            points=ego_points,
            width=ego_lane_width_m,
            links={},
            junction_id=-1,
        )

    for junction in junction_specs:
        element = ET.SubElement(
            root,
            "junction",
            {
                "name": f"inferred_junction_{junction['id']}",
                "id": str(junction["id"]),
            },
        )
        for connection_id, transition in enumerate(junction["connections"], start=1):
            incoming_road_id = transition.get("incoming_road_id")
            if incoming_road_id is None:
                incoming_road_id = str(road_ids[transition["incoming"]])
            connection = ET.SubElement(
                element,
                "connection",
                {
                    "id": str(connection_id),
                    "incomingRoad": str(incoming_road_id),
                    "connectingRoad": str(transition["connector_id"]),
                    "contactPoint": "start",
                },
            )
            ET.SubElement(connection, "laneLink", {"from": "-1", "to": "-1"})

    _indent(root)
    return ET.tostring(root, encoding="unicode")


def _integrate_route_topology(
    route_specs: list[dict[str, Any]],
    junction_specs: list[dict[str, Any]],
    connector_specs: list[dict[str, Any]],
    *,
    road_ids: dict[str, int],
    next_connector_id: int,
    junction_tolerance_m: float,
) -> int:
    """Attach the exact Ego route chain to the selected map junction graph.

    The recorded route can run through the middle of a source road block, so
    replacing it with a map-lane centerline loses control alignment. Keep the
    exact route as several roads, but use junction movements at nearby map
    intersections and at the route anchors. This makes the route a real branch
    of the map graph instead of an isolated synthetic corridor.
    """

    if not route_specs or not junction_specs:
        for previous, current in zip(route_specs, route_specs[1:]):
            if previous["end_index"] != current["start_index"]:
                continue
            previous.setdefault("links", {})["successor"] = _road_link_ref(
                "road", str(current["id"]), "start"
            )
            current.setdefault("links", {})["predecessor"] = _road_link_ref(
                "road", str(previous["id"]), "end"
            )
        return next_connector_id

    junction_by_id = {str(item["id"]): item for item in junction_specs}
    junction_centers = {
        junction_id: _junction_spec_center(spec)
        for junction_id, spec in junction_by_id.items()
    }

    def set_route_link(
        route: dict[str, Any], direction: str, reference: dict[str, str]
    ) -> None:
        links = route.setdefault("links", {})
        existing = links.get(direction)
        if existing is not None and existing != reference:
            raise NuScenesMapError(
                f"route road {route['id']} has conflicting {direction} links: "
                f"{existing!r} vs {reference!r}"
            )
        links[direction] = reference

    def nearest_junction(point: tuple[float, float], tolerance: float) -> str | None:
        candidates = [
            (math.dist(point, center), junction_id)
            for junction_id, center in junction_centers.items()
            if center is not None
        ]
        if not candidates:
            return None
        distance, junction_id = min(candidates)
        return junction_id if distance <= tolerance else None

    def add_route_movement(
        *,
        junction_id: str,
        incoming_road_id: str,
        outgoing_road_id: str,
        incoming_end: tuple[float, float],
        outgoing_start: tuple[float, float],
        incoming_heading: float,
        outgoing_heading: float,
        name_suffix: str,
        width: float,
    ) -> None:
        nonlocal next_connector_id
        connector_id = next_connector_id
        next_connector_id += 1
        connector = {
            "id": connector_id,
            "junction_id": int(junction_id),
            "name": f"inferred_connector_route_{name_suffix}",
            "points": _connector_points(
                {
                    "incoming_end": incoming_end,
                    "outgoing_start": outgoing_start,
                    "incoming_heading": incoming_heading,
                    "outgoing_heading": outgoing_heading,
                }
            ),
            "width": min(max(width, 2.0), 8.0),
            "incoming_road_id": str(incoming_road_id),
            "outgoing_road_id": str(outgoing_road_id),
            "links": {
                "predecessor": _road_link_ref(
                    "road", str(incoming_road_id), "end"
                ),
                "successor": _road_link_ref(
                    "road", str(outgoing_road_id), "start"
                ),
            },
        }
        connector_specs.append(connector)
        junction_by_id[junction_id]["connections"].append(
            {
                "connector_id": connector_id,
                "incoming_road_id": str(incoming_road_id),
                "outgoing_road_id": str(outgoing_road_id),
            }
        )

    # Attach the first route road to a nearby map-junction incoming branch.
    # The route endpoint is allowed to be in the middle of a source block; the
    # new connector carries the short geometric access movement.
    first = route_specs[0]
    start_junction_id = nearest_junction(
        first["points"][0], junction_tolerance_m
    )
    if start_junction_id is not None:
        candidates = [
            transition
            for transition in junction_by_id[start_junction_id]["connections"]
            if transition.get("incoming") in road_ids
            and transition.get("incoming_end") is not None
        ]
        if candidates:
            transition = min(
                candidates,
                key=lambda item: math.dist(
                    item["incoming_end"], first["points"][0]
                ),
            )
            incoming_road_id = str(road_ids[transition["incoming"]])
            set_route_link(
                first,
                "predecessor",
                _road_link_ref("junction", start_junction_id, "start"),
            )
            add_route_movement(
                junction_id=start_junction_id,
                incoming_road_id=incoming_road_id,
                outgoing_road_id=str(first["id"]),
                incoming_end=transition["incoming_end"],
                outgoing_start=first["points"][0],
                incoming_heading=transition["incoming_heading"],
                outgoing_heading=_start_heading(first["points"]),
                name_suffix=f"anchor_in_{incoming_road_id}_to_{first['id']}",
                width=(float(transition["incoming_width"]) + float(first["width"]))
                / 2.0,
            )

    # Use nearby existing junctions for route-road transitions. A junction can
    # host several route movements, which is valid because each route road has
    # its own predecessor/successor slot.
    for previous, current in zip(route_specs, route_specs[1:]):
        if previous["end_index"] != current["start_index"]:
            continue
        boundary = previous["points"][-1]
        junction_id = nearest_junction(boundary, min(junction_tolerance_m, 12.0))
        if junction_id is None:
            set_route_link(
                previous,
                "successor",
                _road_link_ref("road", str(current["id"]), "start"),
            )
            set_route_link(
                current,
                "predecessor",
                _road_link_ref("road", str(previous["id"]), "end"),
            )
            continue
        set_route_link(
            previous,
            "successor",
            _road_link_ref("junction", junction_id, "end"),
        )
        set_route_link(
            current,
            "predecessor",
            _road_link_ref("junction", junction_id, "start"),
        )
        add_route_movement(
            junction_id=junction_id,
            incoming_road_id=str(previous["id"]),
            outgoing_road_id=str(current["id"]),
            incoming_end=previous["points"][-1],
            outgoing_start=current["points"][0],
            incoming_heading=_end_heading(previous["points"]),
            outgoing_heading=_start_heading(current["points"]),
            name_suffix=f"{previous['id']}_to_{current['id']}_at_{junction_id}",
            width=(float(previous["width"]) + float(current["width"])) / 2.0,
        )

    # Attach the last route road to a nearby map-junction outgoing branch.
    last = route_specs[-1]
    end_junction_id = nearest_junction(last["points"][-1], junction_tolerance_m)
    if end_junction_id is not None:
        candidates = [
            transition
            for transition in junction_by_id[end_junction_id]["connections"]
            if transition.get("outgoing") in road_ids
            and transition.get("outgoing_start") is not None
        ]
        if candidates:
            transition = min(
                candidates,
                key=lambda item: math.dist(
                    item["outgoing_start"], last["points"][-1]
                ),
            )
            outgoing_road_id = str(road_ids[transition["outgoing"]])
            set_route_link(
                last,
                "successor",
                _road_link_ref("junction", end_junction_id, "end"),
            )
            add_route_movement(
                junction_id=end_junction_id,
                incoming_road_id=str(last["id"]),
                outgoing_road_id=outgoing_road_id,
                incoming_end=last["points"][-1],
                outgoing_start=transition["outgoing_start"],
                incoming_heading=_end_heading(last["points"]),
                outgoing_heading=transition["outgoing_heading"],
                name_suffix=f"{last['id']}_to_{outgoing_road_id}_anchor_out",
                width=(float(last["width"]) + float(transition["outgoing_width"]))
                / 2.0,
            )

    return next_connector_id


def _junction_spec_center(spec: dict[str, Any]) -> tuple[float, float] | None:
    points = []
    for transition in spec.get("connections", []):
        for key in ("incoming_end", "outgoing_start"):
            point = transition.get(key)
            if point is not None:
                points.append(point)
    if not points:
        return None
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


def _select_lanes(
    map_data: dict[str, Any],
    nodes: dict[str, dict[str, Any]],
    lines: dict[str, dict[str, Any]],
    polygons: dict[str, dict[str, Any]],
    origin: list[Any],
    yaw_deg: float,
    track_points: list[tuple[float, float]],
    radius_m: float,
    source_road_blocks: list[dict[str, Any]] | None = None,
    *,
    connection_tolerance_m: float = 8.0,
    max_turn_deg: float = 135.0,
) -> list[dict[str, Any]]:
    usable_lanes = []
    for record in map_data.get("lane", []):
        if str(record.get("lane_type", "")).upper() != "CAR":
            continue
        try:
            global_points, width = _lane_centerline(record, nodes, lines, polygons)
        except NuScenesMapError:
            continue
        local_points = [_to_local(point, origin, yaw_deg) for point in global_points]
        if len(local_points) < 2:
            continue
        lane = {
            "token": str(record["token"]),
            "points": _deduplicate(local_points),
            "width": min(max(float(width), 2.0), 8.0),
            "source_from_edge_line_token": str(
                record.get("from_edge_line_token", "")
            ),
            "source_to_edge_line_token": str(
                record.get("to_edge_line_token", "")
            ),
            "source_from_edge_node_tokens": tuple(
                str(token)
                for token in (
                    lines.get(str(record.get("from_edge_line_token", "")), {})
                    .get("node_tokens", [])
                )
            ),
            "source_to_edge_node_tokens": tuple(
                str(token)
                for token in (
                    lines.get(str(record.get("to_edge_line_token", "")), {})
                    .get("node_tokens", [])
                )
            ),
            "polygon_vertices": _local_polygon(
                polygons.get(str(record.get("polygon_token", ""))),
                nodes,
                origin,
                yaw_deg,
            ),
        }
        source_block = _source_block_for_point(
            lane["points"][len(lane["points"]) // 2],
            source_road_blocks or [],
        )
        if source_block is not None:
            lane.update(
                {
                    "source_road_block_token": source_block["token"],
                    "source_road_segment_token": source_block[
                        "road_segment_token"
                    ],
                    "source_is_intersection": source_block["is_intersection"],
                }
            )
        if _polyline_length(lane["points"]) <= 0.1:
            continue
        lane["near_scene_tracks"] = (
            _polyline_distance_to_points(lane["points"], track_points) <= radius_m
        )
        usable_lanes.append(lane)

    lanes = [lane for lane in usable_lanes if lane["near_scene_tracks"]]
    selected_tokens = {lane["token"] for lane in lanes}
    boundary_lanes = _extend_selected_boundary_lanes(
        lanes,
        usable_lanes,
        selected_tokens=selected_tokens,
        connection_tolerance_m=connection_tolerance_m,
        max_turn_deg=max_turn_deg,
    )
    for lane in boundary_lanes:
        lane["boundary_extension"] = True
    lanes.extend(boundary_lanes)
    for lane in lanes:
        lane.pop("near_scene_tracks", None)
    return lanes


def _extend_selected_boundary_lanes(
    selected_lanes: list[dict[str, Any]],
    usable_lanes: list[dict[str, Any]],
    *,
    selected_tokens: set[str],
    connection_tolerance_m: float,
    max_turn_deg: float,
) -> list[dict[str, Any]]:
    """Add one-hop source lanes that close a selected road endpoint."""

    additions = []
    for candidate in usable_lanes:
        if candidate["token"] in selected_tokens:
            continue
        if any(
            _lane_endpoint_transition_allowed(
                selected,
                candidate,
                connection_tolerance_m=connection_tolerance_m,
                max_turn_deg=max_turn_deg,
            )
            or _lane_endpoint_transition_allowed(
                candidate,
                selected,
                connection_tolerance_m=connection_tolerance_m,
                max_turn_deg=max_turn_deg,
            )
            for selected in selected_lanes
        ):
            additions.append(candidate)
    return additions


def _lane_endpoint_transition_allowed(
    incoming: dict[str, Any],
    outgoing: dict[str, Any],
    *,
    connection_tolerance_m: float,
    max_turn_deg: float,
) -> bool:
    incoming_block = incoming.get("source_road_block_token")
    outgoing_block = outgoing.get("source_road_block_token")
    if incoming_block is None or outgoing_block is None:
        return False
    if incoming_block is not None and incoming_block == outgoing_block:
        return False
    distance = math.dist(incoming["points"][-1], outgoing["points"][0])
    if distance > connection_tolerance_m:
        return False
    if (
        distance > _MAX_UNVERIFIED_CONNECTION_TOLERANCE_M
        and not _source_edge_line_continuity(incoming, outgoing)
    ):
        return False
    turn_deg = math.degrees(
        _angle_difference(
            _end_heading(incoming["points"]),
            _start_heading(outgoing["points"]),
        )
    )
    return turn_deg <= max_turn_deg


def _prune_block_pair_transitions(
    transitions: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Keep a geometry-minimal lane matching for each source block pair.

    Endpoint proximity alone produces a Cartesian product when several lanes
    leave one source block and several lanes enter the next. That represents
    parallel lanes as every possible movement. A block pair can still be one
    branch among several outgoing blocks, but its lane-level movements should
    be matched one-to-one unless the source data provides stronger evidence.
    """

    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    passthrough: list[dict[str, Any]] = []
    for transition in transitions:
        incoming_block = transition.get("incoming_block")
        outgoing_block = transition.get("outgoing_block")
        if incoming_block is None or outgoing_block is None:
            passthrough.append(transition)
            continue
        grouped.setdefault((incoming_block, outgoing_block), []).append(transition)

    matched: list[dict[str, Any]] = list(passthrough)
    for group in grouped.values():
        matched.extend(_minimum_transition_matching(group))
    return matched


def _minimum_transition_matching(
    transitions: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    incoming_tokens = sorted({item["incoming"] for item in transitions})
    outgoing_tokens = sorted({item["outgoing"] for item in transitions})
    if len(incoming_tokens) <= 1 or len(outgoing_tokens) <= 1:
        return transitions

    by_pair = {(item["incoming"], item["outgoing"]): item for item in transitions}
    outgoing_indices = {token: index for index, token in enumerate(outgoing_tokens)}

    from functools import lru_cache

    @lru_cache(maxsize=None)
    def solve(
        incoming_index: int, used_outgoing_mask: int
    ) -> tuple[int, float, tuple[tuple[str, str], ...]]:
        if incoming_index >= len(incoming_tokens):
            return 0, 0.0, ()

        best = solve(incoming_index + 1, used_outgoing_mask)
        incoming_token = incoming_tokens[incoming_index]
        for outgoing_token in outgoing_tokens:
            outgoing_index = outgoing_indices[outgoing_token]
            bit = 1 << outgoing_index
            if used_outgoing_mask & bit:
                continue
            candidate = by_pair.get((incoming_token, outgoing_token))
            if candidate is None:
                continue
            count, cost, chosen = solve(
                incoming_index + 1,
                used_outgoing_mask | bit,
            )
            candidate_result = (
                count + 1,
                cost
                + float(candidate["distance_m"])
                + 0.02 * float(candidate["turn_deg"]),
                ((incoming_token, outgoing_token),) + chosen,
            )
            if _matching_result_is_better(candidate_result, best):
                best = candidate_result
        return best

    _, _, chosen_pairs = solve(0, 0)
    return [by_pair[pair] for pair in chosen_pairs]


def _matching_result_is_better(
    candidate: tuple[int, float, tuple[tuple[str, str], ...]],
    incumbent: tuple[int, float, tuple[tuple[str, str], ...]],
) -> bool:
    if candidate[0] != incumbent[0]:
        return candidate[0] > incumbent[0]
    if not math.isclose(candidate[1], incumbent[1], rel_tol=0.0, abs_tol=1e-9):
        return candidate[1] < incumbent[1]
    return candidate[2] < incumbent[2]


def _source_road_blocks(
    map_data: dict[str, Any],
    nodes: dict[str, dict[str, Any]],
    polygons: dict[str, dict[str, Any]],
    origin: list[Any],
    yaw_deg: float,
) -> list[dict[str, Any]]:
    segments = _index(map_data.get("road_segment", []))
    result = []
    for block in map_data.get("road_block", []):
        polygon = polygons.get(str(block.get("polygon_token", "")))
        vertices = _local_polygon(polygon, nodes, origin, yaw_deg)
        if len(vertices) < 3:
            continue
        segment_token = str(block.get("road_segment_token", ""))
        segment = segments.get(segment_token, {})
        result.append(
            {
                "token": str(block.get("token", "")),
                "road_segment_token": segment_token,
                "is_intersection": _is_intersection(
                    segment.get("is_intersection")
                ),
                "vertices": vertices,
            }
        )
    return [block for block in result if block["token"]]


def _source_block_for_point(
    point: tuple[float, float], source_road_blocks: list[dict[str, Any]]
) -> dict[str, Any] | None:
    contained = [
        block
        for block in source_road_blocks
        if _point_in_polygon(point, block["vertices"])
    ]
    if not contained:
        return None
    return min(
        contained,
        key=lambda block: _polygon_boundary_distance(point, block["vertices"]),
    )


def _infer_route_alignment_specs(
    scenario_ir: dict[str, Any],
    map_data: dict[str, Any],
    nodes: dict[str, dict[str, Any]],
    polygons: dict[str, dict[str, Any]],
    lanes: list[dict[str, Any]],
    origin: list[Any],
    yaw_deg: float,
    *,
    distance_threshold_m: float,
    heading_threshold_deg: float,
    region_tolerance_m: float,
    sample_spacing_m: float,
) -> list[dict[str, Any]]:
    """Infer separate route roads only where the source lane graph misses Ego.

    nuScenes mini has road-block and intersection polygons but no lane
    connector table.  A recorded Ego path can therefore cross a source road
    region without belonging to any one lane polygon.  Those portions are
    emitted as short, source-labelled roads.  The raw ``nuscenes_lane_*``
    roads remain untouched and are still the authoritative map-lane set.
    """

    trajectory = _ego_route_states(scenario_ir)
    if len(trajectory) < 2:
        return []

    regions = _route_source_regions(map_data, nodes, polygons, origin, yaw_deg)
    if not regions:
        return []

    assignments = []
    for state in trajectory:
        point = (state["x"], state["y"])
        region = _nearest_route_source_region(
            point, regions, tolerance_m=region_tolerance_m
        )
        lane_alignment = _nearest_lane_alignment(point, state, lanes)
        if lane_alignment is None:
            needs_inference = True
        else:
            needs_inference = (
                lane_alignment["distance_m"] > distance_threshold_m
                or lane_alignment["heading_error_deg"] > heading_threshold_deg
            )
        assignments.append(
            {
                "region": region,
                "needs_inference": needs_inference,
            }
        )

    specs: list[dict[str, Any]] = []
    span_start = 0
    sequence = 1
    for index in range(1, len(assignments) + 1):
        at_end = index == len(assignments)
        same_region = (
            not at_end
            and _route_region_key(assignments[index]["region"])
            == _route_region_key(assignments[span_start]["region"])
        )
        if not at_end and same_region:
            continue

        span = assignments[span_start:index]
        region = span[0]["region"]
        if region is not None and any(item["needs_inference"] for item in span):
            start_index = span_start
            end_index = index - 1
            if start_index == end_index:
                start_index = max(0, start_index - 1)
                end_index = min(len(trajectory) - 1, end_index + 1)
            points = _densify_route_states(
                trajectory[start_index : end_index + 1], sample_spacing_m
            )
            if len(points) >= 2 and _polyline_length(points) > 0.1:
                source_kind = str(region["kind"])
                source_token = str(region["token"])
                specs.append(
                    {
                        "name": (
                            f"inferred_route_{source_kind}_{source_token}"
                            f"_segment_{sequence}"
                        ),
                        "points": points,
                        "width": 3.7,
                        "start_index": start_index,
                        "end_index": end_index,
                        "source_kind": source_kind,
                        "source_token": source_token,
                        "links": {},
                    }
                )
                sequence += 1
        span_start = index

    # Adjacent source regions normally meet between two recorded samples.
    # Reuse the boundary sample for the next inferred road so the exported
    # roads share an endpoint and can carry reciprocal OpenDRIVE links.
    for previous, current in zip(specs, specs[1:]):
        if previous["end_index"] + 1 != current["start_index"]:
            continue
        current["start_index"] = previous["end_index"]
        current["points"] = _densify_route_states(
            trajectory[current["start_index"] : current["end_index"] + 1],
            sample_spacing_m,
        )

    return specs


def _build_mixed_route_path(
    scenario_ir: dict[str, Any],
    lanes: list[dict[str, Any]],
    road_ids: dict[str, int],
    connector_specs: list[dict[str, Any]],
    road_links: dict[str, dict[str, dict[str, str]]],
    *,
    sample_spacing_m: float,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Match Ego to a directed map path and emit gaps only where needed.

    The source map contains lane polygons and inferred junction movements, but
    no explicit route relation. A small dynamic program keeps parallel lanes
    from being selected by one-point nearest-neighbour jumps. Transitions are
    limited to the directed map graph, so the returned path can contain
    ``map_lane -> map_connector -> map_lane`` entries rather than a synthetic
    replacement road.
    """

    trajectory = _ego_route_states(scenario_ir)
    if len(trajectory) < 2:
        return [], []

    entities: dict[str, dict[str, Any]] = {}
    for lane in lanes:
        token = str(lane["token"])
        entities[f"lane:{token}"] = {
            "key": f"lane:{token}",
            "kind": "map_lane",
            "road_id": int(road_ids[token]),
            "token": token,
            "points": lane["points"],
            "width": lane["width"],
            "polygon": lane.get("polygon_vertices") or [],
            "source_block": lane.get("source_road_block_token"),
        }

    for connector in connector_specs:
        name = str(connector.get("name", ""))
        if name.startswith("inferred_connector_route_"):
            continue
        connector_id = str(connector["id"])
        entities[f"connector:{connector_id}"] = {
            "key": f"connector:{connector_id}",
            "kind": "map_connector",
            "road_id": int(connector["id"]),
            "token": connector_id,
            "points": connector["points"],
            "width": connector["width"],
            "polygon": [],
            "source_block": None,
        }

    if len(entities) < 2:
        return [], []

    adjacency: dict[str, set[str]] = defaultdict(set)
    for lane in lanes:
        lane_key = f"lane:{lane['token']}"
        reference = road_links.get(str(lane["token"]), {}).get("successor")
        if reference is not None and reference.get("elementType") == "road":
            target_key = next(
                (
                    key
                    for key, entity in entities.items()
                    if str(entity["road_id"]) == str(reference.get("elementId"))
                ),
                None,
            )
            if target_key is not None:
                adjacency[lane_key].add(target_key)
    for connector in connector_specs:
        if str(connector.get("name", "")).startswith(
            "inferred_connector_route_"
        ):
            continue
        connector_key = f"connector:{connector['id']}"
        incoming_key = f"lane:{connector.get('incoming')}"
        outgoing_key = f"lane:{connector.get('outgoing')}"
        if incoming_key in entities:
            adjacency[incoming_key].add(connector_key)
        if outgoing_key in entities:
            adjacency[connector_key].add(outgoing_key)

    def graph_path(start: str, end: str, max_hops: int = 4) -> list[str] | None:
        if start == end:
            return [start]
        queue = deque([(start, [start])])
        visited = {start}
        while queue:
            current, path = queue.popleft()
            if len(path) - 1 >= max_hops:
                continue
            for target in sorted(adjacency.get(current, ())):
                if target in visited:
                    continue
                next_path = path + [target]
                if target == end:
                    return next_path
                visited.add(target)
                queue.append((target, next_path))
        return None

    candidate_observations: list[tuple[int, list[dict[str, Any]]]] = []
    for sample_index, state in enumerate(trajectory):
        point = (state["x"], state["y"])
        yaw = math.radians(state.get("yaw", 0.0))
        candidates = []
        for entity in entities.values():
            alignment = _route_entity_alignment(point, yaw, entity)
            if alignment is None:
                continue
            inside = bool(alignment["inside_polygon"])
            distance_limit = 6.0 if entity["kind"] == "map_connector" else 5.0
            heading_limit = 30.0 if inside else 22.0
            if entity["kind"] == "map_connector":
                heading_limit = 35.0
            if alignment["distance_m"] > distance_limit:
                continue
            if alignment["heading_error_deg"] > heading_limit:
                continue
            candidates.append(
                {
                    "key": entity["key"],
                    "score": (
                        alignment["distance_m"]
                        + 0.04 * alignment["heading_error_deg"]
                        - (1.0 if inside else 0.0)
                    ),
                }
            )
        if candidates:
            candidates.sort(key=lambda item: (item["score"], item["key"]))
            candidate_observations.append((sample_index, candidates))

    if not candidate_observations:
        return [], []

    # DP records retain the directed graph path used to bridge two observed
    # entities. The restart penalty permits a true source-gap when the map
    # graph has no legal transition, while strongly preferring continuity.
    records: list[tuple[int, dict[str, dict[str, Any]]]] = []
    previous_states: dict[str, dict[str, Any]] = {}
    previous_sample_index: int | None = None
    restart_penalty = max(50.0, 4.0 * len(trajectory))
    observation_reward = 8.0
    for sample_index, candidates in candidate_observations:
        current_states: dict[str, dict[str, Any]] = {}
        missing_samples = (
            max(0, sample_index - previous_sample_index - 1)
            if previous_sample_index is not None
            else 0
        )
        for candidate in candidates:
            key = candidate["key"]
            best = {
                "cost": float(candidate["score"])
                + restart_penalty
                - observation_reward
                + 2.0 * missing_samples,
                "prev_key": None,
                "prev_sample_index": previous_sample_index,
                "via_path": [key],
                "restart": True,
            }
            for previous_key, previous in previous_states.items():
                path = graph_path(previous_key, key)
                if path is None:
                    continue
                previous_entity = entities[previous_key]
                current_entity = entities[key]
                switch_penalty = 0.0
                if (
                    previous_entity["kind"] == "map_lane"
                    and current_entity["kind"] == "map_lane"
                    and previous_entity.get("source_block")
                    == current_entity.get("source_block")
                    and previous_key != key
                ):
                    switch_penalty = 12.0
                cost = (
                    previous["cost"]
                    + float(candidate["score"])
                    - observation_reward
                    + 0.75 * max(0, len(path) - 1)
                    + 0.75 * missing_samples
                    + switch_penalty
                )
                if cost < best["cost"]:
                    best = {
                        "cost": cost,
                        "prev_key": previous_key,
                        "prev_sample_index": previous_sample_index,
                        "via_path": path,
                        "restart": False,
                    }
            current_states[key] = best
        records.append((sample_index, current_states))
        previous_states = current_states
        previous_sample_index = sample_index

    final_key = min(
        previous_states,
        key=lambda key: (previous_states[key]["cost"], key),
    )
    selected: list[tuple[int, str, dict[str, Any]]] = []
    record_index = len(records) - 1
    while record_index >= 0 and final_key is not None:
        sample_index, state_records = records[record_index]
        record = state_records.get(final_key)
        if record is None:
            break
        selected.append((sample_index, final_key, record))
        if record["prev_key"] is None:
            break
        final_key = record["prev_key"]
        record_index -= 1
    selected.reverse()

    mapped_sample_count = len(selected)
    if mapped_sample_count < max(3, int(math.ceil(0.5 * len(trajectory)))):
        return [], []

    runs: list[dict[str, Any]] = []
    for sample_index, key, record in selected:
        if record["restart"] or not runs:
            runs.append(
                {
                    "first_sample": sample_index,
                    "last_sample": sample_index,
                    "entities": [key],
                    "spans": {key: [sample_index, sample_index]},
                }
            )
            continue
        run = runs[-1]
        run["last_sample"] = sample_index
        for entity_key in record["via_path"]:
            if entity_key not in run["entities"]:
                run["entities"].append(entity_key)
                run["spans"][entity_key] = [
                    int(record["prev_sample_index"]),
                    sample_index,
                ]
            else:
                span = run["spans"][entity_key]
                span[0] = min(span[0], int(record["prev_sample_index"]))
                span[1] = max(span[1], sample_index)

    map_path_entities = {
        entity_key
        for run in runs
        for entity_key in run["entities"]
        if entities[entity_key]["kind"] in {"map_lane", "map_connector"}
    }
    graph_edge_count = sum(
        max(0, len(run["entities"]) - 1) for run in runs
    )
    if len(map_path_entities) < 2 or graph_edge_count < 1:
        return [], []

    route_path: list[dict[str, Any]] = []
    route_specs: list[dict[str, Any]] = []
    sequence = 1
    previous_run: dict[str, Any] | None = None
    for run_index, run in enumerate(runs):
        if run["first_sample"] > 0:
            gap_start = 0 if previous_run is None else previous_run["last_sample"]
            gap_end = run["first_sample"] - 1
            if gap_end <= gap_start:
                gap_end = min(len(trajectory) - 1, run["first_sample"])
            gap = _make_route_gap_spec(
                trajectory,
                gap_start,
                gap_end,
                sequence=sequence,
                sample_spacing_m=sample_spacing_m,
            )
            if gap is not None:
                route_specs.append(gap)
                route_path.append(
                    {
                        "kind": "source_gap",
                        "road_id": None,
                        "route_spec": gap,
                        "points": gap["points"],
                        "width": gap["width"],
                        "start_index": gap["start_index"],
                        "end_index": gap["end_index"],
                    }
                )
                sequence += 1
        for entity_key in run["entities"]:
            entity = entities[entity_key]
            span = run["spans"].get(
                entity_key,
                [run["first_sample"], run["last_sample"]],
            )
            route_path.append(
                {
                    "kind": entity["kind"],
                    "road_id": entity["road_id"],
                    "entity_key": entity_key,
                    "points": entity["points"],
                    "width": entity["width"],
                    "start_index": span[0],
                    "end_index": span[1],
                }
            )
        previous_run = run

    return route_path, route_specs


def _route_entity_alignment(
    point: tuple[float, float],
    yaw: float,
    entity: dict[str, Any],
) -> dict[str, Any] | None:
    best: dict[str, Any] | None = None
    points = entity.get("points") or []
    for first, second in zip(points, points[1:]):
        if math.dist(first, second) <= 1e-6:
            continue
        distance = _point_segment_distance(point, first, second)
        heading = math.atan2(second[1] - first[1], second[0] - first[0])
        candidate = {
            "distance_m": distance,
            "heading_error_deg": math.degrees(_angle_difference(yaw, heading)),
            "inside_polygon": _point_in_polygon(point, entity.get("polygon") or [])
            if entity.get("polygon")
            else False,
        }
        if best is None or (
            candidate["distance_m"],
            candidate["heading_error_deg"],
        ) < (
            best["distance_m"],
            best["heading_error_deg"],
        ):
            best = candidate
    return best


def _make_route_gap_spec(
    trajectory: list[dict[str, float]],
    start_index: int,
    end_index: int,
    *,
    sequence: int,
    sample_spacing_m: float,
) -> dict[str, Any] | None:
    if len(trajectory) < 2:
        return None
    start_index = max(0, min(start_index, len(trajectory) - 1))
    end_index = max(0, min(end_index, len(trajectory) - 1))
    if start_index > end_index:
        start_index, end_index = end_index, start_index
    if start_index == end_index:
        start_index = max(0, start_index - 1)
        end_index = min(len(trajectory) - 1, end_index + 1)
    points = _densify_route_states(
        trajectory[start_index : end_index + 1], sample_spacing_m
    )
    if len(points) < 2 or _polyline_length(points) <= 0.1:
        return None
    return {
        "name": f"inferred_route_source_gap_trajectory_{sequence}",
        "points": points,
        "width": 3.7,
        "start_index": start_index,
        "end_index": end_index,
        "source_kind": "source_gap",
        "source_token": f"trajectory_gap_{start_index}_{end_index}",
        "links": {},
    }


def _integrate_mixed_route_topology(
    route_path: list[dict[str, Any]],
    junction_specs: list[dict[str, Any]],
    connector_specs: list[dict[str, Any]],
    *,
    road_ids: dict[str, int],
    road_links: dict[str, dict[str, dict[str, str]]],
    next_connector_id: int,
) -> None:
    """Connect source-gap entries to the existing directed map graph."""

    if len(route_path) < 2:
        return
    lane_by_road_id = {
        str(road_id): token for token, road_id in road_ids.items()
    }
    connector_by_road_id = {
        str(connector["id"]): connector
        for connector in connector_specs
    }
    next_junction_id = max(
        (int(junction["id"]) for junction in junction_specs),
        default=0,
    ) + 1

    def links_for(entry: dict[str, Any]) -> dict[str, dict[str, str]]:
        if entry["kind"] == "source_gap":
            return entry["route_spec"].setdefault("links", {})
        if entry["kind"] == "map_lane":
            token = lane_by_road_id[str(entry["road_id"])]
            return road_links.setdefault(token, {})
        return connector_by_road_id[str(entry["road_id"])].setdefault(
            "links", {}
        )

    def set_link(
        entry: dict[str, Any], direction: str, reference: dict[str, str]
    ) -> None:
        links = links_for(entry)
        existing = links.get(direction)
        if existing is not None and existing != reference:
            raise NuScenesMapError(
                f"route path road {entry['road_id']} has conflicting "
                f"{direction} links: {existing!r} vs {reference!r}"
            )
        links[direction] = reference

    for incoming, outgoing in zip(route_path, route_path[1:]):
        if incoming["kind"] != "source_gap" and outgoing["kind"] != "source_gap":
            continue
        if incoming["kind"] == "source_gap" and outgoing["kind"] == "source_gap":
            set_link(
                incoming,
                "successor",
                _road_link_ref("road", str(outgoing["road_id"]), "start"),
            )
            set_link(
                outgoing,
                "predecessor",
                _road_link_ref("road", str(incoming["road_id"]), "end"),
            )
            continue

        junction_id = str(next_junction_id)
        next_junction_id += 1
        connector_id = next_connector_id
        next_connector_id += 1
        incoming_id = str(incoming["road_id"])
        outgoing_id = str(outgoing["road_id"])
        incoming_heading = _end_heading(incoming["points"])
        outgoing_heading = _start_heading(outgoing["points"])
        connector = {
            "id": connector_id,
            "junction_id": int(junction_id),
            "name": f"inferred_connector_route_access_{incoming_id}_to_{outgoing_id}",
            "points": _connector_points(
                {
                    "incoming_end": incoming["points"][-1],
                    "outgoing_start": outgoing["points"][0],
                    "incoming_heading": incoming_heading,
                    "outgoing_heading": outgoing_heading,
                }
            ),
            "width": min(
                max((float(incoming["width"]) + float(outgoing["width"])) / 2.0, 2.0),
                8.0,
            ),
            "incoming_road_id": incoming_id,
            "outgoing_road_id": outgoing_id,
            "links": {
                "predecessor": _road_link_ref("road", incoming_id, "end"),
                "successor": _road_link_ref("road", outgoing_id, "start"),
            },
        }
        connector_specs.append(connector)
        junction_specs.append(
            {
                "id": int(junction_id),
                "connections": [
                    {
                        "connector_id": connector_id,
                        "incoming_road_id": incoming_id,
                        "outgoing_road_id": outgoing_id,
                    }
                ],
            }
        )
        set_link(
            incoming,
            "successor",
            _road_link_ref("junction", junction_id, "end"),
        )
        set_link(
            outgoing,
            "predecessor",
            _road_link_ref("junction", junction_id, "start"),
        )


def _route_path_annotations(
    route_path: list[dict[str, Any]],
) -> dict[str, dict[str, Any]]:
    annotations: dict[str, dict[str, Any]] = {}
    for order, entry in enumerate(route_path, start=1):
        road_id = entry.get("road_id")
        if road_id is None:
            continue
        kind = str(entry["kind"])
        annotations[str(road_id)] = {
            "route_path_order": order,
            "route_path_kind": kind,
            "route_start_sample_index": entry.get("start_index"),
            "route_end_sample_index": entry.get("end_index"),
            "route_geometry_authority": (
                "synthetic_reference_trajectory"
                if kind == "source_gap"
                else (
                    "map_connector_geometry"
                    if kind == "map_connector"
                    else "map_lane_network"
                )
            ),
        }
        if kind == "map_lane":
            annotations[str(road_id)]["route_source_lane_token"] = str(
                entry.get("entity_key", "").removeprefix("lane:")
            )
    return annotations


def _ego_route_states(scenario_ir: dict[str, Any]) -> list[dict[str, float]]:
    raw = (scenario_ir.get("ego") or {}).get("reference_trajectory") or []
    return [
        {
            "x": float(state["x"]),
            "y": float(state["y"]),
            "z": float(state.get("z", 0.0)),
            "yaw": float(state.get("yaw", 0.0)),
        }
        for state in raw
        if isinstance(state, dict) and "x" in state and "y" in state
    ]


def _densify_route_states(
    states: list[dict[str, float]], sample_spacing_m: float
) -> list[tuple[float, float]]:
    if len(states) < 2:
        return []
    points = _densify_route(
        [
            {
                "x": state["x"],
                "y": state["y"],
                "z": state["z"],
                "yaw": state["yaw"],
            }
            for state in states
        ],
        sample_spacing_m,
    )
    return [(point[0], point[1]) for point in points]


def _route_source_regions(
    map_data: dict[str, Any],
    nodes: dict[str, dict[str, Any]],
    polygons: dict[str, dict[str, Any]],
    origin: list[Any],
    yaw_deg: float,
) -> list[dict[str, Any]]:
    regions: list[dict[str, Any]] = []
    # Intersection polygons take precedence over road blocks when they
    # overlap.  Road blocks provide the source identity for ordinary roads.
    for record in map_data.get("road_segment", []):
        if not _is_intersection(record.get("is_intersection")):
            continue
        polygon = polygons.get(str(record.get("polygon_token", "")))
        vertices = _local_polygon(polygon, nodes, origin, yaw_deg)
        if len(vertices) >= 3:
            regions.append(
                {
                    "kind": "intersection",
                    "token": str(record["token"]),
                    "vertices": vertices,
                }
            )
    for record in map_data.get("road_block", []):
        polygon = polygons.get(str(record.get("polygon_token", "")))
        vertices = _local_polygon(polygon, nodes, origin, yaw_deg)
        if len(vertices) >= 3:
            regions.append(
                {
                    "kind": "road_block",
                    "token": str(record["token"]),
                    "vertices": vertices,
                }
            )
    return regions


def _intersection_transition_regions(
    map_data: dict[str, Any],
    nodes: dict[str, dict[str, Any]],
    polygons: dict[str, dict[str, Any]],
    origin: list[Any],
    yaw_deg: float,
) -> list[list[tuple[float, float]]]:
    """Return source intersection polygons used to gate boundary links."""

    return [
        region["vertices"]
        for region in _route_source_regions(
            map_data, nodes, polygons, origin, yaw_deg
        )
        if region["kind"] == "intersection"
    ]


def _local_polygon(
    polygon: dict[str, Any] | None,
    nodes: dict[str, dict[str, Any]],
    origin: list[Any],
    yaw_deg: float,
) -> list[tuple[float, float]]:
    if not polygon:
        return []
    result = []
    for token in polygon.get("exterior_node_tokens", []):
        node = nodes.get(str(token))
        if node is None:
            return []
        result.append(
            _to_local((float(node["x"]), float(node["y"])), origin, yaw_deg)
        )
    return result


def _is_intersection(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes"}


def _route_region_key(region: dict[str, Any] | None) -> tuple[str, str] | None:
    if region is None:
        return None
    return str(region["kind"]), str(region["token"])


def _nearest_route_source_region(
    point: tuple[float, float],
    regions: list[dict[str, Any]],
    *,
    tolerance_m: float,
) -> dict[str, Any] | None:
    contained = [
        region
        for region in regions
        if _point_in_polygon(point, region["vertices"])
    ]
    if contained:
        return contained[0]
    candidates = [
        (
            _polygon_boundary_distance(point, region["vertices"]),
            region,
        )
        for region in regions
    ]
    if not candidates:
        return None
    distance, region = min(candidates, key=lambda item: item[0])
    return region if distance <= tolerance_m else None


def _point_in_polygon(
    point: tuple[float, float], vertices: list[tuple[float, float]]
) -> bool:
    hit = False
    for first, second in zip(vertices, vertices[1:] + vertices[:1]):
        if (first[1] > point[1]) == (second[1] > point[1]):
            continue
        denominator = second[1] - first[1]
        intersection_x = (
            (second[0] - first[0]) * (point[1] - first[1]) / denominator
            + first[0]
        )
        if point[0] < intersection_x:
            hit = not hit
    return hit


def _polygon_boundary_distance(
    point: tuple[float, float], vertices: list[tuple[float, float]]
) -> float:
    if _point_in_polygon(point, vertices):
        return 0.0
    return min(
        _point_segment_distance(point, first, second)
        for first, second in zip(vertices, vertices[1:] + vertices[:1])
    )


def _nearest_lane_alignment(
    point: tuple[float, float],
    state: dict[str, float],
    lanes: list[dict[str, Any]],
) -> dict[str, Any] | None:
    best = None
    for lane in lanes:
        points = lane["points"]
        for first, second in zip(points, points[1:]):
            if math.dist(first, second) <= 1e-6:
                continue
            distance = _point_segment_distance(point, first, second)
            heading = math.atan2(second[1] - first[1], second[0] - first[0])
            heading_error = math.degrees(
                _angle_difference(math.radians(state.get("yaw", 0.0)), heading)
            )
            candidate = {
                "distance_m": distance,
                "heading_error_deg": heading_error,
                "width": lane["width"],
            }
            if best is None or (
                candidate["distance_m"], candidate["heading_error_deg"]
            ) < (best["distance_m"], best["heading_error_deg"]):
                best = candidate
    return best


def _ego_corridor_points(
    scenario_ir: dict[str, Any], *, extension_m: float, sample_spacing_m: float
) -> list[tuple[float, float]]:
    trajectory = _ego_trajectory(scenario_ir)
    points = _densify_route(trajectory, sample_spacing_m)
    if len(points) < 2:
        points = _fallback_route(trajectory, sample_spacing_m)
    points = _extend_route(points, trajectory, extension_m)
    if len(points) < 2:
        raise NuScenesMapError("Scenario IR Ego trajectory cannot form an Ego corridor")
    return [(point[0], point[1]) for point in points]


def _infer_transitions(
    lanes: list[dict[str, Any]],
    *,
    connection_tolerance_m: float,
    boundary_connection_tolerance_m: float | None,
    boundary_region_tolerance_m: float,
    boundary_regions: list[list[tuple[float, float]]],
    max_turn_deg: float,
) -> list[dict[str, Any]]:
    transitions = []
    for incoming in lanes:
        incoming_end = incoming["points"][-1]
        incoming_heading = _end_heading(incoming["points"])
        for outgoing in lanes:
            if incoming is outgoing:
                continue
            incoming_block = incoming.get("source_road_block_token")
            outgoing_block = outgoing.get("source_road_block_token")
            if incoming_block is not None and incoming_block == outgoing_block:
                # Lanes in one nuScenes road block are parallel lane polygons,
                # not predecessor/successor roads. Connecting them creates
                # false lane changes and can collapse an intersection into a
                # single over-connected junction.
                continue
            outgoing_start = outgoing["points"][0]
            distance = math.dist(incoming_end, outgoing_start)
            turn_deg = math.degrees(
                _angle_difference(incoming_heading, _start_heading(outgoing["points"]))
            )
            if turn_deg > max_turn_deg:
                continue
            midpoint = (
                (incoming_end[0] + outgoing_start[0]) / 2.0,
                (incoming_end[1] + outgoing_start[1]) / 2.0,
            )
            source_intersection_index = _intersection_region_index(
                midpoint,
                boundary_regions,
                boundary_region_tolerance_m,
            )
            source_edge_line_continuity = _source_edge_line_continuity(
                incoming, outgoing
            )
            source_intersection_evidence = source_intersection_index is not None
            if (
                distance > _MAX_UNVERIFIED_CONNECTION_TOLERANCE_M
                and not source_intersection_evidence
                and not source_edge_line_continuity
            ):
                continue
            is_boundary_transition = distance > connection_tolerance_m
            if is_boundary_transition:
                if (
                    boundary_connection_tolerance_m is None
                    or distance > boundary_connection_tolerance_m
                    or (
                        source_intersection_index is None
                        and not source_edge_line_continuity
                    )
                ):
                    continue
            if source_intersection_evidence:
                source_evidence = "source_intersection_region"
            elif source_edge_line_continuity:
                source_evidence = "source_edge_line_continuity"
            else:
                source_evidence = "endpoint_heading"
            transitions.append(
                {
                    "incoming": incoming["token"],
                    "outgoing": outgoing["token"],
                    "incoming_block": incoming.get("source_road_block_token"),
                    "outgoing_block": outgoing.get("source_road_block_token"),
                    "distance_m": distance,
                    "turn_deg": turn_deg,
                    "incoming_heading": incoming_heading,
                    "outgoing_heading": _start_heading(outgoing["points"]),
                    "incoming_end": incoming_end,
                    "outgoing_start": outgoing_start,
                    "incoming_width": incoming["width"],
                    "outgoing_width": outgoing["width"],
                    "boundary_inferred": is_boundary_transition,
                    "source_intersection_index": source_intersection_index,
                    "source_edge_line_continuity": source_edge_line_continuity,
                    "source_evidence": source_evidence,
                }
            )
    return transitions


def _source_edge_line_continuity(
    incoming: dict[str, Any], outgoing: dict[str, Any]
) -> bool:
    """Return whether source lane edge metadata explicitly joins two lanes."""

    incoming_line = str(incoming.get("source_to_edge_line_token", ""))
    outgoing_line = str(outgoing.get("source_from_edge_line_token", ""))
    if not incoming_line or not outgoing_line:
        return False
    if incoming_line == outgoing_line:
        return True
    incoming_nodes = set(incoming.get("source_to_edge_node_tokens", ()))
    outgoing_nodes = set(outgoing.get("source_from_edge_node_tokens", ()))
    return bool(incoming_nodes and outgoing_nodes and incoming_nodes & outgoing_nodes)


def _transition_in_source_intersection(
    incoming_end: tuple[float, float],
    outgoing_start: tuple[float, float],
    regions: list[list[tuple[float, float]]],
    tolerance_m: float,
) -> bool:
    midpoint = (
        (incoming_end[0] + outgoing_start[0]) / 2.0,
        (incoming_end[1] + outgoing_start[1]) / 2.0,
    )
    return _intersection_region_index(
        midpoint,
        regions,
        tolerance_m,
    ) is not None


def _intersection_region_index(
    point: tuple[float, float],
    regions: list[list[tuple[float, float]]],
    tolerance_m: float,
) -> int | None:
    """Return the source intersection containing or nearest to a point."""

    if not regions:
        return None
    contained = [
        index
        for index, vertices in enumerate(regions)
        if _point_in_polygon(point, vertices)
    ]
    if contained:
        return min(
            contained,
            key=lambda index: _polygon_boundary_distance(point, regions[index]),
        )
    nearest = min(
        (
            _polygon_boundary_distance(point, vertices),
            index,
        )
        for index, vertices in enumerate(regions)
    )
    return nearest[1] if nearest[0] <= tolerance_m else None


def _connector_points(transition: dict[str, Any]) -> list[tuple[float, float]]:
    """Approximate a lane transition with a tangent-aware cubic polyline."""

    start = transition["incoming_end"]
    end = transition["outgoing_start"]
    distance = math.dist(start, end)
    if distance <= 1e-6:
        heading = transition.get("outgoing_heading", 0.0)
        return [
            start,
            (start[0] + 0.1 * math.cos(heading), start[1] + 0.1 * math.sin(heading)),
        ]

    incoming_heading = transition.get("incoming_heading", 0.0)
    outgoing_heading = transition.get("outgoing_heading", 0.0)
    handle = max(0.5, min(distance / 3.0, 4.0))
    control_a = (
        start[0] + handle * math.cos(incoming_heading),
        start[1] + handle * math.sin(incoming_heading),
    )
    control_b = (
        end[0] - handle * math.cos(outgoing_heading),
        end[1] - handle * math.sin(outgoing_heading),
    )
    points = []
    for index in range(9):
        t = index / 8.0
        inverse = 1.0 - t
        points.append(
            (
                inverse**3 * start[0]
                + 3.0 * inverse**2 * t * control_a[0]
                + 3.0 * inverse * t**2 * control_b[0]
                + t**3 * end[0],
                inverse**3 * start[1]
                + 3.0 * inverse**2 * t * control_a[1]
                + 3.0 * inverse * t**2 * control_b[1]
                + t**3 * end[1],
            )
        )
    return _deduplicate(points)


def _cluster_transitions(
    transitions: list[dict[str, Any]], junction_tolerance_m: float
) -> list[list[dict[str, Any]]]:
    if not transitions:
        return []
    parents = list(range(len(transitions)))

    def find(index: int) -> int:
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parents[right_root] = left_root

    centers = [
        (
            (item["incoming_end"][0] + item["outgoing_start"][0]) / 2.0,
            (item["incoming_end"][1] + item["outgoing_start"][1]) / 2.0,
        )
        for item in transitions
    ]
    for left in range(len(transitions)):
        for right in range(left + 1, len(transitions)):
            left_region = transitions[left].get("source_intersection_index")
            right_region = transitions[right].get("source_intersection_index")
            # A larger endpoint tolerance can bridge nearby source
            # intersections. Keep those movements in separate junctions even
            # when their geometric centers happen to be close. A road with
            # multiple movements is the exception: OpenDRIVE gives it only
            # one successor/predecessor junction reference, so all of its
            # movements must remain in the same group.
            left_incoming = transitions[left].get("incoming")
            right_incoming = transitions[right].get("incoming")
            left_outgoing = transitions[left].get("outgoing")
            right_outgoing = transitions[right].get("outgoing")
            shares_road_slot = (
                left_incoming is not None
                and left_incoming == right_incoming
            ) or (
                left_outgoing is not None
                and left_outgoing == right_outgoing
            )
            if left_region != right_region and not shares_road_slot:
                continue
            if math.dist(centers[left], centers[right]) <= junction_tolerance_m:
                union(left, right)

    groups: dict[int, list[dict[str, Any]]] = {}
    for index, transition in enumerate(transitions):
        groups.setdefault(find(index), []).append(transition)
    return list(groups.values())


def _road_link_ref(
    element_type: str, element_id: str, contact_point: str
) -> dict[str, str]:
    if element_type not in {"road", "junction"}:
        raise ValueError(f"unsupported OpenDRIVE link element type: {element_type}")
    if contact_point not in {"start", "end"}:
        raise ValueError(f"unsupported OpenDRIVE contact point: {contact_point}")
    return {
        "elementType": element_type,
        "elementId": str(element_id),
        "contactPoint": contact_point,
    }


def _normalise_road_link_ref(
    value: dict[str, str] | str, default_contact_point: str
) -> dict[str, str]:
    if isinstance(value, str):
        return _road_link_ref("road", value, default_contact_point)
    if not isinstance(value, dict):
        raise ValueError(f"invalid OpenDRIVE road link reference: {value!r}")
    return _road_link_ref(
        str(value.get("elementType", "road")),
        str(value["elementId"]),
        str(value.get("contactPoint", default_contact_point)),
    )


def _set_road_link(
    road_links: dict[str, dict[str, dict[str, str]]],
    road_token: str,
    direction: str,
    reference: dict[str, str],
) -> None:
    if direction not in {"predecessor", "successor"}:
        raise ValueError(f"invalid road link direction: {direction}")
    links = road_links.setdefault(road_token, {})
    existing = links.get(direction)
    if existing is not None and existing != reference:
        raise NuScenesMapError(
            f"road {road_token} has conflicting {direction} links: "
            f"{existing!r} vs {reference!r}"
        )
    links[direction] = reference


def _add_lane_road(
    root: ET.Element,
    *,
    road_id: int,
    name: str,
    points: list[tuple[float, float]],
    width: float,
    links: dict[str, dict[str, str] | str],
    junction_id: int,
    user_data: dict[str, Any] | None = None,
) -> None:
    length = _polyline_length(points)
    if length <= 1e-6:
        raise ValueError(f"road {road_id} has zero length")
    road = ET.SubElement(
        root,
        "road",
        {
            "name": name,
            "length": _fmt(length),
            "id": str(road_id),
            "junction": str(junction_id),
        },
    )
    ET.SubElement(road, "type", {"s": "0", "type": "town"})

    link = None
    if links:
        link = ET.SubElement(road, "link")
        if "predecessor" in links:
            reference = _normalise_road_link_ref(links["predecessor"], "end")
            ET.SubElement(
                link,
                "predecessor",
                reference,
            )
        if "successor" in links:
            reference = _normalise_road_link_ref(links["successor"], "start")
            ET.SubElement(
                link,
                "successor",
                reference,
            )

    plan_view = ET.SubElement(road, "planView")
    s = 0.0
    for first, second in zip(points, points[1:]):
        segment_length = math.dist(first, second)
        if segment_length <= 1e-6:
            continue
        geometry = ET.SubElement(
            plan_view,
            "geometry",
            {
                "s": _fmt(s),
                "x": _fmt(first[0]),
                "y": _fmt(first[1]),
                "hdg": _fmt(math.atan2(second[1] - first[1], second[0] - first[0])),
                "length": _fmt(segment_length),
            },
        )
        ET.SubElement(geometry, "line")
        s += segment_length

    lanes = ET.SubElement(road, "lanes")
    ET.SubElement(lanes, "laneOffset", {"s": "0", "a": _fmt(width / 2.0), "b": "0", "c": "0", "d": "0"})
    section = ET.SubElement(lanes, "laneSection", {"s": "0"})
    center = ET.SubElement(section, "center")
    center_lane = ET.SubElement(center, "lane", {"id": "0", "type": "none", "level": "false"})
    ET.SubElement(center_lane, "roadMark", {"sOffset": "0", "type": "none", "weight": "standard", "color": "standard", "width": "0"})
    right = ET.SubElement(section, "right")
    driving = ET.SubElement(right, "lane", {"id": "-1", "type": "driving", "level": "false"})
    road_links = {
        direction: value
        for direction, value in links.items()
        if _normalise_road_link_ref(value, "end" if direction == "predecessor" else "start")[
            "elementType"
        ]
        == "road"
    }
    if road_links:
        lane_link = ET.SubElement(driving, "link")
        if "predecessor" in road_links:
            ET.SubElement(lane_link, "predecessor", {"id": "-1"})
        if "successor" in road_links:
            ET.SubElement(lane_link, "successor", {"id": "-1"})
    ET.SubElement(driving, "width", {"sOffset": "0", "a": _fmt(width), "b": "0", "c": "0", "d": "0"})
    ET.SubElement(driving, "roadMark", {"sOffset": "0", "type": "solid", "weight": "standard", "color": "white", "width": "0.12"})
    if user_data:
        metadata = ET.SubElement(road, "userData")
        for key, value in sorted(user_data.items()):
            if value is None or value == "":
                continue
            ET.SubElement(
                metadata,
                "property",
                {"name": str(key), "value": str(value)},
            )


def _track_points(scenario_ir: dict[str, Any]) -> list[tuple[float, float]]:
    tracks = [(scenario_ir.get("ego") or {}).get("reference_trajectory") or []]
    tracks.extend(
        (actor or {}).get("reference_trajectory") or []
        for actor in scenario_ir.get("actors", [])
        if isinstance(actor, dict)
    )
    return [
        (float(state["x"]), float(state["y"]))
        for track in tracks
        for state in track
        if isinstance(state, dict) and "x" in state and "y" in state
    ]


def _start_heading(points: list[tuple[float, float]]) -> float:
    for first, second in zip(points, points[1:]):
        if math.dist(first, second) > 1e-6:
            return math.atan2(second[1] - first[1], second[0] - first[0])
    return 0.0


def _end_heading(points: list[tuple[float, float]]) -> float:
    for first, second in zip(reversed(points), reversed(points[:-1])):
        if math.dist(first, second) > 1e-6:
            return math.atan2(first[1] - second[1], first[0] - second[0])
    return 0.0


def _angle_difference(left: float, right: float) -> float:
    return abs((left - right + math.pi) % (2.0 * math.pi) - math.pi)


def _deduplicate(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    result = []
    for point in points:
        if not result or math.dist(result[-1], point) > 1e-6:
            result.append(point)
    return result


def _polyline_length(points: list[tuple[float, float]]) -> float:
    return sum(math.dist(first, second) for first, second in zip(points, points[1:]))


def _fmt(value: float) -> str:
    return f"{value:.9f}"


def _indent(element: ET.Element, level: int = 0) -> None:
    whitespace = "\n" + level * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = whitespace + "  "
        for child in element:
            _indent(child, level + 1)
        if not element.tail or not element.tail.strip():
            element.tail = whitespace
    elif level and (not element.tail or not element.tail.strip()):
        element.tail = whitespace
