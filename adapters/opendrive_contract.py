"""Fail-closed validation for the OpenDRIVE artifacts used by ClosedLoopBench."""

from __future__ import annotations

import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


class OpenDriveContractError(RuntimeError):
    """Raised when an OpenDRIVE artifact violates its declared scope."""


_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def validate_topology_artifact(
    path: Path,
    *,
    expected_sha256: str | None = None,
    expected_ego_corridor_count: int | None = None,
    require_map_topology: bool = True,
    require_junction_topology: bool = False,
    require_route_chain: bool = False,
    require_route_map_integration: bool = False,
    require_route_source_audit: bool = False,
    scenario_ir_path: Path | None = None,
    require_ego_route_coverage: bool = False,
    max_ego_route_distance_m: float = 1.0,
    max_ego_route_heading_error_deg: float = 45.0,
    max_network_components: int | None = None,
    max_road_link_gap_m: float = 1.0,
    max_junction_endpoint_gap_m: float = 12.0,
    require_boundary_audit: bool = False,
    require_connector_evidence: bool = False,
) -> dict[str, Any]:
    """Validate map/junction links and return a machine-readable summary.

    A clipped local map may legitimately contain boundary road components, so
    structural link validation and network connectivity are reported as
    separate gates. The network count covers map lanes and connector roads;
    the separately named inferred Ego route is reported as its own component
    scope. It does require every emitted junction movement to be traversable
    through its connector. Callers that need one connected map graph can set
    ``max_network_components=1`` explicitly.
    """

    source = Path(path).expanduser().resolve()
    try:
        artifact_sha256 = _sha256_file(source)
    except OSError as exc:
        raise OpenDriveContractError(f"cannot read OpenDRIVE: {source}") from exc
    if expected_sha256 is not None:
        if not isinstance(expected_sha256, str) or not _SHA256.fullmatch(expected_sha256):
            raise ValueError("expected_sha256 must be a lowercase sha256")
    if max_ego_route_distance_m <= 0.0:
        raise ValueError("max_ego_route_distance_m must be positive")
    if not 0.0 < max_ego_route_heading_error_deg < 180.0:
        raise ValueError("max_ego_route_heading_error_deg must be between 0 and 180")
    try:
        root = ET.parse(source).getroot()
    except (OSError, ET.ParseError) as exc:
        raise OpenDriveContractError(f"cannot parse OpenDRIVE: {source}") from exc
    if root.tag != "OpenDRIVE":
        raise OpenDriveContractError(f"not an OpenDRIVE document: {source}")

    roads = root.findall("./road")
    junctions = root.findall("./junction")
    road_by_id: dict[str, ET.Element] = {}
    junction_by_id: dict[str, ET.Element] = {}
    errors: list[str] = []
    if expected_sha256 is not None and artifact_sha256 != expected_sha256:
        errors.append(
            "OpenDRIVE SHA-256 mismatch: "
            f"expected={expected_sha256} actual={artifact_sha256}"
        )

    for road in roads:
        road_id = str(road.attrib.get("id", ""))
        if not road_id:
            errors.append("road without id")
        elif road_id in road_by_id:
            errors.append(f"duplicate road id: {road_id}")
        else:
            road_by_id[road_id] = road
        if not road.findall("./planView/geometry"):
            errors.append(f"road {road_id} has no planView geometry")

    for junction in junctions:
        junction_id = str(junction.attrib.get("id", ""))
        if not junction_id:
            errors.append("junction without id")
        elif junction_id in junction_by_id:
            errors.append(f"duplicate junction id: {junction_id}")
        else:
            junction_by_id[junction_id] = junction

    map_roads = [
        road
        for road in roads
        if road.attrib.get("name", "").startswith("nuscenes_lane_")
    ]
    connector_roads = [
        road
        for road in roads
        if road.attrib.get("name", "").startswith("inferred_connector_")
    ]
    route_roads = [
        road
        for road in roads
        if road.attrib.get("name", "").startswith("inferred_route_")
    ]
    route_path_roads = _ordered_route_path_roads(roads)
    corridor_roads = [
        road for road in roads if road.attrib.get("name") == "ego_route_corridor"
    ]

    endpoints: dict[str, tuple[tuple[float, float], tuple[float, float]]] = {}
    for road_id, road in road_by_id.items():
        try:
            geometries = road.findall("./planView/geometry")
            first = geometries[0]
            last = geometries[-1]
            first_point = (float(first.attrib["x"]), float(first.attrib["y"]))
            last_heading = float(last.attrib["hdg"])
            last_length = float(last.attrib["length"])
            last_point = (
                float(last.attrib["x"]) + last_length * math.cos(last_heading),
                float(last.attrib["y"]) + last_length * math.sin(last_heading),
            )
            endpoints[road_id] = first_point, last_point
        except (IndexError, KeyError, TypeError, ValueError):
            errors.append(f"road {road_id} has invalid planView endpoint data")

    road_link_count = 0
    road_link_gaps: list[float] = []
    lane_link_count = 0

    for road_id, road in road_by_id.items():
        for reference in road.findall("./link/*"):
            if reference.tag not in {"predecessor", "successor"}:
                continue
            road_link_count += 1
            element_type = reference.attrib.get("elementType")
            target_id = reference.attrib.get("elementId")
            if element_type == "junction":
                if target_id not in junction_by_id:
                    errors.append(
                        f"road {road_id} references missing junction {target_id}"
                    )
                continue
            if element_type != "road":
                errors.append(
                    f"road {road_id} has unsupported link elementType {element_type}"
                )
                continue
            target = road_by_id.get(str(target_id))
            if target is None:
                errors.append(f"road {road_id} references missing road {target_id}")
                continue
            reverse_name = (
                "successor" if reference.tag == "predecessor" else "predecessor"
            )
            reverse = target.find(f"./link/{reverse_name}")
            is_connector = road.attrib.get("name", "").startswith(
                "inferred_connector_"
            )
            if not is_connector:
                if reverse is None or reverse.attrib.get("elementId") != road_id:
                    errors.append(
                        f"road {road_id} {reference.tag} link to {target_id} "
                        f"has no reciprocal {reverse_name} link"
                    )
            if road_id in endpoints and str(target_id) in endpoints:
                source_start, source_end = endpoints[road_id]
                target_start, target_end = endpoints[str(target_id)]
                source_point = source_start if reference.tag == "predecessor" else source_end
                target_point = target_end if reference.tag == "predecessor" else target_start
                gap = math.dist(source_point, target_point)
                road_link_gaps.append(gap)
                if gap > max_road_link_gap_m:
                    errors.append(
                        f"road {road_id} {reference.tag} endpoint gap exceeds "
                        f"{max_road_link_gap_m:g} m: {gap:g}"
                    )

        for lane_link in road.findall("./lanes/laneSection/*/lane/link"):
            lane_link_count += 1
            for reference in lane_link:
                if reference.attrib.get("id") != "-1":
                    errors.append(
                        f"road {road_id} contains unsupported lane link id "
                        f"{reference.attrib.get('id')}"
                    )

    junction_connection_count = 0
    junction_endpoint_gaps: list[float] = []
    for junction_id, junction in junction_by_id.items():
        for connection in junction.findall("./connection"):
            junction_connection_count += 1
            incoming_id = connection.attrib.get("incomingRoad")
            connector_id = connection.attrib.get("connectingRoad")
            incoming = road_by_id.get(str(incoming_id))
            connector = road_by_id.get(str(connector_id))
            if incoming is None:
                errors.append(
                    f"junction {junction_id} references missing incoming road {incoming_id}"
                )
            if connector is None:
                errors.append(
                    f"junction {junction_id} references missing connector {connector_id}"
                )
                continue
            if connector.attrib.get("junction") != junction_id:
                errors.append(
                    f"junction {junction_id} connector {connector_id} has junction "
                    f"{connector.attrib.get('junction')}"
                )
            if not connector.attrib.get("name", "").startswith(
                "inferred_connector_"
            ):
                errors.append(
                    f"junction {junction_id} connector {connector_id} is not an inferred connector road"
                )
            lane_links = connection.findall("./laneLink")
            if not lane_links:
                errors.append(
                    f"junction {junction_id} connection {connection.attrib.get('id')} has no laneLink"
                )

            if incoming is not None:
                incoming_successor = incoming.find("./link/successor")
                if not _matches(
                    incoming_successor,
                    element_type="junction",
                    element_id=junction_id,
                ):
                    errors.append(
                        f"junction {junction_id} incoming road {incoming_id} does not link to the junction"
                    )
            predecessor = connector.find("./link/predecessor")
            if not _matches(
                predecessor, element_type="road", element_id=str(incoming_id)
            ):
                errors.append(
                    f"junction {junction_id} connector {connector_id} does not link back to incoming road {incoming_id}"
                )
            successor = connector.find("./link/successor")
            outgoing_id = (
                successor.attrib.get("elementId")
                if successor is not None
                and successor.attrib.get("elementType") == "road"
                else None
            )
            outgoing = road_by_id.get(str(outgoing_id)) if outgoing_id else None
            if outgoing is None:
                errors.append(
                    f"junction {junction_id} connector {connector_id} does not link to an outgoing road"
                )
            else:
                outgoing_predecessor = outgoing.find("./link/predecessor")
                if not _matches(
                    outgoing_predecessor,
                    element_type="junction",
                    element_id=junction_id,
                ):
                    errors.append(
                        f"junction {junction_id} outgoing road {outgoing_id} does not link to the junction"
                    )

            if incoming is not None and str(connector_id) in endpoints:
                gap = math.dist(endpoints[str(incoming_id)][1], endpoints[str(connector_id)][0])
                junction_endpoint_gaps.append(gap)
                if gap > max_junction_endpoint_gap_m:
                    errors.append(
                        f"junction {junction_id} connection endpoint gap exceeds "
                        f"{max_junction_endpoint_gap_m:g} m: {gap:g}"
                    )
            if outgoing is not None and str(connector_id) in endpoints:
                gap = math.dist(endpoints[str(connector_id)][1], endpoints[str(outgoing_id)][0])
                junction_endpoint_gaps.append(gap)
                if gap > max_junction_endpoint_gap_m:
                    errors.append(
                        f"junction {junction_id} connector endpoint gap to outgoing road exceeds "
                        f"{max_junction_endpoint_gap_m:g} m: {gap:g}"
                    )

    if require_map_topology:
        if len(map_roads) < 2:
            errors.append(
                "single-road or corridor-only OpenDRIVE: map topology requires "
                "at least two nuscenes_lane_* roads; "
                f"found {len(map_roads)}"
            )
        if not junctions and road_link_count == 0:
            errors.append("map topology has no road or junction connectivity")
    if require_junction_topology:
        if not junctions:
            errors.append("junction topology requires at least one junction")
        if not connector_roads:
            errors.append("junction topology requires inferred_connector_* roads")
        if junction_connection_count == 0:
            errors.append("junction topology requires at least one junction connection")
    if expected_ego_corridor_count is not None and len(corridor_roads) != expected_ego_corridor_count:
        errors.append(
            "Ego corridor count mismatch: "
            f"expected={expected_ego_corridor_count} actual={len(corridor_roads)}"
        )

    route_chain = (
        _route_path_chain_summary(
            route_path_roads,
            endpoints,
            road_by_id=road_by_id,
            junctions=junctions,
        )
        if route_path_roads
        else _route_chain_summary(
            route_roads,
            endpoints,
            road_by_id=road_by_id,
            junctions=junctions,
        )
    )
    if require_route_chain:
        for error in route_chain["errors"]:
            errors.append(error)
        if not route_roads and not route_path_roads:
            errors.append(
                "route topology requires inferred_route_* roads or a declared route_path"
            )

    route_map_integration = summary_route_map_integration(
        roads,
        junctions,
        map_roads=map_roads,
        connector_roads=connector_roads,
        route_roads=route_roads,
        route_path_roads=route_path_roads,
    )
    if require_route_map_integration:
        for error in route_map_integration["errors"]:
            errors.append(error)

    route_source_audit = _route_source_audit(route_roads)
    if require_route_source_audit:
        for error in route_source_audit["errors"]:
            errors.append(error)

    ego_route = _ego_route_coverage(
        route_path_roads or route_roads,
        scenario_ir_path,
        max_distance_m=(
            max(max_ego_route_distance_m, 5.0)
            if route_path_roads
            else max_ego_route_distance_m
        ),
        max_heading_error_deg=(
            max(max_ego_route_heading_error_deg, 30.0)
            if route_path_roads
            else max_ego_route_heading_error_deg
        ),
    )
    if require_ego_route_coverage:
        if scenario_ir_path is None:
            errors.append(
                "Ego route coverage requires --scenario-ir or scenario_ir_path"
            )
        for error in ego_route["errors"]:
            errors.append(error)

    summary = _summary(
        roads,
        junctions,
        map_roads=map_roads,
        connector_roads=connector_roads,
        route_roads=route_roads,
        route_path_roads=route_path_roads,
        corridor_roads=corridor_roads,
        road_link_count=road_link_count,
        road_link_gaps=road_link_gaps,
        lane_link_count=lane_link_count,
        junction_connection_count=junction_connection_count,
        junction_endpoint_gaps=junction_endpoint_gaps,
    )
    if require_connector_evidence:
        unclassified_connectors = int(
            summary["map_connector_evidence_unclassified_count"]
        )
        if unclassified_connectors:
            errors.append(
                "map connector evidence audit found "
                f"{unclassified_connectors} connector road(s) without supported "
                "topology_evidence metadata"
            )
    network_component_count = int(summary["network_component_count"])
    if require_boundary_audit:
        unclassified = int(summary["isolated_map_lane_unclassified_count"])
        if unclassified:
            errors.append(
                "boundary topology audit found "
                f"{unclassified} isolated map lane road(s) without an explicit "
                "source-boundary classification"
            )
    if max_network_components is not None:
        if max_network_components < 1:
            raise ValueError("max_network_components must be at least 1")
        if network_component_count > max_network_components:
            errors.append(
                "network connectivity requires at most "
                f"{max_network_components} component(s); found "
                f"{network_component_count}"
            )
    warnings = []
    if network_component_count > 1:
        warnings.append(
            "the selected map lane/connector network is split across "
            f"{network_component_count} connected components"
        )
    if route_source_audit["source_gap_road_count"]:
        warnings.append(
            "Ego route contains explicit synthetic reference-trajectory "
            "source-gap geometry; declared map-path roads remain separate"
        )
        linked_count = int(route_map_integration["map_linked_route_road_count"])
        route_count = len(route_path_roads or route_roads)
        if linked_count < route_count:
            warnings.append(
                "only "
                f"{linked_count}/{route_count} inferred route roads have a "
                "direct route-to-map topology link"
            )
    summary.update(
        {
            "schema_version": "opendrive_topology_contract.v1",
            "path": str(source),
            "artifact_sha256": artifact_sha256,
            "expected_artifact_sha256": expected_sha256,
            "status": "passed" if not errors else "failed",
            "network_connectivity_status": (
                "connected" if network_component_count <= 1 else "partial"
            ),
            "all_network_connectivity_status": (
                "connected"
                if int(summary["all_network_component_count"]) <= 1
                else "partial"
            ),
            "route_network_connectivity_status": (
                "connected"
                if int(summary["route_network_component_count"]) <= 1
                else "partial"
            ),
            "network_connectivity_required_max_components": max_network_components,
            "route_chain_status": route_chain["status"],
            "route_chain_road_ids": route_chain["road_ids"],
            "route_chain_link_count": route_chain["link_count"],
            "route_path_road_count": len(route_path_roads),
            "route_path_road_ids": [
                str(road.attrib.get("id")) for road in route_path_roads
            ],
            "route_map_integration_status": route_map_integration["status"],
            "route_map_junction_connection_count": route_map_integration[
                "junction_connection_count"
            ],
            "route_map_linked_route_road_count": route_map_integration[
                "map_linked_route_road_count"
            ],
            "route_map_linked_route_road_ratio": route_map_integration[
                "map_linked_route_road_ratio"
            ],
            "route_map_geometry_road_count": route_map_integration[
                "map_geometry_route_road_count"
            ],
            "route_map_geometry_road_ratio": route_map_integration[
                "map_geometry_route_road_ratio"
            ],
            "route_source_gap_road_count_in_path": route_map_integration[
                "source_gap_route_road_count"
            ],
            "route_map_linked_route_road_ids": route_map_integration[
                "map_linked_route_road_ids"
            ],
            "route_to_route_junction_connection_count": route_map_integration[
                "route_to_route_junction_connection_count"
            ],
            "route_geometry_authority": (
                "mixed_map_and_synthetic_source_gap"
                if route_path_roads and route_source_audit["source_gap_road_count"]
                else (
                    "map_lane_network"
                    if route_path_roads
                    else route_source_audit["geometry_authority"]
                )
            ),
            "route_source_audit_status": route_source_audit["status"],
            "route_source_gap_road_count": route_source_audit[
                "source_gap_road_count"
            ],
            "route_source_gap_road_ids": route_source_audit["source_gap_road_ids"],
            "route_source_unclassified_road_ids": route_source_audit[
                "unclassified_road_ids"
            ],
            "ego_route_sample_count": ego_route["sample_count"],
            "ego_route_coverage_status": ego_route["status"],
            "ego_route_max_distance_m": ego_route["max_distance_m"],
            "ego_route_max_heading_error_deg": ego_route["max_heading_error_deg"],
            "ego_route_missing_sample_count": ego_route["missing_sample_count"],
            "warnings": warnings,
            "errors": errors,
        }
    )
    if errors:
        preview = "; ".join(errors[:8])
        suffix = f" (+{len(errors) - 8} more)" if len(errors) > 8 else ""
        raise OpenDriveContractError(f"invalid OpenDRIVE topology: {preview}{suffix}")
    return summary


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _matches(
    reference: ET.Element | None, *, element_type: str, element_id: str
) -> bool:
    return bool(
        reference is not None
        and reference.attrib.get("elementType") == element_type
        and reference.attrib.get("elementId") == element_id
    )


def _route_chain_summary(
    route_roads: list[ET.Element],
    endpoints: dict[str, tuple[tuple[float, float], tuple[float, float]]],
    *,
    road_by_id: dict[str, ET.Element],
    junctions: list[ET.Element],
) -> dict[str, Any]:
    """Return the ordered route-road chain without treating a corridor as one."""

    route_ids = {str(road.attrib.get("id")) for road in route_roads}
    predecessors: dict[str, set[str]] = {road_id: set() for road_id in route_ids}
    successors: dict[str, set[str]] = {road_id: set() for road_id in route_ids}
    errors: list[str] = []
    for road in route_roads:
        road_id = str(road.attrib.get("id"))
        successor = road.find("./link/successor")
        if successor is not None and successor.attrib.get("elementType") == "road":
            target_id = str(successor.attrib.get("elementId"))
            if target_id in route_ids:
                successors[road_id].add(target_id)
                predecessors[target_id].add(road_id)
        predecessor = road.find("./link/predecessor")
        if predecessor is not None and predecessor.attrib.get("elementType") == "road":
            target_id = str(predecessor.attrib.get("elementId"))
            if target_id in route_ids:
                predecessors[road_id].add(target_id)
                successors[target_id].add(road_id)

    # A route transition through an OpenDRIVE junction has no direct road
    # link between the two route roads. Recover that route edge from the
    # junction connection's incoming road and connector successor.
    for junction in junctions:
        for connection in junction.findall("./connection"):
            incoming_id = str(connection.attrib.get("incomingRoad"))
            connector = road_by_id.get(str(connection.attrib.get("connectingRoad")))
            if connector is None or incoming_id not in route_ids:
                continue
            successor = connector.find("./link/successor")
            if successor is None or successor.attrib.get("elementType") != "road":
                continue
            target_id = str(successor.attrib.get("elementId"))
            if target_id not in route_ids:
                continue
            successors[incoming_id].add(target_id)
            predecessors[target_id].add(incoming_id)

    if not route_ids:
        return {
            "status": "not_present",
            "road_ids": [],
            "link_count": 0,
            "errors": [],
        }
    if any(len(values) > 1 for values in predecessors.values()):
        errors.append("route topology has a road with multiple route predecessors")
    if any(len(values) > 1 for values in successors.values()):
        errors.append("route topology has a road with multiple route successors")

    starts = sorted(road_id for road_id, values in predecessors.items() if not values)
    if len(starts) != 1:
        errors.append(
            "route topology must have exactly one chain start; "
            f"found {len(starts)}"
        )
    order: list[str] = []
    seen: set[str] = set()
    current = starts[0] if len(starts) == 1 else None
    while current is not None:
        if current in seen:
            errors.append("route topology contains a cycle")
            break
        seen.add(current)
        order.append(current)
        next_ids = sorted(successors[current])
        current = next_ids[0] if next_ids else None
    if len(order) != len(route_ids):
        errors.append(
            "route topology is not one ordered chain; "
            f"visited {len(order)} of {len(route_ids)} route roads"
        )

    link_count = sum(len(values) for values in successors.values())
    if link_count != max(0, len(route_ids) - 1):
        errors.append(
            "route topology link count mismatch: "
            f"expected {max(0, len(route_ids) - 1)} found {link_count}"
        )
    for source_id, targets in successors.items():
        for target_id in targets:
            if source_id not in endpoints or target_id not in endpoints:
                continue
            gap = math.dist(endpoints[source_id][1], endpoints[target_id][0])
            if gap > 1.0:
                errors.append(
                    f"route link {source_id}->{target_id} endpoint gap exceeds 1 m: {gap:g}"
                )
    return {
        "status": "passed" if not errors else "failed",
        "road_ids": order if order else sorted(route_ids),
        "link_count": link_count,
        "errors": errors,
    }


def _ordered_route_path_roads(roads: list[ET.Element]) -> list[ET.Element]:
    """Return roads that explicitly declare membership in the Ego path."""

    declared = []
    for road in roads:
        properties = _user_data_properties(road)
        if "route_path_order" not in properties:
            continue
        try:
            order = int(properties["route_path_order"])
        except (TypeError, ValueError):
            order = 10**9
        declared.append((order, str(road.attrib.get("id")), road))
    return [road for _, _, road in sorted(declared, key=lambda item: (item[0], item[1]))]


def _route_path_chain_summary(
    route_path_roads: list[ET.Element],
    endpoints: dict[str, tuple[tuple[float, float], tuple[float, float]]],
    *,
    road_by_id: dict[str, ET.Element],
    junctions: list[ET.Element],
) -> dict[str, Any]:
    """Validate the declared mixed map/source-gap route order.

    A map junction connection names its connecting road separately from the
    outgoing road. The generic route-chain checker would therefore invent an
    extra direct edge when the connecting map road is itself part of the
    declared path. This checker validates each adjacent declared pair against
    the actual road/junction relation instead.
    """

    errors: list[str] = []
    if not route_path_roads:
        return {
            "status": "not_present",
            "road_ids": [],
            "link_count": 0,
            "errors": [],
        }
    properties = [_user_data_properties(road) for road in route_path_roads]
    orders = []
    for road, values in zip(route_path_roads, properties):
        try:
            orders.append(int(values["route_path_order"]))
        except (KeyError, TypeError, ValueError):
            errors.append(
                f"route path road {road.attrib.get('id')} has invalid route_path_order"
            )
            orders.append(None)
    expected_orders = list(range(1, len(route_path_roads) + 1))
    if orders != expected_orders:
        errors.append(
            "route path order must be contiguous from 1; "
            f"found {orders!r}"
        )

    junction_by_id = {str(junction.attrib.get("id")): junction for junction in junctions}

    def has_declared_edge(source: ET.Element, target: ET.Element) -> bool:
        source_id = str(source.attrib.get("id"))
        target_id = str(target.attrib.get("id"))
        successor = source.find("./link/successor")
        if successor is not None:
            if (
                successor.attrib.get("elementType") == "road"
                and str(successor.attrib.get("elementId")) == target_id
            ):
                return True
            if successor.attrib.get("elementType") == "junction":
                junction = junction_by_id.get(str(successor.attrib.get("elementId")))
                if junction is not None:
                    for connection in junction.findall("./connection"):
                        if str(connection.attrib.get("incomingRoad")) != source_id:
                            continue
                        connecting_id = str(connection.attrib.get("connectingRoad"))
                        if connecting_id == target_id:
                            return True
                        connector = road_by_id.get(connecting_id)
                        connector_successor = (
                            connector.find("./link/successor")
                            if connector is not None
                            else None
                        )
                        if (
                            connector_successor is not None
                            and connector_successor.attrib.get("elementType") == "road"
                            and str(connector_successor.attrib.get("elementId"))
                            == target_id
                        ):
                            return True
        return False

    for source, target in zip(route_path_roads, route_path_roads[1:]):
        if not has_declared_edge(source, target):
            errors.append(
                "route path edge is not represented by road/junction topology: "
                f"{source.attrib.get('id')}->{target.attrib.get('id')}"
            )
        if source.attrib.get("id") in endpoints and target.attrib.get("id") in endpoints:
            source_end = endpoints[str(source.attrib.get("id"))][1]
            target_start = endpoints[str(target.attrib.get("id"))][0]
            # Junction connectors can be between the two path entries, so this
            # gap is diagnostic only for direct road links.
            successor = source.find("./link/successor")
            if successor is not None and successor.attrib.get("elementType") == "road":
                gap = math.dist(source_end, target_start)
                if gap > 1.0:
                    errors.append(
                        f"route path direct edge {source.attrib.get('id')}->"
                        f"{target.attrib.get('id')} endpoint gap exceeds 1 m: {gap:g}"
                    )

    return {
        "status": "passed" if not errors else "failed",
        "road_ids": [str(road.attrib.get("id")) for road in route_path_roads],
        "link_count": max(0, len(route_path_roads) - 1),
        "errors": errors,
    }


def summary_route_map_integration(
    roads: list[ET.Element],
    junctions: list[ET.Element],
    *,
    map_roads: list[ET.Element],
    connector_roads: list[ET.Element],
    route_roads: list[ET.Element],
    route_path_roads: list[ET.Element] | None = None,
) -> dict[str, Any]:
    """Report whether the declared Ego path uses the map graph."""

    declared_route_roads = route_path_roads or route_roads
    route_ids = {str(road.attrib.get("id")) for road in declared_route_roads}
    map_ids = {
        str(road.attrib.get("id"))
        for road in (*map_roads, *connector_roads)
        if not road.attrib.get("name", "").startswith("inferred_connector_route_")
    }
    components = _road_components(roads, junctions)
    route_map_ids = route_ids & map_ids
    source_route_ids = route_ids - map_ids
    shared_components = [
        component
        for component in components
        if component & route_ids and component & map_ids
    ]
    road_by_id = {str(road.attrib.get("id")): road for road in roads}
    junction_connection_count = 0
    route_to_route_junction_connection_count = 0
    map_linked_route_road_ids: set[str] = set()
    for junction in junctions:
        for connection in junction.findall("./connection"):
            incoming_id = str(connection.attrib.get("incomingRoad"))
            connector = road_by_id.get(str(connection.attrib.get("connectingRoad")))
            successor = (
                connector.find("./link/successor")
                if connector is not None
                else None
            )
            outgoing_id = (
                str(successor.attrib.get("elementId"))
                if successor is not None
                and successor.attrib.get("elementType") == "road"
                else None
            )
            if incoming_id in source_route_ids and outgoing_id in map_ids:
                junction_connection_count += 1
                map_linked_route_road_ids.add(incoming_id)
            elif incoming_id in map_ids and outgoing_id in source_route_ids:
                junction_connection_count += 1
                map_linked_route_road_ids.add(outgoing_id)
            elif incoming_id in source_route_ids and outgoing_id in source_route_ids:
                route_to_route_junction_connection_count += 1
    map_linked_route_road_ids.update(route_map_ids)
    for road in declared_route_roads:
        route_id = str(road.attrib.get("id"))
        for reference in road.findall("./link/*"):
            if reference.attrib.get("elementType") != "road":
                continue
            target_id = str(reference.attrib.get("elementId"))
            if target_id in map_ids:
                map_linked_route_road_ids.add(route_id)
            elif target_id in route_ids:
                continue
    errors = []
    if declared_route_roads and source_route_ids and not shared_components:
        errors.append(
            "inferred route roads are not connected to the map lane/junction graph"
        )
    if source_route_ids and junction_connection_count == 0:
        errors.append(
            "inferred route roads have no route-to-map junction connection"
        )
    return {
        "status": "passed" if not errors else "failed",
        "shared_component_count": len(shared_components),
        "junction_connection_count": junction_connection_count,
        "map_linked_route_road_count": len(map_linked_route_road_ids),
        "map_linked_route_road_ratio": (
            len(map_linked_route_road_ids) / len(route_ids) if route_ids else None
        ),
        "map_geometry_route_road_count": len(route_map_ids),
        "map_geometry_route_road_ratio": (
            len(route_map_ids) / len(route_ids) if route_ids else None
        ),
        "source_gap_route_road_count": len(source_route_ids),
        "map_linked_route_road_ids": sorted(map_linked_route_road_ids),
        "route_to_route_junction_connection_count": (
            route_to_route_junction_connection_count
        ),
        "errors": errors,
    }


def _route_source_audit(route_roads: list[ET.Element]) -> dict[str, Any]:
    """Classify inferred route roads without treating them as map lanes.

    Route roads intentionally preserve the recorded Ego trajectory where the
    source map does not provide an unambiguous lane centerline. Their presence
    is useful for replay alignment, but it is not evidence of map-lane
    coverage. The metadata gate makes that distinction explicit in the
    artifact rather than inferring it from a road name.
    """

    if not route_roads:
        return {
            "status": "not_present",
            "geometry_authority": "map_lane_network_not_checked",
            "source_gap_road_count": 0,
            "source_gap_road_ids": [],
            "unclassified_road_ids": [],
            "errors": [],
        }
    source_gap_ids = []
    unclassified_ids = []
    for road in route_roads:
        road_id = str(road.attrib.get("id"))
        properties = _user_data_properties(road)
        required = {
            "route_geometry_authority",
            "route_inference_reason",
            "route_source_kind",
            "route_source_token",
            "route_start_sample_index",
            "route_end_sample_index",
        }
        if required.issubset(properties) and properties[
            "route_geometry_authority"
        ] == "synthetic_reference_trajectory":
            source_gap_ids.append(road_id)
        else:
            unclassified_ids.append(road_id)
    errors = []
    if unclassified_ids:
        errors.append(
            "inferred route roads without explicit synthetic source-gap metadata: "
            + ", ".join(sorted(unclassified_ids))
        )
    return {
        "status": "passed" if not errors else "failed",
        "geometry_authority": "synthetic_reference_trajectory",
        "source_gap_road_count": len(source_gap_ids),
        "source_gap_road_ids": sorted(source_gap_ids),
        "unclassified_road_ids": sorted(unclassified_ids),
        "errors": errors,
    }


def _ego_route_coverage(
    route_roads: list[ET.Element],
    scenario_ir_path: Path | None,
    *,
    max_distance_m: float,
    max_heading_error_deg: float,
) -> dict[str, Any]:
    if scenario_ir_path is None:
        return {
            "status": "not_checked",
            "sample_count": 0,
            "max_distance_m": None,
            "max_heading_error_deg": None,
            "missing_sample_count": 0,
            "errors": [],
        }
    source = Path(scenario_ir_path).expanduser().resolve()
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {
            "status": "failed",
            "sample_count": 0,
            "max_distance_m": None,
            "max_heading_error_deg": None,
            "missing_sample_count": 0,
            "errors": [f"cannot read Scenario IR for Ego route coverage: {source}"],
        }
    trajectory = (payload.get("ego") or {}).get("reference_trajectory") or []
    segments = _route_segments(route_roads)
    yaw_unit = str(
        (payload.get("coordinate_frame") or {}).get("units", {}).get("yaw", "radian")
    ).lower()
    yaw_is_degrees = yaw_unit in {"degree", "degrees", "deg"}
    distances: list[float] = []
    headings: list[float] = []
    missing = 0
    for state in trajectory:
        if not isinstance(state, dict) or "x" not in state or "y" not in state:
            continue
        point = (float(state["x"]), float(state["y"]))
        yaw = float(state.get("yaw", 0.0))
        if yaw_is_degrees:
            yaw = math.radians(yaw)
        candidates = []
        for start, end, heading in segments:
            distance = _point_segment_distance(point, start, end)
            heading_error = math.degrees(_angle_difference(yaw, heading))
            candidates.append((distance, heading_error))
        if not candidates:
            missing += 1
            continue
        distance, heading_error = min(candidates, key=lambda value: (value[0], value[1]))
        distances.append(distance)
        headings.append(heading_error)
        if distance > max_distance_m or heading_error > max_heading_error_deg:
            missing += 1
    errors = []
    if not segments:
        errors.append("Ego route coverage has no inferred_route_* geometry")
    if missing:
        errors.append(
            f"Ego route coverage failed for {missing} of {len(trajectory)} samples"
        )
    return {
        "status": "passed" if not errors else "failed",
        "sample_count": len(trajectory),
        "max_distance_m": max(distances, default=None),
        "max_heading_error_deg": max(headings, default=None),
        "missing_sample_count": missing,
        "errors": errors,
    }


def _route_segments(
    route_roads: list[ET.Element],
) -> list[tuple[tuple[float, float], tuple[float, float], float]]:
    segments = []
    for road in route_roads:
        for geometry in road.findall("./planView/geometry"):
            try:
                start = (float(geometry.attrib["x"]), float(geometry.attrib["y"]))
                heading = float(geometry.attrib["hdg"])
                length = float(geometry.attrib["length"])
            except (KeyError, TypeError, ValueError):
                continue
            if length <= 1e-9:
                continue
            end = (
                start[0] + length * math.cos(heading),
                start[1] + length * math.sin(heading),
            )
            segments.append((start, end, heading))
    return segments


def _point_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.dist(point, start)
    t = max(
        0.0,
        min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq),
    )
    projection = (start[0] + t * dx, start[1] + t * dy)
    return math.dist(point, projection)


def _angle_difference(left: float, right: float) -> float:
    return abs((left - right + math.pi) % (2.0 * math.pi) - math.pi)


def _summary(
    roads: list[ET.Element],
    junctions: list[ET.Element],
    *,
    map_roads: list[ET.Element],
    connector_roads: list[ET.Element],
    route_roads: list[ET.Element],
    corridor_roads: list[ET.Element],
    road_link_count: int,
    road_link_gaps: list[float],
    lane_link_count: int,
    junction_connection_count: int,
    junction_endpoint_gaps: list[float],
    route_path_roads: list[ET.Element] | None = None,
) -> dict[str, Any]:
    components = _road_components(roads, junctions)
    corridor_ids = {road.attrib.get("id") for road in corridor_roads}
    road_ids = {road.attrib.get("id") for road in roads}
    route_connector_ids = {
        road.attrib.get("id")
        for road in connector_roads
        if road.attrib.get("name", "").startswith("inferred_connector_route_")
    }
    map_network_ids = {
        road.attrib.get("id") for road in (*map_roads, *connector_roads)
        if road.attrib.get("id") not in route_connector_ids
    }
    route_network_roads = route_path_roads or route_roads
    route_ids = {road.attrib.get("id") for road in route_network_roads}
    all_network_components = [
        component
        for component in components
        if any(
            node in road_ids and node not in corridor_ids for node in component
        )
    ]
    map_components = _road_components(
        roads,
        junctions,
        included_road_ids=map_network_ids,
    )
    map_network_components = [
        component
        for component in map_components
        if any(node in map_network_ids for node in component)
    ]
    route_network_components = [
        component for component in components if any(node in route_ids for node in component)
    ]
    network_sizes = [
        sum(node in map_network_ids for node in component)
        for component in map_network_components
    ]
    all_network_sizes = [
        sum(node in road_ids and node not in corridor_ids for node in component)
        for component in all_network_components
    ]
    route_sizes = [
        sum(node in route_ids for node in component)
        for component in route_network_components
    ]
    isolated_map_lane_road_ids = sorted(
        node
        for component, size in zip(map_network_components, network_sizes)
        if size == 1
        for node in component
        if node in {road.attrib.get("id") for road in map_roads}
    )
    boundary_road_ids = {
        road.attrib.get("id")
        for road in map_roads
        if _user_data_properties(road).get("topology_boundary") == "true"
    }
    isolated_boundary_ids = sorted(
        road_id
        for road_id in isolated_map_lane_road_ids
        if road_id in boundary_road_ids
    )
    isolated_unclassified_ids = sorted(
        road_id
        for road_id in isolated_map_lane_road_ids
        if road_id not in boundary_road_ids
    )
    supported_connector_evidence = {
        "endpoint_heading",
        "source_edge_line_continuity",
        "source_intersection_region",
    }
    connector_evidence_counts: dict[str, int] = {}
    connector_evidence_unclassified_ids = []
    for road in connector_roads:
        road_id = str(road.attrib.get("id"))
        if road_id in route_connector_ids:
            continue
        evidence = _user_data_properties(road).get("topology_evidence", "")
        if evidence not in supported_connector_evidence:
            connector_evidence_unclassified_ids.append(road_id)
            continue
        connector_evidence_counts[evidence] = (
            connector_evidence_counts.get(evidence, 0) + 1
        )
    return {
        "road_count": len(roads),
        "junction_count": len(junctions),
        "road_link_count": road_link_count,
        "road_link_endpoint_gap_count": len(road_link_gaps),
        "road_link_endpoint_gap_max_m": max(road_link_gaps, default=0.0),
        "lane_link_count": lane_link_count,
        "junction_connection_count": junction_connection_count,
        "junction_endpoint_gap_count": len(junction_endpoint_gaps),
        "junction_endpoint_gap_max_m": max(junction_endpoint_gaps, default=0.0),
        "map_lane_road_count": len(map_roads),
        "connector_road_count": len(connector_roads),
        "map_connector_road_count": len(connector_roads) - len(route_connector_ids),
        "route_connector_road_count": len(route_connector_ids),
        "route_inference_road_count": len(route_roads),
        "route_path_road_count": len(route_network_roads),
        "ego_corridor_road_count": len(corridor_roads),
        "network_component_count": len(map_network_components),
        "largest_network_component_road_count": max(network_sizes, default=0),
        "isolated_network_road_count": sum(size == 1 for size in network_sizes),
        "all_network_component_count": len(all_network_components),
        "all_network_largest_component_road_count": max(all_network_sizes, default=0),
        "route_network_component_count": len(route_network_components),
        "route_network_road_count": len(route_ids),
        "route_network_largest_component_road_count": max(route_sizes, default=0),
        "isolated_map_lane_road_ids": isolated_map_lane_road_ids,
        "isolated_map_lane_boundary_ids": isolated_boundary_ids,
        "isolated_map_lane_unclassified_ids": isolated_unclassified_ids,
        "isolated_map_lane_boundary_count": len(isolated_boundary_ids),
        "isolated_map_lane_unclassified_count": len(isolated_unclassified_ids),
        "isolated_map_lane_boundary_status": (
            "passed" if not isolated_unclassified_ids else "failed"
        ),
        "map_connector_evidence_counts": connector_evidence_counts,
        "map_connector_evidence_unclassified_ids": sorted(
            connector_evidence_unclassified_ids
        ),
        "map_connector_evidence_unclassified_count": len(
            connector_evidence_unclassified_ids
        ),
        "map_connector_evidence_status": (
            "passed"
            if not connector_evidence_unclassified_ids
            else "missing"
        ),
    }


def _user_data_properties(road: ET.Element) -> dict[str, str]:
    return {
        str(property_node.attrib["name"]): str(property_node.attrib.get("value", ""))
        for property_node in road.findall("./userData/property")
        if property_node.attrib.get("name")
    }


def _road_components(
    roads: list[ET.Element],
    junctions: list[ET.Element],
    *,
    included_road_ids: set[str] | None = None,
) -> list[set[str]]:
    adjacency: dict[str, set[str]] = {
        str(road.attrib.get("id")): set()
        for road in roads
        if included_road_ids is None
        or str(road.attrib.get("id")) in included_road_ids
    }
    adjacency.update({f"junction:{junction.attrib.get('id')}": set() for junction in junctions})

    def connect(left: str, right: str) -> None:
        if left not in adjacency or right not in adjacency:
            return
        adjacency[left].add(right)
        adjacency[right].add(left)

    for road in roads:
        road_id = str(road.attrib.get("id"))
        for reference in road.findall("./link/*"):
            element_type = reference.attrib.get("elementType")
            target_id = reference.attrib.get("elementId")
            if element_type == "road":
                connect(road_id, str(target_id))
            elif element_type == "junction":
                connect(road_id, f"junction:{target_id}")
    for junction in junctions:
        junction_id = f"junction:{junction.attrib.get('id')}"
        for connection in junction.findall("./connection"):
            connect(junction_id, str(connection.attrib.get("incomingRoad")))
            connect(junction_id, str(connection.attrib.get("connectingRoad")))

    components: list[set[str]] = []
    seen: set[str] = set()
    for node in adjacency:
        if node in seen:
            continue
        stack = [node]
        seen.add(node)
        component: set[str] = set()
        while stack:
            current = stack.pop()
            component.add(current)
            for neighbor in adjacency[current]:
                if neighbor not in seen:
                    seen.add(neighbor)
                    stack.append(neighbor)
        components.append(component)
    return components


def main(argv: list[str] | None = None) -> int:
    import argparse
    import json

    parser = argparse.ArgumentParser(description="Validate an OpenDRIVE topology contract.")
    parser.add_argument("--xodr", required=True, type=Path)
    parser.add_argument(
        "--expected-sha256",
        help="require the exact artifact SHA-256 before accepting the topology",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--no-map-topology", action="store_true")
    parser.add_argument("--require-junction-topology", action="store_true")
    parser.add_argument("--require-route-chain", action="store_true")
    parser.add_argument(
        "--require-route-map-integration",
        action="store_true",
        help="require inferred route roads to share a junction component with map lanes",
    )
    parser.add_argument(
        "--require-route-source-audit",
        action="store_true",
        help="require every inferred route road to declare its synthetic source-gap metadata",
    )
    parser.add_argument("--scenario-ir", type=Path)
    parser.add_argument("--require-ego-route-coverage", action="store_true")
    parser.add_argument("--max-ego-route-distance-m", type=float, default=1.0)
    parser.add_argument(
        "--max-ego-route-heading-error-deg", type=float, default=45.0
    )
    parser.add_argument("--expected-ego-corridor-count", type=int)
    parser.add_argument("--max-network-components", type=int)
    parser.add_argument(
        "--require-boundary-audit",
        action="store_true",
        help="require every isolated map lane road to carry source-boundary metadata",
    )
    parser.add_argument(
        "--require-connector-evidence",
        action="store_true",
        help="require map connector roads to declare supported topology evidence",
    )
    args = parser.parse_args(argv)
    summary = validate_topology_artifact(
        args.xodr,
        expected_sha256=args.expected_sha256,
        expected_ego_corridor_count=args.expected_ego_corridor_count,
        require_map_topology=not args.no_map_topology,
        require_junction_topology=args.require_junction_topology,
        require_route_chain=args.require_route_chain,
        require_route_map_integration=args.require_route_map_integration,
        require_route_source_audit=args.require_route_source_audit,
        scenario_ir_path=args.scenario_ir,
        require_ego_route_coverage=args.require_ego_route_coverage,
        max_ego_route_distance_m=args.max_ego_route_distance_m,
        max_ego_route_heading_error_deg=args.max_ego_route_heading_error_deg,
        max_network_components=args.max_network_components,
        require_boundary_audit=args.require_boundary_audit,
        require_connector_evidence=args.require_connector_evidence,
    )
    rendered = json.dumps(summary, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
