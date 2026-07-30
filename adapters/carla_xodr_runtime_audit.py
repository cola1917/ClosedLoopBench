"""Runtime checks for a generated OpenDRIVE map loaded by CARLA.

The XML contract proves that links and junction records are internally
consistent.  This module consumes CARLA waypoint observations to verify the
separate runtime properties that XML alone cannot prove: route samples remain
on a driving lane, consecutive samples are reachable through CARLA's waypoint
graph, and road/lane changes occur through an explicit junction branch.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import xml.etree.ElementTree as ET


def audit_waypoint_samples(
    samples: Iterable[Mapping[str, Any]],
    *,
    lateral_tolerance_m: float = 0.25,
    continuity_tolerance_m: float = 3.0,
    route_contract: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Audit normalized CARLA waypoint samples without importing CARLA.

    Each sample contains a CARLA-frame ``expected`` point, a normalized
    ``waypoint`` record (or ``None``), the normalized ``next_waypoints``
    returned by ``Waypoint.next``, and the distance to the next recorded
    route sample as ``step_distance_m``.  Keeping the decision logic here
    makes the remote command testable with deterministic fixtures.
    """

    if lateral_tolerance_m < 0.0:
        raise ValueError("lateral_tolerance_m must be non-negative")
    if continuity_tolerance_m <= 0.0:
        raise ValueError("continuity_tolerance_m must be positive")

    rows = [dict(sample) for sample in samples]
    route_sequence, route_edges = _normalise_route_contract(route_contract)
    route_order = {
        road_id: index for index, road_id in enumerate(route_sequence or ())
    }
    lane_failures: list[dict[str, Any]] = []
    missing_waypoints: list[int] = []
    lane_records: list[dict[str, Any]] = []
    route_failures: list[dict[str, Any]] = []
    observed_route_ids: list[str] = []

    for index, sample in enumerate(rows):
        sample_index = int(sample.get("index", index))
        expected = _point(sample.get("expected"))
        waypoint = sample.get("waypoint")
        if waypoint is None:
            missing_waypoints.append(sample_index)
            lane_failures.append(
                {"index": sample_index, "issues": ["missing_waypoint"]}
            )
            if route_sequence is not None:
                route_failures.append(
                    {"index": sample_index, "issues": ["missing_route_waypoint"]}
                )
            continue
        issues: list[str] = []
        actual = _point(waypoint.get("location"))
        if expected is None or actual is None:
            issues.append("missing_waypoint_position")
            center_distance = None
        else:
            center_distance = math.dist(expected, actual)
            width = _finite_number(waypoint.get("lane_width_m"))
            if width is None or width <= 0.0:
                issues.append("invalid_lane_width")
            elif center_distance > width / 2.0 + lateral_tolerance_m:
                issues.append("outside_lane_width")
        if waypoint.get("is_driving_lane") is not True:
            issues.append("non_driving_lane")
        for field in ("road_id", "section_id", "lane_id"):
            if waypoint.get(field) is None:
                issues.append(f"missing_{field}")
        if issues:
            lane_failures.append({"index": sample_index, "issues": issues})
        if route_sequence is not None:
            road_id = waypoint.get("road_id")
            route_issues: list[str] = []
            if road_id is None:
                route_issues.append("missing_route_road_id")
            else:
                normalized_road_id = str(road_id)
                observed_route_ids.append(normalized_road_id)
                if normalized_road_id not in route_order:
                    route_issues.append("waypoint_road_not_in_declared_route")
            if route_issues:
                route_failures.append(
                    {"index": sample_index, "issues": route_issues}
                )
        lane_records.append(
            {
                "index": sample_index,
                "road_id": waypoint.get("road_id"),
                "section_id": waypoint.get("section_id"),
                "lane_id": waypoint.get("lane_id"),
                "is_junction": bool(waypoint.get("is_junction", False)),
                "center_distance_m": center_distance,
                "lane_width_m": waypoint.get("lane_width_m"),
                "inside_lane": not issues,
            }
        )

    continuity_failures: list[dict[str, Any]] = []
    branch_failures: list[dict[str, Any]] = []
    branch_transitions: list[dict[str, Any]] = []
    route_transition_count = 0
    transition_count = 0
    for previous, current in zip(rows, rows[1:]):
        previous_index = int(previous.get("index", transition_count))
        current_index = int(current.get("index", previous_index + 1))
        previous_waypoint = previous.get("waypoint")
        current_waypoint = current.get("waypoint")
        if previous_waypoint is None or current_waypoint is None:
            if route_sequence is not None:
                route_failures.append(
                    {
                        "from_index": previous_index,
                        "to_index": current_index,
                        "issue": "route_transition_missing_waypoint",
                    }
                )
            continue
        transition_count += 1
        candidates = [
            candidate
            for candidate in (previous.get("next_waypoints") or [])
            if isinstance(candidate, Mapping)
        ]
        target = _point(current_waypoint.get("location"))
        candidate_distances = [
            (math.dist(target, point), candidate)
            for candidate in candidates
            if (point := _point(candidate.get("location"))) is not None
            and target is not None
        ]
        step_distance = _finite_number(previous.get("step_distance_m")) or 0.0
        allowed_distance = continuity_tolerance_m + max(0.0, step_distance * 0.25)
        best = min(candidate_distances, key=lambda item: item[0]) if candidate_distances else None
        if best is None or best[0] > allowed_distance:
            continuity_failures.append(
                {
                    "from_index": previous_index,
                    "to_index": current_index,
                    "candidate_count": len(candidates),
                    "nearest_candidate_distance_m": best[0] if best else None,
                    "allowed_distance_m": allowed_distance,
                    "issue": "waypoint_next_discontinuity",
                }
            )
            continue

        declared_junction = False
        if route_sequence is not None:
            route_transition_count += 1
            transition = _route_transition(
                previous_waypoint,
                current_waypoint,
                route_order,
                route_edges,
            )
            if transition is None:
                route_failures.append(
                    {
                        "from_index": previous_index,
                        "to_index": current_index,
                        "from_road_id": previous_waypoint.get("road_id"),
                        "to_road_id": current_waypoint.get("road_id"),
                        "issue": "waypoint_route_transition_not_declared",
                    }
                )
            else:
                declared_junction = bool(transition.get("through_junction"))

        previous_identity = _waypoint_identity(previous_waypoint)
        current_identity = _waypoint_identity(current_waypoint)
        changed = previous_identity != current_identity
        if not changed:
            continue

        is_junction = bool(
            previous_waypoint.get("is_junction", False)
            or current_waypoint.get("is_junction", False)
        )
        candidate_identities = {
            _waypoint_identity(candidate) for _, candidate in candidate_distances
        }
        candidate_branch = len(candidate_identities) > 1
        branch_row = {
            "from_index": previous_index,
            "to_index": current_index,
            "from_waypoint": previous_identity,
            "to_waypoint": current_identity,
            "candidate_count": len(candidates),
            "candidate_branch": candidate_branch,
            "through_junction": is_junction,
        }
        branch_transitions.append(branch_row)
        if not is_junction and not candidate_branch and not declared_junction:
            branch_row = dict(branch_row)
            branch_row["issue"] = "road_or_lane_change_outside_junction"
            branch_failures.append(branch_row)

    if route_sequence is not None:
        if not observed_route_ids:
            route_failures.append(
                {"issue": "no_observed_route_waypoint"}
            )
        else:
            if observed_route_ids[0] != route_sequence[0]:
                route_failures.append(
                    {
                        "issue": "route_start_road_mismatch",
                        "expected": route_sequence[0],
                        "actual": observed_route_ids[0],
                    }
                )
            if observed_route_ids[-1] != route_sequence[-1]:
                route_failures.append(
                    {
                        "issue": "route_end_road_mismatch",
                        "expected": route_sequence[-1],
                        "actual": observed_route_ids[-1],
                    }
                )

    inside_count = sum(1 for record in lane_records if record["inside_lane"])
    lane_status = "passed" if not lane_failures else "failed"
    continuity_status = "passed" if not continuity_failures else "failed"
    branch_status = "passed" if not branch_failures else "failed"
    route_status = (
        "not_checked"
        if route_sequence is None
        else ("passed" if not route_failures else "failed")
    )
    statuses = (lane_status, continuity_status, branch_status)
    if route_sequence is not None:
        statuses += (route_status,)
    return {
        "schema_version": "carla_xodr_runtime_audit.v1",
        "status": (
            "passed"
            if all(status == "passed" for status in statuses)
            else "failed"
        ),
        "sample_count": len(rows),
        "lane_membership": {
            "status": lane_status,
            "waypoint_count": len(lane_records),
            "missing_waypoint_indices": missing_waypoints,
            "inside_lane_count": inside_count,
            "inside_lane_fraction": (
                inside_count / len(lane_records) if lane_records else 0.0
            ),
            "failures": lane_failures,
            "samples": lane_records,
        },
        "waypoint_continuity": {
            "status": continuity_status,
            "transition_count": transition_count,
            "failures": continuity_failures,
        },
        "route_branch": {
            "status": branch_status,
            "transition_count": len(branch_transitions),
            "branch_transitions": branch_transitions,
            "failures": branch_failures,
        },
        "route_topology": {
            "status": route_status,
            "declared_road_sequence": route_sequence or [],
            "observed_road_sequence": observed_route_ids,
            "transition_count": route_transition_count,
            "failures": route_failures,
        },
    }


def load_route_topology_contract(path: Path) -> dict[str, Any]:
    """Read the declared Ego route and its junction bridges from an XODR.

    The route metadata identifies the roads sampled by Scenario IR.  CARLA
    also exposes junction connector roads as waypoints, so the runtime
    sequence expands each declared path edge with the connector selected by
    the corresponding OpenDRIVE junction connection.
    """

    source = Path(path).expanduser().resolve()
    try:
        root = ET.parse(source).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ValueError(f"cannot parse XODR route contract: {source}") from exc
    if root.tag != "OpenDRIVE":
        raise ValueError(f"not an OpenDRIVE document: {source}")

    roads = root.findall("./road")
    road_by_id = {str(road.attrib.get("id")): road for road in roads}
    path_roads = []
    for road in roads:
        properties = {
            str(node.attrib["name"]): str(node.attrib.get("value", ""))
            for node in road.findall("./userData/property")
            if node.attrib.get("name")
        }
        if "route_path_order" not in properties:
            continue
        try:
            order = int(properties["route_path_order"])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"route road {road.attrib.get('id')} has invalid route_path_order"
            ) from exc
        path_roads.append((order, str(road.attrib.get("id")), road))
    path_roads.sort(key=lambda item: (item[0], item[1]))
    path_ids = [road_id for _, road_id, _ in path_roads]
    if len(path_ids) < 2:
        raise ValueError(
            "XODR route contract requires at least two declared route-path roads"
        )
    if path_ids != list(dict.fromkeys(path_ids)):
        raise ValueError("XODR route contract contains duplicate route-path road ids")

    junction_by_id = {
        str(junction.attrib.get("id")): junction
        for junction in root.findall("./junction")
    }
    sequence = [path_ids[0]]
    transitions: list[dict[str, Any]] = []

    for source_id, target_id in zip(path_ids, path_ids[1:]):
        steps = _resolve_route_edge(
            source_id,
            target_id,
            road_by_id=road_by_id,
            junction_by_id=junction_by_id,
        )
        for step in steps:
            from_id = str(step["from_road_id"])
            to_id = str(step["to_road_id"])
            if sequence[-1] != from_id:
                raise ValueError(
                    "XODR route bridge is not contiguous: "
                    f"{sequence[-1]} -> {from_id}"
                )
            sequence.append(to_id)
            transitions.append(step)

    return {
        "schema_version": "carla_xodr_route_topology.v1",
        "xodr_path": str(source),
        "route_path_road_ids": path_ids,
        "road_sequence": sequence,
        "transition_edges": transitions,
        "start_road_id": sequence[0],
        "end_road_id": sequence[-1],
    }


def _resolve_route_edge(
    source_id: str,
    target_id: str,
    *,
    road_by_id: Mapping[str, Any],
    junction_by_id: Mapping[str, Any],
) -> list[dict[str, Any]]:
    source = road_by_id.get(source_id)
    if source is None:
        raise ValueError(f"XODR route references missing source road {source_id}")
    successor = source.find("./link/successor")
    if successor is None:
        raise ValueError(
            f"XODR route edge {source_id}->{target_id} has no source successor"
        )
    element_type = successor.attrib.get("elementType")
    element_id = str(successor.attrib.get("elementId"))
    if element_type == "road" and element_id == target_id:
        return [
            {
                "from_road_id": source_id,
                "to_road_id": target_id,
                "through_junction": False,
                "junction_id": None,
            }
        ]
    if element_type != "junction":
        raise ValueError(
            f"XODR route edge {source_id}->{target_id} has unsupported successor "
            f"{element_type}:{element_id}"
        )
    junction = junction_by_id.get(element_id)
    if junction is None:
        raise ValueError(
            f"XODR route edge {source_id}->{target_id} references missing junction {element_id}"
        )
    for connection in junction.findall("./connection"):
        if str(connection.attrib.get("incomingRoad")) != source_id:
            continue
        connector_id = str(connection.attrib.get("connectingRoad"))
        connector = road_by_id.get(connector_id)
        if connector is None:
            continue
        connector_successor = connector.find("./link/successor")
        connector_target = (
            str(connector_successor.attrib.get("elementId"))
            if connector_successor is not None
            and connector_successor.attrib.get("elementType") == "road"
            else None
        )
        if connector_id == target_id:
            return [
                {
                    "from_road_id": source_id,
                    "to_road_id": target_id,
                    "through_junction": True,
                    "junction_id": element_id,
                }
            ]
        if connector_target == target_id:
            return [
                {
                    "from_road_id": source_id,
                    "to_road_id": connector_id,
                    "through_junction": True,
                    "junction_id": element_id,
                },
                {
                    "from_road_id": connector_id,
                    "to_road_id": target_id,
                    "through_junction": True,
                    "junction_id": element_id,
                },
            ]
    raise ValueError(
        "XODR junction does not declare route movement "
        f"{source_id}->{target_id} at junction {element_id}"
    )


def _normalise_route_contract(
    contract: Mapping[str, Any] | None,
) -> tuple[list[str] | None, dict[tuple[str, str], dict[str, Any]]]:
    if contract is None:
        return None, {}
    raw_sequence = contract.get("road_sequence")
    if not isinstance(raw_sequence, list) or len(raw_sequence) < 2:
        raise ValueError("route contract requires a road_sequence with at least two roads")
    sequence = [str(value) for value in raw_sequence]
    if any(not value for value in sequence) or len(sequence) != len(set(sequence)):
        raise ValueError("route contract road_sequence must contain unique non-empty ids")
    raw_edges = contract.get("transition_edges")
    if not isinstance(raw_edges, list):
        raise ValueError("route contract requires transition_edges")
    edges: dict[tuple[str, str], dict[str, Any]] = {}
    for raw_edge in raw_edges:
        if not isinstance(raw_edge, Mapping):
            raise ValueError("route contract transition edge must be an object")
        source_id = str(raw_edge.get("from_road_id", ""))
        target_id = str(raw_edge.get("to_road_id", ""))
        if not source_id or not target_id:
            raise ValueError("route contract transition edge has an empty road id")
        if (source_id, target_id) in edges:
            raise ValueError(
                f"route contract contains duplicate transition {source_id}->{target_id}"
            )
        edges[(source_id, target_id)] = dict(raw_edge)
    return sequence, edges


def _route_transition(
    previous_waypoint: Mapping[str, Any],
    current_waypoint: Mapping[str, Any],
    route_order: Mapping[str, int],
    route_edges: Mapping[tuple[str, str], Mapping[str, Any]],
) -> dict[str, Any] | None:
    previous_id = str(previous_waypoint.get("road_id"))
    current_id = str(current_waypoint.get("road_id"))
    previous_index = route_order.get(previous_id)
    current_index = route_order.get(current_id)
    if previous_index is None or current_index is None or current_index < previous_index:
        return None
    if previous_id == current_id:
        return {"through_junction": False}
    direct = route_edges.get((previous_id, current_id))
    if direct is not None:
        return dict(direct)
    # Consecutive Scenario IR samples can straddle an unobserved connector.
    # Accept only a contiguous forward walk through the declared XODR route.
    through_junction = False
    for index in range(previous_index, current_index):
        edge = route_edges.get((
            _route_id_at(route_order, index),
            _route_id_at(route_order, index + 1),
        ))
        if edge is None:
            return None
        through_junction = through_junction or bool(edge.get("through_junction"))
    return {"through_junction": through_junction}


def _route_id_at(route_order: Mapping[str, int], index: int) -> str:
    for road_id, road_index in route_order.items():
        if road_index == index:
            return road_id
    raise ValueError(f"route contract has no road at sequence index {index}")


def normalize_waypoint(
    waypoint: Any,
    *,
    carla_module: Any,
) -> dict[str, Any] | None:
    """Convert a CARLA ``Waypoint`` to the serializable audit form."""

    if waypoint is None:
        return None
    transform = getattr(waypoint, "transform", None)
    location = getattr(transform, "location", None)
    rotation = getattr(transform, "rotation", None)
    if location is None:
        return None
    lane_type = getattr(waypoint, "lane_type", None)
    driving_type = getattr(getattr(carla_module, "LaneType", None), "Driving", None)
    is_driving = lane_type == driving_type or "driving" in str(lane_type).lower()
    return {
        "location": {
            "x": float(getattr(location, "x")),
            "y": float(getattr(location, "y")),
            "z": float(getattr(location, "z", 0.0)),
        },
        "yaw_deg": float(getattr(rotation, "yaw", 0.0)),
        "road_id": int(getattr(waypoint, "road_id")),
        "section_id": int(getattr(waypoint, "section_id")),
        "lane_id": int(getattr(waypoint, "lane_id")),
        "lane_type": str(lane_type),
        "is_driving_lane": bool(is_driving),
        "lane_width_m": float(getattr(waypoint, "lane_width")),
        "is_junction": bool(getattr(waypoint, "is_junction", False)),
    }


def canonical_to_carla_point(state: Mapping[str, Any]) -> dict[str, float]:
    """Reflect the canonical y-left scene frame at the CARLA API boundary."""

    return {
        "x": float(state["x"]),
        "y": -float(state["y"]),
        "z": float(state.get("z", 0.0)),
    }


def canonical_to_carla_yaw(state: Mapping[str, Any]) -> float:
    return _normalize_degrees(-float(state.get("yaw", 0.0)))


def _point(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, Mapping):
        return None
    try:
        return float(value["x"]), float(value["y"]), float(value.get("z", 0.0))
    except (KeyError, TypeError, ValueError):
        return None


def _waypoint_identity(value: Mapping[str, Any]) -> tuple[Any, Any, Any]:
    return value.get("road_id"), value.get("section_id"), value.get("lane_id")


def _finite_number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _normalize_degrees(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0
