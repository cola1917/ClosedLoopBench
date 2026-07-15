from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Iterable


class NuScenesMapError(ValueError):
    """Raised when a nuScenes map cannot provide a usable local lane graph."""


def load_nuscenes_map(dataroot: str | Path, location: str) -> dict[str, Any]:
    path = Path(dataroot) / "maps" / f"{location}.json"
    if not path.is_file():
        raise FileNotFoundError(f"nuScenes map not found: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise NuScenesMapError(f"nuScenes map must contain a JSON object: {path}")
    return value


def build_local_opendrive_xml(
    scenario_ir: dict[str, Any],
    map_data: dict[str, Any],
    *,
    radius_m: float = 35.0,
    connection_tolerance_m: float = 2.0,
) -> str:
    """Build a deliberately limited OpenDRIVE 1.4 map around scene tracks.

    Each nuScenes lane polygon becomes one OpenDRIVE road with one driving
    lane. Curves are represented as short line geometries; topology is only
    linked where one selected lane end has a single unambiguous nearby start.
    """

    if radius_m <= 0:
        raise ValueError("radius_m must be positive")
    frame = scenario_ir.get("coordinate_frame", {})
    origin = frame.get("origin_global_translation")
    yaw_deg = frame.get("origin_global_yaw_deg")
    if not isinstance(origin, list) or len(origin) < 2 or yaw_deg is None:
        raise NuScenesMapError("Scenario IR lacks the global-to-local coordinate transform")

    nodes = _index(map_data.get("node", []))
    lines = _index(map_data.get("line", []))
    polygons = _index(map_data.get("polygon", []))
    track_points = _track_points(scenario_ir)
    if not track_points:
        raise NuScenesMapError("Scenario IR has no Ego or Actor trajectory points")

    lanes = []
    for record in map_data.get("lane", []):
        if str(record.get("lane_type", "")).upper() != "CAR":
            continue
        try:
            global_points, width = _lane_centerline(record, nodes, lines, polygons)
        except NuScenesMapError:
            continue
        local_points = [_to_local(point, origin, float(yaw_deg)) for point in global_points]
        if _polyline_distance_to_points(local_points, track_points) <= radius_m:
            lanes.append(
                {
                    "token": str(record["token"]),
                    "points": _deduplicate(local_points),
                    "width": min(max(width, 2.0), 8.0),
                }
            )

    lanes = [lane for lane in lanes if _polyline_length(lane["points"]) > 0.1]
    lanes.sort(key=lambda lane: lane["token"])
    if not lanes:
        raise NuScenesMapError(
            f"no usable nuScenes lanes found within {radius_m:.1f} m of scene trajectories"
        )

    links = _unambiguous_links(lanes, connection_tolerance_m)
    root = ET.Element("OpenDRIVE")
    all_points = [point for lane in lanes for point in lane["points"]]
    xs, ys = [p[0] for p in all_points], [p[1] for p in all_points]
    ET.SubElement(
        root,
        "header",
        {
            "revMajor": "1",
            "revMinor": "4",
            "name": f"nuscenes_local_{scenario_ir.get('scenario_id', 'scene')}",
            "version": "1.00",
            "date": "2026-07-13T00:00:00",
            "north": _fmt(max(ys)),
            "south": _fmt(min(ys)),
            "east": _fmt(max(xs)),
            "west": _fmt(min(xs)),
        },
    )
    for road_id, lane in enumerate(lanes, start=1):
        _add_road(root, road_id, lane, links)

    _indent(root)
    return ET.tostring(root, encoding="unicode")


def _add_road(
    root: ET.Element,
    road_id: int,
    lane: dict[str, Any],
    links: dict[str, dict[str, str]],
) -> None:
    points = lane["points"]
    road = ET.SubElement(
        root,
        "road",
        {
            "name": f"nuscenes_lane_{lane['token']}",
            "length": _fmt(_polyline_length(points)),
            "id": str(road_id),
            "junction": "-1",
        },
    )
    token_links = links.get(lane["token"], {})
    if token_links:
        link = ET.SubElement(road, "link")
        if "predecessor" in token_links:
            ET.SubElement(
                link,
                "predecessor",
                {
                    "elementType": "road",
                    "elementId": token_links["predecessor"],
                    "contactPoint": "end",
                },
            )
        if "successor" in token_links:
            ET.SubElement(
                link,
                "successor",
                {
                    "elementType": "road",
                    "elementId": token_links["successor"],
                    "contactPoint": "start",
                },
            )

    plan_view = ET.SubElement(road, "planView")
    s = 0.0
    for first, second in zip(points, points[1:]):
        length = math.dist(first, second)
        if length <= 1e-6:
            continue
        geometry = ET.SubElement(
            plan_view,
            "geometry",
            {
                "s": _fmt(s),
                "x": _fmt(first[0]),
                "y": _fmt(first[1]),
                "hdg": _fmt(math.atan2(second[1] - first[1], second[0] - first[0])),
                "length": _fmt(length),
            },
        )
        ET.SubElement(geometry, "line")
        s += length

    lanes = ET.SubElement(road, "lanes")
    ET.SubElement(
        lanes,
        "laneOffset",
        {"s": "0.000000", "a": _fmt(lane["width"] / 2.0), "b": "0", "c": "0", "d": "0"},
    )
    section = ET.SubElement(lanes, "laneSection", {"s": "0.000000"})
    center = ET.SubElement(section, "center")
    center_lane = ET.SubElement(center, "lane", {"id": "0", "type": "none", "level": "false"})
    ET.SubElement(
        center_lane,
        "roadMark",
        {
            "sOffset": "0",
            "type": "none",
            "weight": "standard",
            "color": "standard",
            "width": "0",
        },
    )
    right = ET.SubElement(section, "right")
    driving = ET.SubElement(right, "lane", {"id": "-1", "type": "driving", "level": "false"})
    if token_links:
        lane_link = ET.SubElement(driving, "link")
        if "predecessor" in token_links:
            ET.SubElement(lane_link, "predecessor", {"id": "-1"})
        if "successor" in token_links:
            ET.SubElement(lane_link, "successor", {"id": "-1"})
    ET.SubElement(
        driving,
        "width",
        {"sOffset": "0.000000", "a": _fmt(lane["width"]), "b": "0", "c": "0", "d": "0"},
    )
    ET.SubElement(
        driving,
        "roadMark",
        {
            "sOffset": "0",
            "type": "solid",
            "weight": "standard",
            "color": "standard",
            "width": "0.12",
        },
    )


def _lane_centerline(
    lane: dict[str, Any],
    nodes: dict[str, dict[str, Any]],
    lines: dict[str, dict[str, Any]],
    polygons: dict[str, dict[str, Any]],
) -> tuple[list[tuple[float, float]], float]:
    polygon = polygons.get(str(lane.get("polygon_token", "")))
    from_line = lines.get(str(lane.get("from_edge_line_token", "")))
    to_line = lines.get(str(lane.get("to_edge_line_token", "")))
    if not polygon or not from_line or not to_line:
        raise NuScenesMapError("lane is missing polygon or endpoint edges")
    exterior = [str(token) for token in polygon.get("exterior_node_tokens", [])]
    from_tokens = [str(token) for token in from_line.get("node_tokens", [])]
    to_tokens = [str(token) for token in to_line.get("node_tokens", [])]
    if len(exterior) < 4 or len(from_tokens) != 2 or len(to_tokens) != 2:
        raise NuScenesMapError("lane boundary is incomplete")

    adjacency: dict[str, list[str]] = {token: [] for token in exterior}
    for left, right in zip(exterior, exterior[1:] + exterior[:1]):
        if {left, right} in ({*from_tokens}, {*to_tokens}):
            continue
        adjacency[left].append(right)
        adjacency[right].append(left)

    boundaries = []
    for start in from_tokens:
        path = [start]
        previous = None
        current = start
        while current not in to_tokens:
            candidates = [token for token in adjacency.get(current, []) if token != previous]
            if len(candidates) != 1:
                raise NuScenesMapError("lane polygon does not split into two simple boundaries")
            previous, current = current, candidates[0]
            if current in path:
                raise NuScenesMapError("cycle in lane boundary")
            path.append(current)
        boundaries.append([_node_xy(nodes, token) for token in path])

    samples = max(len(boundaries[0]), len(boundaries[1]), 3)
    left = _resample(boundaries[0], samples)
    right = _resample(boundaries[1], samples)
    center = [((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0) for a, b in zip(left, right)]
    width = sum(math.dist(a, b) for a, b in zip(left, right)) / samples
    return center, width


def _node_xy(nodes: dict[str, dict[str, Any]], token: str) -> tuple[float, float]:
    node = nodes.get(token)
    if node is None:
        raise NuScenesMapError(f"map references missing node: {token}")
    return float(node["x"]), float(node["y"])


def _resample(points: list[tuple[float, float]], count: int) -> list[tuple[float, float]]:
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.dist(first, second))
    if cumulative[-1] <= 1e-9:
        return [points[0]] * count
    result = []
    for index in range(count):
        target = cumulative[-1] * index / (count - 1)
        if index == count - 1:
            result.append(points[-1])
            continue
        segment = next(
            (
                i
                for i in range(len(cumulative) - 1)
                if cumulative[i + 1] + 1e-9 >= target
            ),
            len(cumulative) - 2,
        )
        span = cumulative[segment + 1] - cumulative[segment]
        ratio = 0.0 if span <= 1e-9 else (target - cumulative[segment]) / span
        first, second = points[segment], points[segment + 1]
        result.append(
            (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            )
        )
    return result


def _to_local(point: tuple[float, float], origin: list[Any], yaw_deg: float) -> tuple[float, float]:
    dx, dy = point[0] - float(origin[0]), point[1] - float(origin[1])
    yaw = math.radians(yaw_deg)
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy


def _track_points(scenario_ir: dict[str, Any]) -> list[tuple[float, float]]:
    tracks = [scenario_ir.get("ego", {}).get("reference_trajectory", [])]
    tracks.extend(actor.get("reference_trajectory", []) for actor in scenario_ir.get("actors", []))
    return [
        (float(state["x"]), float(state["y"]))
        for track in tracks
        for state in track
        if "x" in state and "y" in state
    ]


def _polyline_distance_to_points(
    polyline: list[tuple[float, float]], points: list[tuple[float, float]]
) -> float:
    return min(
        _point_segment_distance(point, first, second)
        for point in points
        for first, second in zip(polyline, polyline[1:])
    )


def _point_segment_distance(point, first, second) -> float:
    dx, dy = second[0] - first[0], second[1] - first[1]
    denominator = dx * dx + dy * dy
    if denominator <= 1e-12:
        return math.dist(point, first)
    ratio = max(
        0.0,
        min(
            1.0,
            ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) / denominator,
        ),
    )
    projection = (first[0] + ratio * dx, first[1] + ratio * dy)
    return math.dist(point, projection)


def _unambiguous_links(lanes: list[dict[str, Any]], tolerance: float) -> dict[str, dict[str, str]]:
    road_ids = {lane["token"]: str(index) for index, lane in enumerate(lanes, start=1)}
    successor_candidates: dict[str, list[str]] = {}
    predecessor_candidates: dict[str, list[str]] = {lane["token"]: [] for lane in lanes}
    for lane in lanes:
        candidates = [
            other["token"]
            for other in lanes
            if other is not lane and math.dist(lane["points"][-1], other["points"][0]) <= tolerance
        ]
        successor_candidates[lane["token"]] = candidates
        for candidate in candidates:
            predecessor_candidates[candidate].append(lane["token"])
    links: dict[str, dict[str, str]] = {}
    for lane in lanes:
        token = lane["token"]
        if len(successor_candidates[token]) == 1:
            successor = successor_candidates[token][0]
            if len(predecessor_candidates[successor]) == 1:
                links.setdefault(token, {})["successor"] = road_ids[successor]
                links.setdefault(successor, {})["predecessor"] = road_ids[token]
    return links


def _deduplicate(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    result = []
    for point in points:
        if not result or math.dist(result[-1], point) > 1e-6:
            result.append(point)
    return result


def _polyline_length(points: list[tuple[float, float]]) -> float:
    return sum(math.dist(first, second) for first, second in zip(points, points[1:]))


def _index(records: Iterable[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    return {str(record["token"]): record for record in records}


def _fmt(value: float) -> str:
    return f"{value:.6f}"


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
