from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Any


def build_route_aligned_opendrive_xml(
    scenario_ir: dict[str, Any],
    *,
    road_id: int = 1000,
    lane_width_m: float = 3.7,
    extension_m: float = 10.0,
    sample_spacing_m: float = 2.0,
    road_name: str | None = None,
) -> str:
    """Build a bounded single-road Ego control corridor from Scenario IR.

    This mode is intended for deterministic route replay and control
    alignment. It deliberately does not reconstruct or replace a map lane
    graph; callers must keep the multi-road topology artifact separate.
    """

    if road_id < 0:
        raise ValueError("road_id must be non-negative")
    if lane_width_m <= 0.0:
        raise ValueError("lane_width_m must be positive")
    if extension_m < 0.0:
        raise ValueError("extension_m must be non-negative")
    if sample_spacing_m <= 0.0:
        raise ValueError("sample_spacing_m must be positive")

    trajectory = _ego_trajectory(scenario_ir)
    points = _densify(trajectory, sample_spacing_m)
    if len(points) < 2:
        points = _fallback_path(trajectory, sample_spacing_m)
    points = _extend_path(points, trajectory, extension_m)
    if len(points) < 2:
        raise ValueError("Scenario IR ego trajectory does not define a route")

    road_length = _polyline_length(points)
    if road_length <= 1e-6:
        raise ValueError("Scenario IR ego trajectory has zero route length")

    root = ET.Element("OpenDRIVE")
    bounds_x = [point[0] for point in points]
    bounds_y = [point[1] for point in points]
    ET.SubElement(
        root,
        "header",
        {
            "revMajor": "1",
            "revMinor": "4",
            "name": road_name or f"route_aligned_{scenario_ir.get('scenario_id', 'scene')}",
            "version": "1.00",
            "date": "2026-07-29T00:00:00",
            "north": _fmt(max(bounds_y)),
            "south": _fmt(min(bounds_y)),
            "east": _fmt(max(bounds_x)),
            "west": _fmt(min(bounds_x)),
        },
    )

    road = ET.SubElement(
        root,
        "road",
        {
            "name": road_name or "route_aligned_ego_corridor",
            "length": _fmt(road_length),
            "id": str(road_id),
            "junction": "-1",
        },
    )
    ET.SubElement(road, "type", {"s": "0", "type": "town"})

    plan_view = ET.SubElement(road, "planView")
    s = 0.0
    for first, second in zip(points, points[1:]):
        length = math.dist(first[:2], second[:2])
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

    elevation = ET.SubElement(road, "elevationProfile")
    start_z = points[0][2]
    slope = (points[-1][2] - start_z) / road_length
    ET.SubElement(
        elevation,
        "elevation",
        {"s": "0", "a": _fmt(start_z), "b": _fmt(slope), "c": "0", "d": "0"},
    )

    lanes = ET.SubElement(road, "lanes")
    ET.SubElement(
        lanes,
        "laneOffset",
        {"s": "0", "a": _fmt(lane_width_m / 2.0), "b": "0", "c": "0", "d": "0"},
    )
    lane_section = ET.SubElement(lanes, "laneSection", {"s": "0"})
    center = ET.SubElement(lane_section, "center")
    center_lane = ET.SubElement(center, "lane", {"id": "0", "type": "none", "level": "false"})
    ET.SubElement(
        center_lane,
        "roadMark",
        {"sOffset": "0", "type": "none", "weight": "standard", "color": "standard", "width": "0"},
    )
    right = ET.SubElement(lane_section, "right")
    driving = ET.SubElement(right, "lane", {"id": "-1", "type": "driving", "level": "false"})
    ET.SubElement(
        driving,
        "width",
        {"sOffset": "0", "a": _fmt(lane_width_m), "b": "0", "c": "0", "d": "0"},
    )
    ET.SubElement(
        driving,
        "roadMark",
        {
            "sOffset": "0",
            "type": "broken",
            "weight": "standard",
            "color": "white",
            "width": "0.12",
            "laneChange": "both",
        },
    )

    _indent(root)
    return ET.tostring(root, encoding="unicode")


def _ego_trajectory(scenario_ir: dict[str, Any]) -> list[dict[str, float]]:
    raw = (scenario_ir.get("ego") or {}).get("reference_trajectory") or []
    result = []
    for state in raw:
        if not isinstance(state, dict) or "x" not in state or "y" not in state:
            continue
        result.append(
            {
                "x": float(state["x"]),
                "y": float(state["y"]),
                "z": float(state.get("z", 0.0)),
                "yaw": float(state.get("yaw", 0.0)),
            }
        )
    return result


def _densify(trajectory: list[dict[str, float]], spacing_m: float) -> list[tuple[float, float, float]]:
    if not trajectory:
        return []
    result = [(trajectory[0]["x"], trajectory[0]["y"], trajectory[0]["z"])]
    for first, second in zip(trajectory, trajectory[1:]):
        distance = math.dist((first["x"], first["y"]), (second["x"], second["y"]))
        steps = max(1, int(math.ceil(distance / spacing_m)))
        for index in range(1, steps + 1):
            ratio = index / steps
            result.append(
                (
                    first["x"] + ratio * (second["x"] - first["x"]),
                    first["y"] + ratio * (second["y"] - first["y"]),
                    first["z"] + ratio * (second["z"] - first["z"]),
                )
            )
    return _deduplicate(result)


def _fallback_path(trajectory: list[dict[str, float]], spacing_m: float) -> list[tuple[float, float, float]]:
    if not trajectory:
        return []
    first = trajectory[0]
    length = max(20.0, spacing_m)
    yaw = math.radians(first["yaw"])
    second = {
        "x": first["x"] + length * math.cos(yaw),
        "y": first["y"] + length * math.sin(yaw),
        "z": first["z"],
    }
    return [(first["x"], first["y"], first["z"]), (second["x"], second["y"], second["z"])]


def _extend_path(
    points: list[tuple[float, float, float]],
    trajectory: list[dict[str, float]],
    extension_m: float,
) -> list[tuple[float, float, float]]:
    if extension_m <= 0.0 or not points:
        return points
    first = trajectory[0] if trajectory else {"yaw": 0.0}
    last = trajectory[-1] if trajectory else {"yaw": 0.0}
    start_yaw = math.radians(first.get("yaw", 0.0))
    end_yaw = math.radians(last.get("yaw", 0.0))
    start = points[0]
    end = points[-1]
    before = (start[0] - extension_m * math.cos(start_yaw), start[1] - extension_m * math.sin(start_yaw), start[2])
    after = (end[0] + extension_m * math.cos(end_yaw), end[1] + extension_m * math.sin(end_yaw), end[2])
    return [before, *points, after]


def _deduplicate(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    result = []
    for point in points:
        if not result or math.dist(result[-1][:2], point[:2]) > 1e-6:
            result.append(point)
    return result


def _polyline_length(points: list[tuple[float, float, float]]) -> float:
    return sum(math.dist(first[:2], second[:2]) for first, second in zip(points, points[1:]))


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
