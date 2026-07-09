from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Any


def build_minimal_opendrive_xml(scenario_ir: dict[str, Any]) -> str:
    ego_traj = scenario_ir.get("ego", {}).get("reference_trajectory", [])
    start = ego_traj[0] if ego_traj else {"x": 0.0, "y": 0.0, "yaw": 0.0}
    length = max(_trajectory_length(ego_traj), 20.0)

    root = ET.Element("OpenDRIVE")
    ET.SubElement(
        root,
        "header",
        {
            "revMajor": "1",
            "revMinor": "4",
            "name": f"minimal_{scenario_ir['scenario_id']}",
            "version": "1.00",
            "date": "2026-07-08T00:00:00",
            "north": "0",
            "south": "0",
            "east": "0",
            "west": "0",
        },
    )
    road = ET.SubElement(
        root,
        "road",
        {
            "name": "semantic_equivalent_road",
            "length": f"{length:.3f}",
            "id": "1",
            "junction": "-1",
        },
    )
    plan_view = ET.SubElement(road, "planView")
    geometry = ET.SubElement(
        plan_view,
        "geometry",
        {
            "s": "0.000",
            "x": str(start.get("x", 0.0)),
            "y": str(start.get("y", 0.0)),
            "hdg": str(start.get("yaw", 0.0)),
            "length": f"{length:.3f}",
        },
    )
    ET.SubElement(geometry, "line")

    lanes = ET.SubElement(road, "lanes")
    lane_section = ET.SubElement(lanes, "laneSection", {"s": "0.000"})
    center = ET.SubElement(lane_section, "center")
    ET.SubElement(center, "lane", {"id": "0", "type": "none", "level": "false"})
    left = ET.SubElement(lane_section, "left")
    left_lane = ET.SubElement(left, "lane", {"id": "1", "type": "driving", "level": "false"})
    ET.SubElement(left_lane, "width", {"sOffset": "0.000", "a": "3.500", "b": "0", "c": "0", "d": "0"})
    right = ET.SubElement(lane_section, "right")
    right_lane = ET.SubElement(right, "lane", {"id": "-1", "type": "driving", "level": "false"})
    ET.SubElement(right_lane, "width", {"sOffset": "0.000", "a": "3.500", "b": "0", "c": "0", "d": "0"})

    _indent(root)
    return ET.tostring(root, encoding="unicode")


def _trajectory_length(states: list[dict[str, Any]]) -> float:
    if len(states) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(states, states[1:]):
        total += math.hypot(float(b.get("x", 0.0)) - float(a.get("x", 0.0)), float(b.get("y", 0.0)) - float(a.get("y", 0.0)))
    return total


def _indent(elem, level: int = 0) -> None:
    text = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = text + "  "
        for child in elem:
            _indent(child, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = text
    elif level and (not elem.tail or not elem.tail.strip()):
        elem.tail = text
