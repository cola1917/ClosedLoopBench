from __future__ import annotations

import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
XODR_PATH = PROJECT_ROOT / "outputs" / "scene-0061" / "road.xodr"


def _user_data(road: ET.Element) -> dict[str, str]:
    return {
        item.attrib["code"]: item.attrib["value"]
        for item in road.findall("userData")
    }


def _plan_view_end(road: ET.Element) -> tuple[float, float]:
    geometry = road.findall("./planView/geometry")[-1]
    x = float(geometry.attrib["x"])
    y = float(geometry.attrib["y"])
    heading = float(geometry.attrib["hdg"])
    length = float(geometry.attrib["length"])
    arc = geometry.find("arc")
    if arc is None:
        return x + length * math.cos(heading), y + length * math.sin(heading)
    curvature = float(arc.attrib["curvature"])
    end_heading = heading + curvature * length
    return (
        x + (math.sin(end_heading) - math.sin(heading)) / curvature,
        y - (math.cos(end_heading) - math.cos(heading)) / curvature,
    )


class Scene0061NativeOpenDriveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        root = ET.parse(XODR_PATH).getroot()
        cls.roads = {road.attrib["id"]: road for road in root.findall("road")}
        cls.junction = root.find("junction[@id='1']")

    def test_source_two_to_one_merge_is_explicit(self) -> None:
        target = self.roads["12"]
        target_data = _user_data(target)
        self.assertEqual(target_data["topologyRole"], "two_to_one_merge_target")
        self.assertEqual(target_data["mergeIncomingRoads"], "56,58")

        expected = {"56": ("24", "straight"), "58": ("4", "left")}
        for connector_id, (incoming_id, maneuver) in expected.items():
            connector = self.roads[connector_id]
            data = _user_data(connector)
            self.assertEqual(data["topologyRole"], "merge_incoming")
            self.assertEqual(data["mergeTargetRoad"], "12")
            self.assertEqual(data["inferredManeuver"], maneuver)
            self.assertEqual(
                connector.find("./link/predecessor").attrib["elementId"],
                incoming_id,
            )
            self.assertEqual(
                connector.find("./link/successor").attrib["elementId"], "12"
            )

    def test_merge_connectors_have_distinct_junction_entries(self) -> None:
        self.assertIsNotNone(self.junction)
        connections = {
            item.attrib["connectingRoad"]: item
            for item in self.junction.findall("connection")
        }
        self.assertEqual(connections["56"].attrib["incomingRoad"], "24")
        self.assertEqual(connections["58"].attrib["incomingRoad"], "4")
        for connector_id in ("56", "58"):
            lane_link = connections[connector_id].find("laneLink")
            self.assertEqual(lane_link.attrib, {"from": "1", "to": "1"})

    def test_both_connector_endpoints_coincide_with_merge_target(self) -> None:
        target_start = self.roads["12"].find("./planView/geometry")
        expected = (
            float(target_start.attrib["x"]),
            float(target_start.attrib["y"]),
        )
        for connector_id in ("56", "58"):
            actual = _plan_view_end(self.roads[connector_id])
            self.assertLess(math.dist(actual, expected), 1e-3)


if __name__ == "__main__":
    unittest.main()
