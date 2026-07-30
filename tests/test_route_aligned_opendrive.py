import math
import unittest
import xml.etree.ElementTree as ET


class RouteAlignedOpenDriveTests(unittest.TestCase):
    def test_densifies_route_and_writes_radian_headings(self):
        from adapters.route_aligned_opendrive import build_route_aligned_opendrive_xml

        scenario_ir = {
            "scenario_id": "route-test",
            "ego": {
                "reference_trajectory": [
                    {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                    {"x": 4.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                    {"x": 4.0, "y": 4.0, "z": 0.0, "yaw": 90.0},
                ]
            },
        }

        root = ET.fromstring(
            build_route_aligned_opendrive_xml(
                scenario_ir,
                extension_m=0.0,
                sample_spacing_m=1.0,
            )
        )
        road = root.find("road")
        self.assertIsNotNone(road)
        geometries = road.findall("planView/geometry")
        self.assertGreaterEqual(len(geometries), 8)
        self.assertAlmostEqual(float(geometries[0].attrib["hdg"]), 0.0, places=6)
        self.assertTrue(
            any(
                abs(float(item.attrib["hdg"]) - math.pi / 2.0) < 1e-6
                for item in geometries
            )
        )
        self.assertEqual(road.find("lanes/laneSection/right/lane").attrib["id"], "-1")

    def test_degrees_are_not_written_as_open_drive_heading(self):
        from adapters.route_aligned_opendrive import build_route_aligned_opendrive_xml

        scenario_ir = {
            "scenario_id": "heading-test",
            "ego": {
                "reference_trajectory": [
                    {"x": 0.0, "y": 0.0, "yaw": 90.0},
                    {"x": 0.0, "y": 10.0, "yaw": 90.0},
                ]
            },
        }
        root = ET.fromstring(
            build_route_aligned_opendrive_xml(scenario_ir, extension_m=0.0)
        )
        heading = float(root.find("road/planView/geometry").attrib["hdg"])
        self.assertAlmostEqual(heading, math.pi / 2.0, places=6)


if __name__ == "__main__":
    unittest.main()
