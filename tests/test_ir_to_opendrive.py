import unittest
import xml.etree.ElementTree as ET

from tests.test_ir_to_carla import minimal_scenario_ir


class OpenDriveExportContractTests(unittest.TestCase):
    def test_builds_minimal_parseable_opendrive_from_scenario_ir(self):
        from adapters.ir_to_opendrive import build_minimal_opendrive_xml

        scenario_ir = minimal_scenario_ir()
        xml_text = build_minimal_opendrive_xml(scenario_ir)
        root = ET.fromstring(xml_text)

        self.assertEqual(root.tag, "OpenDRIVE")
        self.assertIsNotNone(root.find("header"))
        road = root.find("road")
        self.assertIsNotNone(road)
        self.assertGreater(float(road.attrib["length"]), 1.0)
        self.assertIsNotNone(root.find("road/planView/geometry/line"))
        self.assertIsNotNone(root.find("road/lanes/laneSection/center/lane"))


if __name__ == "__main__":
    unittest.main()
