import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


class OpenDriveExportContractTests(unittest.TestCase):
    def test_builds_minimal_parseable_opendrive_from_scenario_ir(self):
        from adapters.ir_to_opendrive import build_minimal_opendrive_xml

        scenario_ir = json.loads(
            Path("E:/code/TriggerEngine/outputs/scenario_ir/scene-1077.mvp.json").read_text(encoding="utf-8")
        )

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
