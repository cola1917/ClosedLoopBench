import unittest
import xml.etree.ElementTree as ET


def _scenario_ir():
    return {
        "scenario_id": "unit-scenario",
        "windows": {"event": {"start_sec": 1.0, "end_sec": 8.25}},
        "ego": {
            "initial_state": {
                "x": 1.0,
                "y": 2.0,
                "z": 0.0,
                "yaw": 0.5,
                "speed_mps": 3.5,
            }
        },
        "actors": [
            {
                "actor_id": "actor-1",
                "dimensions": {"length": 4.8, "width": 2.1, "height": 1.7},
                "initial_state": {
                    "x": 10.0,
                    "y": -2.0,
                    "z": 0.4,
                    "yaw": -0.1,
                    "speed_mps": 5.0,
                },
            }
        ],
    }


class OpenScenarioExportContractTests(unittest.TestCase):
    def test_builds_parseable_openscenario_xml_from_scenario_ir(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        xml_text = build_openscenario_xml(_scenario_ir(), road_file="road.xodr")
        root = ET.fromstring(xml_text)

        self.assertEqual(root.tag, "OpenSCENARIO")
        self.assertIsNotNone(root.find("FileHeader"))
        self.assertIsNotNone(root.find("RoadNetwork/LogicFile"))
        self.assertEqual(root.find("RoadNetwork/LogicFile").attrib["filepath"], "road.xodr")
        entities = root.findall("Entities/ScenarioObject")
        self.assertGreaterEqual(len(entities), 2)
        self.assertEqual(entities[0].attrib["name"], "ego")
        self.assertIsNotNone(root.find("Storyboard/Init"))
        self.assertIsNotNone(root.find("Storyboard/StopTrigger"))

    def test_entities_include_complete_vehicle_bounding_boxes(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        root = ET.fromstring(build_openscenario_xml(_scenario_ir()))

        for scenario_object in root.findall("Entities/ScenarioObject"):
            vehicle = scenario_object.find("Vehicle")
            self.assertIsNotNone(vehicle, scenario_object.attrib["name"])
            self.assertIsNotNone(vehicle.find("BoundingBox/Center"), scenario_object.attrib["name"])
            dimensions = vehicle.find("BoundingBox/Dimensions")
            self.assertIsNotNone(dimensions, scenario_object.attrib["name"])
            self.assertIn("length", dimensions.attrib)
            self.assertIn("width", dimensions.attrib)
            self.assertIn("height", dimensions.attrib)

    def test_stop_trigger_uses_event_window_end(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        root = ET.fromstring(build_openscenario_xml(_scenario_ir()))

        condition = root.find(
            "Storyboard/StopTrigger/ConditionGroup/Condition/ByValueCondition/SimulationTimeCondition"
        )
        self.assertIsNotNone(condition)
        self.assertEqual(condition.attrib["value"], "8.25")
        self.assertEqual(condition.attrib["rule"], "greaterThan")

    def test_init_contains_teleport_and_speed_action_for_every_entity(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        root = ET.fromstring(build_openscenario_xml(_scenario_ir()))
        entity_names = [entity.attrib["name"] for entity in root.findall("Entities/ScenarioObject")]

        for entity_name in entity_names:
            private = root.find(f"Storyboard/Init/Actions/Private[@entityRef='{entity_name}']")
            self.assertIsNotNone(private, entity_name)
            self.assertIsNotNone(private.find("PrivateAction/TeleportAction"), entity_name)
            self.assertIsNotNone(private.find("PrivateAction/LongitudinalAction/SpeedAction"), entity_name)

    def test_maneuver_group_contains_required_actors_element(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        root = ET.fromstring(build_openscenario_xml(_scenario_ir()))
        maneuver_group = root.find("Storyboard/Story/Act/ManeuverGroup")

        self.assertIsNotNone(maneuver_group)
        actors = maneuver_group.find("Actors")
        self.assertIsNotNone(actors)
        self.assertEqual(actors.attrib["selectTriggeringEntities"], "false")
        self.assertEqual(actors.find("EntityRef").attrib["entityRef"], "ego")


if __name__ == "__main__":
    unittest.main()
