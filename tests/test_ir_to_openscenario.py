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
                "policy": "replay",
                "closed_loop_level": "replay",
                "reference_trajectory": [
                    {"t_sec": 1.0, "x": 10.0, "y": -2.0, "z": 0.4, "yaw": -0.1},
                    {"t_sec": 2.5, "x": 17.5, "y": -1.5, "z": 0.4, "yaw": 0.0},
                ],
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
        self.assertIsNotNone(root.find("ParameterDeclarations"))
        self.assertIsNotNone(root.find("CatalogLocations"))
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

    def test_vehicle_model_name_is_carla_blueprint_not_entity_id(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        root = ET.fromstring(build_openscenario_xml(_scenario_ir()))

        for scenario_object in root.findall("Entities/ScenarioObject"):
            vehicle = scenario_object.find("Vehicle")
            self.assertEqual(vehicle.attrib["name"], "vehicle.tesla.model3")
            self.assertNotEqual(vehicle.attrib["name"], scenario_object.attrib["name"])

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

    def test_replay_maneuver_group_targets_actor_not_ego(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        root = ET.fromstring(build_openscenario_xml(_scenario_ir()))
        maneuver_group = root.find("Storyboard/Story/Act/ManeuverGroup")

        self.assertIsNotNone(maneuver_group)
        actors = maneuver_group.find("Actors")
        self.assertIsNotNone(actors)
        self.assertEqual(actors.attrib["selectTriggeringEntities"], "false")
        self.assertEqual(actors.find("EntityRef").attrib["entityRef"], "actor_actor_1")
        self.assertNotEqual(actors.find("EntityRef").attrib["entityRef"], "ego")

    def test_replay_actor_gets_timed_follow_trajectory_action(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        root = ET.fromstring(build_openscenario_xml(_scenario_ir()))
        follow = root.find(
            "Storyboard/Story/Act/ManeuverGroup/Maneuver/Event/Action/PrivateAction/"
            "RoutingAction/FollowTrajectoryAction"
        )

        self.assertIsNotNone(follow)
        vertices = follow.findall("Trajectory/Shape/Polyline/Vertex")
        self.assertEqual([vertex.attrib["time"] for vertex in vertices], ["1", "2.5"])
        positions = [vertex.find("Position/WorldPosition").attrib for vertex in vertices]
        self.assertEqual(positions[0], {"x": "10", "y": "-2", "z": "0.4", "h": "-0.1"})
        self.assertEqual(positions[1], {"x": "17.5", "y": "-1.5", "z": "0.4", "h": "0"})
        timing = follow.find("TimeReference/Timing")
        self.assertEqual(timing.attrib["domainAbsoluteRelative"], "absolute")

    def test_ego_never_gets_follow_trajectory_action(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        scenario = _scenario_ir()
        scenario["ego"]["reference_trajectory"] = [
            {"t_sec": 0.0, "x": 1.0, "y": 2.0},
            {"t_sec": 1.0, "x": 4.0, "y": 2.0},
        ]
        root = ET.fromstring(build_openscenario_xml(scenario))

        refs = [
            ref.attrib["entityRef"]
            for ref in root.findall("Storyboard/Story/Act/ManeuverGroup/Actors/EntityRef")
        ]
        self.assertNotIn("ego", refs)
        self.assertEqual(len(root.findall(".//FollowTrajectoryAction")), 1)

    def test_scripted_and_traffic_manager_actors_are_init_only(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        scenario = _scenario_ir()
        trajectory = scenario["actors"][0]["reference_trajectory"]
        scenario["actors"].extend(
            [
                {
                    "actor_id": "scripted",
                    "initial_state": trajectory[0],
                    "policy": "scripted_trigger",
                    "closed_loop_level": "scripted",
                    "reference_trajectory": trajectory,
                },
                {
                    "actor_id": "tm",
                    "initial_state": trajectory[0],
                    "policy": "reactive_rule_based",
                    "closed_loop_level": "traffic_manager_reactive",
                    "reference_trajectory": trajectory,
                },
            ]
        )
        root = ET.fromstring(build_openscenario_xml(scenario))

        refs = [
            ref.attrib["entityRef"]
            for ref in root.findall("Storyboard/Story/Act/ManeuverGroup/Actors/EntityRef")
        ]
        self.assertEqual(refs, ["actor_actor_1"])
        self.assertIsNotNone(root.find("Storyboard/Init/Actions/Private[@entityRef='actor_scripted']"))
        self.assertIsNotNone(root.find("Storyboard/Init/Actions/Private[@entityRef='actor_tm']"))

    def test_act_and_event_have_simulation_time_start_triggers(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        root = ET.fromstring(build_openscenario_xml(_scenario_ir()))
        act_start = root.find(
            "Storyboard/Story/Act/StartTrigger/ConditionGroup/Condition/ByValueCondition/SimulationTimeCondition"
        )
        event_start = root.find(
            "Storyboard/Story/Act/ManeuverGroup/Maneuver/Event/StartTrigger/ConditionGroup/"
            "Condition/ByValueCondition/SimulationTimeCondition"
        )
        act_condition = root.find(
            "Storyboard/Story/Act/StartTrigger/ConditionGroup/Condition"
        )

        self.assertEqual(act_start.attrib, {"value": "0", "rule": "greaterThan"})
        self.assertEqual(act_condition.attrib["conditionEdge"], "none")
        self.assertEqual(event_start.attrib, {"value": "1", "rule": "greaterThan"})

    def test_degree_yaw_is_converted_to_openscenario_radians(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        scenario = _scenario_ir()
        scenario["coordinate_frame"] = {"units": {"yaw": "degree"}}
        scenario["ego"]["initial_state"]["yaw"] = 90.0
        scenario["actors"][0]["initial_state"]["yaw"] = -90.0
        scenario["actors"][0]["reference_trajectory"][0]["yaw"] = 180.0
        root = ET.fromstring(build_openscenario_xml(scenario))

        ego_position = root.find(
            "Storyboard/Init/Actions/Private[@entityRef='ego']/PrivateAction/TeleportAction/"
            "Position/WorldPosition"
        )
        actor_position = root.find(
            "Storyboard/Init/Actions/Private[@entityRef='actor_actor_1']/PrivateAction/TeleportAction/"
            "Position/WorldPosition"
        )
        trajectory_position = root.find(
            ".//ManeuverGroup/Maneuver/Event/Action/PrivateAction/RoutingAction/"
            "FollowTrajectoryAction/Trajectory/Shape/Polyline/Vertex/Position/WorldPosition"
        )

        self.assertAlmostEqual(float(ego_position.attrib["h"]), 1.570796, places=6)
        self.assertAlmostEqual(float(actor_position.attrib["h"]), -1.570796, places=6)
        self.assertAlmostEqual(float(trajectory_position.attrib["h"]), 3.141593, places=6)

    def test_placeholder_maneuver_is_not_emitted(self):
        from adapters.ir_to_openscenario import build_openscenario_xml

        xml_text = build_openscenario_xml(_scenario_ir())

        self.assertNotIn("placeholder", xml_text)


if __name__ == "__main__":
    unittest.main()
