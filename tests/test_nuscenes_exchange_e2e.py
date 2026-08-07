import json
import os
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_NUSCENES_ROOT = Path("E:/code/nuscenes-mini")
# The current Singapore map export produces 67 local lane roads with the
# canonical 50 m trajectory selection. Keep a margin for map packaging
# updates while still rejecting a corridor-only or otherwise tiny map.
MIN_SCENE_0061_LANE_ROADS = 60


class NuScenesExchangeEndToEndTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        dataroot = Path(os.environ.get("NUSCENES_DATAROOT", DEFAULT_NUSCENES_ROOT))
        version_root = dataroot / "v1.0-mini"
        if not version_root.is_dir() or not (dataroot / "maps").is_dir():
            raise unittest.SkipTest(
                "nuScenes mini is unavailable; set NUSCENES_DATAROOT to run the P1 integration test"
            )

        from runners.build_nuscenes_exchange import build_nuscenes_exchange

        cls._temporary_directory = tempfile.TemporaryDirectory()
        cls.output_dir = Path(cls._temporary_directory.name) / "scene-0061"
        cls.paths = build_nuscenes_exchange(
            dataroot,
            "v1.0-mini",
            "scene-0061",
            cls.output_dir,
        )

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, "_temporary_directory"):
            cls._temporary_directory.cleanup()

    def test_scene_0061_builds_a_complete_portable_exchange(self):
        self.assertEqual(
            set(self.paths),
            {"scene_ir", "opendrive", "openscenario", "scene_package"},
        )
        self.assertTrue(all(path.is_file() for path in self.paths.values()))

        scene_ir = json.loads(self.paths["scene_ir"].read_text(encoding="utf-8"))
        self.assertEqual(scene_ir["scenario_id"], scene_ir["source"]["scene_token"])
        self.assertEqual(scene_ir["source"]["scene_name"], "scene-0061")
        self.assertEqual(scene_ir["source"]["sample_count"], 39)
        self.assertEqual(len(scene_ir["ego"]["reference_trajectory"]), 39)
        self.assertEqual(len(scene_ir["actors"]), 227)
        self.assertAlmostEqual(scene_ir["windows"]["event"]["end_sec"], 19.149566)
        self.assertEqual(
            scene_ir["coordinate_frame"]["units"],
            {"position": "meter", "time": "second", "yaw": "degree"},
        )

        xosc = ET.parse(self.paths["openscenario"]).getroot()
        self.assertEqual(
            xosc.find("RoadNetwork/LogicFile").attrib["filepath"],
            "road.xodr",
        )
        self.assertEqual(len(xosc.findall("Entities/ScenarioObject")), 228)
        self.assertEqual(len(xosc.findall(".//FollowTrajectoryAction")), 212)
        self.assertTrue(
            all(
                condition.attrib["conditionEdge"] == "none"
                for condition in xosc.findall(".//Condition")
                if condition.find("ByValueCondition/SimulationTimeCondition") is not None
                and float(
                    condition.find("ByValueCondition/SimulationTimeCondition").attrib["value"]
                )
                == 0.0
            )
        )

        opendrive = ET.parse(self.paths["opendrive"]).getroot()
        self.assertEqual(opendrive.tag, "OpenDRIVE")
        roads = opendrive.findall("road")
        lane_count = sum(
            road.attrib.get("name", "").startswith("nuscenes_lane_")
            for road in roads
        )
        connector_count = sum(
            road.attrib.get("name", "").startswith("inferred_connector_")
            for road in roads
        )
        route_count = sum(
            road.attrib.get("name", "").startswith("inferred_route_")
            for road in roads
        )
        corridor_count = sum(
            road.attrib.get("name") == "ego_route_corridor" for road in roads
        )
        self.assertGreaterEqual(lane_count, 2)
        self.assertGreaterEqual(lane_count, MIN_SCENE_0061_LANE_ROADS)
        self.assertGreater(connector_count, 0)
        self.assertGreater(route_count, 0)
        self.assertEqual(route_count, 1)
        self.assertEqual(corridor_count, 0)
        self.assertEqual(len(roads), lane_count + connector_count + route_count)
        self.assertGreater(len(opendrive.findall("junction")), 0)
        self.assertGreater(len(opendrive.findall("./junction/connection")), 0)

        route_path = sorted(
            (
                int(
                    next(
                        property_node.attrib["value"]
                        for property_node in road.findall("./userData/property")
                        if property_node.attrib.get("name") == "route_path_order"
                    )
                ),
                road,
            )
            for road in roads
            if any(
                property_node.attrib.get("name") == "route_path_order"
                for property_node in road.findall("./userData/property")
            )
        )
        self.assertEqual(
            [order for order, _ in route_path],
            [1, 2, 3, 4, 5, 6],
        )
        self.assertEqual(
            [
                next(
                    property_node.attrib["value"]
                    for property_node in road.findall("./userData/property")
                    if property_node.attrib.get("name") == "route_path_kind"
                )
                for _, road in route_path
            ],
            ["source_gap", "map_lane", "map_connector", "map_lane", "map_connector", "map_lane"],
        )
        self.assertEqual(
            [
                next(
                    property_node.attrib["value"]
                    for property_node in road.findall("./userData/property")
                    if property_node.attrib.get("name") == "route_geometry_authority"
                )
                for _, road in route_path
            ],
            [
                "synthetic_reference_trajectory",
                "map_lane_network",
                "map_connector_geometry",
                "map_lane_network",
                "map_connector_geometry",
                "map_lane_network",
            ],
        )

        package = json.loads(self.paths["scene_package"].read_text(encoding="utf-8"))
        self.assertEqual(package["scene_id"], scene_ir["source"]["scene_token"])
        self.assertEqual(
            package["map"]["source"],
            "nuscenes_topology_map_expansion_with_mixed_route_path",
        )
        exchange_paths = (
            package["motion"]["scene_ir"],
            package["map"]["opendrive"],
            package["scenario"]["openscenario"],
        )
        self.assertEqual(exchange_paths, ("scene_ir.json", "road.xodr", "scenario.xosc"))
        self.assertTrue(all(not Path(path).is_absolute() for path in exchange_paths))
        self.assertTrue(all((self.output_dir / path).is_file() for path in exchange_paths))

    def test_scene_0061_executes_headless_in_esmini(self):
        from tools.esmini import find_esmini

        esmini = find_esmini()
        if esmini is None:
            self.skipTest("esmini is unavailable")
        completed = subprocess.run(
            [
                str(esmini),
                "--osc",
                str(self.paths["openscenario"]),
                "--headless",
                "--fixed_timestep",
                "0.05",
                "--log_level",
                "warn",
            ],
            cwd=PROJECT_ROOT,
            text=True,
            capture_output=True,
            timeout=30,
            check=False,
        )
        self.assertEqual(
            completed.returncode,
            0,
            msg=f"esmini stdout:\n{completed.stdout}\nesmini stderr:\n{completed.stderr}",
        )


if __name__ == "__main__":
    unittest.main()
