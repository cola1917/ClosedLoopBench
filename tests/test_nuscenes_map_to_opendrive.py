import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


def _fixture_map():
    nodes = [
        {"token": "a0", "x": 0.0, "y": 2.0},
        {"token": "a1", "x": 10.0, "y": 2.0},
        {"token": "a2", "x": 10.0, "y": -2.0},
        {"token": "a3", "x": 0.0, "y": -2.0},
        {"token": "b0", "x": 10.0, "y": 2.0},
        {"token": "b1", "x": 20.0, "y": 2.0},
        {"token": "b2", "x": 20.0, "y": -2.0},
        {"token": "b3", "x": 10.0, "y": -2.0},
        {"token": "far0", "x": 100.0, "y": 2.0},
        {"token": "far1", "x": 110.0, "y": 2.0},
        {"token": "far2", "x": 110.0, "y": -2.0},
        {"token": "far3", "x": 100.0, "y": -2.0},
        {"token": "bike0", "x": 2.0, "y": 3.0},
        {"token": "bike1", "x": 8.0, "y": 3.0},
        {"token": "bike2", "x": 8.0, "y": 2.5},
        {"token": "bike3", "x": 2.0, "y": 2.5},
    ]
    lines = []
    polygons = []
    lanes = []
    for prefix in ("a", "b", "far"):
        lines.extend(
            [
                {"token": f"{prefix}-from", "node_tokens": [f"{prefix}0", f"{prefix}3"]},
                {"token": f"{prefix}-to", "node_tokens": [f"{prefix}1", f"{prefix}2"]},
            ]
        )
        polygons.append({"token": f"{prefix}-poly", "exterior_node_tokens": [f"{prefix}0", f"{prefix}1", f"{prefix}2", f"{prefix}3"], "holes": []})
        lanes.append({"token": f"{prefix}-lane", "polygon_token": f"{prefix}-poly", "from_edge_line_token": f"{prefix}-from", "to_edge_line_token": f"{prefix}-to", "lane_type": "CAR"})
    lines.extend(
        [
            {"token": "bike-from", "node_tokens": ["bike0", "bike3"]},
            {"token": "bike-to", "node_tokens": ["bike1", "bike2"]},
        ]
    )
    polygons.append(
        {
            "token": "bike-poly",
            "exterior_node_tokens": ["bike0", "bike1", "bike2", "bike3"],
            "holes": [],
        }
    )
    lanes.append(
        {
            "token": "bike-lane",
            "polygon_token": "bike-poly",
            "from_edge_line_token": "bike-from",
            "to_edge_line_token": "bike-to",
            "lane_type": "BIKE",
        }
    )
    return {"node": nodes, "line": lines, "polygon": polygons, "lane": lanes}


def _fixture_ir():
    return {
        "scenario_id": "scene-test",
        "map_context": {"location": "test-map"},
        "coordinate_frame": {"origin_global_translation": [0.0, 0.0, 0.0], "origin_global_yaw_deg": 0.0},
        "ego": {"reference_trajectory": [{"x": 1.0, "y": 0.0}, {"x": 19.0, "y": 0.0}]},
        "actors": [],
    }


class NuScenesMapToOpenDriveTests(unittest.TestCase):
    def test_builds_local_parseable_roads_and_unique_connection(self):
        from adapters.nuscenes_map_to_opendrive import build_local_opendrive_xml

        root = ET.fromstring(build_local_opendrive_xml(_fixture_ir(), _fixture_map(), radius_m=5.0))
        roads = root.findall("road")
        self.assertEqual(root.tag, "OpenDRIVE")
        self.assertEqual(root.find("header").attrib["revMinor"], "4")
        self.assertEqual(len(roads), 2)
        self.assertEqual(roads[0].find("link/successor").attrib["elementId"], "2")
        self.assertEqual(roads[1].find("link/predecessor").attrib["elementId"], "1")
        self.assertEqual(roads[0].find("lanes/laneSection/right/lane").attrib["type"], "driving")
        self.assertGreater(len(roads[0].findall("planView/geometry")), 0)

    def test_cli_writer_uses_existing_ir_without_devkit(self):
        from runners.build_nuscenes_opendrive import write_nuscenes_opendrive

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "maps").mkdir()
            (root / "maps" / "test-map.json").write_text(json.dumps(_fixture_map()), encoding="utf-8")
            ir_path = root / "scene_ir.json"
            ir_path.write_text(json.dumps(_fixture_ir()), encoding="utf-8")
            output = root / "road.xodr"
            write_nuscenes_opendrive(root, output, scenario_ir_path=ir_path, radius_m=5.0)
            self.assertEqual(ET.parse(output).getroot().tag, "OpenDRIVE")

    def test_local_nuscenes_mini_when_available(self):
        from adapters.nuscenes_map_to_opendrive import build_local_opendrive_xml, load_nuscenes_map
        from adapters.nuscenes_scene import build_scene_ir

        dataroot = Path("E:/code/nuscenes-mini")
        if not (dataroot / "v1.0-mini" / "scene.json").is_file():
            self.skipTest("local nuScenes mini dataset is not available")
        scenario_ir = build_scene_ir(dataroot, "scene-0061")
        map_data = load_nuscenes_map(dataroot, scenario_ir["map_context"]["location"])
        root = ET.fromstring(build_local_opendrive_xml(scenario_ir, map_data, radius_m=25.0))
        self.assertGreater(len(root.findall("road")), 0)


if __name__ == "__main__":
    unittest.main()
