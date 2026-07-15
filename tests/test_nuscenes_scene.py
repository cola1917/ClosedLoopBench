import json
import math
import tempfile
import unittest
from pathlib import Path


SCENE_TOKEN = "cc8c0bf57f984915a77078b10eb33198"


def _write_table(root, name, records):
    (root / f"{name}.json").write_text(json.dumps(records), encoding="utf-8")


def _fixture(root):
    version = root / "v1.0-mini"
    version.mkdir()
    yaw_90 = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]
    tables = {
        "scene": [{"token": SCENE_TOKEN, "log_token": "log", "nbr_samples": 2, "first_sample_token": "s0", "last_sample_token": "s1", "name": "scene-test", "description": "fixture"}],
        "sample": [
            {"token": "s0", "timestamp": 1_000_000, "prev": "", "next": "s1", "scene_token": SCENE_TOKEN},
            {"token": "s1", "timestamp": 1_500_000, "prev": "s0", "next": "", "scene_token": SCENE_TOKEN},
        ],
        "sample_data": [
            {"token": "sd0", "sample_token": "s0", "ego_pose_token": "ep0", "calibrated_sensor_token": "cal", "is_key_frame": True},
            {"token": "sd1", "sample_token": "s1", "ego_pose_token": "ep1", "calibrated_sensor_token": "cal", "is_key_frame": True},
        ],
        "ego_pose": [
            {"token": "ep0", "translation": [10.0, 20.0, 1.0], "rotation": yaw_90},
            {"token": "ep1", "translation": [10.0, 22.0, 1.0], "rotation": yaw_90},
        ],
        "calibrated_sensor": [{"token": "cal", "sensor_token": "lidar"}],
        "sensor": [{"token": "lidar", "channel": "LIDAR_TOP", "modality": "lidar"}],
        "sample_annotation": [
            {"token": "a0", "sample_token": "s0", "instance_token": "inst", "translation": [9.0, 22.0, 1.0], "rotation": yaw_90, "size": [2.0, 4.0, 1.5]},
            {"token": "a1", "sample_token": "s1", "instance_token": "inst", "translation": [9.0, 23.0, 1.0], "rotation": yaw_90, "size": [2.0, 4.0, 1.5]},
        ],
        "instance": [{"token": "inst", "category_token": "cat"}],
        "category": [{"token": "cat", "name": "vehicle.car"}],
        "log": [{"token": "log", "location": "singapore-onenorth"}],
    }
    for name, records in tables.items():
        _write_table(version, name, records)


class NuScenesSceneTests(unittest.TestCase):
    def test_extracts_normalized_scene_by_name_and_token(self):
        from adapters.nuscenes_scene import build_scene_ir

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _fixture(root)
            by_name = build_scene_ir(root, "scene-test")
            by_token = build_scene_ir(root, SCENE_TOKEN)

        self.assertEqual(by_name, by_token)
        self.assertEqual(by_name["scenario_id"], SCENE_TOKEN)
        self.assertEqual(by_name["source"]["scene_name"], "scene-test")
        self.assertEqual(by_name["source"]["scene_token"], SCENE_TOKEN)
        self.assertEqual(by_name["source"]["sample_count"], 2)
        self.assertEqual(by_name["map_context"]["location"], "singapore-onenorth")
        self.assertAlmostEqual(by_name["ego"]["initial_state"]["x"], 0.0)
        self.assertAlmostEqual(by_name["ego"]["reference_trajectory"][1]["x"], 2.0)
        self.assertAlmostEqual(by_name["ego"]["reference_trajectory"][1]["y"], 0.0)
        self.assertAlmostEqual(by_name["ego"]["reference_trajectory"][1]["speed_mps"], 4.0)
        self.assertEqual(by_name["actors"][0]["category"], "vehicle.car")
        self.assertEqual(by_name["actors"][0]["dimensions"]["length"], 4.0)
        self.assertAlmostEqual(by_name["actors"][0]["initial_state"]["x"], 2.0)
        self.assertAlmostEqual(by_name["actors"][0]["initial_state"]["y"], 1.0)

    def test_cli_output_is_stable_json(self):
        from runners.build_scene_ir_from_nuscenes import write_scene_ir

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _fixture(root)
            first = root / "first.json"
            second = root / "second.json"
            write_scene_ir(root, "v1.0-mini", "scene-test", first)
            write_scene_ir(root, "v1.0-mini", SCENE_TOKEN, second)
            self.assertEqual(first.read_text(encoding="utf-8"), second.read_text(encoding="utf-8"))

    def test_local_nuscenes_mini_scene_0061_when_available(self):
        from adapters.nuscenes_scene import build_scene_ir

        dataroot = Path("E:/code/nuscenes-mini")
        if not (dataroot / "v1.0-mini" / "scene.json").is_file():
            self.skipTest("local nuScenes mini dataset is not available")
        scenario_ir = build_scene_ir(dataroot, "scene-0061")
        self.assertEqual(scenario_ir["source"]["sample_count"], 39)
        self.assertEqual(scenario_ir["map_context"]["location"], "singapore-onenorth")
        self.assertEqual(scenario_ir["ego"]["initial_state"]["x"], 0.0)
        self.assertGreater(len(scenario_ir["actors"]), 0)


if __name__ == "__main__":
    unittest.main()
