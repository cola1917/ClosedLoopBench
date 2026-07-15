import json
import hashlib
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch


def _scene_ir():
    return {
        "schema_version": "scenario_ir.v1",
        "scenario_id": "cc8c0bf57f984915a77078b10eb33198",
        "source": {
            "dataset": "nuscenes",
            "scene_name": "scene-test",
            "scene_token": "cc8c0bf57f984915a77078b10eb33198",
        },
        "coordinate_frame": {
            "name": "scene_local_ego_start",
            "units": {"position": "meter", "time": "second", "yaw": "degree"},
            "handedness": "right",
            "x_axis": "initial_ego_forward",
            "y_axis": "initial_ego_left",
            "origin_global_translation": [10.0, 20.0, 1.0],
            "origin_global_rotation_wxyz": [0.70710678, 0.0, 0.0, 0.70710678],
            "origin_global_yaw_deg": 90.0,
            "transform": "local_xy = R(-origin_yaw) * (global_xy - origin_xy)",
        },
        "map_context": {"location": "test-map"},
        "windows": {"event": {"start_sec": 0.0, "end_sec": 1.0}},
        "ego": {
            "initial_state": {"x": 0.0, "y": 0.0, "yaw": 90.0, "speed_mps": 0.0},
            "reference_trajectory": [],
        },
        "actors": [],
    }


class BuildNuScenesExchangeTests(unittest.TestCase):
    def test_builds_from_external_ir_and_materializes_reconstruction_package(self):
        from runners.build_nuscenes_exchange import build_exchange_from_scenario_ir

        def write_road(dataroot, output, **kwargs):
            output.write_text("<OpenDRIVE/>\n", encoding="utf-8")
            return output

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ir_path = root / "source" / "scenario_ir.json"
            ir_path.parent.mkdir()
            ir_path.write_text(json.dumps(_scene_ir()), encoding="utf-8")
            usdz = root / "reconstruction-source" / "reconstruction" / "last.usdz"
            usdz.parent.mkdir(parents=True)
            usdz.write_bytes(b"usdz")
            reconstruction = {
                "schema_version": "reconstruction_package.v1",
                "scene_id": "cc8c0bf57f984915a77078b10eb33198",
                "source": {
                    "dataset": "nuscenes",
                    "scene_name": "scene-test",
                    "scene_token": "cc8c0bf57f984915a77078b10eb33198",
                },
                "artifacts": [{
                    "role": "nurec_usdz",
                    "path": "reconstruction/last.usdz",
                    "media_type": "model/vnd.usdz+zip",
                    "sha256": hashlib.sha256(b"usdz").hexdigest(),
                    "size_bytes": 4,
                }],
                "alignment": {"status": "pending_runtime_alignment"},
            }
            reconstruction_path = root / "reconstruction-source" / "reconstruction_package.json"
            reconstruction_path.write_text(json.dumps(reconstruction), encoding="utf-8")
            output = root / "bundle"
            with patch(
                "runners.build_nuscenes_exchange.write_nuscenes_opendrive",
                side_effect=write_road,
            ):
                paths = build_exchange_from_scenario_ir(
                    root / "nuscenes",
                    ir_path,
                    output,
                    reconstruction_package_path=reconstruction_path,
                )

            package = json.loads(paths["scene_package"].read_text(encoding="utf-8"))
            self.assertEqual(package["schema_version"], "closed_loop_scene_package.v1")
            self.assertEqual(package["visual"]["nurec_usdz"], "reconstruction/last.usdz")
            self.assertEqual(package["visual"]["reconstruction_package"], "reconstruction_package.json")
            self.assertTrue((output / "reconstruction" / "last.usdz").is_file())
            self.assertEqual(package["alignment"]["status"], "log_to_sim_defined")

            result = {
                "schema_version": "reconstruction_result.v1",
                "payload": {
                    "scene_id": "cc8c0bf57f984915a77078b10eb33198",
                    "status": "succeeded",
                    "artifacts": [{
                        "role": "reconstruction_package",
                        "path": "reconstruction-source/reconstruction_package.json",
                        "sha256": hashlib.sha256(
                            reconstruction_path.read_bytes()
                        ).hexdigest(),
                        "size_bytes": reconstruction_path.stat().st_size,
                    }],
                },
            }
            result_path = root / "reconstruction_result.json"
            result_path.write_text(json.dumps(result), encoding="utf-8")
            result_output = root / "bundle-from-result"
            with patch(
                "runners.build_nuscenes_exchange.write_nuscenes_opendrive",
                side_effect=write_road,
            ):
                result_paths = build_exchange_from_scenario_ir(
                    root / "nuscenes",
                    ir_path,
                    result_output,
                    reconstruction_result_path=result_path,
                    exchange_root=root,
                )
            result_package = json.loads(
                result_paths["scene_package"].read_text(encoding="utf-8")
            )
            self.assertEqual(
                result_package["visual"]["reconstruction_package"],
                "reconstruction_package.json",
            )

    def test_builds_portable_four_file_exchange(self):
        from runners.build_nuscenes_exchange import build_nuscenes_exchange

        calls = {}

        def write_ir(dataroot, version, scene, output):
            calls["ir"] = (dataroot, version, scene)
            output.write_text(json.dumps(_scene_ir()), encoding="utf-8")
            return output

        def write_road(dataroot, output, **kwargs):
            calls["road"] = (dataroot, kwargs)
            output.write_text("<OpenDRIVE/>\n", encoding="utf-8")
            return output

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            dataroot = root / "nuscenes"
            output_dir = root / "exchange" / "scene-test"
            with patch("runners.build_nuscenes_exchange.write_scene_ir", side_effect=write_ir), patch(
                "runners.build_nuscenes_exchange.write_nuscenes_opendrive", side_effect=write_road
            ):
                paths = build_nuscenes_exchange(
                    dataroot,
                    "v1.0-mini",
                    "scene-test",
                    output_dir,
                    radius_m=42.0,
                )

            self.assertEqual(set(paths), {"scene_ir", "opendrive", "openscenario", "scene_package"})
            self.assertTrue(all(path.is_file() for path in paths.values()))
            self.assertEqual(calls["ir"], (dataroot, "v1.0-mini", "scene-test"))
            self.assertEqual(calls["road"][1]["scenario_ir_path"], paths["scene_ir"])
            self.assertEqual(calls["road"][1]["radius_m"], 42.0)

            xosc = ET.parse(paths["openscenario"]).getroot()
            self.assertEqual(xosc.find("RoadNetwork/LogicFile").attrib["filepath"], "road.xodr")
            self.assertAlmostEqual(float(xosc.find("Storyboard/Init/Actions/Private/PrivateAction/TeleportAction/Position/WorldPosition").attrib["h"]), 1.570796)

            package = json.loads(paths["scene_package"].read_text(encoding="utf-8"))
            self.assertEqual(package["scene_id"], "cc8c0bf57f984915a77078b10eb33198")
            self.assertEqual(package["motion"]["scene_ir"], "scene_ir.json")
            self.assertEqual(package["map"]["opendrive"], "road.xodr")
            self.assertEqual(package["scenario"]["openscenario"], "scenario.xosc")
            self.assertFalse(any(Path(value).is_absolute() for value in (
                package["motion"]["scene_ir"],
                package["map"]["opendrive"],
                package["scenario"]["openscenario"],
            )))


if __name__ == "__main__":
    unittest.main()
