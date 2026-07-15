import unittest


def _coordinate_frame(*, origin=(0.0, 0.0, 0.0), yaw=0.0):
    return {
        "name": "scene_local_ego_start",
        "units": {"position": "meter", "time": "second", "yaw": "degree"},
        "handedness": "right",
        "x_axis": "initial_ego_forward",
        "y_axis": "initial_ego_left",
        "origin_global_translation": list(origin),
        "origin_global_rotation_wxyz": [1.0, 0.0, 0.0, 0.0],
        "origin_global_yaw_deg": yaw,
        "transform": "local_xy = R(-origin_yaw) * (global_xy - origin_xy)",
    }


class ScenePackageTests(unittest.TestCase):
    def test_builds_row_major_log_to_sim_transform(self):
        from adapters.scene_package import build_scene_package

        scene_ir = {
            "scenario_id": "c" * 32,
            "source": {
                "dataset": "nuscenes",
                "scene_name": "scene-test",
                "scene_token": "c" * 32,
            },
            "coordinate_frame": _coordinate_frame(origin=(10.0, 20.0, 1.0), yaw=90.0),
            "map_context": {},
        }
        package = build_scene_package(
            scene_ir,
            scene_ir_path="scene_ir.json",
            openscenario_path="scenario.xosc",
            opendrive_path="road.xodr",
            map_source="fixture",
        )
        matrix = package["alignment"]["sim_from_log_transform"]

        def apply(point):
            x, y, z = point
            return [
                matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
                matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
                matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
            ]

        self.assertEqual(package["alignment"]["status"], "log_to_sim_defined")
        for value in apply((10.0, 20.0, 1.0)):
            self.assertAlmostEqual(value, 0.0)
        forward = apply((10.0, 21.0, 1.0))
        self.assertAlmostEqual(forward[0], 1.0)
        self.assertAlmostEqual(forward[1], 0.0)

    def test_requires_explicit_nuscenes_scene_token(self):
        from adapters.scene_package import build_scene_package

        scene_ir = {
            "scenario_id": "cc8c0bf57f984915a77078b10eb33198",
            "source": {"dataset": "nuscenes", "scene_name": "scene-0061"},
            "coordinate_frame": {},
            "map_context": {},
        }
        with self.assertRaisesRegex(ValueError, "source.scene_token is required"):
            build_scene_package(
                scene_ir,
                scene_ir_path="scene_ir.json",
                openscenario_path="scenario.xosc",
                opendrive_path=None,
                map_source="fixture",
            )

    def test_rejects_nuscenes_scenario_id_that_differs_from_scene_token(self):
        from adapters.scene_package import build_scene_package

        scene_ir = {
            "scenario_id": "scene-0061",
            "source": {
                "dataset": "nuscenes",
                "scene_name": "scene-0061",
                "scene_token": "cc8c0bf57f984915a77078b10eb33198",
            },
            "coordinate_frame": {},
            "map_context": {},
        }
        with self.assertRaisesRegex(ValueError, "must equal source.scene_token"):
            build_scene_package(
                scene_ir,
                scene_ir_path="scene_ir.json",
                openscenario_path="scenario.xosc",
                opendrive_path=None,
                map_source="fixture",
            )

    def test_builds_scene_handoff_without_requiring_nurec(self):
        from adapters.scene_package import build_scene_package

        scene_ir = {
            "scenario_id": "cc8c0bf57f984915a77078b10eb33198",
            "source": {
                "dataset": "nuscenes",
                "scene_name": "scene-0061",
                "scene_token": "cc8c0bf57f984915a77078b10eb33198",
            },
            "coordinate_frame": _coordinate_frame(),
            "map_context": {"location": "singapore-onenorth"},
        }
        package = build_scene_package(
            scene_ir,
            scene_ir_path="scene_ir.json",
            openscenario_path="scenario.xosc",
            opendrive_path="road.xodr",
            map_source="nuscenes_map_expansion",
        )

        self.assertEqual(package["schema_version"], "closed_loop_scene_package.v1")
        self.assertEqual(package["scene_id"], "cc8c0bf57f984915a77078b10eb33198")
        self.assertEqual(package["source"]["scene_name"], "scene-0061")
        self.assertEqual(package["source"]["dataset"], "nuscenes")
        self.assertEqual(package["map"]["location"], "singapore-onenorth")
        self.assertEqual(package["map"]["opendrive"], "road.xodr")
        self.assertIsNone(package["visual"]["nurec_usdz"])
        self.assertEqual(package["alignment"]["status"], "log_to_sim_defined")


if __name__ == "__main__":
    unittest.main()
