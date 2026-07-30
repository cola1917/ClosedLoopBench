from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from PIL import Image

from runners.render_closed_loop_bev_snapshot import MAP_COLORS, RGB_COLORS, render_snapshot


XODR = """<?xml version=\"1.0\"?>
<OpenDRIVE>
  <road id=\"0\" length=\"200\">
    <planView><geometry x=\"0\" y=\"0\" hdg=\"0\" length=\"200\"><line /></geometry></planView>
    <lanes><laneSection s=\"0\"><right><lane id=\"-1\" type=\"driving\"><width a=\"3.5\" /></lane></right></laneSection></lanes>
  </road>
</OpenDRIVE>
"""


class RenderClosedLoopBevSnapshotTests(unittest.TestCase):
    def test_renders_every_runtime_actor_with_a_class_color(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            (root / "road.xodr").write_text(XODR, encoding="utf-8")
            frame = {
                "simulation_time_sec": 0.05,
                "ego_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "actor_states": {
                    "vehicle-1": {
                        "actor_type": "vehicle",
                        "pose": {"x": 5.0, "y": 1.0, "yaw": 0.0},
                        "extent_m": {"x": 2.0, "y": 1.0},
                    },
                    "pedestrian-1": {
                        "actor_type": "pedestrian",
                        "pose": {"x": 2.0, "y": -1.0, "yaw": 15.0},
                        "extent_m": {"x": 0.2, "y": 0.2},
                    },
                },
            }
            (root / "frame_trace.jsonl").write_text(
                json.dumps(frame) + "\n", encoding="utf-8"
            )
            output = root / "bev.png"
            result = render_snapshot(
                run_dir=root,
                opendrive_path=root / "road.xodr",
                output_path=output,
                width=480,
                height=320,
            )

            colors = set(Image.open(output).convert("RGB").get_flattened_data())

        self.assertEqual(result["runtime_actor_count"], 2)
        self.assertEqual(result["runtime_actor_type_counts"], {"vehicle": 1, "pedestrian": 1})
        self.assertIn(RGB_COLORS["ego"], colors)
        self.assertIn(RGB_COLORS["vehicle"], colors)
        self.assertIn(RGB_COLORS["pedestrian"], colors)

    def test_prefers_direct_nuscenes_polygons_for_visual_road_geometry(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            frame = {
                "simulation_time_sec": 0.05,
                "ego_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "actor_states": {},
            }
            (root / "frame_trace.jsonl").write_text(
                json.dumps(frame) + "\n", encoding="utf-8"
            )
            nodes = {
                "drive": [(-6, -4), (12, -4), (12, 4), (-6, 4)],
                "road": [(-6, -2), (12, -2), (12, 2), (-6, 2)],
                "intersection": [(1, -2), (4, -2), (4, 2), (1, 2)],
                "lane": [(5, -1), (11, -1), (11, 1), (5, 1)],
            }
            node_rows = []
            polygon_rows = []
            for name, points in nodes.items():
                node_tokens = []
                for index, (x, y) in enumerate(points):
                    token = f"{name}-{index}"
                    node_rows.append({"token": token, "x": x, "y": y})
                    node_tokens.append(token)
                polygon_rows.append({"token": name, "exterior_node_tokens": node_tokens})
            (root / "map.json").write_text(
                json.dumps(
                    {
                        "node": node_rows,
                        "polygon": polygon_rows,
                        "drivable_area": [{"polygon_tokens": ["drive"]}],
                        "road_block": [],
                        "road_segment": [
                            {"polygon_token": "road", "is_intersection": False},
                            {"polygon_token": "intersection", "is_intersection": True},
                        ],
                        "lane": [{"lane_type": "CAR", "polygon_token": "lane"}],
                    }
                ),
                encoding="utf-8",
            )
            (root / "scene_ir.json").write_text(
                json.dumps(
                    {
                        "coordinate_frame": {
                            "origin_global_translation": [0.0, 0.0, 0.0],
                            "origin_global_yaw_deg": 0.0,
                        }
                    }
                ),
                encoding="utf-8",
            )
            output = root / "nuscenes-bev.png"
            result = render_snapshot(
                run_dir=root,
                nuscenes_map_path=root / "map.json",
                scenario_ir_path=root / "scene_ir.json",
                output_path=output,
                width=480,
                height=320,
            )
            colors = set(Image.open(output).convert("RGB").get_flattened_data())

        self.assertEqual(result["map_source"], "nuscenes_map_geometry_visual_only")
        self.assertEqual(
            result["map_feature_counts"],
            {
                "drivable_area": 1,
                "road_block": 0,
                "road_segment": 1,
                "intersection": 1,
                "lane": 1,
            },
        )
        self.assertIn(MAP_COLORS["road_segment"], colors)
        self.assertIn(MAP_COLORS["intersection"], colors)

    def test_can_limit_displayed_actors_without_changing_runtime_counts(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            (root / "road.xodr").write_text(XODR, encoding="utf-8")
            frame = {
                "ego_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "actor_states": {
                    "near": {
                        "actor_type": "vehicle",
                        "pose": {"x": 5.0, "y": 0.0, "yaw": 0.0},
                        "extent_m": {"x": 2.0, "y": 1.0},
                    },
                    "far": {
                        "actor_type": "pedestrian",
                        "pose": {"x": 50.0, "y": 0.0, "yaw": 0.0},
                        "extent_m": {"x": 0.2, "y": 0.2},
                    },
                },
            }
            (root / "frame_trace.jsonl").write_text(
                json.dumps(frame) + "\n", encoding="utf-8"
            )
            result = render_snapshot(
                run_dir=root,
                opendrive_path=root / "road.xodr",
                output_path=root / "limited.png",
                max_actor_distance_m=20.0,
            )

        self.assertEqual(result["runtime_actor_count"], 2)
        self.assertEqual(result["displayed_actor_count"], 1)
        self.assertEqual(result["displayed_actor_type_counts"], {"vehicle": 1})


if __name__ == "__main__":
    unittest.main()
