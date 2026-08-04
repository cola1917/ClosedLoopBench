from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from PIL import Image

from runners.render_closed_loop_bev_snapshot import RGB_COLORS
from runners.render_six_camera_bev_snapshot import CAMERA_ORDER, render_six_camera_bev_snapshot


XODR = """<?xml version=\"1.0\"?>
<OpenDRIVE>
  <road id=\"0\" length=\"200\">
    <planView><geometry x=\"0\" y=\"0\" hdg=\"0\" length=\"200\"><line /></geometry></planView>
    <lanes><laneSection s=\"0\"><right><lane id=\"-1\" type=\"driving\"><width a=\"3.5\" /></lane></right></laneSection></lanes>
  </road>
</OpenDRIVE>
"""


def _pixels(image: Image.Image):
    getter = getattr(image, "get_flattened_data", None)
    return getter() if callable(getter) else list(image.getdata())


class RenderSixCameraBevSnapshotTests(unittest.TestCase):
    def test_renders_hash_bound_six_camera_grid_with_calibrated_box_outlines(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            payload_root = root / "payload_root"
            camera_dir = payload_root / "frames"
            camera_dir.mkdir(parents=True)
            cameras = []
            for index, camera_name in enumerate(CAMERA_ORDER):
                path = camera_dir / f"{camera_name}.jpg"
                Image.new("RGB", (320, 180), (30 + index * 10, 45, 60)).save(path)
                cameras.append(
                    {
                        "sensor_id": camera_name,
                        "relative_path": f"frames/{camera_name}.jpg",
                        "payload_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    }
                )
            frame = {
                "world_tick_frame": 123,
                "simulation_time_sec": 0.05,
                "ego_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "actor_states": {
                    "vehicle-1": {
                        "actor_type": "vehicle",
                        "pose": {"x": 5.0, "y": 0.0, "yaw": 0.0},
                        "extent_m": {"x": 2.0, "y": 1.0},
                    }
                },
            }
            (root / "frame_trace.jsonl").write_text(json.dumps(frame) + "\n", encoding="utf-8")
            (root / "road.xodr").write_text(XODR, encoding="utf-8")
            audit = {
                "frames": [
                    {
                        "world_tick_frame": 123,
                        "simulation_time_sec": 0.05,
                        "cameras": cameras,
                    }
                ],
                "observations": [
                    {
                        "object_id": "vehicle-1",
                        "camera": "camera_front",
                        "frame_id": 123,
                        "safety_relevant": True,
                        "observation_kind": "calibrated_3d_box_projection",
                        "projection": {
                            "bbox_xyxy_px": [80.0, 40.0, 220.0, 150.0],
                            "distance_to_ego_m": 5.0,
                        },
                    },
                    {
                        "object_id": "unbound-static",
                        "camera": "camera_front",
                        "frame_id": 123,
                        "safety_relevant": True,
                        "observation_kind": "calibrated_3d_box_projection",
                        "projection": {
                            "bbox_xyxy_px": [10.0, 20.0, 70.0, 80.0],
                            "distance_to_ego_m": 5.0,
                        },
                    },
                ],
            }
            audit_path = root / "visibility_audit.json"
            audit_path.write_text(json.dumps(audit), encoding="utf-8")
            output = root / "snapshot.png"
            result = render_six_camera_bev_snapshot(
                run_dir=root,
                visibility_audit_path=audit_path,
                payload_root=payload_root,
                opendrive_path=root / "road.xodr",
                output_path=output,
                camera_cell_width=160,
                bev_width=320,
            )
            colors = set(_pixels(Image.open(output).convert("RGB")))
            metadata = json.loads(output.with_suffix(".json").read_text(encoding="utf-8"))

        self.assertEqual(result["world_tick_frame"], 123)
        self.assertEqual(result["max_distance_m"], 20.0)
        self.assertFalse(result["include_unbound_objects"])
        self.assertEqual(result["calibrated_bbox_candidate_count"], 2)
        self.assertEqual(result["excluded_unbound_count"], 1)
        self.assertEqual(result["calibrated_bbox_count"], 1)
        self.assertEqual(result["map_source"], "xodr_lane_strips_visual_only")
        self.assertEqual(metadata["camera_payloads"]["camera_front"]["calibrated_bbox_count"], 1)
        self.assertIn(RGB_COLORS["vehicle"], colors)


if __name__ == "__main__":
    unittest.main()
