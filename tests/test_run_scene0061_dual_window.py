from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from runners.run_scene0061_dual_window import (
    _actor_state,
    _bbox_corners,
    _changed_tracks,
    _dynamic_delta,
    _same_frame_gate,
    _sample_xodr,
    _validate_map_contract,
)


def _frame(track_id: str, x: float) -> dict:
    return {
        "scene_id": "scene-0061",
        "frame_id": 38,
        "simulation_time_sec": 19.149566,
        "pose_interval_sec": {"start": 19.1, "end": 19.2},
        "shared_dynamic_objects": [
            {
                "track_id": track_id,
                "pose_pair": {
                    "start": {"position_m": {"x": x, "y": 2.0, "z": 1.0}},
                    "end": {"position_m": {"x": x, "y": 2.0, "z": 1.0}},
                },
            }
        ],
    }


class Scene0061DualWindowTests(unittest.TestCase):
    def test_same_frame_delta_and_changed_track(self) -> None:
        track_id = "c1958768d48640948f6053d04cffd35b"
        baseline = _frame(track_id, 10.0)
        moved = _frame(track_id, 11.0)

        _same_frame_gate(baseline, moved)
        self.assertEqual(_changed_tracks(baseline, moved), [track_id])
        self.assertEqual(_dynamic_delta(baseline, moved, track_id), (1.0, 0.0, 0.0))

    def test_actor_state_carries_runtime_identity_and_bbox(self) -> None:
        source = {
            "actor_id": "track-a",
            "type": "vehicle",
            "dimensions": {"length": 4.0, "width": 2.0, "height": 1.5},
            "reference_trajectory": [
                {"t_sec": 0.0, "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0, "speed_mps": 3.0}
            ],
        }
        actor = _actor_state(
            source,
            0.0,
            {"track-a": {"runtime_actor_id": 29}},
            controlled=True,
        )

        self.assertEqual(actor.carla_actor_id, 29)
        self.assertTrue(actor.controlled)
        self.assertEqual(
            _bbox_corners(actor),
            [(-1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (-1.0, 3.0)],
        )

    def test_samples_line_and_arc_opendrive(self) -> None:
        xml = """<OpenDRIVE><road><planView>
        <geometry x="0" y="0" hdg="0" length="10"><line/></geometry>
        <geometry x="10" y="0" hdg="0" length="5"><arc curvature="0.1"/></geometry>
        </planView></road></OpenDRIVE>"""
        with tempfile.TemporaryDirectory() as raw_root:
            path = Path(raw_root) / "road.xodr"
            path.write_text(xml, encoding="utf-8")
            roads = _sample_xodr(path)

        self.assertEqual(len(roads), 2)
        self.assertEqual(roads[0][0], (0.0, 0.0))
        self.assertAlmostEqual(roads[0][-1][0], 10.0)
        self.assertGreater(roads[1][-1][1], 0.0)

    def test_scene_package_map_contract_requires_road_xodr(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            path = Path(raw_root) / "road.xodr"
            path.write_text("<OpenDRIVE/>", encoding="utf-8")
            report = _validate_map_contract(
                {"map": {"opendrive": "road.xodr", "source": "nuscenes_map_expansion"}},
                path,
            )
        self.assertEqual(report["status"], "matched")


if __name__ == "__main__":
    unittest.main()
