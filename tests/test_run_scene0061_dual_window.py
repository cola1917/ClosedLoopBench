from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from runners.run_scene0061_dual_window import (
    ActorState,
    FramePacket,
    RoadGeometry,
    _actor_state,
    _bbox_corners,
    _carla_display_contract,
    _changed_tracks,
    _dynamic_delta,
    _map_display_label,
    _render_carla_window,
    _scene_yaw_radians,
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

    def test_bbox_uses_scene_ir_degree_yaw(self) -> None:
        actor = ActorState(
            track_id="track-a",
            actor_type="vehicle",
            carla_actor_id=29,
            x=0.0,
            y=0.0,
            z=0.0,
            yaw=90.0,
            speed_mps=0.0,
            length=4.0,
            width=2.0,
            height=1.5,
            controlled=False,
            trajectory=(),
        )

        corners = _bbox_corners(actor)

        self.assertAlmostEqual(_scene_yaw_radians(actor.yaw), 1.57079632679)
        self.assertAlmostEqual(corners[0][0], 1.0)
        self.assertAlmostEqual(corners[0][1], -2.0)
        self.assertAlmostEqual(corners[2][0], -1.0)
        self.assertAlmostEqual(corners[2][1], 2.0)

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
        self.assertEqual(roads[0].points[0], (0.0, 0.0))
        self.assertAlmostEqual(roads[0].points[-1][0], 10.0)
        self.assertGreater(roads[1].points[-1][1], 0.0)
        self.assertEqual(roads[0].width_m, 3.5)

    def test_scene_package_map_contract_requires_road_xodr(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            path = Path(raw_root) / "road.xodr"
            path.write_text("<OpenDRIVE/>", encoding="utf-8")
            report = _validate_map_contract(
                {"map": {"opendrive": "road.xodr", "source": "nuscenes_map_expansion"}},
                path,
            )
        self.assertEqual(report["status"], "matched")

    def test_state_explainer_renders_lane_surface_compact_hud_and_controlled_trace(self) -> None:
        try:
            import cv2
            import numpy as np
            using_real_cv2 = True
        except ImportError:
            import numpy as np

            using_real_cv2 = False

            class Cv2Recorder:
                LINE_AA = 16
                FONT_HERSHEY_SIMPLEX = 0

                def __init__(self) -> None:
                    self.calls: list[tuple[str, tuple]] = []

                def fillPoly(self, *args, **kwargs) -> None:
                    self.calls.append(("fillPoly", args))

                def polylines(self, *args, **kwargs) -> None:
                    self.calls.append(("polylines", args))

                def line(self, *args, **kwargs) -> None:
                    self.calls.append(("line", args))

                def arrowedLine(self, *args, **kwargs) -> None:
                    self.calls.append(("arrowedLine", args))

                def putText(self, *args, **kwargs) -> None:
                    self.calls.append(("putText", args))

                def rectangle(self, *args, **kwargs) -> None:
                    self.calls.append(("rectangle", args))

            cv2 = Cv2Recorder()

        ego = ActorState(
            track_id="ego",
            actor_type="ego",
            carla_actor_id=25,
            x=0.0,
            y=0.0,
            z=0.0,
            yaw=0.0,
            speed_mps=2.0,
            length=4.6,
            width=1.9,
            height=1.6,
            controlled=False,
            trajectory=((0.0, 0.0),),
        )
        controlled = ActorState(
            track_id="c1958768d48640948f6053d04cffd35b",
            actor_type="vehicle",
            carla_actor_id=29,
            x=8.0,
            y=1.0,
            z=0.0,
            yaw=0.0,
            speed_mps=3.2,
            length=4.7,
            width=1.9,
            height=1.8,
            controlled=True,
            trajectory=((2.0, 1.0), (5.0, 1.0), (8.0, 1.0)),
        )
        packet = FramePacket(
            state_name="baseline",
            frame_id=38,
            simulation_time_sec=19.149566,
            timestamp_us=1532402946747716,
            ego=ego,
            actors=(controlled,),
            cameras={},
            camera_jpegs={},
            camera_records=(),
        )
        roads = [
            RoadGeometry(
                road_id="1:0",
                points=((-30.0, 0.0), (30.0, 0.0)),
                width_m=4.0,
            )
        ]

        canvas = _render_carla_window(
            packet,
            roads,
            cv2=cv2,
            np=np,
            map_label="singapore-onenorth | road.xodr abcdef01",
        )

        self.assertEqual(canvas.shape, (720, 1280, 3))
        if using_real_cv2:
            self.assertGreater(np.count_nonzero(np.all(canvas == (52, 57, 64), axis=2)), 100)
            self.assertGreater(np.count_nonzero(np.all(canvas == (0, 178, 255), axis=2)), 20)
        else:
            names = [name for name, _ in cv2.calls]
            text = [args[1] for name, args in cv2.calls if name == "putText"]
            self.assertIn("fillPoly", names)
            self.assertIn("arrowedLine", names)
            self.assertIn("line", names)
            self.assertTrue(any("CARLA STATE / OPENDRIVE" in value for value in text))
            self.assertTrue(any("CONTROLLED  CARLA 29" in value for value in text))

    def test_map_display_label_carries_location_source_and_identity_prefix(self) -> None:
        label = _map_display_label(
            {
                "location": "singapore-onenorth",
                "map_source": "nuscenes_map_expansion",
                "selected_sha256": "abcdef0123456789",
            }
        )
        self.assertEqual(
            label,
            "singapore-onenorth | nuscenes_map_expansion | road.xodr abcdef01",
        )

    def test_carla_display_contract_distinguishes_state_explanation_from_sensor_output(self) -> None:
        contract = _carla_display_contract()

        self.assertEqual(contract["purpose"], "world_state_explanation_not_camera_sensor_output")
        self.assertEqual(contract["map"], "width_derived_opendrive_local_driving_lanes")
        self.assertEqual(contract["canvas"], {"width": 1280, "height": 720})
        self.assertIn("controlled_nurec_track_id", contract["annotations"]["hud"])


if __name__ == "__main__":
    unittest.main()
