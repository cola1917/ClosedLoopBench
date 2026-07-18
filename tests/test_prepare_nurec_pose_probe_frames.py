import json
from pathlib import Path
import tempfile
import unittest

from adapters.nurec_multimodal import validate_nurec_multimodal_frame
from runners.prepare_nurec_pose_probe_frames import prepare_probe_frames
from tests.test_actor_binding import VEHICLE_TRACK


class PrepareNuRecPoseProbeFramesTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.root = Path(self.tempdir.name)
        version = self.root / "v1.0-mini"
        version.mkdir()
        scene_id = "cc8c0bf57f984915a77078b10eb33198"
        self._write("scene", [{"token": scene_id, "name": "scene-0061", "first_sample_token": "s0", "nbr_samples": 2}])
        self._write(
            "sample",
            [
                {"token": "s0", "timestamp": 1_000_000, "next": "s1"},
                {"token": "s1", "timestamp": 1_500_000, "next": ""},
            ],
        )
        self._write(
            "sample_data",
            [
                {"token": "cam0", "sample_token": "s0", "is_key_frame": True, "timestamp": 1_000_000, "ego_pose_token": "ego0", "calibrated_sensor_token": "cal_cam"},
                {"token": "lid0", "sample_token": "s0", "is_key_frame": True, "timestamp": 1_000_000, "ego_pose_token": "ego0", "calibrated_sensor_token": "cal_lid"},
                {"token": "cam1", "sample_token": "s1", "is_key_frame": True, "timestamp": 1_500_000, "ego_pose_token": "ego1", "calibrated_sensor_token": "cal_cam"},
                {"token": "lid1", "sample_token": "s1", "is_key_frame": True, "timestamp": 1_500_000, "ego_pose_token": "ego1", "calibrated_sensor_token": "cal_lid"},
            ],
        )
        self._write(
            "sample_annotation",
            [
                {"token": "a0", "sample_token": "s0", "instance_token": VEHICLE_TRACK, "translation": [20, 0, 0], "rotation": [1, 0, 0, 0]},
                {"token": "a1", "sample_token": "s1", "instance_token": VEHICLE_TRACK, "translation": [11, 0, 0], "rotation": [1, 0, 0, 0]},
            ],
        )
        self._write(
            "ego_pose",
            [
                {"token": "ego0", "translation": [0, 0, 0], "rotation": [1, 0, 0, 0]},
                {"token": "ego1", "translation": [10, 0, 0], "rotation": [1, 0, 0, 0]},
            ],
        )
        self._write(
            "calibrated_sensor",
            [
                {"token": "cal_cam", "sensor_token": "sensor_cam", "translation": [1, 0, 1], "rotation": [1, 0, 0, 0]},
                {"token": "cal_lid", "sensor_token": "sensor_lid", "translation": [0, 0, 2], "rotation": [1, 0, 0, 0]},
            ],
        )
        self._write(
            "sensor",
            [
                {"token": "sensor_cam", "channel": "CAM_FRONT", "modality": "camera"},
                {"token": "sensor_lid", "channel": "LIDAR_TOP", "modality": "lidar"},
            ],
        )

    def tearDown(self):
        self.tempdir.cleanup()

    def _write(self, name, value):
        (self.root / "v1.0-mini" / f"{name}.json").write_text(json.dumps(value), encoding="utf-8")

    def test_uses_closest_sample_and_real_sensor_extrinsics(self):
        baseline, moved, context = prepare_probe_frames(
            self.root,
            version="v1.0-mini",
            scene_name="scene-0061",
            track_id=VEHICLE_TRACK,
            actor_type="vehicle",
            camera_channels=["CAM_FRONT"],
            runtime_scene_start_us=900_000,
        )
        validate_nurec_multimodal_frame(baseline)
        validate_nurec_multimodal_frame(moved)
        self.assertEqual(context["sample_token"], "s1")
        self.assertEqual(context["scene_start_us"], 900_000)
        self.assertEqual(context["sample_time_origin_us"], 1_000_000)
        self.assertAlmostEqual(context["actor_ego_distance_m"], 1.0)
        camera_pose = baseline["modalities"]["rgb"]["requests"][0]["sensor"]["pose_pair"]["start"]
        self.assertEqual(camera_pose["position_m"], {"x": 11.0, "y": 0.0, "z": 1.0})
        lidar_pose = baseline["modalities"]["lidar"]["requests"][0]["sensor"]["pose_pair"]["start"]
        self.assertEqual(lidar_pose["position_m"], {"x": 10.0, "y": 0.0, "z": 2.0})
        self.assertNotEqual(
            baseline["shared_dynamic_object_sha256"], moved["shared_dynamic_object_sha256"]
        )

    def test_rejects_sub_threshold_pose_delta(self):
        with self.assertRaisesRegex(ValueError, "at least 0.05"):
            prepare_probe_frames(
                self.root,
                version="v1.0-mini",
                scene_name="scene-0061",
                track_id=VEHICLE_TRACK,
                actor_type="vehicle",
                camera_channels=["CAM_FRONT"],
                delta_m=0.01,
            )


if __name__ == "__main__":
    unittest.main()
