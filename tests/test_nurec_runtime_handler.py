import unittest

from adapters.nurec_multimodal import (
    build_nurec_multimodal_evidence,
    materialize_nurec_rpc_requests,
)
from tests.test_actor_binding import SCENE_TOKEN, VEHICLE_TRACK
from tests.test_nurec_multimodal import _binding_set, _scene_package


IDENTITY = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
]


def _context():
    actor_start = {"x": 5.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    actor_end = {"x": 5.1, "y": 0.0, "z": 0.0, "yaw": 0.0}
    return {
        "schema_version": "carla_nurec_frame_context.v1",
        "scene_id": SCENE_TOKEN,
        "frame_id": 7,
        "tick_index": 6,
        "simulation_time_sec": 0.35,
        "interval_start_sec": 0.30,
        "ego_pose_pair": {
            "start": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
            "end": {"x": 1.1, "y": 2.0, "z": 0.0, "yaw": 0.0},
        },
        "actor_samples": {
            VEHICLE_TRACK: {
                "source": "carla_runtime_actor_pose",
                "pose_pair": {"start": actor_start, "end": actor_end},
            }
        },
        "clock": "carla_snapshot",
    }


class NuRecRuntimeHandlerTests(unittest.TestCase):
    def test_builds_calibrated_frame_and_returns_dispatch_evidence(self):
        from adapters.nurec_runtime_handler import make_nurec_sensor_frame_handler

        captured = []

        def dispatch(frame):
            captured.append(frame)
            responses = [
                {
                    "request_id": payload["request_id"],
                    "status": "ok",
                    "frame_id": payload["frame_id"],
                    "dynamic_object_sha256": payload["dynamic_object_sha256"],
                    "payload_sha256": "e" * 64,
                    "latency_ms": 4.0,
                }
                for payload in materialize_nurec_rpc_requests(frame)
            ]
            return build_nurec_multimodal_evidence(frame, responses)

        camera_extrinsic = list(IDENTITY)
        camera_extrinsic[3] = 1.0
        camera_extrinsic[11] = 1.5
        handler = make_nurec_sensor_frame_handler(
            _scene_package(),
            _binding_set(),
            camera_specs=[{
                "sensor_id": "camera_front",
                "model": "recorded_pinhole",
                "width": 1600,
                "height": 900,
                "sensor_to_ego": camera_extrinsic,
            }],
            lidar_specs=[{
                "sensor_id": "lidar_top",
                "model": "remote_verified_profile",
                "sensor_to_ego": IDENTITY,
            }],
            dispatch_frame=dispatch,
        )
        evidence = handler(_context())

        self.assertEqual(evidence["status"], "passed")
        self.assertEqual(handler.runtime_contract["clock"], "carla_snapshot")
        frame = captured[0]
        camera = frame["modalities"]["rgb"]["requests"][0]["sensor"]
        lidar = frame["modalities"]["lidar"]["requests"][0]["sensor"]
        camera_position = camera["pose_pair"]["end"]["position_m"]
        lidar_position = lidar["pose_pair"]["end"]["position_m"]
        self.assertAlmostEqual(camera_position["x"] - lidar_position["x"], 1.0)
        self.assertAlmostEqual(camera_position["z"] - lidar_position["z"], 1.5)

    def test_requires_explicit_sensor_calibration_and_scene_identity(self):
        from adapters.nurec_multimodal import NuRecMultimodalError
        from adapters.nurec_runtime_handler import make_nurec_sensor_frame_handler

        with self.assertRaisesRegex(NuRecMultimodalError, "sensor_to_ego"):
            make_nurec_sensor_frame_handler(
                _scene_package(),
                _binding_set(),
                camera_specs=[{"sensor_id": "cam", "model": "pinhole"}],
                lidar_specs=[{"sensor_id": "lidar", "model": "verified", "sensor_to_ego": IDENTITY}],
                dispatch_frame=lambda frame: {},
            )

        handler = make_nurec_sensor_frame_handler(
            _scene_package(),
            _binding_set(),
            camera_specs=[{"sensor_id": "cam", "model": "pinhole", "sensor_to_ego": IDENTITY}],
            lidar_specs=[{"sensor_id": "lidar", "model": "verified", "sensor_to_ego": IDENTITY}],
            dispatch_frame=lambda frame: {},
        )
        context = _context()
        context["scene_id"] = "f" * 32
        with self.assertRaisesRegex(NuRecMultimodalError, "scene_id"):
            handler(context)


if __name__ == "__main__":
    unittest.main()
