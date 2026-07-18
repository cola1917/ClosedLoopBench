from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from runners.capture_runtime_alignment_observations import capture_observations


class _Location:
    def __init__(self, *, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Map:
    name = "OpenDriveMap"

    def get_waypoint(self, location, *, project_to_road, lane_type):
        self.last_query = (location, project_to_road, lane_type)
        return SimpleNamespace(
            transform=SimpleNamespace(
                location=location,
                rotation=SimpleNamespace(yaw=-10.0),
            ),
            road_id=1,
            lane_id=-1,
            is_junction=False,
            lane_width=3.7,
        )


class CaptureRuntimeAlignmentObservationsTests(unittest.TestCase):
    def test_converts_canonical_y_left_at_carla_boundary_and_hashes_artifact(self):
        package = {
            "scene_id": "cc8c0bf57f984915a77078b10eb33198",
            "alignment": {
                "sim_from_log_transform": [
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1,
                ]
            },
        }
        landmarks = [
            {
                "landmark_id": f"p{index}",
                "sample_token": f"s{index}",
                "sample_data_token": f"sd{index}",
                "timestamp_us": index,
                "log_global": {"x": x, "y": y, "z": 1.0, "yaw_deg": 10.0},
            }
            for index, (x, y) in enumerate(((0, 0), (1, 0), (0, 1)))
        ]
        carla_module = SimpleNamespace(
            Location=_Location, LaneType=SimpleNamespace(Driving="driving")
        )
        carla_map = _Map()
        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "last.usdz"
            artifact.write_bytes(b"artifact")
            result = capture_observations(
                package,
                landmarks,
                carla_map=carla_map,
                carla_module=carla_module,
                simulator_version="0.9.16",
                renderer_version="26.4.146",
                artifact_path=artifact,
                carla_frame=123,
            )

        self.assertEqual(result["capture"]["landmark_count"], 3)
        self.assertEqual(result["landmarks"][2]["sim_measured"]["y"], 1.0)
        self.assertEqual(result["landmarks"][0]["sim_measured"]["yaw_deg"], 10.0)
        self.assertEqual(len(result["runtime"]["nurec_artifact_sha256"]), 64)
        self.assertEqual(carla_map.last_query[0].y, -1.0)

    def test_uses_nurec_runtime_pose_for_3d_waypoint_query(self):
        package = {
            "scene_id": "cc8c0bf57f984915a77078b10eb33198",
            "alignment": {
                "sim_from_log_transform": [
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1,
                ]
            },
        }
        landmarks = []
        for index, (x, y) in enumerate(((0, 0), (1, 0), (0, 1))):
            landmarks.append(
                {
                    "landmark_id": f"p{index}",
                    "sample_token": f"s{index}",
                    "sample_data_token": f"sd{index}",
                    "timestamp_us": index,
                    "log_global": {"x": x, "y": y, "z": 0.0, "yaw_deg": 10.0},
                    "nurec_runtime": {
                        "x": x + 0.1,
                        "y": y + 0.2,
                        "z": 1.5,
                        "yaw_deg": 10.0,
                        "timestamp_us": index,
                        "pose_source": "nurec_usdz_rig_trajectory_interpolated",
                    },
                }
            )
        carla_module = SimpleNamespace(
            Location=_Location, LaneType=SimpleNamespace(Driving="driving")
        )
        carla_map = _Map()
        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "last.usdz"
            artifact.write_bytes(b"artifact")
            result = capture_observations(
                package,
                landmarks,
                carla_map=carla_map,
                carla_module=carla_module,
                simulator_version="0.9.16",
                renderer_version="26.4.146",
                artifact_path=artifact,
                carla_frame=123,
            )

        self.assertEqual(
            result["capture"]["runtime_reference"], "nurec_usdz_ego_trajectory"
        )
        self.assertEqual(result["capture"]["vertical_reference"], "nurec_usdz_ego_trajectory")
        self.assertEqual(carla_map.last_query[0].z, 1.5)
        self.assertEqual(carla_map.last_query[0].y, -1.2)


if __name__ == "__main__":
    unittest.main()
