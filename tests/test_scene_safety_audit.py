import json
import tempfile
import unittest
from pathlib import Path


class SceneSafetyAuditTests(unittest.TestCase):
    def test_lane_audit_rejects_missing_sensor_and_invasion(self):
        from adapters.scene_safety_audit import audit_lane_tick

        base = {
            "frame_id": 1,
            "simulation_time_sec": 0.05,
            "lane_state": {
                "road_id": 3,
                "section_id": 0,
                "lane_id": -1,
                "lane_type": "Driving",
                "is_on_road": True,
                "inside_lane": True,
                "lane_width_m": 3.5,
                "center_distance_m": 0.2,
                "route_progress": 0.1,
                "lane_invasion_events": [],
                "lane_invasion_sensor_available": True,
            },
        }
        self.assertEqual(audit_lane_tick(base)["status"], "passed")
        base["lane_state"]["lane_invasion_events"] = [{"sensor_event_frame": 1}]
        failed = audit_lane_tick(base)
        self.assertEqual(failed["status"], "failed")
        self.assertIn("lane_invasion", failed["issues"])

    def test_writer_requires_all_four_streams_and_writes_jsonl(self):
        from runners.audit_scene_safety import audit_m8_evidence, write_m8_evidence

        registry = {
            "records": [
                {
                    "object_id": "truck",
                    "role": "static_obstacle",
                    "carla": {"collision_policy": "required"},
                }
            ]
        }
        raw = {
            "frame_id": 1,
            "simulation_time_sec": 0.05,
            "collision": {
                "ego_state": {
                    "pose": {"x": 0, "y": 0, "z": 0, "yaw": 0},
                    "extent_m": {"x": 1, "y": 1, "z": 1},
                },
                "object_states": [
                    {
                        "object_id": "truck",
                        "carla_runtime_actor_id": 4,
                        "pose": {"x": 5, "y": 0, "z": 0, "yaw": 0},
                        "extent_m": {"x": 1, "y": 1, "z": 1},
                    }
                ],
                "collision_events": [],
                "collision_detected": False,
            },
            "lane": {
                "lane_state": {
                    "road_id": 1,
                    "lane_id": -1,
                    "lane_type": "Driving",
                    "is_on_road": True,
                    "inside_lane": True,
                    "lane_width_m": 3.5,
                    "center_distance_m": 0.1,
                    "route_progress": 0.2,
                    "lane_invasion_events": [],
                    "lane_invasion_sensor_available": True,
                }
            },
            "visibility": {
                "projections": [
                    {
                        "object_id": "truck",
                        "camera": "camera_front",
                        "observation_kind": "calibrated_3d_box_projection",
                        "projection": {"bbox_xyxy_px": [1, 2, 3, 4]},
                        "evidence": {
                            "nre_payload_sha256": "a" * 64,
                            "calibrated_sensor_token": "front",
                            "intrinsics_table_sha256": "b" * 64,
                        },
                    }
                ]
            },
            "lidar_world": {
                "expected_world_objects": [
                    {"object_id": "truck", "expected_lidar_support": True}
                ],
                "lidar_occupancy": [{"object_id": "truck", "point_count": 2}],
            },
        }
        streams = audit_m8_evidence(registry, [raw])
        with tempfile.TemporaryDirectory() as directory:
            summary = write_m8_evidence(streams, Path(directory))
            self.assertEqual(summary["status"], "passed")
            for name in (
                "collision_audit.v1.jsonl",
                "lane_audit.v1.jsonl",
                "visibility_audit.v1.jsonl",
                "lidar_world_audit.v1.jsonl",
            ):
                path = Path(directory) / name
                self.assertEqual(len(path.read_text(encoding="utf-8").splitlines()), 1)
                self.assertIsInstance(json.loads(path.read_text(encoding="utf-8")), dict)


if __name__ == "__main__":
    unittest.main()
