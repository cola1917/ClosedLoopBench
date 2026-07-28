import unittest


class DeriveM8SafetyConfigTests(unittest.TestCase):
    def test_all_actor_control_contracts_follow_runtime_binding_reference(self):
        from runners.derive_scene0061_m8_safety_config import derive_m8_safety_config

        source = {
            "schema_version": "carla_run_config.mvp.v0",
            "runtime": {"m7_actor_pose_audit_required": True},
            "actor_control_contract": {
                "actors": [
                    {
                        "actor_id": "vehicle",
                        "sensor_pose_source": "scenario_ir_reference_trajectory",
                        "sensor_pose_reference": "source_track_frame",
                    },
                    {
                        "actor_id": "pedestrian",
                        "sensor_pose_source": "scenario_ir_reference_trajectory",
                        "sensor_pose_reference": "source_track_frame",
                    },
                ]
            },
            "actors": [
                {
                    "actor_id": "vehicle",
                    "source_track_id": "vehicle",
                    "type": "vehicle",
                    "closed_loop_level": "replay",
                    "control_mode_contract": {
                        "runner_executor": "trajectory_replay_vehicle_control",
                        "sensor_pose_reference": "source_track_frame",
                    },
                },
                {
                    "actor_id": "pedestrian",
                    "source_track_id": "pedestrian",
                    "type": "pedestrian",
                    "closed_loop_level": "replay",
                    "control_mode_contract": {
                        "runner_executor": "trajectory_replay_walker_control",
                        "sensor_pose_reference": "carla_bounding_box_bottom",
                    },
                },
            ],
        }

        derived = derive_m8_safety_config(source)

        for actor in derived["actors"]:
            self.assertEqual(
                actor["binding"]["sensor_pose_source"],
                "carla_runtime_actor_pose",
            )
            self.assertEqual(
                actor["binding"]["sensor_pose_reference"],
                "carla_bounding_box_center",
            )
            self.assertEqual(
                actor["control_mode_contract"]["sensor_pose_source"],
                "carla_runtime_actor_pose",
            )
            self.assertEqual(
                actor["control_mode_contract"]["sensor_pose_reference"],
                "carla_bounding_box_center",
            )
        for row in derived["actor_control_contract"]["actors"]:
            self.assertEqual(row["sensor_pose_source"], "carla_runtime_actor_pose")
            self.assertEqual(row["sensor_pose_reference"], "carla_bounding_box_center")


if __name__ == "__main__":
    unittest.main()
