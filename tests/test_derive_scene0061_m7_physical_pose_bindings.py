import unittest


class DeriveM7PhysicalPoseBindingsTests(unittest.TestCase):
    def test_replay_control_is_retained_while_request_pose_becomes_physical(self):
        from runners.derive_scene0061_m7_physical_pose_bindings import derive_m7_physical_pose_bindings

        actor_ids = ["car", "ped"]
        config = {
            "actor_binding": {"selected_actor_ids": actor_ids},
            "actors": [
                {"actor_id": "car", "type": "vehicle", "closed_loop_level": "replay", "binding": {}},
                {"actor_id": "ped", "type": "pedestrian", "closed_loop_level": "replay", "binding": {}},
            ],
        }
        bindings = {
            "bindings": [
                {"actor_id": "car", "actor_type": "vehicle", "sensor_sync": {}},
                {"actor_id": "ped", "actor_type": "pedestrian", "sensor_sync": {}},
            ]
        }
        derived, sidecar = derive_m7_physical_pose_bindings(config, bindings)
        self.assertTrue(derived["runtime"]["m7_actor_pose_audit_required"])
        self.assertEqual(derived["actors"][0]["closed_loop_level"], "replay")
        self.assertEqual(derived["actors"][0]["binding"]["sensor_pose_source"], "carla_runtime_actor_pose")
        self.assertEqual(derived["actors"][1]["binding"]["sensor_pose_reference"], "carla_bounding_box_center")
        self.assertEqual(sidecar["bindings"][0]["sensor_sync"]["pose_reference"], "carla_bounding_box_center")
        self.assertEqual(sidecar["bindings"][0]["sensor_sync"]["replay_render_pose_mode"], "carla_runtime_physical")


if __name__ == "__main__":
    unittest.main()
