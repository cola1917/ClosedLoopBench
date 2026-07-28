import unittest


class StaticPlacementAuditTests(unittest.TestCase):
    def test_requires_collision_runtime_and_binds_visible_projection(self):
        from adapters.static_placement_audit import audit_static_placement

        registry = {"scene_id": "scene", "records": [{"object_id": "truck", "semantic_class": "vehicle", "role": "static_obstacle", "carla": {"placement": {"x": 1}, "collision_policy": "required"}}]}
        runtime = {"static_records": [{"object_id": "truck", "carla_runtime_actor_id": 9, "status": "passed"}]}
        visibility = {"observations": [{"object_id": "truck", "observation_kind": "calibrated_3d_box_projection", "evidence": {"nre_payload_sha256": "a", "calibrated_sensor_token": "b"}}]}
        report = audit_static_placement(registry, runtime, visibility)
        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["geometrically_observed_static_count"], 1)


if __name__ == "__main__":
    unittest.main()
