import unittest


def _request(*, source="carla_runtime_actor_pose", reference="carla_bounding_box_center", offset=0.0):
    pose = {"x": 5.0, "y": 1.0, "z": 0.5, "roll": 0.0, "pitch": 0.0, "yaw": 10.0}
    return {
        "schema_version": "nurec_dynamic_pose_request.v1",
        "scene_id": "scene",
        "frame_id": 9,
        "tick_index": 2,
        "simulation_time_sec": 0.15,
        "pose_interval_sec": {"start": 0.10, "end": 0.15},
        "coordinate_contract": {
            "carla_to_nurec_global_transform": [
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ]
        },
        "actor_pose_pairs": [{
            "actor_id": "car",
            "nurec_track_id": "track",
            "actor_type": "vehicle",
            "carla_runtime_actor_id": 22,
            "carla_physical_pose_reference": "carla_bounding_box_center",
            "carla_physical_pose_pair": {"start": pose, "end": pose},
            "nurec_pose_source": source,
            "nurec_pose_reference": reference,
            "nurec_request_pose_pair": {
                endpoint: {
                    "position_m": {"x": 5.0 + offset, "y": 1.0, "z": 0.5},
                    "orientation_xyzw": {"x": 0.0, "y": 0.0, "z": 0.0871557427, "w": 0.9961946981},
                }
                for endpoint in ("start", "end")
            },
        }],
    }


class ActorPoseAuditTests(unittest.TestCase):
    def test_matching_runtime_pose_passes(self):
        from adapters.actor_pose_audit import audit_actor_pose_request

        rows = audit_actor_pose_request(_request(), expected_actor_ids={"car"})
        self.assertEqual(rows[0]["status"], "passed")
        self.assertAlmostEqual(rows[0]["endpoints"]["end"]["translation_error_m"], 0.0)

    def test_source_replay_pose_fails_even_when_coordinates_match(self):
        from adapters.actor_pose_audit import audit_actor_pose_request

        rows = audit_actor_pose_request(
            _request(source="scenario_ir_reference_trajectory", reference="source_track_frame")
        )
        self.assertEqual(rows[0]["status"], "failed")
        self.assertIn("nonphysical_nurec_pose_source", rows[0]["issues"])
        self.assertIn("physical_nurec_pose_reference_mismatch", rows[0]["issues"])

    def test_translation_threshold_fails(self):
        from adapters.actor_pose_audit import audit_actor_pose_request

        rows = audit_actor_pose_request(_request(offset=0.51))
        self.assertEqual(rows[0]["status"], "failed")
        self.assertIn("end:translation_threshold_exceeded", rows[0]["issues"])

    def test_declared_annotation_absence_is_not_a_missing_request_failure(self):
        from adapters.actor_pose_audit import audit_actor_pose_request

        request = _request()
        request["actor_absences"] = [{"actor_id": "ped", "reason": "outside_source_annotation_window"}]
        rows = audit_actor_pose_request(request, expected_actor_ids={"car", "ped"})
        pedestrian = next(row for row in rows if row["actor_id"] == "ped")
        self.assertEqual(pedestrian["status"], "not_applicable")


if __name__ == "__main__":
    unittest.main()
