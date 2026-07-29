import unittest


def _registry():
    return {
        "schema_version": "scene_object_registry.v1",
        "scene_id": "scene-0061",
        "records": [
            {
                "object_id": "pedestrian",
                "role": "controlled_pedestrian",
                "safety_relevant": True,
                "time_interval": {"start_sec": 1.05, "end_sec": 2.0},
                "nurec": {"track_id": "ped-track"},
            },
            {
                "object_id": "lead",
                "role": "controlled_lead_vehicle",
                "safety_relevant": True,
                "time_interval": {"start_sec": 0.0, "end_sec": 2.0},
                "nurec": {"track_id": "lead-track"},
            },
        ],
    }


def _support(*, pedestrian_exact=(0, 3, 2, 4, 4), lead_exact=(2, 2, 2, 2, 2)):
    ends = (100_000, 150_000, 200_000, 250_000, 300_000)
    times = (0.05, 1.10, 1.15, 1.20, 1.25)
    rows = []
    for end, time, ped, lead in zip(ends, times, pedestrian_exact, lead_exact):
        rows.append(
            {
                "source_lidar_frame_end_us": end,
                "simulation_time_sec": time,
                "track_support": [
                    {
                        "track_id": "ped-track",
                        "annotation_status": "annotated_source_cuboid",
                        "source_cuboid_available": True,
                        "exact_box_hit_points": ped,
                        "padded_box_hit_points": max(ped, 5 if ped else 0),
                    },
                    {
                        "track_id": "lead-track",
                        "annotation_status": "annotated_source_cuboid",
                        "source_cuboid_available": True,
                        "exact_box_hit_points": lead,
                        "padded_box_hit_points": lead + 1,
                    },
                ],
            }
        )
    return {"schema_version": "ncore_dynamic_lidar_support_audit.v2", "source_lidar_frames": rows}


class LidarQualityWindowTests(unittest.TestCase):
    def test_lifecycle_and_consecutive_quality_window(self):
        from adapters.lidar_quality_windows import build_lidar_quality_window_manifest

        report = build_lidar_quality_window_manifest(
            _registry(),
            _support(),
            min_exact_points=1,
            min_padded_points=1,
            min_consecutive_frames=3,
        )
        self.assertEqual(report["status"], "passed")
        pedestrian = next(row for row in report["tracks"] if row["object_id"] == "pedestrian")
        self.assertEqual(pedestrian["active_frame_count"], 4)
        self.assertEqual(pedestrian["editable_frame_indices"], [1, 2, 3, 4])
        self.assertEqual(pedestrian["editable_windows"][0]["frame_count"], 4)
        self.assertEqual(pedestrian["frames"][1]["editable_quality_window"], True)
        self.assertEqual(pedestrian["frames"][1]["lidar_world_closed_loop_eligible"], True)
        self.assertEqual(report["window_semantics"]["name"], "editable_quality_window")
        self.assertEqual(report["window_semantics"]["display_name"], "local_lidar_editable_window")
        self.assertEqual(report["window_semantics"]["scope"], "ego_corridor_actor_interaction")
        self.assertTrue(report["window_semantics"]["lidar_world_closed_loop_claim_allowed_only_inside_window"])
        self.assertEqual(
            sorted(report["frames"][0]["track_support"]),
            ["lead-track", "ped-track"],
        )

    def test_sparse_returns_do_not_become_editable(self):
        from adapters.lidar_quality_windows import build_lidar_quality_window_manifest

        report = build_lidar_quality_window_manifest(
            _registry(),
            _support(pedestrian_exact=(0, 1, 0, 1, 0)),
            min_exact_points=1,
            min_padded_points=1,
            min_consecutive_frames=3,
        )
        self.assertEqual(report["status"], "failed")
        pedestrian = next(row for row in report["tracks"] if row["object_id"] == "pedestrian")
        self.assertEqual(pedestrian["editable_windows"], [])
        self.assertIn("pedestrian:required_object_has_no_editable_quality_window", report["issues"])

    def test_missing_time_mapping_fails_closed(self):
        from adapters.lidar_quality_windows import build_lidar_quality_window_manifest

        support = _support()
        for row in support["source_lidar_frames"]:
            row.pop("simulation_time_sec")
        report = build_lidar_quality_window_manifest(_registry(), support)
        self.assertEqual(report["status"], "failed")
        self.assertTrue(any("simulation_time_sec_missing" in issue for issue in report["issues"]))

    def test_required_manifest_validation_preserves_physics_separation(self):
        from adapters.lidar_quality_windows import (
            build_lidar_quality_window_manifest,
            validate_lidar_quality_window_manifest,
        )

        report = build_lidar_quality_window_manifest(_registry(), _support())
        validate_lidar_quality_window_manifest(
            report,
            scene_id="scene-0061",
            required_object_ids=["pedestrian", "lead"],
        )
        self.assertTrue(report["policy"]["quality_is_not_a_carla_physics_filter"])
        self.assertTrue(report["policy"]["registry_objects_preserved"])
        self.assertTrue(report["policy"]["local_window_is_not_complete_scene_scope"])
        self.assertTrue(report["policy"]["ego_corridor_selection_is_not_object_deletion"])
        self.assertEqual(report["editable_quality_windows"], report["editable_windows"])


if __name__ == "__main__":
    unittest.main()
