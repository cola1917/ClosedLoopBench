import unittest


def _registry():
    return {
        "schema_version": "scene_object_registry.v1",
        "scene_id": "scene-0061",
        "records": [
            {"object_id": "lead", "role": "controlled_lead_vehicle", "semantic_class": "vehicle", "safety_relevant": True, "nurec": {"track_id": "lead-track"}},
            {"object_id": "background", "role": "background_replay", "semantic_class": "vehicle", "safety_relevant": True, "nurec": {"track_id": "background-track"}},
            {"object_id": "far", "role": "background_replay", "semantic_class": "vehicle", "safety_relevant": True, "nurec": {"track_id": "far-track"}},
            {"object_id": "static", "role": "static_obstacle", "semantic_class": "vehicle", "safety_relevant": True, "source": {"source_track_id": "static-track"}, "carla": {"collision_policy": "required"}},
            {"object_id": "road", "role": "road_boundary", "semantic_class": "road_boundary", "safety_relevant": True, "nurec": {"track_id": None}},
        ],
    }


def _quality():
    rows = [
        ("lead-track", "ncore_dynamic_lidar_supported", 10, 10, 10, 20, 40),
        ("background-track", "ncore_dynamic_lidar_sparse", 8, 7, 7, 12, 20),
        ("far-track", "ncore_dynamic_lidar_absent", 8, 0, 0, 0, 0),
    ]
    return {"tracks": [{"track_id": t, "status": s, "matched_lidar_frame_count": m, "nonzero_exact_box_frame_count": e, "nonzero_padded_box_frame_count": p, "sum_exact_box_hit_points": se, "sum_padded_box_hit_points": sp, "max_exact_box_hit_points": 5, "max_padded_box_hit_points": 7} for t, s, m, e, p, se, sp in rows]}


class RenderSelectionTests(unittest.TestCase):
    def test_keeps_mandatory_and_selects_quality_eligible_corridor_objects(self):
        from adapters.nurec_render_selection import build_render_selection

        visibility = {"observations": [
            {"object_id": "lead", "expected_visible": True, "frame_id": 1, "projection": {"distance_to_ego_m": 12}},
            {"object_id": "background", "expected_visible": True, "frame_id": 1, "projection": {"distance_to_ego_m": 30}},
            {"object_id": "far", "expected_visible": True, "frame_id": 1, "projection": {"distance_to_ego_m": 90}},
        ]}
        quality = _quality()
        quality["tracks"].append({"track_id": "static-track", "status": "ncore_dynamic_lidar_supported", "matched_lidar_frame_count": 10, "nonzero_exact_box_frame_count": 10, "nonzero_padded_box_frame_count": 10, "sum_exact_box_hit_points": 20, "sum_padded_box_hit_points": 30, "max_exact_box_hit_points": 5, "max_padded_box_hit_points": 7})
        visibility["observations"].append({"object_id": "static", "expected_visible": True, "frame_id": 1, "projection": {"distance_to_ego_m": 20}})
        report = build_render_selection(_registry(), visibility, quality, quality)
        by_id = {row["object_id"]: row for row in report["objects"]}
        self.assertEqual(by_id["lead"]["decision"], "selected")
        self.assertEqual(by_id["background"]["decision"], "selected")
        self.assertEqual(by_id["far"]["decision"], "excluded_outside_ego_corridor")
        self.assertEqual(by_id["road"]["decision"], "selected")
        self.assertEqual(by_id["static"]["track_id"], "static-track")
        self.assertEqual(by_id["static"]["decision"], "selected")

    def test_mandatory_low_quality_is_a_blocker_not_a_silent_exclusion(self):
        from adapters.nurec_render_selection import build_render_selection

        visibility = {"observations": [{"object_id": "lead", "expected_visible": True, "frame_id": 1, "projection": {"distance_to_ego_m": 12}}]}
        poor = {"tracks": [{"track_id": "lead-track", "status": "ncore_dynamic_lidar_absent", "matched_lidar_frame_count": 3, "nonzero_exact_box_frame_count": 0, "nonzero_padded_box_frame_count": 0, "sum_exact_box_hit_points": 0, "sum_padded_box_hit_points": 0, "max_exact_box_hit_points": 0, "max_padded_box_hit_points": 0}]}
        report = build_render_selection(_registry(), visibility, poor, poor)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["objects"][0]["quality_decision"]["status"], "quality_insufficient")
        self.assertIn("lead", report["blockers"])

    def test_lifecycle_quality_uses_active_ticks_and_defers_early_frames(self):
        from adapters.nurec_render_selection import build_lifecycle_quality_manifest

        registry = {
            "schema_version": "scene_object_registry.v1",
            "scene_id": "scene-0061",
            "records": [{
                "object_id": "pedestrian",
                "role": "controlled_pedestrian",
                "semantic_class": "pedestrian",
                "safety_relevant": True,
                "time_interval": {"start_sec": 1.0, "end_sec": 2.0},
                "nurec": {"track_id": "ped-track"},
                "carla": {"representation": "physical_actor", "collision_policy": "required"},
            }],
        }
        support = {
            "schema_version": "ncore_dynamic_lidar_support_audit.v2",
            "source_lidar_frames": [
                {"source_lidar_frame_index": 0, "source_lidar_frame_end_us": 100_000,
                 "track_support": [{"track_id": "ped-track", "annotation_status": "outside_source_annotation_window",
                                    "source_cuboid_available": False, "exact_box_hit_points": None, "padded_box_hit_points": None}]},
                {"source_lidar_frame_index": 1, "source_lidar_frame_end_us": 1_100_000,
                 "track_support": [{"track_id": "ped-track", "annotation_status": "annotated_source_cuboid",
                                    "source_cuboid_available": True, "exact_box_hit_points": 1, "padded_box_hit_points": 3}]},
                {"source_lidar_frame_index": 2, "source_lidar_frame_end_us": 1_150_000,
                 "track_support": [{"track_id": "ped-track", "annotation_status": "annotated_source_cuboid",
                                    "source_cuboid_available": True, "exact_box_hit_points": 2, "padded_box_hit_points": 4}]},
                {"source_lidar_frame_index": 3, "source_lidar_frame_end_us": 2_100_000,
                 "track_support": [{"track_id": "ped-track", "annotation_status": "outside_source_annotation_window",
                                    "source_cuboid_available": False, "exact_box_hit_points": None, "padded_box_hit_points": None}]},
            ],
        }
        report = build_lifecycle_quality_manifest(registry, support, min_supported_ticks=2)
        self.assertEqual(report["status"], "passed")
        row = report["objects"][0]
        self.assertEqual(row["status"], "supported")
        self.assertEqual(row["supported_tick_indices"], [1, 2])
        self.assertEqual(row["frames"][0]["status"], "deferred")
        self.assertEqual(row["frames"][3]["status"], "deferred")
        self.assertTrue(report["selection_does_not_change_carla_physics"])

    def test_lifecycle_quality_reports_missing_per_frame_evidence(self):
        from adapters.nurec_render_selection import build_lifecycle_quality_manifest

        registry = {
            "schema_version": "scene_object_registry.v1",
            "scene_id": "scene-0061",
            "records": [{"object_id": "ped", "role": "background_replay", "time_interval": {"start_sec": 0, "end_sec": 1}, "nurec": {"track_id": "ped-track"}}],
        }
        report = build_lifecycle_quality_manifest(registry, {"schema_version": "ncore_dynamic_lidar_support_audit.v2"})
        self.assertEqual(report["status"], "evidence_unavailable")
        self.assertEqual(report["reason"], "source_lidar_frames_not_provided")

    def test_lifecycle_windows_require_consecutive_supported_ticks(self):
        from adapters.nurec_render_selection import build_lifecycle_quality_manifest

        registry = {
            "schema_version": "scene_object_registry.v1",
            "scene_id": "scene-0061",
            "records": [{
                "object_id": "ped",
                "role": "controlled_pedestrian",
                "time_interval": {"start_sec": 0, "end_sec": 1},
                "nurec": {"track_id": "ped-track"},
            }],
        }
        rows = []
        for index, exact in enumerate((2, 0, 3, 4)):
            rows.append({
                "source_lidar_frame_index": index,
                "source_lidar_frame_end_us": (index + 1) * 100_000,
                "track_support": [{
                    "track_id": "ped-track",
                    "source_cuboid_available": True,
                    "annotation_status": "annotated_source_cuboid",
                    "exact_box_hit_points": exact,
                    "padded_box_hit_points": max(exact, 1),
                }],
            })
        report = build_lifecycle_quality_manifest(
            registry,
            {"schema_version": "ncore_dynamic_lidar_support_audit.v2", "source_lidar_frames": rows},
            min_supported_ticks=2,
        )
        row = report["objects"][0]
        self.assertEqual(row["status"], "supported")
        self.assertEqual(row["supported_tick_windows"][-1]["tick_count"], 2)
        self.assertEqual(row["supported_tick_windows"][-1]["start_frame_index"], 2)


if __name__ == "__main__":
    unittest.main()
