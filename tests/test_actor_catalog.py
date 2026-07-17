import unittest


TOKEN = "c" * 32
VEHICLE = "1" * 32
PEDESTRIAN = "2" * 32


def _state(t, x, y, speed=1.0):
    return {"t_sec": t, "x": x, "y": y, "z": 0.0, "yaw": 0.0, "speed_mps": speed}


def _base(actors):
    return {
        "schema_version": "scenario_ir.v1",
        "scenario_id": TOKEN,
        "scenario_type": "nuscenes_reconstructed_scene",
        "source": {"dataset": "nuscenes", "scene_id": TOKEN, "version": "v1.0-mini", "scene_name": "scene-test", "scene_token": TOKEN, "sample_count": 12},
        "coordinate_frame": {"name": "scene_local_ego_start", "units": {"position": "meter", "time": "second", "yaw": "degree"}, "handedness": "right", "x_axis": "initial_ego_forward", "y_axis": "initial_ego_left", "origin_global_translation": [0, 0, 0], "origin_global_rotation_wxyz": [1, 0, 0, 0], "origin_global_yaw_deg": 0, "transform": "local_xy = R(-origin_yaw) * (global_xy - origin_xy)"},
        "windows": {"event": {"start_sec": 0, "end_sec": 5}, "warmup": {"start_sec": 0, "end_sec": 1}, "reconstruction": {"start_sec": 0, "end_sec": 5}},
        "ego": {"track_id": "ego", "initial_state": _state(0, 0, 0), "reference_trajectory": [_state(i, i * 2, 0, 2) for i in range(12)], "route": {}},
        "actors": actors,
        "map_context": {"feature_counts": {}, "features": []},
        "sensors": {"available_capabilities": ["camera", "lidar"]},
        "events": {"trigger": None, "mined_events": []},
        "data_requirements": {"reconstruction": {"required": ["camera_images", "camera_calibration", "ego_pose", "actor_tracks"]}, "closed_loop": {"required": ["ego_initial_state", "actor_initial_states", "map_context"]}},
        "risk_metrics": {"trigger_time_sec": 0, "trigger_tag": None, "actor_count": len(actors), "ego_reference_state_count": 12},
        "dataset_refs": {"source": {"dataset": "nuscenes", "root": None, "scene_id": TOKEN}, "sample_refs": {"status": "deferred", "refs": []}, "index_refs": {"status": "deferred", "refs": []}},
        "evaluation": {"metrics": []},
        "variants": {"mvp": {}, "final_closed_loop": {}},
    }


def _actor(actor_id, actor_type, category, points):
    return {
        "actor_id": actor_id,
        "source_track_id": actor_id,
        "role": "context",
        "type": actor_type,
        "category": category,
        "initial_state": points[0],
        "reference_trajectory": points,
        "policy_hints": {"mvp": "replay"},
    }


class ActorCatalogTests(unittest.TestCase):
    def test_ranks_long_same_lane_vehicle_and_close_pedestrian(self):
        from adapters.actor_catalog import rank_actor_candidates

        vehicle_points = [_state(i, i * 2 + 20, 1.0, 3.0) for i in range(12)]
        pedestrian_points = [_state(i, 8.0, 3.0, 1.0) for i in range(12)]
        far_vehicle = [_state(i, i * 2 + 10, 20.0, 3.0) for i in range(12)]
        full = _base([
            _actor(VEHICLE, "vehicle", "vehicle.car", vehicle_points),
            _actor(PEDESTRIAN, "pedestrian", "human.pedestrian.adult", pedestrian_points),
            _actor("3" * 32, "vehicle", "vehicle.car", far_vehicle),
        ])
        audit = rank_actor_candidates(full)

        self.assertEqual(audit["recommendations"]["primary_vehicle_actor_id"], VEHICLE)
        self.assertEqual(audit["recommendations"]["bounded_pedestrian_actor_id"], PEDESTRIAN)
        pedestrian = next(item for item in audit["candidates"] if item["actor_id"] == PEDESTRIAN)
        self.assertEqual(pedestrian["closed_loop_scope"], "speed_pause_yield_abort_on_source_corridor")

    def test_repairs_event_referenced_and_explicit_actor_gaps(self):
        from adapters.actor_catalog import repair_scenario_actor_catalog

        full = _base([
            _actor(VEHICLE, "vehicle", "vehicle.car", [_state(i, i + 10, 0) for i in range(12)]),
            _actor(PEDESTRIAN, "pedestrian", "human.pedestrian.adult", [_state(i, i, 3) for i in range(12)]),
        ])
        mined = _base([])
        mined["events"]["mined_events"] = [{"subject_id": f"ego:{VEHICLE}", "metadata": {"target_id": VEHICLE}}]
        repaired = repair_scenario_actor_catalog(
            mined,
            full,
            additional_actor_ids=[PEDESTRIAN],
        )

        self.assertEqual({actor["actor_id"] for actor in repaired["actors"]}, {VEHICLE, PEDESTRIAN})
        diagnostics = repaired["diagnostics"]["actor_catalog_repair"]
        self.assertEqual(diagnostics["added_actor_count"], 2)
        self.assertEqual(diagnostics["unresolved_event_actor_ids"], [])
        self.assertEqual(repaired["risk_metrics"]["actor_count"], 2)

    def test_rejects_unknown_selected_actor(self):
        from adapters.actor_catalog import ActorCatalogError, repair_scenario_actor_catalog

        with self.assertRaisesRegex(ActorCatalogError, "absent"):
            repair_scenario_actor_catalog(_base([]), _base([]), additional_actor_ids=["f" * 32])


if __name__ == "__main__":
    unittest.main()
