import unittest
import json
import struct
import tempfile
from pathlib import Path


REGISTRY = {
    "records": [
        {"object_id": "truck", "role": "static_obstacle", "time_interval": {"start_sec": 0.0, "end_sec": None}, "carla": {"collision_policy": "required"}},
        {"object_id": "road", "role": "road_boundary", "carla": {"collision_policy": "not_applicable"}},
    ]
}


def _tick():
    return {
        "frame_id": 10,
        "simulation_time_sec": 0.5,
        "ego_state": {"pose": {"x": 0, "y": 0, "z": 0}, "extent_m": {"x": 1, "y": 1, "z": 1}},
        "object_states": [{"object_id": "truck", "carla_runtime_actor_id": 4, "pose": {"x": 5, "y": 0, "z": 0}, "extent_m": {"x": 1, "y": 1, "z": 1}}],
        "collision_events": [],
        "collision_detected": False,
    }


class SceneSafetyAuditTests(unittest.TestCase):
    def test_collision_requires_full_physical_registry_coverage_and_event_attribution(self):
        from adapters.scene_safety_audit import audit_collision_tick

        self.assertEqual(audit_collision_tick(REGISTRY, _tick())["status"], "passed")
        overlap = _tick()
        overlap["object_states"][0]["pose"]["x"] = 1
        failure = audit_collision_tick(REGISTRY, overlap)
        self.assertEqual(failure["status"], "failed")
        self.assertIn("unattributed_geometric_overlap:truck", failure["issues"])
        overlap["collision_events"] = [{"object_id": "truck"}]
        self.assertEqual(audit_collision_tick(REGISTRY, overlap)["status"], "passed")

    def test_lane_rejects_missing_or_offroad_evidence(self):
        from adapters.scene_safety_audit import audit_lane_tick

        base = {"frame_id": 10, "simulation_time_sec": 0.5}
        self.assertEqual(audit_lane_tick(base)["status"], "failed")
        base["lane_state"] = {"road_id": 1, "lane_id": 2, "is_on_road": True, "signed_centerline_distance_m": 0.1, "signed_boundary_distance_m": 1.0, "route_progress": 0.2, "lane_invasion_events": [], "lane_invasion_sensor_available": True}
        self.assertEqual(audit_lane_tick(base)["status"], "passed")
        base["lane_state"]["is_on_road"] = False
        self.assertIn("off_road", audit_lane_tick(base)["issues"])

    def test_visibility_requires_payload_bound_calibrated_projection(self):
        from adapters.scene_safety_audit import audit_visibility_tick

        base = {"frame_id": 10, "simulation_time_sec": 0.5, "projections": [{"object_id": "truck", "camera": "camera_front"}]}
        self.assertEqual(audit_visibility_tick(base)["status"], "failed")
        base["projections"][0].update({
            "observation_kind": "calibrated_3d_box_projection",
            "projection": {"bbox_xyxy_px": [1, 2, 3, 4]},
            "evidence": {"nre_payload_sha256": "a" * 64, "calibrated_sensor_token": "front", "intrinsics_table_sha256": "b" * 64},
        })
        self.assertEqual(audit_visibility_tick(base)["status"], "passed")

    def test_visibility_empty_frame_fails_closed(self):
        from adapters.scene_safety_audit import audit_visibility_tick

        result = audit_visibility_tick({"frame_id": 10, "simulation_time_sec": 0.5, "projections": []})
        self.assertEqual(result["status"], "failed")

    def test_lidar_world_requires_declared_occupancy(self):
        from adapters.scene_safety_audit import audit_lidar_world_tick

        base = {"frame_id": 10, "simulation_time_sec": 0.5, "expected_world_objects": [{"object_id": "truck", "expected_lidar_support": True}], "lidar_occupancy": []}
        self.assertEqual(audit_lidar_world_tick(base)["status"], "failed")
        base["lidar_occupancy"] = [{"object_id": "truck", "point_count": 3}]
        self.assertEqual(audit_lidar_world_tick(base)["status"], "passed")

    def test_lidar_world_rejects_source_carla_observability_conflict(self):
        from adapters.scene_safety_audit import audit_lidar_world_tick

        tick = {
            "frame_id": 10,
            "simulation_time_sec": 0.5,
            "expected_world_objects": [{
                "object_id": "truck",
                "expected_lidar_support": False,
                "source_lidar_observability": {
                    "issues": ["source_observed_carla_occluded"],
                },
            }],
            "lidar_occupancy": [{"object_id": "truck", "point_count": 0}],
        }
        result = audit_lidar_world_tick(tick)
        self.assertEqual(result["status"], "failed")
        self.assertIn("source_observed_carla_occluded:truck", result["issues"])

    def test_lidar_world_rejects_unknown_or_duplicate_occupancy(self):
        from adapters.scene_safety_audit import audit_lidar_world_tick, SceneSafetyAuditError

        base = {"frame_id": 10, "simulation_time_sec": 0.5, "expected_world_objects": [{"object_id": "truck", "expected_lidar_support": False}]}
        with self.assertRaises(SceneSafetyAuditError):
            audit_lidar_world_tick({**base, "lidar_occupancy": [{"object_id": "truck", "point_count": 0}, {"object_id": "truck", "point_count": 0}]})
        result = audit_lidar_world_tick({**base, "lidar_occupancy": [{"object_id": "unknown", "point_count": 1}]})
        self.assertEqual(result["status"], "failed")
        self.assertIn("unexpected_lidar_occupancy:unknown", result["issues"])

    def test_runner_writes_four_immutable_tick_streams(self):
        from runners.audit_scene_safety import audit_m8_evidence, write_m8_evidence

        raw = _tick()
        raw["lane"] = {"lane_state": {"road_id": 1, "lane_id": 2, "is_on_road": True, "signed_centerline_distance_m": 0.1, "signed_boundary_distance_m": 1.0, "route_progress": 0.2, "lane_invasion_events": [], "lane_invasion_sensor_available": True}}
        raw["visibility"] = {"projections": [{"object_id": "truck", "camera": "camera_front", "observation_kind": "calibrated_3d_box_projection", "projection": {"bbox_xyxy_px": [1, 2, 3, 4]}, "evidence": {"nre_payload_sha256": "a" * 64, "calibrated_sensor_token": "front", "intrinsics_table_sha256": "b" * 64}}]}
        raw["lidar_world"] = {"expected_world_objects": [{"object_id": "truck", "expected_lidar_support": True}], "lidar_occupancy": [{"object_id": "truck", "point_count": 2}]}
        raw["collision"] = {key: value for key, value in raw.items() if key not in {"lane", "visibility", "lidar_world"}}
        raw["frame_id"] = raw["collision"].pop("frame_id")
        raw["simulation_time_sec"] = raw["collision"].pop("simulation_time_sec")
        result = audit_m8_evidence(REGISTRY, [raw])
        with tempfile.TemporaryDirectory() as directory:
            summary = write_m8_evidence(result, Path(directory))
            self.assertEqual(summary["status"], "passed")
            for filename in ("collision_audit.v1.jsonl", "lane_audit.v1.jsonl", "visibility_audit.v1.jsonl", "lidar_world_audit.v1.jsonl"):
                self.assertEqual(len((Path(directory) / filename).read_text(encoding="utf-8").splitlines()), 1)

    def test_runner_rejects_mismatched_stream_frames(self):
        from runners.audit_scene_safety import write_m8_evidence

        rows = {name: [{"frame_id": 10, "status": "passed"}] for name, _, _ in __import__("runners.audit_scene_safety", fromlist=["AUDITS"]).AUDITS}
        rows["lidar_world"] = [{"frame_id": 11, "status": "passed"}]
        with tempfile.TemporaryDirectory() as directory:
            summary = write_m8_evidence(rows, Path(directory))
            self.assertEqual(summary["status"], "failed")
            self.assertTrue(summary["frame_set_mismatch"])

    def test_normalized_lidar_is_counted_in_scene_world_boxes(self):
        from adapters.lidar_world_support import lidar_occupancy_from_xyzi, summarize_xyzi_payload

        identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        payload = struct.pack("<4f", 5.0, 0.0, 0.0, 1.0) + struct.pack("<4f", 15.0, 0.0, 0.0, 1.0)
        rows = lidar_occupancy_from_xyzi(
            payload,
            ego_pose={"x": 0, "y": 0, "z": 0, "yaw": 0},
            sensor_to_ego=identity,
            object_states=[{"object_id": "truck", "carla_runtime_actor_id": 4, "pose": {"x": 5, "y": 0, "z": 0, "yaw": 0}, "extent_m": {"x": 1, "y": 1, "z": 1}}],
        )
        self.assertEqual(rows[0]["point_count"], 1)
        self.assertEqual(summarize_xyzi_payload(payload)["sensor_bounds_m"]["x"], [5.0, 15.0])

    def test_lidar_occupancy_keeps_carla_right_axis_when_composing_world_pose(self):
        from adapters.lidar_world_support import lidar_occupancy_from_xyzi

        identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        payload = struct.pack("<4f", 0.0, 5.0, 0.0, 1.0)
        rows = lidar_occupancy_from_xyzi(
            payload,
            ego_pose={"x": 0, "y": 0, "z": 0, "yaw": 0},
            sensor_to_ego=identity,
            object_states=[{"object_id": "right", "carla_runtime_actor_id": 4, "pose": {"x": 0, "y": 5, "z": 0, "yaw": 0}, "extent_m": {"x": 1, "y": 1, "z": 1}}],
        )
        self.assertEqual(rows[0]["point_count"], 1)

    def test_lidar_expectation_excludes_occluded_and_out_of_range_boxes(self):
        from adapters.lidar_world_support import expected_lidar_support_from_physical_boxes

        identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        objects = [
            {"object_id": "near", "carla_runtime_actor_id": 1, "pose": {"x": 5, "y": 0, "z": 0, "yaw": 0}, "extent_m": {"x": 1, "y": 1, "z": 1}},
            {"object_id": "occluded", "carla_runtime_actor_id": 2, "pose": {"x": 10, "y": 0, "z": 0, "yaw": 0}, "extent_m": {"x": 1, "y": 1, "z": 1}},
            {"object_id": "distant", "carla_runtime_actor_id": 3, "pose": {"x": 100, "y": 0, "z": 0, "yaw": 0}, "extent_m": {"x": 1, "y": 1, "z": 1}},
        ]
        rows = expected_lidar_support_from_physical_boxes(ego_pose={"x": 0, "y": 0, "z": 0, "yaw": 0}, sensor_to_ego=identity, object_states=objects, max_range_m=80)
        by_id = {row["object_id"]: row for row in rows}
        self.assertTrue(by_id["near"]["expected_lidar_support"])
        self.assertFalse(by_id["occluded"]["expected_lidar_support"])
        self.assertEqual(by_id["occluded"]["reason"], "carla_occluded")
        self.assertFalse(by_id["distant"]["expected_lidar_support"])
        self.assertEqual(by_id["distant"]["reason"], "outside_declared_lidar_range")

    def test_lidar_occupancy_runner_requires_same_frame_payload_with_hash(self):
        from runners.build_m8_lidar_occupancy import build_m8_lidar_occupancy

        with tempfile.TemporaryDirectory() as directory:
            payload = Path(directory) / "lidar.bin"
            payload.write_bytes(struct.pack("<4f", 5.0, 0.0, 0.0, 1.0))
            digest = __import__("hashlib").sha256(payload.read_bytes()).hexdigest()
            runtime = [{"frame_id": 10, "simulation_time_sec": 0.5, "ego_state": {"pose": {"x": 0, "y": 0, "z": 0, "yaw": 0}}, "object_states": [{"object_id": "truck", "carla_runtime_actor_id": 4, "pose": {"x": 5, "y": 0, "z": 0, "yaw": 0}, "extent_m": {"x": 1, "y": 1, "z": 1}}]}]
            trace = [{"frame_id": 10, "status": "passed", "records": [{"modality": "lidar", "sensor_id": "lidar_top", "status": "passed", "response_metadata": {"materialized_payload": {"path": str(payload), "sha256": digest, "coordinate_frame": "sensor_local", "axis_convention": "carla_sensor"}}}]}]
            identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            rows = build_m8_lidar_occupancy(runtime, trace, {"nurec_runtime": {"lidar_specs": [{"sensor_id": "lidar_top", "sensor_to_ego": identity}]}})
            self.assertEqual(rows[0]["occupancy"][0]["point_count"], 1)

    def test_m8_config_derivation_preserves_replay_and_requires_m7(self):
        from runners.derive_scene0061_m8_safety_config import (
            build_m8_actor_binding_manifest,
            derive_m8_safety_config,
        )

        source = {
            "schema_version": "carla_run_config.mvp.v0",
            "run_id": "base",
            "scenario_id": "a" * 32,
            "runtime": {"m7_actor_pose_audit_required": True},
            "actor_binding": {"selected_actor_ids": ["car"]},
            "actors": [
                {
                    "actor_id": "car",
                    "source_track_id": "car",
                    "type": "vehicle",
                    "closed_loop_level": "replay",
                },
                {
                    "actor_id": "walker",
                    "source_track_id": "walker",
                    "type": "pedestrian",
                    "closed_loop_level": "replay",
                },
            ],
        }
        derived = derive_m8_safety_config(source)
        self.assertTrue(derived["runtime"]["m8_safety_audit_required"])
        self.assertEqual(
            derived["runtime"]["dynamic_actor_lifecycle"],
            "source_annotation_window",
        )
        self.assertEqual(
            derived["runtime"]["static_obstacle_lifecycle"],
            "source_annotation_window",
        )
        self.assertEqual(derived["actors"][0]["closed_loop_level"], "replay")
        self.assertFalse(derived["nurec_runtime"]["lidar_instant_sampling"])
        bindings = {
            row["actor_id"]: row
            for row in build_m8_actor_binding_manifest(derived)["bindings"]
        }
        self.assertEqual(set(bindings), {"car", "walker"})
        self.assertEqual(bindings["car"]["nurec"]["track_id"], "car")
        self.assertEqual(
            bindings["car"]["sensor_sync"]["pose_reference"],
            "carla_bounding_box_center",
        )
        self.assertEqual(
            bindings["walker"]["sensor_sync"]["pose_reference"],
            "carla_bounding_box_center",
        )

    def test_m8_runtime_adapter_retains_physical_box_state(self):
        from runners.build_m8_expected_visibility import physical_frames_from_m8_runtime

        frames = physical_frames_from_m8_runtime([{"frame_id": 9, "simulation_time_sec": 0.5, "ego_state": {"pose": {"x": 0}}, "object_states": [{"object_id": "truck", "pose": {"x": 5}, "extent_m": {"x": 1}}]}])
        self.assertEqual(frames[0]["actor_states"]["truck"]["pose"]["x"], 5)

    def test_source_annotation_lifecycle_has_no_unmatched_grace_window(self):
        from runners.run_carla_basic_agent import _actor_temporal_lifecycle_state

        window = (1.0, 2.0)
        self.assertEqual(_actor_temporal_lifecycle_state(window, 0.99), "deferred")
        self.assertEqual(_actor_temporal_lifecycle_state(window, 1.0), "active")
        self.assertEqual(_actor_temporal_lifecycle_state(window, 2.0), "active")
        self.assertEqual(_actor_temporal_lifecycle_state(window, 2.0001), "despawned")

    def test_static_obstacle_lifecycle_uses_its_declared_source_window(self):
        from runners.run_carla_basic_agent import (
            _advance_static_obstacle_temporal_lifecycle,
            _initialize_static_obstacle_temporal_lifecycle,
            _static_obstacle_runtime_evidence,
            _static_obstacle_windows_fit_requested_horizon,
        )
        from tests.test_basic_agent_runtime_loop import FakeCarlaModule, FakeWorld

        events = []
        obstacle = {
            "object_id": "source-singleton",
            "placement": {"x": 2.0, "y": 1.0, "z": 0.0, "yaw": 0.0},
            "blueprint": "static.prop.*",
            "collision_policy": "required",
            "time_interval": {"start_sec": 1.0, "end_sec": 2.0},
        }
        spawned = {}
        lifecycle = _initialize_static_obstacle_temporal_lifecycle([obstacle])
        _advance_static_obstacle_temporal_lifecycle(
            FakeCarlaModule(events), FakeWorld(events), [obstacle], spawned, lifecycle,
            scenario_time_sec=0.5,
        )
        self.assertEqual(lifecycle["source-singleton"]["state"], "deferred")
        self.assertFalse(spawned)

        _advance_static_obstacle_temporal_lifecycle(
            FakeCarlaModule(events), FakeWorld(events), [obstacle], spawned, lifecycle,
            scenario_time_sec=1.0,
        )
        self.assertEqual(lifecycle["source-singleton"]["state"], "active")
        self.assertIn("source-singleton", spawned)
        incomplete = _static_obstacle_runtime_evidence(
            {"static_obstacles": [obstacle]}, spawned,
            temporal_lifecycle=lifecycle,
            require_window_entered=True,
            require_window_completed=True,
            observed_horizon_sec=1.0,
        )
        self.assertEqual(incomplete["status"], "failed")
        self.assertIn(
            "source-singleton:source_annotation_window_not_completed",
            incomplete["issues"],
        )

        _advance_static_obstacle_temporal_lifecycle(
            FakeCarlaModule(events), FakeWorld(events), [obstacle], spawned, lifecycle,
            scenario_time_sec=2.01,
        )
        self.assertEqual(lifecycle["source-singleton"]["state"], "despawned")
        self.assertFalse(spawned)
        audit = _static_obstacle_runtime_evidence(
            {"static_obstacles": [obstacle]}, spawned,
            temporal_lifecycle=lifecycle,
            require_window_entered=True,
            require_window_completed=True,
            observed_horizon_sec=2.01,
        )
        self.assertEqual(audit["status"], "passed")
        self.assertFalse(
            _static_obstacle_windows_fit_requested_horizon(
                [obstacle], requested_horizon_sec=0.15,
            )
        )
        self.assertTrue(
            _static_obstacle_windows_fit_requested_horizon(
                [obstacle], requested_horizon_sec=2.01,
            )
        )

    def test_static_obstacle_short_horizon_defers_future_source_window(self):
        from runners.run_carla_basic_agent import (
            _initialize_static_obstacle_temporal_lifecycle,
            _static_obstacle_runtime_evidence,
        )

        obstacle = {
            "object_id": "future-static",
            "placement": {"x": 2.0, "y": 1.0, "z": 0.0, "yaw": 0.0},
            "blueprint": "static.prop.*",
            "collision_policy": "required",
            "time_interval": {"start_sec": 1.0, "end_sec": 2.0},
        }
        lifecycle = _initialize_static_obstacle_temporal_lifecycle([obstacle])
        audit = _static_obstacle_runtime_evidence(
            {"static_obstacles": [obstacle]},
            {},
            temporal_lifecycle=lifecycle,
            observed_horizon_sec=0.15,
        )

        self.assertEqual(audit["status"], "passed")
        self.assertEqual(audit["records"][0]["status"], "deferred")
        self.assertEqual(
            audit["records"][0]["source_annotation_window_requirement"],
            "deferred_outside_observed_horizon",
        )

    def test_replay_executor_is_explicitly_not_required_before_source_window(self):
        from runners.run_carla_basic_agent import _actor_control_execution_evidence

        preflight = {
            "status": "passed",
            "records": [{"actor_id": "future-car", "issues": []}],
            "issues": [],
        }
        evidence = _actor_control_execution_evidence(
            {},
            {},
            preflight,
            temporal_lifecycle={"future-car": {"state": "deferred", "events": []}},
        )
        self.assertEqual(evidence["status"], "passed")
        self.assertEqual(
            evidence["records"][0]["execution_requirement"],
            "not_required_outside_source_annotation_window",
        )

    def test_m8_artifact_join_keeps_same_frame_geometry_and_lidar_inputs(self):
        from runners.build_m8_evidence_from_artifacts import build_m8_evidence_from_artifacts

        runtime = [_tick()]
        visibility = {"observations": [{"frame_id": 10, "object_id": "truck", "camera": "camera_front", "observation_kind": "calibrated_3d_box_projection", "projection": {"bbox_xyxy_px": [1, 2, 3, 4]}, "evidence": {"nre_payload_sha256": "a" * 64, "calibrated_sensor_token": "front", "intrinsics_table_sha256": "b" * 64}}]}
        expected = [{"frame_id": 10, "expected_world_objects": [{"object_id": "truck", "expected_lidar_support": True}]}]
        occupancy = [{"frame_id": 10, "occupancy": [{"object_id": "truck", "point_count": 2}]}]
        rows = build_m8_evidence_from_artifacts(runtime, visibility, expected, occupancy)

        self.assertEqual(rows[0]["visibility"]["projections"][0]["frame_id"], 10)
        self.assertEqual(rows[0]["lidar_world"]["lidar_occupancy"][0]["point_count"], 2)

    def test_m8_artifact_join_rejects_empty_visibility_and_extra_frames(self):
        from runners.build_m8_evidence_from_artifacts import build_m8_evidence_from_artifacts

        runtime = [_tick()]
        expected = [{"frame_id": 10, "expected_world_objects": [{"object_id": "truck", "expected_lidar_support": True}]}]
        occupancy = [{"frame_id": 10, "occupancy": [{"object_id": "truck", "point_count": 2}]}]
        with self.assertRaisesRegex(Exception, "calibrated visibility"):
            build_m8_evidence_from_artifacts(runtime, {"observations": []}, expected, occupancy)
        visibility = {"observations": [{"frame_id": 10, "object_id": "truck", "camera": "camera_front", "observation_kind": "calibrated_3d_box_projection", "projection": {}, "evidence": {"nre_payload_sha256": "a" * 64, "calibrated_sensor_token": "front", "intrinsics_table_sha256": "b" * 64}}]}
        with self.assertRaisesRegex(Exception, "absent from runtime"):
            build_m8_evidence_from_artifacts(runtime, {"observations": visibility["observations"] + [{**visibility["observations"][0], "frame_id": 11}]}, expected, occupancy)

    def test_detector_evidence_requires_pinned_model_and_iou_match(self):
        from adapters.rgb_detector_evidence import match_detector_evidence

        projection = [{"object_id": "truck", "frame_id": 9, "camera": "camera_front", "semantic_class": "vehicle", "projection": {"bbox_xyxy_px": [10, 10, 50, 50]}}]
        model = {"name": "detector", "version": "1", "weight_sha256": "a" * 64, "class_mapping_sha256": "b" * 64, "score_threshold": 0.5}
        evidence = {"schema_version": "rgb_detector_evidence.v1", "model": model, "detections": [{"frame_id": 9, "camera": "camera_front", "semantic_class": "car", "score": 0.9, "bbox_xyxy_px": [12, 12, 48, 48]}]}
        self.assertEqual(match_detector_evidence(projection, evidence)[0]["object_id"], "truck")


if __name__ == "__main__":
    unittest.main()
