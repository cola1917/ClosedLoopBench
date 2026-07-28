from __future__ import annotations


def test_source_backed_expectation_requires_same_tick_source_support():
    from runners.build_m8_expected_lidar_support import build_expected_lidar_support

    identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    manifest_sha = "a" * 64
    report_sha = "b" * 64
    config = {
        "nurec_runtime": {
            "lidar_specs": [{"sensor_id": "lidar_top", "sensor_to_ego": identity}],
            "native_scan_manifest": {"sha256": manifest_sha},
        },
        "actors": [{"actor_id": "car", "source_track_id": "source-car"}],
        "static_obstacles": [],
    }
    runtime = [{
        "frame_id": 10,
        "simulation_time_sec": 0.05,
        "ego_state": {"pose": {"x": 0, "y": 0, "z": 0, "yaw": 0}},
        "object_states": [{"object_id": "car", "carla_runtime_actor_id": 4, "pose": {"x": 5, "y": 0, "z": 0, "yaw": 0}, "extent_m": {"x": 1, "y": 1, "z": 1}}],
    }]
    nurec = [{
        "frame_id": 10,
        "status": "passed",
        "dispatch": {"temporal_alignment": {"status": "aligned", "manifest_sha256": manifest_sha, "wire_start_us": 1_000, "wire_end_us": 1_050, "native_scan_index": 0, "midpoint_error_us": 0, "max_midpoint_error_us": 30_000}},
    }]
    source = {
        "schema_version": "ncore_dynamic_lidar_support_audit.v2",
        "source_lidar_frames": [{
            "source_lidar_frame_end_us": 1_050,
            "track_support": [{"track_id": "source-car", "annotation_status": "annotated_source_cuboid", "source_cuboid_available": True, "exact_box_hit_points": 1, "padded_box_hit_points": 1}],
        }],
    }

    rows = build_expected_lidar_support(
        runtime,
        config,
        nurec_rows=nurec,
        source_lidar_support=source,
        source_lidar_support_sha256=report_sha,
    )

    assert rows[0]["schema_version"] == "m8_expected_lidar_support.v2"
    assert rows[0]["expected_world_objects"][0]["expected_lidar_support"] is True
    assert rows[0]["source_lidar_alignment"]["wire_end_us"] == 1_050


def test_static_source_unavailability_disables_only_lidar_expectation():
    from runners.build_m8_expected_lidar_support import build_expected_lidar_support

    identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    manifest_sha = "a" * 64
    report_sha = "b" * 64
    static_report_sha = "c" * 64
    config = {
        "nurec_runtime": {
            "lidar_specs": [{"sensor_id": "lidar_top", "sensor_to_ego": identity}],
            "native_scan_manifest": {"sha256": manifest_sha},
        },
        "actors": [{"actor_id": "car", "source_track_id": "source-car"}],
        "static_obstacles": [{
            "object_id": "barrier",
            "source": {"source_track_id": "source-barrier"},
        }],
    }
    runtime = [{
        "frame_id": 10,
        "simulation_time_sec": 0.05,
        "ego_state": {"pose": {"x": 0, "y": 0, "z": 0, "yaw": 0}},
        "object_states": [
            {"object_id": "car", "carla_runtime_actor_id": 4, "pose": {"x": 5, "y": 0, "z": 0, "yaw": 0}, "extent_m": {"x": 1, "y": 1, "z": 1}},
            {"object_id": "barrier", "carla_runtime_actor_id": 5, "pose": {"x": 6, "y": 0, "z": 0, "yaw": 0}, "extent_m": {"x": 1, "y": 1, "z": 1}},
        ],
    }]
    nurec = [{
        "frame_id": 10,
        "status": "passed",
        "dispatch": {"temporal_alignment": {"status": "aligned", "manifest_sha256": manifest_sha, "wire_start_us": 1_000, "wire_end_us": 1_050, "native_scan_index": 0, "midpoint_error_us": 0, "max_midpoint_error_us": 30_000}},
    }]
    dynamic_source = {
        "schema_version": "ncore_dynamic_lidar_support_audit.v2",
        "source_lidar_frames": [{
            "source_lidar_frame_end_us": 1_050,
            "track_support": [{"track_id": "source-car", "annotation_status": "annotated_source_cuboid", "source_cuboid_available": True, "exact_box_hit_points": 1, "padded_box_hit_points": 1}],
        }],
    }
    static_source = {
        "schema_version": "ncore_dynamic_lidar_support_audit.v2",
        "source_lidar_frames": [{
            "source_lidar_frame_end_us": 1_050,
            "track_support": [{"track_id": "source-barrier", "annotation_status": "outside_source_annotation_window", "source_cuboid_available": False, "exact_box_hit_points": None, "padded_box_hit_points": None}],
        }],
    }

    rows = build_expected_lidar_support(
        runtime,
        config,
        nurec_rows=nurec,
        source_lidar_support=dynamic_source,
        source_lidar_support_sha256=report_sha,
        static_source_lidar_support=static_source,
        static_source_lidar_support_sha256=static_report_sha,
    )

    by_id = {item["object_id"]: item for item in rows[0]["expected_world_objects"]}
    assert by_id["car"]["expected_lidar_support"] is True
    assert by_id["barrier"]["expected_lidar_support"] is False
    assert by_id["barrier"]["source_lidar_observability"]["status"] == "unavailable"
