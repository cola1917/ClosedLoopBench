import copy
import unittest

from adapters.actor_binding import build_actor_binding_set
from adapters.nurec_multimodal import make_pose_pair
from tests.test_actor_binding import SCENE_TOKEN, VEHICLE_TRACK, _scenario_ir
from tests.test_runtime_alignment import _package


def _binding_set(mode="scripted"):
    return build_actor_binding_set(
        _scenario_ir(),
        selected_actor_ids=[VEHICLE_TRACK],
        nurec_track_ids=[VEHICLE_TRACK],
        control_modes={VEHICLE_TRACK: mode},
    )


def _scene_package():
    package = _package()
    package["alignment"]["status"] = "runtime_validated"
    package["alignment"]["validation_evidence"] = "runtime_alignment_evidence.json"
    return package


def _frame():
    from adapters.nurec_multimodal import build_nurec_multimodal_frame

    sensor_pair = make_pose_pair(
        {"x": 0.0, "y": 0.0, "z": 1.5, "yaw": 0.0},
        {"x": 0.1, "y": 0.0, "z": 1.5, "yaw": 1.0},
    )
    actor_pair = make_pose_pair(
        {"x": 5.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        {"x": 5.1, "y": 0.0, "z": 0.0, "yaw": 1.0},
    )
    return build_nurec_multimodal_frame(
        _scene_package(),
        _binding_set(),
        frame_id=42,
        simulation_time_sec=2.10,
        interval_start_sec=2.05,
        camera_specs=[{
            "sensor_id": "camera_front",
            "model": "recorded_pinhole",
            "width": 1600,
            "height": 900,
            "pose_pair": sensor_pair,
        }],
        lidar_specs=[{
            "sensor_id": "lidar_top",
            "model": "runtime_verified_lidar_profile",
            "scan_frequency_hz": 20.0,
            "pose_pair": sensor_pair,
        }],
        actor_samples={
            VEHICLE_TRACK: {
                "source": "carla_runtime_actor_pose",
                "pose_pair": actor_pair,
            }
        },
    )


class NuRecMultimodalTests(unittest.TestCase):
    def test_rgb_and_lidar_materialize_the_same_dynamic_object_payload(self):
        from adapters.nurec_multimodal import materialize_nurec_rpc_requests

        frame = _frame()
        requests = materialize_nurec_rpc_requests(frame)
        self.assertEqual({item["modality"] for item in requests}, {"rgb", "lidar"})
        self.assertEqual(len({item["dynamic_object_sha256"] for item in requests}), 1)
        self.assertEqual(requests[0]["dynamic_objects"], requests[1]["dynamic_objects"])
        self.assertEqual(requests[0]["frame_id"], requests[1]["frame_id"])
        self.assertEqual(requests[0]["pose_interval_sec"], requests[1]["pose_interval_sec"])

    def test_sim_to_nurec_transform_is_applied_to_actor_and_sensor_pose(self):
        frame = _frame()
        actor_start = frame["shared_dynamic_objects"][0]["pose_pair"]["start"]["position_m"]
        camera_start = frame["modalities"]["rgb"]["requests"][0]["sensor"]["pose_pair"]["start"]["position_m"]
        # Test package sim_from_log translates global (10, 20, 1) to local origin.
        self.assertEqual(actor_start, {"x": 15.0, "y": 20.0, "z": 1.0})
        self.assertEqual(camera_start, {"x": 10.0, "y": 20.0, "z": 2.5})

    def test_sensor_extrinsic_is_composed_with_each_ego_pose_endpoint(self):
        from adapters.nurec_multimodal import sensor_pose_pair_from_ego

        ego = make_pose_pair(
            {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
            {"x": 2.0, "y": 2.0, "z": 0.0, "yaw": 90.0},
        )
        sensor_to_ego = [
            1.0, 0.0, 0.0, 1.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 1.5,
            0.0, 0.0, 0.0, 1.0,
        ]
        pair = sensor_pose_pair_from_ego(ego, sensor_to_ego)
        start = pair["start"]["matrix_row_major_4x4"]
        end = pair["end"]["matrix_row_major_4x4"]

        self.assertEqual([start[3], start[7], start[11]], [2.0, 2.0, 1.5])
        self.assertAlmostEqual(end[3], 2.0)
        self.assertAlmostEqual(end[7], 3.0)
        self.assertAlmostEqual(end[11], 1.5)

    def test_runtime_alignment_and_actor_pose_source_fail_closed(self):
        from adapters.nurec_multimodal import NuRecMultimodalError, build_nurec_multimodal_frame

        package = _scene_package()
        package["alignment"]["status"] = "log_to_sim_defined"
        package["alignment"].pop("validation_evidence")
        pair = make_pose_pair({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        kwargs = dict(
            frame_id=1,
            simulation_time_sec=0.05,
            interval_start_sec=0.0,
            camera_specs=[{"sensor_id": "cam", "model": "pinhole", "width": 10, "pose_pair": pair}],
            lidar_specs=[{"sensor_id": "lidar", "model": "verified", "pose_pair": pair}],
            actor_samples={VEHICLE_TRACK: {"source": "carla_runtime_actor_pose", "pose_pair": pair}},
        )
        with self.assertRaisesRegex(NuRecMultimodalError, "runtime_validated"):
            build_nurec_multimodal_frame(package, _binding_set(), **kwargs)

        package = _scene_package()
        kwargs["actor_samples"][VEHICLE_TRACK]["source"] = "scenario_ir_reference_trajectory"
        with self.assertRaisesRegex(NuRecMultimodalError, "pose source"):
            build_nurec_multimodal_frame(package, _binding_set(), **kwargs)

    def test_digest_tampering_is_rejected(self):
        from adapters.nurec_multimodal import NuRecMultimodalError, validate_nurec_multimodal_frame

        frame = _frame()
        frame["shared_dynamic_objects"][0]["pose_pair"]["end"]["position_m"]["x"] += 1.0
        with self.assertRaisesRegex(NuRecMultimodalError, "digest"):
            validate_nurec_multimodal_frame(frame)

    def test_complete_matching_responses_create_passed_evidence(self):
        from adapters.nurec_multimodal import (
            assert_nurec_multimodal_evidence,
            build_nurec_multimodal_evidence,
            materialize_nurec_rpc_requests,
        )

        frame = _frame()
        responses = [
            {
                "request_id": item["request_id"],
                "status": "ok",
                "frame_id": item["frame_id"],
                "dynamic_object_sha256": item["dynamic_object_sha256"],
                "payload_sha256": "b" * 64,
                "latency_ms": 12.5,
            }
            for item in materialize_nurec_rpc_requests(frame)
        ]
        evidence = build_nurec_multimodal_evidence(frame, responses, max_latency_ms=50.0)
        self.assertEqual(evidence["status"], "passed")
        self.assertEqual(evidence["modalities"]["rgb"]["passed_count"], 1)
        self.assertEqual(evidence["modalities"]["lidar"]["passed_count"], 1)
        assert_nurec_multimodal_evidence(evidence)

    def test_missing_or_cross_frame_response_fails_evidence(self):
        from adapters.nurec_multimodal import (
            NuRecMultimodalError,
            assert_nurec_multimodal_evidence,
            build_nurec_multimodal_evidence,
            materialize_nurec_rpc_requests,
        )

        frame = _frame()
        first = materialize_nurec_rpc_requests(frame)[0]
        response = {
            "request_id": first["request_id"],
            "status": "ok",
            "frame_id": first["frame_id"] + 1,
            "dynamic_object_sha256": first["dynamic_object_sha256"],
            "payload_sha256": "c" * 64,
            "latency_ms": 1.0,
        }
        evidence = build_nurec_multimodal_evidence(frame, [response])
        self.assertEqual(evidence["status"], "failed")
        self.assertTrue(any("frame_id_mismatch" in issue for issue in evidence["issues"]))
        self.assertTrue(any("missing_response" in issue for issue in evidence["issues"]))
        with self.assertRaises(NuRecMultimodalError):
            assert_nurec_multimodal_evidence(evidence)


if __name__ == "__main__":
    unittest.main()
