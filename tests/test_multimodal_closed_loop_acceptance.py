import copy
import unittest


SCENE_TOKEN = "cc8c0bf57f984915a77078b10eb33198"


def _result(actor_type="vehicle"):
    decision = {
        "reason": "ego_gap_or_ttc_reactive",
        "motion_constraint": (
            "source_reference_corridor"
            if actor_type == "pedestrian"
            else "carla_lane_or_reference_route"
        ),
        "allowed_actions": (
            ["speed", "pause", "yield", "abort"]
            if actor_type == "pedestrian"
            else ["speed", "throttle", "brake", "steer", "yield", "abort"]
        ),
    }
    evidence = []
    for frame_id in (10, 11):
        records = []
        for modality, count in (("rgb", 6), ("lidar", 1)):
            for index in range(count):
                records.append(
                    {
                        "request_id": f"{frame_id}:{modality}:{index}",
                        "modality": modality,
                        "sensor_id": f"{modality}-{index}",
                        "status": "passed",
                        "latency_ms": 1.0,
                        "payload_sha256": ("1" if modality == "rgb" else "2") * 64,
                        "issues": [],
                    }
                )
        evidence.append({
            "schema_version": "nurec_multimodal_evidence.v1",
            "scene_id": SCENE_TOKEN,
            "frame_id": frame_id,
            "simulation_time_sec": frame_id * 0.05,
            "dynamic_object_sha256": ("a" if frame_id == 10 else "b") * 64,
            "dynamic_object_count": 1,
            "records": records,
            "modalities": {
                "rgb": {"requested_count": 6, "passed_count": 6},
                "lidar": {"requested_count": 1, "passed_count": 1},
            },
            "max_latency_ms": 50.0,
            "status": "passed",
            "issues": [],
        })
    return {
        "status": "interactive_closed_loop",
        "scenario_id": SCENE_TOKEN,
        "nurec_multimodal_trace": evidence,
        "report": {
            "runtime": {
                "frame_trace_count": 2,
                "actor_physical_response": {"trigger": {"displacement_m": 1.0}},
                "actor_runtime_binding": {
                    "status": "passed",
                    "records": [{
                        "actor_id": "trigger",
                        "actor_type": actor_type,
                        "source_track_id": "source-track",
                        "nurec_track_id": "source-track",
                        "sensor_pose_source": "carla_runtime_actor_pose",
                        "required_modalities": ["rgb", "lidar"],
                        "carla": {"runtime_actor_id": 101},
                        "status": "passed",
                    }],
                },
                "multimodal_sensor": {
                    "required": True,
                    "status": "passed",
                    "sensor_closed_loop": True,
                    "frame_count": 2,
                    "modalities": ["rgb", "lidar"],
                },
            },
            "metrics": [
                {"actor_decisions": {"trigger": decision}},
                {"actor_decisions": {"trigger": decision}},
            ],
        },
    }


class MultimodalClosedLoopAcceptanceTests(unittest.TestCase):
    def test_accepts_complete_explainable_vehicle_and_pedestrian_runs(self):
        from runners.validate_multimodal_closed_loop import validate_multimodal_closed_loop_result

        vehicle = validate_multimodal_closed_loop_result(_result("vehicle"))
        pedestrian = validate_multimodal_closed_loop_result(_result("pedestrian"))
        self.assertEqual(vehicle["status"], "passed")
        self.assertEqual(pedestrian["status"], "passed")
        self.assertEqual(vehicle["modalities"], ["rgb", "lidar"])

    def test_rejects_missing_lidar_or_incomplete_frame_coverage(self):
        from runners.validate_multimodal_closed_loop import (
            MultimodalClosedLoopError,
            validate_multimodal_closed_loop_result,
        )

        missing_lidar = _result()
        missing_lidar["nurec_multimodal_trace"][0]["modalities"]["lidar"]["passed_count"] = 0
        with self.assertRaisesRegex(MultimodalClosedLoopError, "invalid NuRec frame evidence"):
            validate_multimodal_closed_loop_result(missing_lidar)

        uncovered = _result()
        uncovered["nurec_multimodal_trace"].pop()
        with self.assertRaisesRegex(MultimodalClosedLoopError, "every CARLA frame"):
            validate_multimodal_closed_loop_result(uncovered)

    def test_rejects_control_only_actor_and_unexplainable_decision(self):
        from runners.validate_multimodal_closed_loop import (
            MultimodalClosedLoopError,
            validate_multimodal_closed_loop_result,
        )

        no_physics = _result()
        no_physics["report"]["runtime"]["actor_physical_response"] = {}
        with self.assertRaisesRegex(MultimodalClosedLoopError, "physical response"):
            validate_multimodal_closed_loop_result(no_physics)

        no_reason = _result()
        no_reason["report"]["metrics"][0]["actor_decisions"]["trigger"].pop("reason")
        with self.assertRaisesRegex(MultimodalClosedLoopError, "reason/constraint"):
            validate_multimodal_closed_loop_result(no_reason)

    def test_rejects_pedestrian_free_space_edit(self):
        from runners.validate_multimodal_closed_loop import (
            MultimodalClosedLoopError,
            validate_multimodal_closed_loop_result,
        )

        result = _result("pedestrian")
        result["report"]["metrics"][0]["actor_decisions"]["trigger"]["motion_constraint"] = "free_space"
        with self.assertRaisesRegex(MultimodalClosedLoopError, "source corridor"):
            validate_multimodal_closed_loop_result(result)


if __name__ == "__main__":
    unittest.main()
