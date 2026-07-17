import unittest
from types import SimpleNamespace

from tests.test_basic_agent_runtime_loop import (
    BasicAgentRuntimeLoopTests,
    FakeBasicAgent,
    FakeCarlaModule,
    FakeClient,
    FakeWorld,
)
from tests.test_actor_binding import SCENE_TOKEN


def _plan():
    plan = BasicAgentRuntimeLoopTests()._interactive_plan()
    plan["scenario_id"] = SCENE_TOKEN
    actor = plan["actors"][0]
    actor["source_track_id"] = "track-trigger"
    actor["role_name"] = "actor.trigger"
    actor["binding"] = {
        "schema_version": "actor_runtime_binding.v1",
        "nurec_track_id": "track-trigger",
        "sensor_pose_source": "carla_runtime_actor_pose",
        "required_modalities": ["rgb", "lidar"],
        "same_dynamic_object_for_all_modalities": True,
        "declared_status": "ready",
    }
    plan["actor_binding"] = {
        "schema_version": "actor_binding_set.v1",
        "scene_id": plan["scenario_id"],
        "readiness": {"status": "ready", "blockers": []},
        "selected_actor_ids": ["trigger"],
    }
    plan["runtime"]["multimodal_sensor_required"] = True
    return plan


def _evidence(context, *, passed=True):
    status = "passed" if passed else "failed"
    issues = [] if passed else ["synthetic_rpc_failure"]
    records = []
    for modality in ("rgb", "lidar"):
        records.append(
            {
                "request_id": f"{context['frame_id']}:{modality}",
                "modality": modality,
                "sensor_id": "CAM_FRONT" if modality == "rgb" else "LIDAR_TOP",
                "status": status,
                "latency_ms": 1.0,
                "payload_sha256": ("1" if modality == "rgb" else "2") * 64,
                "issues": [] if passed else ["synthetic_rpc_failure"],
            }
        )
    return {
        "schema_version": "nurec_multimodal_evidence.v1",
        "scene_id": context["scene_id"],
        "frame_id": context["frame_id"],
        "simulation_time_sec": context["simulation_time_sec"],
        "dynamic_object_sha256": "d" * 64,
        "dynamic_object_count": len(context["actor_samples"]),
        "records": records,
        "modalities": {
            "rgb": {"requested_count": 1, "passed_count": 1 if passed else 0},
            "lidar": {"requested_count": 1, "passed_count": 1 if passed else 0},
        },
        "max_latency_ms": None,
        "issues": issues,
        "status": status,
    }


class BoundWorld(FakeWorld):
    def __init__(self, events):
        super().__init__(events)
        self.frame = 0

    def tick(self):
        self.frame += 1
        self.events.append("world.tick")
        return self.frame

    def _vehicle_for_blueprint(self, blueprint):
        entity = super()._vehicle_for_blueprint(blueprint)
        role_name = blueprint.attributes.get("role_name", "vehicle")
        if role_name != "ego_vehicle":
            entity.id = 101
            entity.attributes = {"role_name": role_name}
        return entity


class BoundClient(FakeClient):
    def __init__(self, events, host, port):
        super().__init__(events, host, port)
        self.world = BoundWorld(events)


class BoundCarla(FakeCarlaModule):
    def Client(self, host, port):
        return BoundClient(self.events, host, port)


class NuRecRunnerIntegrationTests(unittest.TestCase):
    def test_sensor_handler_is_closed_and_audited(self):
        from runners.run_carla_basic_agent import run_basic_agent

        class Handler:
            def __init__(self):
                self.closed = False

            def __call__(self, context):
                return _evidence(context)

            def close(self):
                self.closed = True

        handler = Handler()
        result = run_basic_agent(
            _plan(),
            carla_module=BoundCarla([]),
            agent_module=FakeBasicAgent,
            sensor_frame_handler=handler,
        )

        self.assertTrue(handler.closed)
        self.assertIn(
            {"action": "sensor_frame_handler.close", "status": "succeeded"},
            result["cleanup_audit"],
        )

    def test_absolute_carla_clock_keeps_relative_scenario_time_and_local_pose_interval(self):
        from runners.run_carla_basic_agent import run_basic_agent

        class AbsoluteClockWorld(BoundWorld):
            def get_snapshot(self):
                return SimpleNamespace(
                    frame=self.frame,
                    timestamp=SimpleNamespace(
                        elapsed_seconds=100.0 + (self.frame - 1) * 0.05,
                        delta_seconds=0.05,
                    ),
                )

        class AbsoluteClockClient(BoundClient):
            def __init__(self, events, host, port):
                super().__init__(events, host, port)
                self.world = AbsoluteClockWorld(events)

        class AbsoluteClockCarla(BoundCarla):
            def Client(self, host, port):
                return AbsoluteClockClient(self.events, host, port)

        contexts = []
        result = run_basic_agent(
            _plan(),
            carla_module=AbsoluteClockCarla([]),
            agent_module=FakeBasicAgent,
            sensor_frame_handler=lambda context: contexts.append(context) or _evidence(context),
        )

        self.assertEqual(result["status"], "interactive_closed_loop")
        self.assertAlmostEqual(contexts[0]["simulation_time_sec"], 100.0)
        self.assertAlmostEqual(contexts[0]["interval_start_sec"], 99.95)
        self.assertAlmostEqual(contexts[0]["scenario_time_sec"], 0.05)
        self.assertAlmostEqual(contexts[1]["interval_start_sec"], 100.0)

    def test_required_handler_receives_physical_pose_pairs_and_proves_both_modalities(self):
        from runners.run_carla_basic_agent import run_basic_agent

        contexts = []

        def handler(context):
            contexts.append(context)
            return _evidence(context)

        result = run_basic_agent(
            _plan(),
            carla_module=BoundCarla([]),
            agent_module=FakeBasicAgent,
            sensor_frame_handler=handler,
        )

        self.assertEqual(result["status"], "interactive_closed_loop")
        self.assertEqual(len(contexts), 2)
        self.assertEqual([item["frame_id"] for item in contexts], [1, 2])
        sample = contexts[0]["actor_samples"]["trigger"]
        self.assertEqual(sample["source"], "carla_runtime_actor_pose")
        self.assertEqual(sample["carla_runtime_actor_id"], 101)
        self.assertEqual(sample["nurec_track_id"], "track-trigger")
        self.assertNotEqual(sample["pose_pair"]["start"]["x"], sample["pose_pair"]["end"]["x"])
        runtime = result["report"]["runtime"]
        self.assertEqual(runtime["actor_runtime_binding"]["status"], "passed")
        self.assertEqual(runtime["multimodal_sensor"]["status"], "passed")
        self.assertTrue(runtime["multimodal_sensor"]["sensor_closed_loop"])
        self.assertEqual(runtime["multimodal_sensor"]["modalities"], ["rgb", "lidar"])

    def test_required_mode_rejects_missing_handler_or_failed_rpc_evidence(self):
        from runners.run_carla_basic_agent import run_basic_agent

        missing = run_basic_agent(
            _plan(),
            carla_module=BoundCarla([]),
            agent_module=FakeBasicAgent,
        )
        self.assertEqual(missing["status"], "failed")
        self.assertIn("no sensor_frame_handler", missing["detail"])

        failed = run_basic_agent(
            _plan(),
            carla_module=BoundCarla([]),
            agent_module=FakeBasicAgent,
            sensor_frame_handler=lambda context: _evidence(context, passed=False),
        )
        self.assertEqual(failed["status"], "failed")
        self.assertIn("required NuRec multimodal frame failed", failed["detail"])
        self.assertEqual(
            failed["report"]["runtime"]["multimodal_sensor"]["status"],
            "failed",
        )

    def test_frame_identity_must_be_strictly_increasing(self):
        from runners.run_carla_basic_agent import run_basic_agent

        class ConstantFrameWorld(BoundWorld):
            def tick(self):
                self.events.append("world.tick")
                return 1

        class ConstantFrameClient(BoundClient):
            def __init__(self, events, host, port):
                super().__init__(events, host, port)
                self.world = ConstantFrameWorld(events)

        class ConstantFrameCarla(BoundCarla):
            def Client(self, host, port):
                return ConstantFrameClient(self.events, host, port)

        result = run_basic_agent(
            _plan(),
            carla_module=ConstantFrameCarla([]),
            agent_module=FakeBasicAgent,
            sensor_frame_handler=lambda context: _evidence(context),
        )
        self.assertEqual(result["status"], "failed")
        self.assertIn("not strictly increasing", result["detail"])

    def test_runtime_role_name_mismatch_fails_before_rendering(self):
        from runners.run_carla_basic_agent import run_basic_agent

        class WrongRoleWorld(BoundWorld):
            def _vehicle_for_blueprint(self, blueprint):
                entity = super()._vehicle_for_blueprint(blueprint)
                role_name = blueprint.attributes.get("role_name", "vehicle")
                if role_name != "ego_vehicle":
                    entity.attributes = {"role_name": "wrong-role"}
                return entity

        class WrongRoleClient(BoundClient):
            def __init__(self, events, host, port):
                super().__init__(events, host, port)
                self.world = WrongRoleWorld(events)

        class WrongRoleCarla(BoundCarla):
            def Client(self, host, port):
                return WrongRoleClient(self.events, host, port)

        calls = []
        result = run_basic_agent(
            _plan(),
            carla_module=WrongRoleCarla([]),
            agent_module=FakeBasicAgent,
            sensor_frame_handler=lambda context: calls.append(context) or _evidence(context),
        )
        self.assertEqual(result["status"], "failed")
        self.assertIn("carla_role_name_mismatch", result["detail"])
        self.assertEqual(calls, [])

    def test_source_and_nurec_track_identity_mismatch_fails_before_rendering(self):
        from runners.run_carla_basic_agent import run_basic_agent

        plan = _plan()
        plan["actors"][0]["binding"]["nurec_track_id"] = "different-track"
        calls = []
        result = run_basic_agent(
            plan,
            carla_module=BoundCarla([]),
            agent_module=FakeBasicAgent,
            sensor_frame_handler=lambda context: calls.append(context) or _evidence(context),
        )

        self.assertEqual(result["status"], "failed")
        self.assertIn("source_nurec_track_mismatch", result["detail"])
        self.assertEqual(calls, [])


if __name__ == "__main__":
    unittest.main()
