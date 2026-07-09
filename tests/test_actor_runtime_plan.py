import json
import unittest


class ActorRuntimePlanTests(unittest.TestCase):
    def test_replay_actor_plan_is_half_closed_loop_and_uses_reference_trajectory(self):
        from actors.policy_config import build_actor_policy_config
        from actors.runtime_plan import build_actor_runtime_plan

        actor = {
            "actor_id": "background-1",
            "role": "context",
            "type": "vehicle",
            "policy": "replay",
            "initial_state": {"x": 1.0, "y": 2.0, "yaw": 0.1, "speed_mps": 4.0},
            "reference_trajectory": [{"t_sec": 0.0, "x": 1.0, "y": 2.0}],
        }
        policy = build_actor_policy_config(actor)

        plan = build_actor_runtime_plan(actor, policy)

        self.assertEqual(plan["actor_id"], "background-1")
        self.assertEqual(plan["runtime_mode"], "replay")
        self.assertEqual(plan["closed_loop_level"], "replay")
        self.assertFalse(plan["interactive_candidate"])
        self.assertEqual(plan["controller"]["type"], "trajectory_replay")
        self.assertEqual(plan["controller"]["reference_source"], "scenario_ir")
        self.assertEqual(plan["controller"]["trajectory_points"], 1)
        self.assertFalse(plan["requires_carla_runtime"])

    def test_scripted_actor_plan_is_interactive_candidate_without_rule_engine(self):
        from actors.policy_config import build_actor_policy_config
        from actors.runtime_plan import build_actor_runtime_plan

        actor = {
            "actor_id": "cut-in",
            "role": "trigger",
            "type": "vehicle",
            "policy": "scripted_trigger",
            "initial_state": {"x": 0.0, "y": 0.0, "yaw": 0.0, "speed_mps": 6.0},
            "reference_trajectory": [{"t_sec": 0.0, "x": 0.0, "y": 0.0}],
        }
        policy = build_actor_policy_config(actor, style="aggressive")

        plan = build_actor_runtime_plan(actor, policy)

        self.assertEqual(plan["runtime_mode"], "scripted")
        self.assertEqual(plan["closed_loop_level"], "scripted")
        self.assertTrue(plan["interactive_candidate"])
        self.assertEqual(plan["controller"]["type"], "scripted_reference_conditioned")
        self.assertIn("trigger_conditions", plan["controller"])
        self.assertNotIn("rules", plan["controller"])
        self.assertEqual(plan["style_profile"]["name"], "aggressive")

    def test_traffic_manager_actor_plan_contains_low_code_configuration(self):
        from actors.policy_config import build_actor_policy_config
        from actors.runtime_plan import build_actor_runtime_plan

        actor = {
            "actor_id": "reactive-1",
            "role": "interactive",
            "type": "vehicle",
            "initial_state": {"x": 0.0, "y": 0.0, "yaw": 0.0, "speed_mps": 8.0},
            "reference_trajectory": [{"t_sec": 0.0, "x": 0.0, "y": 0.0}],
        }
        policy = build_actor_policy_config(actor, style="defensive")

        plan = build_actor_runtime_plan(actor, policy)

        self.assertEqual(plan["runtime_mode"], "traffic_manager")
        self.assertEqual(plan["closed_loop_level"], "traffic_manager_reactive")
        self.assertTrue(plan["interactive_candidate"])
        self.assertEqual(plan["controller"]["type"], "carla_traffic_manager")
        self.assertEqual(plan["controller"]["runtime_binding"], "deferred")
        self.assertIn("desired_time_headway_sec", plan["controller"]["parameters"])
        self.assertIn("min_gap_m", plan["controller"]["parameters"])
        self.assertNotIn("carla_actor_id", plan["controller"])

    def test_build_actor_runtime_plan_set_from_run_config(self):
        from actors.runtime_plan import build_actor_runtime_plan_set

        run_config = {
            "schema_version": "carla_run_config.mvp.v0",
            "scenario_id": "scenario-1",
            "actors": [
                {
                    "actor_id": "ctx",
                    "role": "context",
                    "type": "vehicle",
                    "policy": "replay",
                    "reference_trajectory": [],
                },
                {
                    "actor_id": "trigger",
                    "role": "trigger",
                    "type": "vehicle",
                    "policy": "scripted_trigger",
                    "reference_trajectory": [],
                },
            ],
        }

        plan_set = build_actor_runtime_plan_set(run_config, style="normal")

        self.assertEqual(plan_set["schema_version"], "actor_runtime_plan.mvp.v0")
        self.assertEqual(plan_set["scenario_id"], "scenario-1")
        self.assertEqual(plan_set["summary"]["actor_count"], 2)
        self.assertEqual(plan_set["summary"]["runtime_modes"]["replay"], 1)
        self.assertEqual(plan_set["summary"]["runtime_modes"]["scripted"], 1)
        self.assertEqual(plan_set["summary"]["interactive_candidate_count"], 1)
        json.dumps(plan_set)


if __name__ == "__main__":
    unittest.main()
