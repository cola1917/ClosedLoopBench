import unittest


class ActorPolicyConfigTests(unittest.TestCase):
    def test_trigger_actor_uses_scripted_trigger_policy_with_style_profile(self):
        from actors.policy_config import build_actor_policy_config

        actor = {
            "id": "actor-cut-in",
            "role": "trigger",
            "reference_trajectory": [{"t": 0.0, "x": 0.0, "y": 0.0}],
        }

        config = build_actor_policy_config(actor, style="aggressive")

        self.assertEqual(config["actor_id"], "actor-cut-in")
        self.assertEqual(config["role"], "trigger")
        self.assertEqual(config["policy_mode"], "scripted_trigger")
        self.assertEqual(config["closed_loop_level"], "scripted")
        self.assertTrue(config["closed_loop"]["ego_responsive"])
        self.assertTrue(config["closed_loop"]["requires_carla_runtime"])
        self.assertEqual(config["conditioning"], "reference")
        self.assertEqual(config["style"], "aggressive")
        self.assertEqual(config["style_profile"]["name"], "aggressive")
        self.assertIn("min_gap_m", config["style_profile"])

    def test_context_actor_uses_replay_policy_with_style_profile(self):
        from actors.policy_config import build_actor_policy_config

        actor = {
            "name": "background-car",
            "role": "context",
        }

        config = build_actor_policy_config(actor, style="normal")

        self.assertEqual(config["actor_id"], "background-car")
        self.assertEqual(config["role"], "context")
        self.assertEqual(config["policy_mode"], "replay")
        self.assertEqual(config["closed_loop_level"], "replay")
        self.assertFalse(config["closed_loop"]["ego_responsive"])
        self.assertFalse(config["closed_loop"]["requires_carla_runtime"])
        self.assertEqual(config["conditioning"], "reference")
        self.assertEqual(config["style_profile"]["name"], "normal")

    def test_non_context_actor_can_use_reactive_rule_based_policy(self):
        from actors.policy_config import build_actor_policy_config

        actor = {
            "id": "actor-1",
            "role": "interactive",
        }

        config = build_actor_policy_config(actor, style="assertive")

        self.assertEqual(config["policy_mode"], "reactive_rule_based")
        self.assertEqual(config["closed_loop_level"], "traffic_manager_reactive")
        self.assertTrue(config["closed_loop"]["actor_state_mutates_from_ego"])
        self.assertEqual(config["style_profile"]["name"], "assertive")


if __name__ == "__main__":
    unittest.main()
