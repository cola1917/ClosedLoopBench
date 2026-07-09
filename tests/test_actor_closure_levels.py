import unittest


class ActorClosureLevelTests(unittest.TestCase):
    def test_declares_three_actor_closed_loop_levels(self):
        from actors.closure_levels import available_actor_closure_levels, get_actor_closure_level

        self.assertEqual(
            set(available_actor_closure_levels()),
            {"replay", "scripted", "traffic_manager_reactive"},
        )

        replay = get_actor_closure_level("replay")
        scripted = get_actor_closure_level("scripted")
        reactive = get_actor_closure_level("traffic_manager_reactive")

        self.assertFalse(replay.ego_responsive)
        self.assertFalse(replay.actor_state_mutates_from_ego)
        self.assertFalse(replay.requires_carla_runtime)
        self.assertTrue(scripted.ego_responsive)
        self.assertTrue(scripted.requires_carla_runtime)
        self.assertTrue(reactive.ego_responsive)
        self.assertTrue(reactive.actor_state_mutates_from_ego)

    def test_maps_policy_modes_to_actor_closure_levels(self):
        from actors.closure_levels import actor_closure_level_for_policy

        self.assertEqual(actor_closure_level_for_policy("replay").name, "replay")
        self.assertEqual(actor_closure_level_for_policy("scripted_trigger").name, "scripted")
        self.assertEqual(
            actor_closure_level_for_policy("reactive_rule_based").name,
            "traffic_manager_reactive",
        )


if __name__ == "__main__":
    unittest.main()
