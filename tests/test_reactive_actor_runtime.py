import unittest


class ReactiveActorRuntimeTests(unittest.TestCase):
    def test_defensive_and_normal_yield_or_slow_when_ego_is_close(self):
        from actors.reactive_actor import plan_reactive_actor_control

        actor_state = {"speed_mps": 10.0}
        close_ego = {"distance_m": 4.0, "relative_speed_mps": 2.0}

        defensive = plan_reactive_actor_control(actor_state, close_ego, style="defensive")
        normal = plan_reactive_actor_control(actor_state, close_ego, style="normal")

        self.assertTrue(defensive["should_yield"])
        self.assertTrue(defensive["brake"])
        self.assertTrue(defensive["should_abort"])
        self.assertLess(defensive["desired_speed_mps"], actor_state["speed_mps"])

        self.assertTrue(normal["should_yield"])
        self.assertTrue(normal["brake"])
        self.assertTrue(normal["should_abort"])
        self.assertLess(normal["desired_speed_mps"], actor_state["speed_mps"])

    def test_aggressive_accepts_smaller_gap_and_keeps_lane_change_enabled(self):
        from actors.reactive_actor import plan_reactive_actor_control

        actor_state = {"speed_mps": 10.0}
        close_but_acceptable_for_aggressive = {"distance_m": 3.0, "relative_speed_mps": 1.0}

        normal = plan_reactive_actor_control(actor_state, close_but_acceptable_for_aggressive, style="normal")
        aggressive = plan_reactive_actor_control(
            actor_state,
            close_but_acceptable_for_aggressive,
            style="aggressive",
        )

        self.assertTrue(normal["should_yield"])
        self.assertFalse(aggressive["should_yield"])
        self.assertFalse(aggressive["should_abort"])
        self.assertTrue(aggressive["lane_change_enabled"])
        self.assertLess(aggressive["min_gap_m"], normal["min_gap_m"])

    def test_missing_ego_state_falls_back_to_reference_or_safe_default(self):
        from actors.reactive_actor import plan_reactive_actor_control

        with_reference = plan_reactive_actor_control(
            {"speed_mps": 7.0},
            None,
            style="defensive",
            reference_speed_mps=5.5,
        )
        without_reference = plan_reactive_actor_control({}, None, style="normal")

        self.assertFalse(with_reference["should_yield"])
        self.assertFalse(with_reference["brake"])
        self.assertEqual(with_reference["desired_speed_mps"], 5.5)
        self.assertEqual(with_reference["reason"], "no_ego_state_reference_fallback")

        self.assertFalse(without_reference["should_yield"])
        self.assertFalse(without_reference["brake"])
        self.assertEqual(without_reference["desired_speed_mps"], 0.0)
        self.assertEqual(without_reference["reason"], "no_ego_state_safe_default")

    def test_non_closing_ego_reports_missing_ttc_instead_of_infinity(self):
        from actors.reactive_actor import plan_reactive_actor_control

        decision = plan_reactive_actor_control(
            {"speed_mps": 8.0},
            {"distance_m": 20.0, "relative_speed_mps": 0.0},
            style="normal",
        )

        self.assertIsNone(decision["ttc_sec"])
        self.assertEqual(decision["reason"], "ego_state_within_style_gap")


if __name__ == "__main__":
    unittest.main()
