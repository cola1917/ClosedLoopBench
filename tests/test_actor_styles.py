import unittest


class ActorStyleProfileTests(unittest.TestCase):
    def test_returns_required_style_templates(self):
        from actors.style_profiles import ActorStyleProfile, available_actor_styles

        expected_styles = {
            "defensive",
            "normal",
            "assertive",
            "aggressive",
            "delayed",
            "noncompliant",
        }

        self.assertEqual(set(available_actor_styles()), expected_styles)

        for style in expected_styles:
            profile = ActorStyleProfile.for_style(style)
            self.assertEqual(profile.name, style)
            self.assertIsInstance(profile.desired_time_headway_sec, float)
            self.assertIsInstance(profile.min_gap_m, float)
            self.assertIsInstance(profile.reaction_time_sec, float)
            self.assertIsInstance(profile.yield_ttc_threshold_sec, float)
            self.assertIsInstance(profile.lane_change_gap_acceptance_m, float)
            self.assertIsInstance(profile.abort_on_low_ttc, bool)

    def test_style_relationships_match_behavioral_intent(self):
        from actors.style_profiles import ActorStyleProfile

        defensive = ActorStyleProfile.for_style("defensive")
        aggressive = ActorStyleProfile.for_style("aggressive")
        normal = ActorStyleProfile.for_style("normal")
        delayed = ActorStyleProfile.for_style("delayed")

        self.assertLess(aggressive.min_gap_m, defensive.min_gap_m)
        self.assertGreater(delayed.reaction_time_sec, normal.reaction_time_sec)

    def test_unknown_style_is_rejected(self):
        from actors.style_profiles import ActorStyleProfile

        with self.assertRaises(ValueError):
            ActorStyleProfile.for_style("mystery")


if __name__ == "__main__":
    unittest.main()
