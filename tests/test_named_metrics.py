import math
import unittest


class NamedSafetyMetricTests(unittest.TestCase):
    def test_ttc_uses_distance_over_positive_closing_speed(self):
        from metrics.named import time_to_collision

        self.assertEqual(time_to_collision(distance_m=30.0, closing_speed_mps=10.0), 3.0)

    def test_ttc_returns_none_when_not_closing(self):
        from metrics.named import time_to_collision

        self.assertIsNone(time_to_collision(distance_m=30.0, closing_speed_mps=0.0))
        self.assertIsNone(time_to_collision(distance_m=30.0, closing_speed_mps=-2.0))

    def test_drac_uses_relative_speed_squared_over_two_distance(self):
        from metrics.named import drac

        self.assertEqual(drac(distance_m=20.0, closing_speed_mps=10.0), 2.5)

    def test_drac_handles_non_positive_distance_safely(self):
        from metrics.named import drac

        self.assertTrue(math.isinf(drac(distance_m=0.0, closing_speed_mps=5.0)))
        self.assertEqual(drac(distance_m=0.0, closing_speed_mps=0.0), 0.0)

    def test_tet_counts_time_exposed_below_ttc_threshold(self):
        from metrics.named import time_exposed_ttc

        ttcs = [3.0, 1.8, None, 1.2, 2.5]

        self.assertAlmostEqual(time_exposed_ttc(ttcs, threshold_s=2.0, dt_s=0.1), 0.2)

    def test_tit_integrates_ttc_deficit_below_threshold(self):
        from metrics.named import time_integrated_ttc

        ttcs = [3.0, 1.8, None, 1.2, 2.5]

        self.assertAlmostEqual(time_integrated_ttc(ttcs, threshold_s=2.0, dt_s=0.1), 0.1)


class NamedComfortMetricTests(unittest.TestCase):
    def test_jerk_from_acceleration_sequence(self):
        from metrics.named import jerk_from_acceleration

        self.assertEqual(
            jerk_from_acceleration([0.0, 1.0, -1.0, -4.0], dt_s=0.5),
            [2.0, -4.0, -6.0],
        )

    def test_hard_brake_count_uses_longitudinal_acceleration_threshold(self):
        from metrics.named import hard_brake_count

        self.assertEqual(
            hard_brake_count([0.0, -2.9, -3.0, -4.5], threshold_mps2=-3.0),
            2,
        )


if __name__ == "__main__":
    unittest.main()
