import unittest


def _config():
    return {"scenario_id": "extended-metrics", "actors": []}


class ExtendedMetricTests(unittest.TestCase):
    def test_summarizes_duration_safety_latency_actor_and_sensor_metrics(self):
        from metrics.report import build_closed_loop_report

        rows = [
            {
                "t_sec": 0.0,
                "ego": {"speed_mps": 0.0},
                "actor_distances_m": {"lead": 12.0},
                "collision": False,
                "ttc": 4.0,
                "route_progress": 0.0,
                "following": False,
                "hard_brake": False,
                "jerk": 0.0,
                "control_latency_ms": 10.0,
                "control_timeout": False,
                "control_fallback": False,
                "sensor_dropped_frames": 0,
                "synchronization_error_us": 100,
            },
            {
                "t_sec": 0.1,
                "ego": {"speed_mps": 5.0},
                "actor_distances_m": {"lead": 8.0},
                "collision": False,
                "ttc": 2.0,
                "route_progress": 0.5,
                "following": True,
                "hard_brake": True,
                "jerk": -5.0,
                "post_encroachment_time_sec": 1.5,
                "drac_mps2": 2.5,
                "control_latency_ms": 20.0,
                "control_timeout": True,
                "control_fallback": True,
                "actor_outcomes": {"ped": "yield"},
                "sensor_dropped_frames": 1,
                "synchronization_error_ms": 0.3,
            },
            {
                "t_sec": 0.2,
                "ego": {"speed_mps": 10.0},
                "actor_distances_m": {"lead": 5.0},
                "collision": False,
                "ttc": 1.2,
                "route_progress": 1.0,
                "following": True,
                "hard_brake": True,
                "jerk": 10.0,
                "control_latency_ms": 30.0,
                "control_timeout": False,
                "control_fallback": False,
                "sensor_dropped_frames": 0,
                "synchronization_error_ms": 0.2,
            },
        ]

        summary = build_closed_loop_report(_config(), rows, status="completed")["summary"]

        self.assertAlmostEqual(summary["route_completion_time_sec"], 0.2)
        self.assertAlmostEqual(summary["average_speed_mps"], 5.0)
        self.assertAlmostEqual(summary["stopped_time_sec"], 0.1)
        self.assertAlmostEqual(summary["following_time_sec"], 0.1)
        self.assertEqual(summary["min_distance_m"], 5.0)
        self.assertEqual(summary["min_pet_sec"], 1.5)
        self.assertEqual(summary["max_drac_mps2"], 2.5)
        self.assertEqual(summary["hard_brake_count"], 1)
        self.assertEqual(summary["max_jerk"], 10.0)
        self.assertEqual(summary["control_latency_p50_ms"], 20.0)
        self.assertEqual(summary["control_latency_p95_ms"], 29.0)
        self.assertEqual(summary["control_timeout_count"], 1)
        self.assertAlmostEqual(summary["control_timeout_rate"], 1 / 3)
        self.assertEqual(summary["control_fallback_count"], 1)
        self.assertEqual(summary["actor_outcomes"], {"yield": 1})
        self.assertEqual(summary["sensor_dropped_frame_count"], 1)
        self.assertEqual(summary["max_synchronization_error_ms"], 0.3)

    def test_hard_brake_is_event_based_and_jerk_ignores_nonfinite_or_reversed_samples(self):
        from metrics.report import build_closed_loop_report

        rows = [
            {"t_sec": 0.0, "collision": False, "ttc": 3.0, "route_progress": 0.0, "hard_brake": True, "jerk": 3.0},
            {"t_sec": 0.1, "collision": False, "ttc": 3.0, "route_progress": 0.2, "hard_brake": True, "jerk": float("nan")},
            {"t_sec": 0.2, "collision": False, "ttc": 3.0, "route_progress": 0.4, "hard_brake": False, "jerk": 4.0},
            {"t_sec": 0.15, "collision": False, "ttc": 3.0, "route_progress": 0.6, "hard_brake": True, "jerk": 999.0},
            {"t_sec": 0.3, "collision": False, "ttc": 3.0, "route_progress": 1.0, "hard_brake": True, "jerk": -5.0},
        ]
        summary = build_closed_loop_report(_config(), rows, status="completed")["summary"]
        self.assertEqual(summary["hard_brake_count"], 2)
        self.assertEqual(summary["max_jerk"], 5.0)

    def test_missing_sources_are_explicitly_unavailable_and_fail_closed(self):
        from metrics.report import build_closed_loop_report

        report = build_closed_loop_report(
            _config(),
            [{"t_sec": 0.0, "route_progress": 1.0}],
            status="completed",
        )
        summary = report["summary"]
        self.assertIsNone(summary["collision_count"])
        self.assertIsNone(summary["min_ttc"])
        self.assertIsNone(summary["control_timeout_count"])
        self.assertFalse(summary["metric_availability"]["collision_count"])
        self.assertFalse(summary["metric_availability"]["min_ttc"])
        self.assertEqual(report["evaluation"]["overall_result"], "unknown")

    def test_collector_preserves_extended_fields(self):
        from metrics.collector import build_tick_row

        row = build_tick_row(
            t_sec=1.0,
            ego_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ego_speed_mps=2.0,
            ego_control={"throttle": 0.1, "brake": 0.0, "steer": 0.0},
            actor_distances_m={},
            ttc=None,
            collision=None,
            route_progress=0.1,
            hard_brake=False,
            jerk=None,
            following=True,
            post_encroachment_time_sec=2.0,
            drac_mps2=1.5,
            control_latency_ms=8.0,
            control_timeout=False,
            control_fallback=False,
            actor_outcomes={"ped": "crossing"},
            sensor_dropped_frames=2,
            synchronization_error_ms=0.4,
        )
        self.assertTrue(row["following"])
        self.assertEqual(row["post_encroachment_time_sec"], 2.0)
        self.assertEqual(row["actor_outcomes"], {"ped": "crossing"})
        self.assertEqual(row["sensor_dropped_frames"], 2)


if __name__ == "__main__":
    unittest.main()
