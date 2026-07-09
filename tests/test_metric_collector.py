import unittest


class MetricCollectorTests(unittest.TestCase):
    def test_tick_row_contains_minimal_closed_loop_fields(self):
        from metrics.collector import build_tick_row

        row = build_tick_row(
            t_sec=1.2,
            ego_pose={"x": 10.0, "y": 2.0, "yaw": 0.1},
            ego_speed_mps=6.5,
            ego_control={"throttle": 0.2, "brake": 0.0, "steer": -0.1},
            actor_distances_m={"actor-1": 12.0, "actor-2": 25.5},
            ttc=3.4,
            collision=False,
            route_progress=0.42,
            hard_brake=False,
            jerk=1.7,
        )

        self.assertEqual(row["t_sec"], 1.2)
        self.assertEqual(row["ego"]["pose"]["x"], 10.0)
        self.assertEqual(row["ego"]["speed_mps"], 6.5)
        self.assertEqual(row["ego"]["control"]["steer"], -0.1)
        self.assertEqual(row["actor_distances_m"]["actor-1"], 12.0)
        self.assertEqual(row["ttc"], 3.4)
        self.assertFalse(row["collision"])
        self.assertEqual(row["route_progress"], 0.42)
        self.assertFalse(row["hard_brake"])
        self.assertEqual(row["jerk"], 1.7)

    def test_collector_aggregates_rows_for_report(self):
        from metrics.collector import TickMetricCollector

        collector = TickMetricCollector()
        collector.add_tick(
            t_sec=0.0,
            ego_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ego_speed_mps=5.0,
            ego_control={"throttle": 0.4, "brake": 0.0, "steer": 0.0},
            actor_distances_m={"actor-1": 20.0},
            ttc=4.0,
            collision=False,
            route_progress=0.1,
            hard_brake=False,
            jerk=0.0,
        )
        collector.add_tick(
            t_sec=0.1,
            ego_pose={"x": 0.5, "y": 0.0, "yaw": 0.0},
            ego_speed_mps=4.5,
            ego_control={"throttle": 0.0, "brake": 0.8, "steer": 0.0},
            actor_distances_m={"actor-1": 8.0},
            ttc=1.2,
            collision=True,
            route_progress=0.2,
            hard_brake=True,
            jerk=-8.0,
        )

        rows = collector.to_report_rows()

        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["min_ttc"], 4.0)
        self.assertEqual(rows[1]["min_ttc"], 1.2)
        self.assertTrue(rows[1]["collision"])
        self.assertTrue(rows[1]["hard_brake"])

    def test_collector_handoff_to_closed_loop_report(self):
        from metrics.collector import TickMetricCollector
        from metrics.report import build_closed_loop_report

        run_config = {
            "scenario_id": "scene-test-collector",
            "actors": [{"actor_id": "a1", "policy": "replay", "closed_loop_level": "replay"}],
        }
        collector = TickMetricCollector()
        collector.add_tick(
            t_sec=0.0,
            ego_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ego_speed_mps=3.0,
            ego_control={"throttle": 0.1, "brake": 0.0, "steer": 0.0},
            actor_distances_m={"a1": 10.0},
            ttc=2.5,
            collision=False,
            route_progress=0.5,
            hard_brake=False,
            jerk=2.0,
        )

        report = build_closed_loop_report(
            run_config,
            tick_metrics=collector.to_report_rows(),
            status="ego_closed_loop",
        )

        self.assertEqual(report["status"], "ego_closed_loop")
        self.assertEqual(report["summary"]["min_ttc"], 2.5)
        self.assertEqual(report["summary"]["route_progress"], 0.5)
        self.assertEqual(report["metrics"][0]["ego"]["speed_mps"], 3.0)


if __name__ == "__main__":
    unittest.main()
