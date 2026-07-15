import unittest


class RuntimeMetricSourceTests(unittest.TestCase):
    def test_route_progress_projects_physical_pose_onto_route(self):
        from runners.run_carla_basic_agent import _route_progress_from_pose

        route = [
            {"x": 0.0, "y": 0.0},
            {"x": 10.0, "y": 0.0},
            {"x": 10.0, "y": 10.0},
        ]

        self.assertAlmostEqual(_route_progress_from_pose(route, {"x": 5.0, "y": 0.0}), 0.25)
        self.assertAlmostEqual(_route_progress_from_pose(route, {"x": 10.0, "y": 5.0}), 0.75)
        self.assertAlmostEqual(_route_progress_from_pose(route, {"x": 10.0, "y": 10.0}), 1.0)

    def test_collision_tracker_reports_only_ticks_with_events(self):
        from runners.run_carla_basic_agent import _CollisionTracker

        tracker = _CollisionTracker()
        self.assertFalse(tracker.consume_tick())
        tracker.on_collision(object())
        self.assertTrue(tracker.consume_tick())
        self.assertFalse(tracker.consume_tick())

    def test_runtime_report_keeps_collision_unknown_without_sensor_samples(self):
        from metrics.report import build_closed_loop_report

        report = build_closed_loop_report(
            {"scenario_id": "missing-collision-sensor", "actors": []},
            tick_metrics=[{"collision": None, "route_progress": 0.5}],
            status="ego_closed_loop",
        )

        self.assertIsNone(report["summary"]["collision_count"])
        collision = next(
            item
            for item in report["evaluation"]["criteria"]
            if item["metric"] == "collision_count"
        )
        self.assertEqual(collision["result"], "unknown")

    def test_world_weather_accepts_named_carla_preset(self):
        from runners.run_carla_basic_agent import _apply_world_weather

        clear_noon = object()

        class FakeCarla:
            WeatherParameters = type("WeatherParameters", (), {"ClearNoon": clear_noon})

        class FakeWorld:
            def set_weather(self, weather):
                self.weather = weather

        world = FakeWorld()
        _apply_world_weather(FakeCarla, world, "ClearNoon")
        self.assertIs(world.weather, clear_noon)


if __name__ == "__main__":
    unittest.main()
