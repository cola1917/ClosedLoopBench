import unittest
from pathlib import Path


class ReferencePurePursuitPluginTests(unittest.TestCase):
    def _config(self, profile="short", **extra):
        return {
            "repo_path": str(Path(__file__).resolve().parents[1]),
            "profile": profile,
            **extra,
        }

    def _observation(self, *, frame_id=1, speed=2.0):
        return {
            "frame_id": frame_id,
            "timestamp": frame_id * 0.05,
            "rgb": {},
            "lidar": None,
            "ego_state": {
                "speed_mps": speed,
                "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            },
            "route": {
                "route_waypoints": [[0.0, 0.0], [3.0, 0.0], [6.0, 2.0], [10.0, 5.0]],
                "route_command": "LANE_FOLLOW",
                "target_point": [10.0, 5.0],
            },
        }

    def test_short_and_long_profiles_select_different_targets(self):
        from agents.reference_pure_pursuit import create_plugin

        short = create_plugin(self._config("short"))
        long = create_plugin(self._config("long"))
        short.reset({})
        long.reset({})
        short_control = short.predict_control(self._observation())
        long_control = long.predict_control(self._observation())
        self.assertEqual(short_control["diagnostics"]["target_index"], 2)
        self.assertEqual(long_control["diagnostics"]["target_index"], 3)
        self.assertNotEqual(short_control["steer"], long_control["steer"])

    def test_speed_controller_brakes_above_target(self):
        from agents.reference_pure_pursuit import create_plugin

        plugin = create_plugin(self._config("short", target_speed_mps=5.0))
        plugin.reset({})
        control = plugin.predict_control(self._observation(speed=8.0))
        self.assertEqual(control["throttle"], 0.0)
        self.assertGreater(control["brake"], 0.0)

    def test_reset_is_deterministic_and_clears_progress(self):
        from agents.reference_pure_pursuit import create_plugin

        plugin = create_plugin(self._config())
        plugin.reset({"seed": 1})
        first = plugin.predict_control(self._observation())
        plugin.reset({"seed": 1})
        second = plugin.predict_control(self._observation())
        self.assertEqual(first, second)
        self.assertEqual(len(plugin.diagnostics), 1)


if __name__ == "__main__":
    unittest.main()
