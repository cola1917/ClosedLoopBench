import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path


IDENTITY_4X4 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def calibration():
    from agents.ego_observation import build_pinhole_calibration

    return build_pinhole_calibration(
        width=900, height=256, fov_deg=100.0, sensor_to_ego=IDENTITY_4X4
    )


def ego_state():
    return {
        "speed_mps": 5.0,
        "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0},
        "velocity": {"x": 5.0, "y": 0.0, "z": 0.0},
        "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


def route():
    return {
        "route_waypoints": [[1.0, 2.0], [5.0, 2.0]],
        "route_command": "LANE_FOLLOW",
        "target_point": [5.0, 2.0],
    }


class EgoObservationTests(unittest.TestCase):
    def _aggregator(self, **kwargs):
        from agents.ego_observation import TickObservationAggregator

        return TickObservationAggregator(required_cameras=["rgb_front"], **kwargs)

    def _complete(self, aggregator, *, frame=17, camera_t=1.0, state_t=1.0, route_t=1.0):
        aggregator.receive_camera(
            "rgb_front", b"image", frame_id=frame, t_sec=camera_t, calibration=calibration()
        )
        aggregator.receive_ego_state(ego_state(), frame_id=frame, t_sec=state_t)
        aggregator.receive_route(route(), frame_id=frame, t_sec=route_t)

    def test_builds_calibrated_same_tick_observation(self):
        aggregator = self._aggregator()
        self._complete(aggregator)
        result = aggregator.build(now_sec=1.0, expected_frame_id=17)
        self.assertEqual(result["status"], "ready")
        observation = result["observation"]
        self.assertEqual(observation["source"], "carla_current_tick")
        self.assertEqual(observation["frame_id"], 17)
        self.assertEqual(observation["calibration"]["rgb_front"]["width"], 900)

    def test_missing_channel_is_fail_closed(self):
        aggregator = self._aggregator()
        aggregator.receive_ego_state(ego_state(), frame_id=1, t_sec=1.0)
        result = aggregator.build(now_sec=1.0)
        self.assertEqual(result["reason"], "missing_channel")
        self.assertIn("camera:rgb_front", result["detail"]["missing"])

    def test_mixed_carla_ticks_are_rejected(self):
        aggregator = self._aggregator()
        self._complete(aggregator)
        aggregator.receive_route(route(), frame_id=18, t_sec=1.0)
        result = aggregator.build(now_sec=1.0)
        self.assertEqual(result["reason"], "tick_mismatch")

    def test_stale_and_timestamp_skew_are_rejected(self):
        stale = self._aggregator(timeout_sec=0.2)
        self._complete(stale)
        self.assertEqual(stale.build(now_sec=1.21)["reason"], "stale_observation")

        skewed = self._aggregator(max_skew_sec=0.01)
        self._complete(skewed, route_t=1.02)
        self.assertEqual(skewed.build(now_sec=1.02)["reason"], "timestamp_skew")

    def test_missing_state_fields_and_bad_calibration_are_rejected(self):
        aggregator = self._aggregator()
        aggregator.receive_camera(
            "rgb_front", b"image", frame_id=1, t_sec=1.0, calibration=calibration()
        )
        aggregator.receive_ego_state({"speed_mps": 1.0}, frame_id=1, t_sec=1.0)
        aggregator.receive_route(route(), frame_id=1, t_sec=1.0)
        self.assertEqual(aggregator.build(now_sec=1.0)["reason"], "invalid_ego_state")

        invalid_calibration = self._aggregator()
        self._complete(invalid_calibration)
        invalid_calibration.receive_camera(
            "rgb_front", b"image", frame_id=17, t_sec=1.0, calibration={"width": 900}
        )
        self.assertEqual(
            invalid_calibration.build(now_sec=1.0)["reason"], "invalid_calibration"
        )

    def test_sensor_specs_are_explicit_and_cli_is_model_free(self):
        from agents.ego_observation import build_camera_sensor_specs
        from runners.build_ego_observation_contract import main

        specs = build_camera_sensor_specs(["rgb_front"])
        self.assertEqual(specs["rgb_front"]["blueprint"], "sensor.camera.rgb")
        self.assertEqual(specs["rgb_front"]["runtime_calibration"]["intrinsic_shape"], [3, 3])

        with tempfile.TemporaryDirectory() as tmpdir:
            output = Path(tmpdir) / "observation.json"
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                self.assertEqual(main(["--output", str(output)]), 0)
            contract = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(contract["aggregation"]["policy"], "same_carla_tick")
        self.assertIn("not_validated", contract["aggregation"]["runtime_binding"])


if __name__ == "__main__":
    unittest.main()
