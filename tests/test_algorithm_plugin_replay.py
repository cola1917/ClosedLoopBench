import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path


def observation(frame_id):
    return {
        "frame_id": frame_id,
        "timestamp": frame_id * 0.05,
        "rgb": {},
        "lidar": None,
        "ego_state": {
            "speed_mps": 2.0,
            "pose": {"x": float(frame_id - 1), "y": 0.0, "yaw": 0.0},
        },
        "route": {
            "route_waypoints": [[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]],
            "route_command": "LANE_FOLLOW",
            "target_point": [10.0, 0.0],
        },
        "sensor_validity": {"lidar": True},
        "synchronization": {"frame_id": frame_id},
    }


class AlgorithmPluginReplayTests(unittest.TestCase):
    def test_jsonl_replay_outputs_offline_trace_and_simulated_safe_stop(self):
        from runners.replay_algorithm_plugin import main

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            config = root / "config.json"
            observations = root / "observations.jsonl"
            trace = root / "control.jsonl"
            report = root / "report.json"
            config.write_text(
                json.dumps(
                    {
                        "repo_path": str(Path(__file__).resolve().parents[1]),
                        "profile": "short",
                    }
                ),
                encoding="utf-8",
            )
            observations.write_text(
                "".join(json.dumps(observation(frame)) + "\n" for frame in (1, 2, 3)),
                encoding="utf-8",
            )
            with contextlib.redirect_stdout(io.StringIO()):
                exit_code = main(
                    [
                        "--plugin",
                        "agents.reference_pure_pursuit:create_plugin",
                        "--config",
                        str(config),
                        "--observations",
                        str(observations),
                        "--control-trace",
                        str(trace),
                        "--report",
                        str(report),
                        "--simulate-timeout-frame",
                        "2",
                    ]
                )
            payload = json.loads(report.read_text(encoding="utf-8"))
            rows = [json.loads(line) for line in trace.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(exit_code, 0)
        self.assertEqual(payload["evidence_classification"], "offline_conformance")
        self.assertFalse(payload["real_carla_nurec_closed_loop"])
        self.assertEqual(payload["fallback_count"], 1)
        self.assertEqual(rows[1]["control"]["reason"], "timeout")
        self.assertEqual(rows[1]["control"]["brake"], 1.0)

    def test_determinism_verification_passes_for_reference_plugin(self):
        from runners.replay_algorithm_plugin import verify_determinism

        result = verify_determinism(
            plugin_spec="agents.reference_pure_pursuit:create_plugin",
            plugin_config={"repo_path": str(Path(__file__).resolve().parents[1])},
            observations=[observation(1), observation(2)],
        )
        self.assertTrue(result["passed"])

    def test_cli_refuses_to_overwrite_historical_trace(self):
        from runners.replay_algorithm_plugin import main

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            config = root / "config.json"
            observations = root / "observations.jsonl"
            trace = root / "existing-control.jsonl"
            report = root / "report.json"
            config.write_text(json.dumps({"repo_path": str(root)}), encoding="utf-8")
            observations.write_text(json.dumps(observation(1)) + "\n", encoding="utf-8")
            trace.write_text("historical\n", encoding="utf-8")
            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main(
                        [
                            "--plugin",
                            "agents.reference_pure_pursuit:create_plugin",
                            "--config",
                            str(config),
                            "--observations",
                            str(observations),
                            "--control-trace",
                            str(trace),
                            "--report",
                            str(report),
                        ]
                    )
            self.assertEqual(trace.read_text(encoding="utf-8"), "historical\n")


if __name__ == "__main__":
    unittest.main()
