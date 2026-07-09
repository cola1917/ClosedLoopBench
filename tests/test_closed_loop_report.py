import json
import tempfile
import unittest
from pathlib import Path


def _minimal_run_config(reconstruction_package=None):
    return {
        "schema_version": "carla_run_config.mvp.v0",
        "scenario_id": "scene-test-001",
        "actors": [
            {
                "actor_id": "trigger",
                "policy": "reactive_rule_based",
                "closed_loop_level": "traffic_manager_reactive",
            },
            {"actor_id": "context", "policy": "replay", "closed_loop_level": "replay"},
            {"actor_id": "parked", "policy": "replay", "closed_loop_level": "replay"},
        ],
        "metrics": ["collision", "min_ttc", "route_progress"],
        "reconstruction_package": reconstruction_package
        or {
            "enabled": False,
            "package_path": None,
        },
    }


class ClosedLoopReportTests(unittest.TestCase):
    def test_builds_not_run_closed_loop_report_from_run_config(self):
        from metrics.report import build_closed_loop_report

        report = build_closed_loop_report(_minimal_run_config())

        self.assertEqual(report["schema_version"], "closed_loop_report.mvp.v0")
        self.assertEqual(report["scenario_id"], "scene-test-001")
        self.assertEqual(report["status"], "not_run")
        self.assertEqual(report["summary"]["collision_count"], 0)
        self.assertIsNone(report["summary"]["min_ttc"])
        self.assertEqual(report["summary"]["route_progress"], 0.0)
        self.assertEqual(
            report["summary"]["actor_policy_modes"],
            {"reactive_rule_based": 1, "replay": 2},
        )
        self.assertEqual(
            report["summary"]["actor_closed_loop_levels"],
            {"traffic_manager_reactive": 1, "replay": 2},
        )
        self.assertEqual(report["metrics"], [])
        self.assertIn("run_config", report["artifacts"])
        self.assertNotIn("reconstruction_package", report["artifacts"])

    def test_report_includes_reconstruction_package_artifact_when_enabled(self):
        from metrics.report import build_closed_loop_report

        package_path = "E:/code/NeuralSceneBridge/outputs/scene-test/reconstruction_package.json"
        report = build_closed_loop_report(
            _minimal_run_config(
                {
                    "enabled": True,
                    "package_path": package_path,
                }
            )
        )

        self.assertEqual(report["artifacts"]["reconstruction_package"], package_path)

    def test_report_summarizes_tick_metrics_when_provided(self):
        from metrics.report import build_closed_loop_report

        report = build_closed_loop_report(
            _minimal_run_config(),
            tick_metrics=[
                {
                    "t_sec": 0.0,
                    "collision": False,
                    "ttc": 4.5,
                    "route_progress": 0.1,
                    "hard_brake_count": 1,
                    "jerk": 2.0,
                },
                {
                    "t_sec": 0.1,
                    "collision": True,
                    "ttc": 2.0,
                    "route_progress": 0.3,
                    "hard_brake": True,
                    "max_jerk": 5.0,
                },
                {
                    "t_sec": 0.2,
                    "collision_count": 2,
                    "min_ttc": 1.5,
                    "route_progress": 0.2,
                    "jerk": -7.0,
                },
            ],
            status="completed",
        )

        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["summary"]["collision_count"], 3)
        self.assertEqual(report["summary"]["min_ttc"], 1.5)
        self.assertEqual(report["summary"]["route_progress"], 0.3)
        self.assertEqual(report["summary"]["hard_brake_count"], 2)
        self.assertEqual(report["summary"]["max_jerk"], 7.0)
        self.assertEqual(len(report["metrics"]), 3)

    def test_dry_run_cli_writes_report_without_carla(self):
        from runners.run_closed_loop import write_closed_loop_report

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            config_path = tmp_path / "carla_run_config.json"
            output_path = tmp_path / "closed_loop_report.json"
            config_path.write_text(
                json.dumps(_minimal_run_config(), ensure_ascii=False),
                encoding="utf-8",
            )

            written = write_closed_loop_report(config_path, output_path, dry_run=True)

            self.assertEqual(written, output_path)
            report = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(report["schema_version"], "closed_loop_report.mvp.v0")
            self.assertEqual(report["scenario_id"], "scene-test-001")
            self.assertEqual(report["status"], "not_run")
            self.assertEqual(report["summary"]["collision_count"], 0)


if __name__ == "__main__":
    unittest.main()
