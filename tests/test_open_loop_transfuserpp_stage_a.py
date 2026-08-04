import hashlib
import json
import tempfile
import unittest
from pathlib import Path


def _fixture_ir() -> dict:
    trajectory = [
        {"t_sec": 0.0, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "speed_mps": 2.0},
        {"t_sec": 0.05, "x": 0.1, "y": 0.0, "z": 0.0, "yaw": 0.0, "speed_mps": 2.0},
        {"t_sec": 0.10, "x": 0.2, "y": 0.01, "z": 0.0, "yaw": 2.0, "speed_mps": 2.0},
    ]
    return {
        "schema_version": "scenario_ir.v1",
        "scenario_id": "scene-test-open-loop-tfpp",
        "source": {
            "scene_token": "scene-test-open-loop-tfpp",
            "scene_name": "scene-test-tfpp",
            "version": "v1.0-test",
        },
        "ego": {"reference_trajectory": trajectory},
        "actors": [],
    }


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class OpenLoopTransFuserPPStageATests(unittest.TestCase):
    def _inputs(self, root: Path) -> tuple[Path, Path, str, str]:
        ir = root / "scene_ir.json"
        xodr = root / "road.xodr"
        ir.write_text(json.dumps(_fixture_ir(), indent=2) + "\n", encoding="utf-8")
        xodr.write_text(
            "<OpenDRIVE><road name='test' id='1' length='1'/></OpenDRIVE>\n",
            encoding="utf-8",
        )
        return ir, xodr, _sha256(ir), _sha256(xodr)

    def test_missing_runtime_and_native_trace_are_blocked_not_faked(self):
        from runners.run_open_loop_transfuserpp_stage_a import (
            run_open_loop_transfuserpp_stage_a,
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            ir, xodr, ir_sha, xodr_sha = self._inputs(Path(tmpdir))
            report = run_open_loop_transfuserpp_stage_a(
                scenario_ir_path=ir,
                opendrive_path=xodr,
                expected_scenario_ir_sha256=ir_sha,
                expected_opendrive_sha256=xodr_sha,
                run_id="test-open-loop-tfpp-blocked",
            )

        self.assertEqual(report["execution_status"], "blocked")
        self.assertEqual(report["evidence_classification"], "open_loop_multimodal")
        self.assertIn("runtime_config_missing", report["blockers"])
        self.assertIn("native_stage_a_observation_trace_missing", report["blockers"])
        self.assertFalse(report["real_tfpp_checkpoint_loaded"])
        self.assertFalse(report["real_carla_stage_a_open_loop"])
        self.assertFalse(report["control_affects_next_ego_pose"])
        self.assertFalse(report["claims_m8"])
        self.assertFalse(report["claims_m9"])

    def test_waypoint_conversion_declares_model_right_axis(self):
        from runners.run_open_loop_transfuserpp_stage_a import _waypoints_to_world

        result = _waypoints_to_world(
            [[1.0, 2.0]],
            {"x": 10.0, "y": 20.0, "yaw": 0.0},
            horizon_spacing_sec=0.5,
        )

        self.assertEqual(result, [{"horizon_sec": 0.5, "x": 11.0, "y": 18.0}])

    def test_open_loop_config_mismatch_is_reported_as_blocker(self):
        from runners.run_open_loop_transfuserpp_stage_a import (
            run_open_loop_transfuserpp_stage_a,
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            ir, xodr, ir_sha, xodr_sha = self._inputs(root)
            config = root / "runtime.json"
            config.write_text(
                json.dumps(
                    {
                        "open_loop": {
                            "evidence_classification": "perception_eligible",
                            "scenario_ir_sha256": ir_sha,
                            "opendrive_sha256": xodr_sha,
                            "control_affects_next_ego_pose": False,
                            "claims_m8": False,
                            "claims_m9": False,
                            "sensor_source": "carla_stage_a_native_rgb_lidar",
                        },
                        "experiment": {"scenario_ir_sha256": "0" * 64},
                    }
                ),
                encoding="utf-8",
            )
            report = run_open_loop_transfuserpp_stage_a(
                scenario_ir_path=ir,
                opendrive_path=xodr,
                expected_scenario_ir_sha256=ir_sha,
                expected_opendrive_sha256=xodr_sha,
                runtime_config_path=config,
                run_id="test-open-loop-tfpp-config",
            )

        self.assertEqual(report["execution_status"], "blocked")
        self.assertIn(
            "runtime_config.open_loop.evidence_classification_mismatch",
            report["blockers"],
        )
        self.assertIn(
            "runtime_config.experiment.scenario_ir_sha256_mismatch",
            report["blockers"],
        )


if __name__ == "__main__":
    unittest.main()
