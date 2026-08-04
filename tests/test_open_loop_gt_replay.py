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
        "scenario_id": "scene-test-open-loop",
        "source": {
            "scene_token": "scene-test-open-loop",
            "scene_name": "scene-test",
            "version": "v1.0-test",
        },
        "ego": {"reference_trajectory": trajectory},
        "actors": [
            {
                "actor_id": "actor-1",
                "reference_trajectory": [
                    {"t_sec": 0.0, "x": 3.0, "y": 1.0, "z": 0.0, "yaw": 0.0, "speed_mps": 1.0},
                    {"t_sec": 0.10, "x": 3.1, "y": 1.0, "z": 0.0, "yaw": 0.0, "speed_mps": 1.0},
                ],
            }
        ],
    }


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class OpenLoopGtReplayTests(unittest.TestCase):
    def _write_inputs(self, root: Path) -> tuple[Path, Path, str, str]:
        ir = root / "scene_ir.json"
        xodr = root / "road.xodr"
        ir.write_text(json.dumps(_fixture_ir(), indent=2) + "\n", encoding="utf-8")
        xodr.write_text("<OpenDRIVE><road name='test' id='1' length='1'/></OpenDRIVE>\n", encoding="utf-8")
        return ir, xodr, _sha256(ir), _sha256(xodr)

    def test_gt_pose_owns_next_pose_after_predict_control(self):
        from runners.run_open_loop_gt_replay import run_open_loop_gt_replay

        with tempfile.TemporaryDirectory() as tmpdir:
            ir, xodr, ir_sha, xodr_sha = self._write_inputs(Path(tmpdir))
            report = run_open_loop_gt_replay(
                scenario_ir_path=ir,
                opendrive_path=xodr,
                expected_scenario_ir_sha256=ir_sha,
                expected_opendrive_sha256=xodr_sha,
                plugin_config={"profile": "short"},
                run_id="test-open-loop",
            )

        self.assertEqual(report["execution_status"], "completed")
        self.assertEqual(report["evidence_classification"], "open_loop_multimodal")
        self.assertFalse(report["control_affects_next_ego_pose"])
        self.assertFalse(report["claims_m8"])
        self.assertFalse(report["claims_m9"])
        self.assertEqual(report["control_application"], "not_applied")
        self.assertEqual(report["pose_ownership"]["max_pose_error_m"], 0.0)
        for frame in report["frames"][:-1]:
            self.assertEqual(frame["ego_pose_before_control"], frame["ego_pose_after_control"])
            self.assertEqual(frame["next_ego_pose_expected"], frame["next_ego_pose_actual"])
            self.assertEqual(
                frame["next_ego_pose_source"],
                "scenario_ir_reference_trajectory",
            )

    def test_pin_mismatch_fails_closed(self):
        from runners.run_open_loop_gt_replay import OpenLoopReplayError, load_pinned_inputs

        with tempfile.TemporaryDirectory() as tmpdir:
            ir, xodr, _, xodr_sha = self._write_inputs(Path(tmpdir))
            with self.assertRaisesRegex(OpenLoopReplayError, "Scenario IR SHA-256 mismatch"):
                load_pinned_inputs(
                    ir,
                    xodr,
                    expected_scenario_ir_sha256="0" * 64,
                    expected_opendrive_sha256=xodr_sha,
                )


if __name__ == "__main__":
    unittest.main()
