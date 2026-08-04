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
        "scenario_id": "scene-test-open-loop-ros",
        "source": {
            "scene_token": "scene-test-open-loop-ros",
            "scene_name": "scene-test-ros",
            "version": "v1.0-test",
        },
        "ego": {"reference_trajectory": trajectory},
        "actors": [],
    }


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class OpenLoopRosSmokeTests(unittest.TestCase):
    def test_pure_pursuit_crosses_bridge_with_frame_matched_control(self):
        from runners.run_open_loop_ros_smoke import run_open_loop_ros_smoke

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            ir = root / "scene_ir.json"
            xodr = root / "road.xodr"
            ir.write_text(json.dumps(_fixture_ir(), indent=2) + "\n", encoding="utf-8")
            xodr.write_text(
                "<OpenDRIVE><road name='test' id='1' length='1'/></OpenDRIVE>\n",
                encoding="utf-8",
            )
            report = run_open_loop_ros_smoke(
                scenario_ir_path=ir,
                opendrive_path=xodr,
                expected_scenario_ir_sha256=_sha256(ir),
                expected_opendrive_sha256=_sha256(xodr),
                plugin_config={"profile": "short"},
                run_id="test-open-loop-ros",
            )

        self.assertEqual(report["schema_version"], "open_loop_ros_boundary_report.v1")
        self.assertEqual(report["execution_status"], "completed")
        self.assertEqual(report["evidence_classification"], "open_loop_multimodal")
        self.assertFalse(report["real_ros2_transport"])
        self.assertFalse(report["control_affects_next_ego_pose"])
        self.assertFalse(report["claims_m8"])
        self.assertFalse(report["claims_m9"])
        self.assertEqual(report["frame_sync"]["matched_frame_count"], 3)
        self.assertEqual(report["frame_sync"]["scored_frame_mismatch_count"], 0)
        self.assertEqual(report["pose_ownership"]["max_pose_error_m"], 0.0)
        for frame in report["frames"]:
            self.assertEqual(frame["observation_frame_id"], frame["control_source_frame_id"])
            self.assertEqual(frame["bridge_status"], "control")

    def test_report_validator_rejects_control_owned_pose(self):
        from runners.run_open_loop_ros_smoke import (
            OpenLoopRosSmokeError,
            validate_open_loop_ros_report,
        )

        report = {
            "schema_version": "open_loop_ros_boundary_report.v1",
            "scene_id": "scene",
            "scenario_id": "scenario",
            "execution_status": "completed",
            "evidence_classification": "open_loop_multimodal",
            "real_ros2_transport": False,
            "ego_pose_source": "scenario_ir_reference_trajectory",
            "control_affects_next_ego_pose": True,
            "claims_m8": False,
            "claims_m9": False,
            "frame_sync": {"scored_frame_mismatch_count": 0},
            "pose_ownership": {"control_applied": False},
        }
        with self.assertRaisesRegex(OpenLoopRosSmokeError, "control_affects_next_ego_pose"):
            validate_open_loop_ros_report(report)


if __name__ == "__main__":
    unittest.main()
