import tempfile
import unittest
import contextlib
import io
import subprocess
from pathlib import Path


HASH = "a" * 64
SENSOR_TO_EGO = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 2.5,
    0.0, 0.0, 0.0, 1.0,
]


def _calibration():
    return {
        "camera_sensor_id": "camera_front",
        "camera_width": 800,
        "camera_height": 450,
        "camera_sensor_to_ego": list(SENSOR_TO_EGO),
        "camera_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
        "camera_adaptation": "center_crop_800x450_to_800x400_then_resize_to_model_config",
        "lidar_sensor_id": "lidar_top",
        "lidar_sensor_to_ego": list(SENSOR_TO_EGO),
    }


def _observation():
    return {
        "frame_id": 7,
        "timestamp": 0.35,
        "rgb": {"camera_front": {
            "path": "front.jpg",
            "sha256": HASH,
            "byte_count": 1,
            "encoding": "jpeg",
            "coordinate_frame": "camera_optical",
        }},
        "lidar": {
            "path": "lidar.bin",
            "sha256": HASH,
            "byte_count": 16,
            "encoding": "float32_xyzi_little_endian",
            "coordinate_frame": "sensor_local",
            "axis_convention": "carla_sensor",
            "sensor_to_ego": list(SENSOR_TO_EGO),
        },
        "calibration": _calibration(),
        "ego_state": {"speed_mps": 3.0, "pose": {"x": 1.0, "y": 2.0, "yaw": 5.0}},
        "route": {
            "route_command": "LANE_FOLLOW",
            "target_point_ego_m": [8.0, 0.5],
            "target_point_coordinate_frame": "carla_ego",
        },
        "synchronization": {
            "frame_id": 7,
            "error_ms": 0.2,
            "dynamic_object_sha256": HASH,
        },
        "run_context": {
            "run_id": "scene0061-attempt-01",
            "scene_id": "cc8c0bf57f984915a77078b10eb33198",
            "case_id": "S0_original_replay",
            "seed": 41,
            "identity": {
                name: HASH
                for name in (
                    "artifact_sha256",
                    "scene_package_sha256",
                    "scenario_ir_sha256",
                    "immutable_matrix_sha256",
                    "source_run_config_sha256",
                    "variant_config_sha256",
                    "run_config_sha256",
                )
            },
        },
    }


class TransFuserPPContractTests(unittest.TestCase):
    def test_capability_matches_official_leaderboard2_sensor_boundary(self):
        from agents.transfuserpp_contract import capability

        value = capability()
        self.assertEqual(value["required_rgb_cameras"], ["camera_front"])
        self.assertTrue(value["requires_lidar"])
        self.assertFalse(value["full_3d_occupancy_output"])

    def test_observation_requires_verified_lidar_axis_and_same_frame(self):
        from agents.transfuserpp_contract import (
            TransFuserPPContractError,
            validate_observation,
        )

        validate_observation(_observation())
        invalid = _observation()
        invalid["lidar"]["axis_convention"] = "unknown"
        with self.assertRaisesRegex(TransFuserPPContractError, "axis_convention"):
            validate_observation(invalid)
        invalid = _observation()
        invalid["synchronization"]["frame_id"] = 8
        with self.assertRaisesRegex(TransFuserPPContractError, "frame_mismatch"):
            validate_observation(invalid)

    def test_observation_rejects_camera_or_lidar_calibration_drift(self):
        from agents.transfuserpp_contract import (
            TransFuserPPContractError,
            validate_observation,
        )

        invalid = _observation()
        invalid["calibration"]["camera_width"] = 640
        with self.assertRaisesRegex(TransFuserPPContractError, "800x450"):
            validate_observation(invalid)
        invalid = _observation()
        invalid["calibration"]["lidar_sensor_to_ego"][3] = 1.0
        with self.assertRaisesRegex(TransFuserPPContractError, "mismatch"):
            validate_observation(invalid)

    def test_unbound_manifest_is_fail_closed(self):
        from agents.transfuserpp_contract import build_runtime_manifest

        manifest = build_runtime_manifest(
            {
                "repo_path": "missing",
                "checkpoint_path": "missing.pth",
                "model_config_path": "missing.json",
                "intermediate_output_dir": "output",
            }
        )
        self.assertEqual(manifest["execution_status"], "blocked")
        self.assertFalse(manifest["real_checkpoint_loaded"])
        self.assertIn("repo_revision_invalid", manifest["problems"])
        self.assertIn("upstream_reference_invalid", manifest["problems"])

    def test_formal_camera_adaptation_window_is_deterministic(self):
        from agents.transfuserpp_contract import camera_center_crop_window

        self.assertEqual(camera_center_crop_window(800, 450, 1024, 512), [0, 25, 800, 425])

    def test_carla_navigation_snapshot_hash_is_path_independent(self):
        from agents.transfuserpp_contract import directory_snapshot_sha256

        with tempfile.TemporaryDirectory() as left_dir, tempfile.TemporaryDirectory() as right_dir:
            for root_value in (left_dir, right_dir):
                root = Path(root_value)
                (root / "navigation").mkdir()
                (root / "navigation" / "__init__.py").write_text("", encoding="utf-8")
                (root / "navigation" / "global_route_planner.py").write_text(
                    "VALUE = 1\n", encoding="utf-8"
                )
            self.assertEqual(
                directory_snapshot_sha256(left_dir),
                directory_snapshot_sha256(right_dir),
            )

    def test_upstream_origin_and_clean_worktree_are_verified(self):
        from agents.transfuserpp_contract import (
            UPSTREAM_REFERENCE,
            _git_is_ancestor,
            _git_origin,
            _git_reference_commit,
            _git_worktree_clean,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            subprocess.run(["git", "init", str(root)], check=True, capture_output=True)
            subprocess.run(
                ["git", "-C", str(root), "remote", "add", "origin",
                 "git@github.com:autonomousvision/carla_garage.git"],
                check=True,
                capture_output=True,
            )
            self.assertEqual(
                _git_origin(root),
                "https://github.com/autonomousvision/carla_garage",
            )
            self.assertTrue(_git_worktree_clean(root))
            subprocess.run(
                ["git", "-C", str(root), "config", "user.email", "test@example.com"],
                check=True,
            )
            subprocess.run(
                ["git", "-C", str(root), "config", "user.name", "Test"], check=True
            )
            (root / "tracked.py").write_text("VALUE = 1\n", encoding="utf-8")
            subprocess.run(["git", "-C", str(root), "add", "tracked.py"], check=True)
            subprocess.run(["git", "-C", str(root), "commit", "-m", "base"], check=True, capture_output=True)
            base = subprocess.run(
                ["git", "-C", str(root), "rev-parse", "HEAD"],
                check=True, capture_output=True, text=True,
            ).stdout.strip()
            subprocess.run(
                ["git", "-C", str(root), "update-ref", UPSTREAM_REFERENCE, base], check=True
            )
            resolved = _git_reference_commit(root, UPSTREAM_REFERENCE)
            self.assertEqual(resolved, base)
            self.assertTrue(_git_is_ancestor(root, base, resolved))
            (root / "tracked.py").write_text("VALUE = 2\n", encoding="utf-8")
            subprocess.run(["git", "-C", str(root), "add", "tracked.py"], check=True)
            subprocess.run(
                ["git", "-C", str(root), "commit", "-m", "not-on-ref"],
                check=True,
                capture_output=True,
            )
            head = subprocess.run(
                ["git", "-C", str(root), "rev-parse", "HEAD"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
            self.assertFalse(_git_is_ancestor(root, head, resolved))
            (root / "untracked.py").write_text("VALUE = 1\n", encoding="utf-8")
            self.assertFalse(_git_worktree_clean(root))

    def test_manifest_cli_writes_blocked_template_without_claiming_inference(self):
        import json
        from runners.build_transfuserpp_runtime_manifest import main

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "config.json"
            output = root / "manifest.json"
            config.write_text(
                json.dumps(
                    {
                        "repo_path": "missing",
                        "checkpoint_path": "missing",
                        "model_config_path": "missing",
                        "intermediate_output_dir": str(root / "out"),
                    }
                ),
                encoding="utf-8",
            )
            with contextlib.redirect_stdout(io.StringIO()):
                code = main(["--config", str(config), "--output", str(output)])
            result = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(code, 0)
        self.assertEqual(result["evidence_classification"], "remote_validation_required")
        self.assertFalse(result["real_checkpoint_loaded"])


if __name__ == "__main__":
    unittest.main()
