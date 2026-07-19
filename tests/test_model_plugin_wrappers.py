import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path


class ModelPluginWrapperTests(unittest.TestCase):
    def test_unbound_manifest_is_fail_closed_and_explicitly_not_real(self):
        from agents.model_plugin_wrappers import build_external_model_runtime_manifest

        manifest = build_external_model_runtime_manifest(
            "tcp", {"repo_path": "missing", "checkpoint_path": "missing.pth"}
        )
        self.assertEqual(manifest["execution_status"], "blocked")
        self.assertEqual(manifest["evidence_classification"], "remote_validation_required")
        self.assertFalse(manifest["real_checkpoint_loaded"])
        self.assertTrue(manifest["remote_gpu_validation_required"])
        self.assertIn("repo_sha256_missing", manifest["problems"])

    def test_checkpoint_hash_mismatch_is_rejected(self):
        from agents.model_plugin_wrappers import build_external_model_runtime_manifest

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            checkpoint = root / "model.pth"
            checkpoint.write_bytes(b"weights")
            manifest = build_external_model_runtime_manifest(
                "transfuser",
                {
                    "repo_path": str(root),
                    "checkpoint_path": str(checkpoint),
                    "repo_sha256": "repo-hash",
                    "checkpoint_sha256": "wrong",
                },
            )
        self.assertIn("checkpoint_sha256_mismatch", manifest["problems"])

    def test_fake_recorded_backends_normalize_common_boundary(self):
        from agents.model_plugin_wrappers import create_tcp_plugin, create_transfuser_plugin

        config = {
            "allow_test_backend": True,
            "recorded_controls": [
                {
                    "throttle": 0.1,
                    "steer": -0.2,
                    "brake": 0.0,
                    "hand_brake": False,
                    "reverse": False,
                }
            ],
        }
        base = {
            "frame_id": 7,
            "timestamp": 0.35,
            "rgb": {"rgb_front": "frame.png"},
            "lidar": "points.bin",
            "calibration": {},
            "ego_state": {"speed_mps": 2.0},
            "route": {"route_command": "LEFT", "target_point": [1.0, 2.0]},
        }
        for factory in (create_tcp_plugin, create_transfuser_plugin):
            plugin = factory(config)
            plugin.reset({})
            control = plugin.predict_control(base)
            self.assertEqual(control["source_frame_id"], 7)
            self.assertEqual(control["status"], "test_recorded_control")
            self.assertFalse(plugin.health_check()["real_checkpoint_loaded"])

    def test_transfuser_declares_lidar_while_tcp_does_not(self):
        from agents.model_plugin_wrappers import ExternalModelPluginWrapper

        self.assertFalse(ExternalModelPluginWrapper("tcp").capability["requires_lidar"])
        self.assertTrue(ExternalModelPluginWrapper("transfuser").capability["requires_lidar"])

    def test_common_executor_rejects_external_plugin_without_identity_hashes(self):
        from agents.model_plugin_wrappers import create_tcp_plugin
        from agents.plugin_contract import AlgorithmPluginExecutor, PluginContractError

        plugin = create_tcp_plugin({"allow_test_backend": True})
        with self.assertRaisesRegex(PluginContractError, "repo_sha256"):
            AlgorithmPluginExecutor(plugin, {"repo_path": "."}, already_initialized=True)

    def test_wrapper_refuses_false_real_checkpoint_claim(self):
        from agents.model_plugin_wrappers import create_tcp_plugin

        with self.assertRaisesRegex(ValueError, "cannot claim real_checkpoint_loaded"):
            create_tcp_plugin({"real_checkpoint_loaded": True})

    def test_manifest_cli_writes_remote_validation_boundary(self):
        from runners.build_external_model_plugin_manifest import main

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            config = root / "tcp.json"
            output = root / "manifest.json"
            config.write_text(
                json.dumps({"repo_path": "missing", "checkpoint_path": "missing.pth"}),
                encoding="utf-8",
            )
            with contextlib.redirect_stdout(io.StringIO()):
                exit_code = main(
                    [
                        "--algorithm",
                        "tcp",
                        "--config",
                        str(config),
                        "--output",
                        str(output),
                    ]
                )
            manifest = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(exit_code, 0)
        self.assertEqual(manifest["evidence_classification"], "remote_validation_required")
        self.assertFalse(manifest["real_checkpoint_loaded"])


if __name__ == "__main__":
    unittest.main()
