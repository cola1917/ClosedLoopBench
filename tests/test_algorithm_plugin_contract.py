import tempfile
import unittest
from pathlib import Path


class AlgorithmPluginContractTests(unittest.TestCase):
    def _config(self, **extra):
        return {"repo_path": str(Path(__file__).resolve().parents[1]), **extra}

    def _observation(self, frame_id=1):
        return {
            "frame_id": frame_id,
            "timestamp": frame_id * 0.05,
            "rgb": {},
            "lidar": None,
            "ego_state": {
                "speed_mps": 2.0,
                "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            },
            "route": {
                "route_waypoints": [[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]],
                "route_command": "LANE_FOLLOW",
                "target_point": [10.0, 0.0],
            },
            "sensor_validity": {"lidar": True},
            "synchronization": {"frame_id": frame_id},
        }

    def test_existing_loader_loads_reference_plugin_and_contract_executes(self):
        from agents.algorithm_backend import load_backend
        from agents.plugin_contract import AlgorithmPluginExecutor

        config = self._config(profile="short")
        backend = load_backend("agents.reference_pure_pursuit:create_plugin", config)
        executor = AlgorithmPluginExecutor(
            backend,
            config,
            already_initialized=True,
            evidence_classification="offline_conformance",
        )
        executor.initialize()
        executor.reset({"scene_id": "scene-0061"})
        result = executor.predict(self._observation())
        executor.close()

        self.assertEqual(result["execution_status"], "control")
        self.assertEqual(result["evidence_classification"], "offline_conformance")
        self.assertEqual(result["control"]["source_frame_id"], 1)
        self.assertFalse(backend.capability["is_perception_algorithm"])
        self.assertEqual(result["plugin_identity"]["checkpoint_sha256"], "not_applicable")
        self.assertEqual(len(result["plugin_identity"]["config_sha256"]), 64)
        self.assertEqual(len(result["plugin_identity"]["repo_sha256"]), 64)

    def test_stale_and_synchronization_mismatch_fail_safe(self):
        from agents.algorithm_backend import load_backend
        from agents.plugin_contract import AlgorithmPluginExecutor, SAFE_STOP_CONTROL

        config = self._config()
        backend = load_backend("agents.reference_pure_pursuit:create_plugin", config)
        executor = AlgorithmPluginExecutor(
            backend,
            config,
            already_initialized=True,
            evidence_classification="offline_conformance",
        )
        executor.initialize()
        executor.reset({})
        self.assertEqual(executor.predict(self._observation(2))["execution_status"], "control")
        stale = executor.predict(self._observation(2))
        executor.reset({})
        mismatch_observation = self._observation(3)
        mismatch_observation["synchronization"]["frame_id"] = 4
        mismatch = executor.predict(mismatch_observation)
        executor.close()

        self.assertEqual(stale["control"]["reason"], "stale_frame")
        self.assertEqual(mismatch["control"]["reason"], "frame_mismatch")
        for result in (stale, mismatch):
            for field, expected in SAFE_STOP_CONTROL.items():
                self.assertEqual(result["control"][field], expected)

    def test_file_hash_identity_is_exact(self):
        from agents.plugin_contract import file_sha256

        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "checkpoint.bin"
            path.write_bytes(b"checkpoint")
            self.assertEqual(
                file_sha256(path),
                "47320987f9a49d5b00119b960f247a956773f57543982b8bfcb6da5bb3afd9ef",
            )

    def test_repo_identity_is_path_independent_when_revision_is_frozen(self):
        from agents.plugin_contract import build_plugin_identity
        from agents.reference_pure_pursuit import ReferencePurePursuitPlugin

        capability = ReferencePurePursuitPlugin().capability
        left = build_plugin_identity(
            {"repo_path": "C:/checkout-a", "repo_revision": "frozen-revision"},
            capability,
        )
        right = build_plugin_identity(
            {"repo_path": "D:/checkout-b", "repo_revision": "frozen-revision"},
            capability,
        )

        self.assertEqual(left["repo_sha256"], right["repo_sha256"])
        self.assertNotEqual(left["config_sha256"], right["config_sha256"])


if __name__ == "__main__":
    unittest.main()
