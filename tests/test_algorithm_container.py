import contextlib
import io
import os
import tempfile
import unittest
import json
from pathlib import Path
from unittest.mock import patch


class AlgorithmContainerTests(unittest.TestCase):
    def _runtime(self, root: Path, source: str, module_name: str = "test_plugin") -> dict[str, str]:
        repo = root / "repo"
        checkpoint_dir = root / "checkpoints"
        shared = root / "sim-data"
        repo.mkdir()
        checkpoint_dir.mkdir()
        shared.mkdir()
        (repo / f"{module_name}.py").write_text(source, encoding="utf-8")
        checkpoint = checkpoint_dir / "model.pth"
        checkpoint.write_bytes(b"checkpoint")
        return {
            "ALGORITHM_PLUGIN": f"{module_name}:create_backend",
            "ALGORITHM_REPO_PATH": str(repo),
            "ALGORITHM_CHECKPOINT_PATH": str(checkpoint),
            "SIM_DATA_PATH": str(shared),
            "ALGORITHM_ID": "test-baseline",
        }

    def test_preflight_loads_mounted_model_plugin(self):
        from runners.run_algorithm_container import preflight

        source = """
class Backend:
    def predict_control(self, observation):
        return observation
    def health_check(self):
        return {"status": "ready"}
    def run(self):
        return None
def create_backend(config):
    assert config["checkpoint_path"].endswith("model.pth")
    return Backend()
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            config, backend = preflight(self._runtime(Path(tmpdir), source, "healthy_plugin"))
        self.assertEqual(config["algorithm_id"], "test-baseline")
        self.assertTrue(callable(backend.predict_control))

    def test_missing_checkpoint_fails_instead_of_starting_stub(self):
        from agents.algorithm_backend import AlgorithmBackendError
        from runners.run_algorithm_container import build_runtime_config

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            environment = self._runtime(root, "")
            Path(environment["ALGORITHM_CHECKPOINT_PATH"]).unlink()
            with self.assertRaisesRegex(AlgorithmBackendError, "checkpoint is not a file"):
                build_runtime_config(environment)

    def test_plugin_must_expose_predict_control(self):
        from agents.algorithm_backend import AlgorithmBackendError
        from runners.run_algorithm_container import preflight

        source = """
class Backend:
    def run(self):
        return None
def create_backend(config):
    return Backend()
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            with self.assertRaisesRegex(AlgorithmBackendError, "predict_control"):
                preflight(self._runtime(Path(tmpdir), source, "invalid_plugin"))

    def test_run_requires_real_plugin_lifecycle(self):
        from agents.algorithm_backend import AlgorithmBackendError, run_backend

        class PredictOnlyBackend:
            def predict_control(self, observation):
                return observation

        with self.assertRaisesRegex(AlgorithmBackendError, "blocking run"):
            run_backend(PredictOnlyBackend())

    def test_cli_preflight_returns_nonzero_for_missing_mounts(self):
        from runners.run_algorithm_container import main

        environment = {
            "ALGORITHM_PLUGIN": "missing:create_backend",
            "ALGORITHM_REPO_PATH": "Z:/missing/repo",
            "ALGORITHM_CHECKPOINT_PATH": "Z:/missing/model.pth",
            "SIM_DATA_PATH": "Z:/missing/sim-data",
        }
        stderr = io.StringIO()
        with patch.dict(os.environ, environment, clear=True), contextlib.redirect_stderr(stderr):
            result = main(["preflight", "--ready-file", "Z:/missing/ready"])
        self.assertEqual(result, 2)
        self.assertIn('"status": "failed"', stderr.getvalue())

    def test_compose_does_not_define_a_carla_service(self):
        compose = (Path(__file__).parents[1] / "docker" / "compose.algorithm.yml").read_text(
            encoding="utf-8"
        )
        self.assertIn("ego-algorithm:", compose)
        self.assertNotIn("carla-server:", compose)
        self.assertIn('network_mode: "${ALGORITHM_NETWORK_MODE:-host}"', compose)
        self.assertIn("E:/sim-data", compose)

    def test_health_refreshes_algorithm_heartbeat(self):
        from runners.run_algorithm_container import main

        with tempfile.TemporaryDirectory() as tmpdir:
            ready = Path(tmpdir) / "algorithm.ready.json"
            ready.write_text(
                json.dumps({"status": "ready", "algorithm_id": "tcp", "heartbeat_unix": 1.0}),
                encoding="utf-8",
            )
            with patch("runners.run_algorithm_container.time.time", return_value=123.0):
                result = main(["health", "--ready-file", str(ready)])
            self.assertEqual(result, 0)
            self.assertEqual(
                json.loads(ready.read_text(encoding="utf-8"))["heartbeat_unix"],
                123.0,
            )


if __name__ == "__main__":
    unittest.main()
