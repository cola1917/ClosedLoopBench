from __future__ import annotations

import tempfile
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace


HASH = "a" * 64


class TransFuserPPRuntimeUnitTests(unittest.TestCase):
    def test_reset_clears_official_private_pid_windows(self) -> None:
        from agents.transfuserpp_runtime import TransFuserPPModelRuntime

        controller = SimpleNamespace(
            _window=deque([1.0, 2.0], maxlen=4),
            _saved_window=[3.0, 4.0],
        )
        runtime = TransFuserPPModelRuntime.__new__(TransFuserPPModelRuntime)
        runtime.net = SimpleNamespace(lateral_pid_controller=controller)
        runtime._last_frame_id = 17
        runtime.reset()
        self.assertEqual(list(controller._window), [])
        self.assertEqual(controller._saved_window, [])
        self.assertIsNone(runtime._last_frame_id)

    def test_run_activation_requires_full_formal_identity(self) -> None:
        from agents.transfuserpp_runtime import (
            TransFuserPPModelRuntime,
            TransFuserPPRuntimeError,
        )

        identity = {
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
        }
        with tempfile.TemporaryDirectory() as directory:
            runtime = TransFuserPPModelRuntime.__new__(TransFuserPPModelRuntime)
            runtime.experiment = {
                "scene_id": "cc8c0bf57f984915a77078b10eb33198",
                "case_id": "S0_original_replay",
                "seed": 41,
                **identity,
            }
            runtime.net = SimpleNamespace()
            runtime._active_run_id = None
            runtime.output_root = Path(directory)
            bad = {
                "run_context": {
                    "run_id": "attempt-01",
                    "scene_id": runtime.experiment["scene_id"],
                    "case_id": runtime.experiment["case_id"],
                    "seed": 41,
                    "identity": {**identity, "artifact_sha256": "b" * 64},
                }
            }
            with self.assertRaisesRegex(
                TransFuserPPRuntimeError, "artifact_sha256 mismatch"
            ):
                runtime._activate_run(bad)
            good = {"run_context": {**bad["run_context"], "identity": identity}}
            runtime._activate_run(good)
            self.assertEqual(runtime._active_run_id, "attempt-01")
            self.assertTrue((Path(directory) / "attempt-01").is_dir())

    def test_shared_output_reference_is_attempt_relative(self) -> None:
        from agents.transfuserpp_runtime import TransFuserPPModelRuntime

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "case" / "attempt" / "frame.npz"
            target.parent.mkdir(parents=True)
            target.write_bytes(b"npz")
            runtime = TransFuserPPModelRuntime.__new__(TransFuserPPModelRuntime)
            runtime.shared_data_root = root
            self.assertEqual(
                runtime._shared_relative_path(target),
                "case/attempt/frame.npz",
            )


if __name__ == "__main__":
    unittest.main()
