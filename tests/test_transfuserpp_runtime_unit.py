from __future__ import annotations

import tempfile
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np


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

    def test_physical_camera_resize_requires_1600x900_source(self) -> None:
        from agents.transfuserpp_contract import camera_adaptation_contract
        from agents.transfuserpp_runtime import (
            TransFuserPPModelRuntime,
            TransFuserPPRuntimeError,
        )

        calls = []

        def resize(image, size, *, interpolation):
            calls.append((image.shape, size, interpolation))
            return np.zeros((size[1], size[0], image.shape[2]), dtype=image.dtype)

        runtime = TransFuserPPModelRuntime.__new__(TransFuserPPModelRuntime)
        runtime.cv2 = SimpleNamespace(INTER_LINEAR=7, resize=resize)
        output = runtime._adapt_physical_camera_image(
            np.zeros((900, 1600, 3), dtype=np.uint8), camera_adaptation_contract()
        )
        self.assertEqual(output.shape, (450, 800, 3))
        self.assertEqual(calls, [((900, 1600, 3), (800, 450), 7)])
        with self.assertRaisesRegex(TransFuserPPRuntimeError, "1600x900"):
            runtime._adapt_physical_camera_image(
                np.zeros((450, 800, 3), dtype=np.uint8),
                camera_adaptation_contract(),
            )

    def test_warmup_does_not_write_intermediate_or_consume_frame(self) -> None:
        from agents.transfuserpp_runtime import TransFuserPPModelRuntime

        class Cuda:
            def synchronize(self, _device):
                pass

        runtime = TransFuserPPModelRuntime.__new__(TransFuserPPModelRuntime)
        runtime._closed = False
        runtime.max_sync_error_ms = 1.0
        runtime.device = "cuda:0"
        runtime.torch = SimpleNamespace(cuda=Cuda())
        runtime._successful_inference_count = 0
        runtime._last_frame_id = None
        runtime._activate_run = mock.Mock()
        runtime.reset = mock.Mock()
        runtime._preprocess = mock.Mock(return_value=({}, {}))
        runtime._forward = mock.Mock(return_value=tuple(range(10)))

        with mock.patch(
            "agents.transfuserpp_runtime.validate_observation",
            return_value={"frame_id": 4},
        ):
            result = runtime.warmup({"frame_id": 4}, iterations=2)

        self.assertEqual(result["intermediate_count"], 0)
        self.assertTrue(result["formal_frame_excluded"])
        self.assertEqual(result["frame_id"], 4)
        self.assertEqual(runtime._successful_inference_count, 2)
        self.assertIsNone(runtime._last_frame_id)
        self.assertEqual(runtime._preprocess.call_count, 2)
        self.assertEqual(runtime._forward.call_count, 2)


if __name__ == "__main__":
    unittest.main()
