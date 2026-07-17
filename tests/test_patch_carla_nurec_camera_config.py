from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from tools.patch_carla_nurec_camera_config import (
    NEW_CALL,
    NEW_CLI,
    NEW_CONFIG_OPEN,
    NEW_SENSOR_BRANCH,
    NEW_SIGNATURE,
    OLD_CALL,
    OLD_CLI,
    OLD_CONFIG_OPEN,
    OLD_SENSOR_BRANCH,
    OLD_SIGNATURE,
    apply_patch,
)


class CameraConfigPatchTests(unittest.TestCase):
    def _target(self, text: str) -> tuple[tempfile.TemporaryDirectory[str], Path]:
        temporary = tempfile.TemporaryDirectory()
        target = Path(temporary.name) / "example.py"
        target.write_text(text, encoding="utf-8")
        return temporary, target

    def test_legacy_script_is_patched_idempotently(self) -> None:
        temporary, target = self._target(
            OLD_SIGNATURE + OLD_CONFIG_OPEN + OLD_SENSOR_BRANCH + OLD_CLI + OLD_CALL
        )
        with temporary:
            self.assertEqual(
                apply_patch(target),
                "patched:signature,config-path,recorded-camera,cli,call",
            )
            self.assertEqual(apply_patch(target), "already_patched")

    def test_grid_enhanced_recorded_camera_branch_is_already_patched(self) -> None:
        grid_branch = NEW_SENSOR_BRANCH.replace(
            "                    output_dir,\n                ),",
            "                    output_dir,\n"
            "                    pygame_dims=grid_size,\n"
            "                ),",
        )
        temporary, target = self._target(
            NEW_SIGNATURE + NEW_CONFIG_OPEN + grid_branch + NEW_CLI + NEW_CALL
        )
        with temporary:
            self.assertEqual(apply_patch(target), "already_patched")
            self.assertFalse(target.with_suffix(".py.pre-camera-config").exists())


if __name__ == "__main__":
    unittest.main()
