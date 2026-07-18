from pathlib import Path
import tempfile
import unittest

from tools.patch_carla_nurec_server_command import (
    NEW_COMMAND,
    NEW_IMPORT,
    OLD_COMMAND,
    OLD_IMPORT,
    PREVIOUS_COMMAND,
    apply_patch,
)


class PatchCarlaNuRecServerCommandTests(unittest.TestCase):
    def _apply(self, body: str) -> tuple[str, str]:
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "nurec_render_service.py"
            target.write_text(body, encoding="utf-8")
            result = apply_patch(target)
            patched = target.read_text(encoding="utf-8")
            self.assertEqual(apply_patch(target), "already_patched")
            self.assertTrue(
                target.with_suffix(".py.pre-closedloopbench").is_file()
            )
            return result, patched

    def test_patches_unmodified_release(self):
        result, patched = self._apply(OLD_IMPORT + OLD_COMMAND)
        self.assertEqual(result, "patched")
        self.assertIn(NEW_IMPORT, patched)
        self.assertIn(NEW_COMMAND, patched)
        self.assertIn("NUREC_SERVER_ARGS", patched)

    def test_upgrades_previous_closedloopbench_patch(self):
        result, patched = self._apply(NEW_IMPORT + PREVIOUS_COMMAND)
        self.assertEqual(result, "upgraded:server-args")
        self.assertIn(NEW_COMMAND, patched)
        self.assertNotIn(PREVIOUS_COMMAND, patched)


if __name__ == "__main__":
    unittest.main()
