from pathlib import Path
import tempfile
import unittest

from tools.patch_carla_nurec_ego_warmup import (
    BASE_EGO_GUARD,
    PATCHED_EGO_GUARD,
    apply_patch,
)


class PatchCarlaNuRecEgoWarmupTests(unittest.TestCase):
    def test_disables_ego_physics_immediately_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "nurec_integration.py"
            target.write_text(BASE_EGO_GUARD, encoding="utf-8")
            self.assertEqual(
                apply_patch(target), "patched:immediate-ego-physics-disable"
            )
            self.assertIn(PATCHED_EGO_GUARD, target.read_text(encoding="utf-8"))
            self.assertEqual(apply_patch(target), "already_patched")
            self.assertTrue(target.with_suffix(".py.pre-ego-warmup").is_file())


if __name__ == "__main__":
    unittest.main()
