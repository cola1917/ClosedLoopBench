from pathlib import Path
import tempfile
import unittest

from tools.patch_carla_nurec_actor_inventory import (
    BASE_CLI,
    BASE_EXCEPTION,
    BASE_FINALLY,
    BASE_STATE,
    BASE_TICK,
    PATCHED_CLI,
    PATCHED_EXCEPTION,
    PATCHED_FINALLY,
    PATCHED_STATE,
    PATCHED_TICK,
    apply_patch,
)


class PatchCarlaNuRecActorInventoryTests(unittest.TestCase):
    def test_patches_current_overlap_variant_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "example.py"
            target.write_text(
                "\n".join(
                    (BASE_CLI, BASE_STATE, BASE_TICK, BASE_EXCEPTION, BASE_FINALLY)
                ),
                encoding="utf-8",
            )
            result = apply_patch(target)
            patched = target.read_text(encoding="utf-8")
            self.assertEqual(result, "patched:cli,state,sampling,fail-closed,report")
            for marker in (
                PATCHED_CLI,
                PATCHED_STATE,
                PATCHED_TICK,
                PATCHED_EXCEPTION,
                PATCHED_FINALLY,
            ):
                self.assertIn(marker, patched)
            self.assertEqual(apply_patch(target), "already_patched")
            self.assertTrue(target.with_suffix(".py.pre-actor-inventory").is_file())


if __name__ == "__main__":
    unittest.main()
