from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from pathlib import Path

from tools.scene0061_sync import Scene0061SyncError, create_bundle, git_sha, verify_bundle


def _git(repo: Path, *args: str) -> None:
    subprocess.run(("git", *args), cwd=repo, check=True, capture_output=True, text=True)


class Scene0061SyncTests(unittest.TestCase):
    def _repository(self) -> tuple[tempfile.TemporaryDirectory[str], Path, str, str]:
        directory = tempfile.TemporaryDirectory()
        repo = Path(directory.name)
        _git(repo, "init")
        _git(repo, "config", "user.email", "tests@example.invalid")
        _git(repo, "config", "user.name", "ClosedLoopBench tests")
        (repo / "state.txt").write_text("base\n", encoding="utf-8")
        _git(repo, "add", "state.txt")
        _git(repo, "commit", "-m", "base")
        base = git_sha(repo)
        (repo / "state.txt").write_text("head\n", encoding="utf-8")
        _git(repo, "commit", "-am", "head")
        return directory, repo, base, git_sha(repo)

    def test_create_bundle_writes_hash_bound_manifest(self) -> None:
        directory, repo, base, head = self._repository()
        self.addCleanup(directory.cleanup)
        bundle = repo / "scene0061.bundle"
        result = create_bundle(repo, base, bundle)
        self.assertTrue(result["verified"])
        self.assertEqual(result["base_commit"], base)
        self.assertEqual(result["head_commit"], head)
        self.assertTrue(any(row["commit"] == head for row in result["heads"]))
        manifest = Path(result["manifest_path"])
        self.assertTrue(manifest.is_file())
        self.assertEqual(json.loads(manifest.read_text(encoding="utf-8"))["bundle_sha256"], result["bundle_sha256"])
        self.assertEqual(verify_bundle(repo, bundle, expected_head=head)["heads"], result["heads"])

    def test_create_bundle_refuses_to_overwrite_existing_artifacts(self) -> None:
        directory, repo, base, _ = self._repository()
        self.addCleanup(directory.cleanup)
        bundle = repo / "scene0061.bundle"
        bundle.write_bytes(b"do not overwrite")
        with self.assertRaisesRegex(Scene0061SyncError, "refusing to overwrite"):
            create_bundle(repo, base, bundle)

    def test_verify_rejects_an_unadvertised_expected_head(self) -> None:
        directory, repo, base, _ = self._repository()
        self.addCleanup(directory.cleanup)
        bundle = repo / "scene0061.bundle"
        create_bundle(repo, base, bundle)
        with self.assertRaisesRegex(Scene0061SyncError, "does not advertise"):
            verify_bundle(repo, bundle, expected_head="f" * 40)


if __name__ == "__main__":
    unittest.main()
