import json
import tempfile
import unittest
from pathlib import Path


def _smoke(status="passed"):
    return {
        "schema_version": "nurec_reconstruction_smoke.v1",
        "status": status,
        "scene_id": "scene-0061",
        "editable_quality_windows": {
            "required": True,
            "status": "passed" if status == "passed" else "failed",
        },
    }


def _summary(root: Path, *, failed=False, mismatched=False):
    artifacts = {}
    for stream in ("collision", "lane", "visibility", "lidar_world"):
        path = root / f"{stream}.jsonl"
        frame_id = 11 if mismatched and stream == "lidar_world" else 10
        path.write_text(
            json.dumps({"frame_id": frame_id, "status": "failed" if failed else "passed"}) + "\n",
            encoding="utf-8",
        )
        artifacts[stream] = {
            "path": str(path),
            "tick_count": 1,
            "failed_tick_count": 1 if failed else 0,
        }
    return {
        "schema_version": "scene_safety_audit_summary.v1",
        "status": "failed" if failed else "passed",
        "artifacts": artifacts,
    }


class M8PromotionTests(unittest.TestCase):
    def test_requires_all_four_streams_and_artifact(self):
        from adapters.m8_promotion import evaluate_m8_promotion

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "last.usdz"
            artifact.write_bytes(b"candidate")
            report = evaluate_m8_promotion(
                _smoke(), _summary(root), artifact_path=artifact, scene_id="scene-0061"
            )
            self.assertEqual(report["status"], "passed")
            self.assertTrue(report["formal_reconstruction_allowed"])
            self.assertEqual(report["artifact"]["size_bytes"], 9)

    def test_failed_lidar_stream_blocks_promotion(self):
        from adapters.m8_promotion import evaluate_m8_promotion

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "last.usdz"
            artifact.write_bytes(b"candidate")
            report = evaluate_m8_promotion(
                _smoke(), _summary(root, failed=True), artifact_path=artifact
            )
            self.assertEqual(report["status"], "failed")
            self.assertIn("m8_four_stream_summary_failed", report["issues"])
            self.assertFalse(report["formal_reconstruction_allowed"])

    def test_old_smoke_without_quality_window_is_not_promotable(self):
        from adapters.m8_promotion import evaluate_m8_promotion

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "last.usdz"
            artifact.write_bytes(b"candidate")
            smoke = _smoke()
            smoke["editable_quality_windows"] = {"status": "not_provided", "required": False}
            report = evaluate_m8_promotion(smoke, _summary(root), artifact_path=artifact)
            self.assertEqual(report["status"], "failed")
            self.assertIn("editable_quality_window_not_required", report["issues"])

    def test_mismatched_stream_frames_block_promotion(self):
        from adapters.m8_promotion import evaluate_m8_promotion

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "last.usdz"
            artifact.write_bytes(b"candidate")
            report = evaluate_m8_promotion(
                _smoke(), _summary(root, mismatched=True), artifact_path=artifact
            )
            self.assertEqual(report["status"], "failed")
            self.assertIn("m8_stream_frame_sets_mismatch", report["issues"])


if __name__ == "__main__":
    unittest.main()
