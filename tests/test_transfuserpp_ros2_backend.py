import json
import tempfile
import unittest
from pathlib import Path


class TransFuserPPRos2BackendTests(unittest.TestCase):
    def test_backend_exception_trace_cannot_claim_matched_control(self):
        from agents.transfuserpp_ros2_backend import TransFuserPPRos2Backend

        with tempfile.TemporaryDirectory() as directory:
            backend = TransFuserPPRos2Backend.__new__(TransFuserPPRos2Backend)
            backend._failure_count = 0
            backend._last_failure = None
            backend._failure_root = Path(directory) / "backend_failures"
            backend._node = None
            try:
                raise RuntimeError("synthetic model failure")
            except RuntimeError as exc:
                backend._record_failure(
                    {
                        "frame_id": 17,
                        "timestamp": 0.85,
                        "run_context": {
                            "run_id": "attempt-01",
                            "scene_id": "scene",
                            "case_id": "S0_original_replay",
                            "seed": 41,
                            "identity": {"artifact_sha256": "a" * 64},
                        },
                    },
                    "obs-000001",
                    exc,
                )
            failure_trace = backend._failure_root / "attempt-01.jsonl"
            row = json.loads(failure_trace.read_text(encoding="utf-8"))
        self.assertEqual(row["exception_type"], "RuntimeError")
        self.assertIn("synthetic model failure", row["error"])
        self.assertFalse(row["safe_stop_published_as_valid_control"])
        self.assertEqual(row["frame_id"], 17)
        self.assertEqual(row["run_id"], "attempt-01")
        self.assertEqual(backend._failure_count, 1)


if __name__ == "__main__":
    unittest.main()
