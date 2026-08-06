import hashlib
import tempfile
import unittest
from pathlib import Path


class OpenLoopTransFuserPPStageBPathTests(unittest.TestCase):
    def test_container_payload_ref_keeps_capture_directory_mount_prefix(self):
        from runners.capture_open_loop_transfuserpp_stage_b import _container_payload_ref

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output_dir = root / "m7-seed-43-capture"
            payload = output_dir / "payloads" / "frame_00000000" / "camera_front.jpg"
            payload.parent.mkdir(parents=True)
            payload.write_bytes(b"payload")
            reference = {
                "path": str(payload),
                "relative_path": "payloads/frame_00000000/camera_front.jpg",
                "sha256": hashlib.sha256(b"payload").hexdigest(),
                "byte_count": len(b"payload"),
                "encoding": "jpeg",
            }

            result = _container_payload_ref(reference, output_dir, "/sim-data")

        self.assertEqual(
            result["path"],
            "/sim-data/m7-seed-43-capture/payloads/frame_00000000/camera_front.jpg",
        )
        self.assertEqual(result["relative_path"], "payloads/frame_00000000/camera_front.jpg")


if __name__ == "__main__":
    unittest.main()
