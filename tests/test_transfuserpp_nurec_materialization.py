import struct
import tempfile
import unittest
from pathlib import Path


class Response:
    def __init__(self):
        self.point_xyzs = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        self.point_intensities = [0.25, 0.75]


class TransFuserPPNuRecMaterializationTests(unittest.TestCase):
    def test_legacy_lidar_materializes_canonical_little_endian_xyzi(self):
        from adapters.nurec_260_client import _lidar_xyzi_bytes

        data = _lidar_xyzi_bytes(Response(), b"unused")
        self.assertEqual(len(data), 32)
        self.assertEqual(struct.unpack("<ffff", data[:16]), (1.0, 2.0, 3.0, 0.25))
        self.assertEqual(struct.unpack("<ffff", data[16:]), (4.0, 5.0, 6.0, 0.75))

    def test_materialized_payload_has_attempt_relative_container_reference(self):
        from adapters.nurec_260_client import NuRec260Client

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            attempt = root / "attempt-01"
            client = NuRec260Client.__new__(NuRec260Client)
            client._payload_output_dir = attempt / "algorithm_sensor_payloads"
            client._payload_reference_root = root.resolve()
            result = client._materialize_payload(
                {"frame_id": 7, "sensor": {"sensor_id": "camera_front"}},
                b"payload",
                suffix=".jpg",
            )
        self.assertEqual(
            result["relative_path"],
            "attempt-01/algorithm_sensor_payloads/frame_00000007/camera_front.jpg",
        )


if __name__ == "__main__":
    unittest.main()
