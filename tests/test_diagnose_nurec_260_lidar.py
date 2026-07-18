import struct
import unittest

from runners.diagnose_nurec_260_lidar import (
    _invert_rigid,
    _matmul,
    _matrix_to_pose,
    _pose_to_matrix,
    _response_wire_layout,
)


def _varint(value):
    result = bytearray()
    while value > 0x7F:
        result.append((value & 0x7F) | 0x80)
        value >>= 7
    result.append(value)
    return bytes(result)


def _wire_varint(field, value):
    return _varint(field << 3) + _varint(value)


def _wire_bytes(field, value):
    return _varint((field << 3) | 2) + _varint(len(value)) + value


class _Response:
    point_xyzs = ()
    point_intensities = ()


class NuRecLidarDiagnosticTests(unittest.TestCase):
    def test_reports_buffered_wire_layout(self):
        xyz = struct.pack("<6f", 1, 2, 3, 4, 5, 6)
        intensity = struct.pack("<2f", 0.25, 0.75)
        body = _wire_varint(3, 2) + _wire_bytes(4, xyz) + _wire_bytes(5, intensity)

        result = _response_wire_layout(_Response(), body)

        self.assertEqual(result["buffered_num_points"], 2)
        self.assertEqual(result["point_xyzs_buffer_bytes"], 24)
        self.assertEqual(result["point_intensities_buffer_bytes"], 8)
        self.assertEqual([row["field_number"] for row in result["top_level_fields"]], [3, 4, 5])

    def test_pose_matrix_round_trip_and_rigid_inverse(self):
        pose = {
            "position_m": {"x": 10.0, "y": -2.0, "z": 1.5},
            "orientation_xyzw": {"x": 0.0, "y": 0.0, "z": 0.3826834324, "w": 0.9238795325},
        }
        matrix = _pose_to_matrix(pose)
        recovered = _matrix_to_pose(matrix)
        identity = _matmul(_invert_rigid(matrix), matrix)

        for axis in ("x", "y", "z"):
            self.assertAlmostEqual(recovered["position_m"][axis], pose["position_m"][axis])
        for row in range(4):
            for column in range(4):
                self.assertAlmostEqual(identity[row][column], 1.0 if row == column else 0.0)


if __name__ == "__main__":
    unittest.main()
