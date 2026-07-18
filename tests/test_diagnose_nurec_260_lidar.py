import struct
from pathlib import Path
import unittest

from runners.diagnose_nurec_260_lidar import (
    diagnose_lidar,
    _invert_rigid,
    _matmul,
    _matrix_to_pose,
    _pose_to_matrix,
    _run_rpc_case,
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

    def __init__(self, body=b""):
        self.body = body

    def SerializeToString(self):
        return self.body


class _Descriptor:
    full_name = "test.Request"


class _Request:
    DESCRIPTOR = _Descriptor()

    def SerializeToString(self):
        return b"request"


class _Client:
    def __init__(self, response):
        self.response = response
        self.scene_start_us = 1532402927598150
        self.runtime_scene_id = "scene-0061"

    def encode_lidar(self, payload):
        return {"wire_request": _Request()}

    def render_lidar(self, request):
        return self.response

    @staticmethod
    def response_bytes(response):
        return response.SerializeToString()

    @staticmethod
    def inspect_response(payload, response, body):
        raise ValueError("semantic LiDAR validation failed")


class NuRecLidarDiagnosticTests(unittest.TestCase):
    def test_diagnostic_requires_a_target_and_lidar_device(self):
        missing = Path("does-not-need-to-exist.json")
        with self.assertRaisesRegex(ValueError, "NRE target"):
            diagnose_lidar(
                missing,
                missing,
                missing,
                missing,
                targets=[],
                device_types=["PANDAR128"],
            )
        with self.assertRaisesRegex(ValueError, "LiDAR device type"):
            diagnose_lidar(
                missing,
                missing,
                missing,
                missing,
                targets=["127.0.0.1:46443"],
                device_types=[],
            )

    def test_reports_buffered_wire_layout(self):
        xyz = struct.pack("<6f", 1, 2, 3, 4, 5, 6)
        intensity = struct.pack("<2f", 0.25, 0.75)
        body = _wire_varint(3, 2) + _wire_bytes(4, xyz) + _wire_bytes(5, intensity)

        result = _response_wire_layout(_Response(), body)

        self.assertEqual(result["buffered_num_points"], 2)
        self.assertEqual(result["point_xyzs_buffer_bytes"], 24)
        self.assertEqual(result["point_intensities_buffer_bytes"], 8)
        self.assertEqual([row["field_number"] for row in result["top_level_fields"]], [3, 4, 5])
        self.assertEqual(
            [row["wire_type"] for row in result["top_level_fields"]],
            ["varint", "length_delimited", "length_delimited"],
        )

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

    def test_rpc_failure_keeps_received_payload_evidence(self):
        body = _wire_varint(3, 1) + _wire_bytes(4, struct.pack("<3f", 1, 2, 3))
        response = _Response(body)
        payload = {
            "modality": "lidar",
            "sensor": {
                "sensor_id": "lidar_top",
                "model": "PANDAR128",
                "pose_pair": {
                    "start": {
                        "position_m": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation_xyzw": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "end": {
                        "position_m": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation_xyzw": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                },
            },
            "dynamic_objects": [],
            "pose_interval_sec": {"start": 0.0, "end": 0.01},
        }

        result = _run_rpc_case(
            _Client(response),
            payload,
            case_name="malformed",
            sequence=1,
            device_type="PANDAR128",
            pose_source="test",
        )

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["response"]["serialized_bytes"], len(body))
        self.assertEqual(result["response"]["payload_sha256"], __import__("hashlib").sha256(body).hexdigest())
        self.assertEqual(result["response"]["wire_layout"]["buffered_num_points"], 1)


if __name__ == "__main__":
    unittest.main()
