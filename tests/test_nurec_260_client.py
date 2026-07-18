import struct
import unittest
from types import SimpleNamespace

from tests.test_actor_binding import VEHICLE_TRACK
from tests.test_nurec_multimodal import _frame


class _Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class _Response:
    def __init__(self, wire_bytes, **kwargs):
        self._wire_bytes = wire_bytes
        self.__dict__.update(kwargs)

    def SerializeToString(self):
        return self._wire_bytes


class _Stub:
    def __init__(self, *, rgb_width=1600, rgb_height=900):
        self.calls = []
        self.rgb_width = rgb_width
        self.rgb_height = rgb_height

    def render_rgb(self, request, *, timeout):
        self.calls.append(("rgb", request, timeout))
        return _Response(
            b"rgb-protobuf",
            image_bytes=_jpeg(self.rgb_width, self.rgb_height),
        )

    def render_lidar(self, request, *, timeout):
        self.calls.append(("lidar", request, timeout))
        return _Response(
            b"lidar-protobuf",
            point_xyzs=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            point_intensities=[0.25, 0.75],
        )


class _BufferedLidarStub(_Stub):
    def __init__(self, *, point_count=2, xyz_bytes=None, intensity_bytes=None):
        super().__init__()
        self.point_count = point_count
        self.xyz_bytes = (
            xyz_bytes
            if xyz_bytes is not None
            else struct.pack("<6f", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        )
        self.intensity_bytes = (
            intensity_bytes
            if intensity_bytes is not None
            else struct.pack("<2f", 0.25, 0.75)
        )

    def render_lidar(self, request, *, timeout):
        self.calls.append(("lidar", request, timeout))
        body = (
            _wire_varint(3, self.point_count)
            + _wire_bytes(4, self.xyz_bytes)
            + _wire_bytes(5, self.intensity_bytes)
        )
        return _Response(body, point_xyzs=[], point_intensities=[])


class _InventoryStub(_Stub):
    def get_version(self, request, *, timeout):
        return SimpleNamespace(
            version_id="26.04",
            git_hash="abc123",
            grpc_api_version=SimpleNamespace(major=1, minor=2, patch=3),
        )

    def get_available_scenes(self, request, *, timeout):
        return SimpleNamespace(scene_ids=["scene-0062", "scene-0061"])

    def get_available_cameras(self, request, *, timeout):
        return SimpleNamespace(
            available_cameras=[
                SimpleNamespace(
                    logical_id="camera_front",
                    trajectory_idx=0,
                    intrinsics=SimpleNamespace(resolution_w=1600, resolution_h=900),
                )
            ]
        )


def _protobuf_module():
    return SimpleNamespace(
        RGBRenderRequest=_Message,
        LidarRenderRequest=_Message,
        LidarSpec=_Message,
        PosePair=_Message,
        DynamicObject=_Message,
        AvailableCamerasRequest=_Message,
        JPEG=2,
        PANDAR128=0,
        AT128=1,
    )


def _jpeg(width, height):
    # Minimal SOF0 segment; the adapter only parses dimensions from the header.
    return (
        b"\xff\xd8\xff\xc0\x00\x0b\x08"
        + int(height).to_bytes(2, "big")
        + int(width).to_bytes(2, "big")
        + b"\x01\x01\x11\x00\xff\xd9"
    )


def _varint(value):
    result = bytearray()
    while value > 0x7F:
        result.append((value & 0x7F) | 0x80)
        value >>= 7
    result.append(value)
    return bytes(result)


def _wire_varint(field_number, value):
    return _varint(field_number << 3) + _varint(value)


def _wire_bytes(field_number, value):
    return _varint((field_number << 3) | 2) + _varint(len(value)) + value


def _wire_fixed64(field_number, value):
    return _varint((field_number << 3) | 1) + value


class NuRec260ClientTests(unittest.TestCase):
    def _client(self, stub=None):
        from adapters.nurec_260_client import NuRec260Client

        return NuRec260Client(
            target="127.0.0.1:46435",
            runtime_scene_id="scene-0061",
            scene_start_us=1_000_000,
            timeout_sec=12.0,
            protobuf_module=_protobuf_module(),
            common_protobuf_module=SimpleNamespace(Empty=_Message),
            stub=stub or _Stub(),
            camera_specs={
                "camera_front": SimpleNamespace(
                    resolution_w=1600,
                    resolution_h=900,
                )
            },
        )

    def test_dispatches_real_260_shapes_with_one_time_window_and_actor_pose(self):
        stub = _Stub()
        client = self._client(stub)
        evidence = client.dispatch_frame(_frame())

        self.assertEqual(evidence["status"], "passed")
        self.assertEqual(evidence["dispatch"]["runtime_scene_id"], "scene-0061")
        self.assertEqual(evidence["dispatch"]["canonical_scene_id"], evidence["scene_id"])
        self.assertEqual(evidence["dispatch"]["nre_api"], "SensorsimService/26.04")
        records = {item["modality"]: item for item in evidence["records"]}
        self.assertEqual(
            records["rgb"]["response_metadata"],
            {"width": 1600, "height": 900, "encoding": "jpeg"},
        )
        self.assertEqual(
            records["lidar"]["response_metadata"],
            {"point_count": 2, "encoding": "float_xyz_intensity"},
        )

        rgb = stub.calls[0][1]
        lidar = stub.calls[1][1]
        self.assertEqual((rgb.frame_start_us, rgb.frame_end_us), (3_050_000, 3_100_000))
        self.assertEqual(
            (lidar.frame_start_us, lidar.frame_end_us),
            (rgb.frame_start_us, rgb.frame_end_us),
        )
        self.assertEqual(rgb.scene_id, "scene-0061")
        self.assertEqual(lidar.scene_id, "scene-0061")
        self.assertEqual(rgb.image_quality, 0.95)
        self.assertEqual(rgb.dynamic_objects[0].track_id, VEHICLE_TRACK)
        self.assertEqual(lidar.dynamic_objects[0].track_id, VEHICLE_TRACK)
        self.assertEqual(
            rgb.dynamic_objects[0].pose_pair.start_pose,
            lidar.dynamic_objects[0].pose_pair.start_pose,
        )

    def test_time_window_is_strictly_positive_after_microsecond_rounding(self):
        client = self._client()
        self.assertEqual(
            client._time_window_us(
                {"pose_interval_sec": {"start": 2.0, "end": 2.0}}
            ),
            (3_000_000, 3_000_001),
        )

    def test_wrong_rgb_dimensions_fail_the_frame_closed(self):
        evidence = self._client(_Stub(rgb_width=800, rgb_height=450)).dispatch_frame(
            _frame()
        )
        self.assertEqual(evidence["status"], "failed")
        self.assertEqual(evidence["modalities"]["rgb"]["passed_count"], 0)
        self.assertEqual(evidence["modalities"]["lidar"]["passed_count"], 1)

    def test_rejects_rgb_quality_outside_the_nre_fractional_range(self):
        frame = _frame()
        frame["modalities"]["rgb"]["requests"][0]["sensor"]["parameters"][
            "image_quality"
        ] = 95.0

        evidence = self._client().dispatch_frame(frame)

        self.assertEqual(evidence["status"], "failed")
        rgb = next(row for row in evidence["records"] if row["modality"] == "rgb")
        self.assertIn("rpc_status_not_ok", rgb["issues"])

    def test_accepts_nre_260_buffered_lidar_response_with_old_client_proto(self):
        evidence = self._client(_BufferedLidarStub()).dispatch_frame(_frame())

        self.assertEqual(evidence["status"], "passed")
        lidar = next(row for row in evidence["records"] if row["modality"] == "lidar")
        self.assertEqual(
            lidar["response_metadata"],
            {"point_count": 2, "encoding": "float_xyz_intensity"},
        )

    def test_rejects_malformed_nre_260_buffered_lidar_response(self):
        evidence = self._client(
            _BufferedLidarStub(point_count=2, xyz_bytes=struct.pack("<3f", 1, 2, 3))
        ).dispatch_frame(_frame())

        self.assertEqual(evidence["status"], "failed")
        lidar = next(row for row in evidence["records"] if row["modality"] == "lidar")
        self.assertIn("rpc_status_not_ok", lidar["issues"])
        self.assertIn("payload_sha256_invalid", lidar["issues"])

    def test_rejects_buffer_field_with_wrong_protobuf_wire_type(self):
        from adapters.nurec_260_client import _inspect_lidar_response
        from adapters.nurec_multimodal import NuRecMultimodalError

        xyz = struct.pack("<6f", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        intensity = struct.pack("<2f", 0.25, 0.75)
        body = (
            _wire_varint(3, 2)
            + _wire_bytes(4, xyz)
            + _wire_fixed64(5, intensity)
        )

        with self.assertRaisesRegex(NuRecMultimodalError, "wrong protobuf wire type"):
            _inspect_lidar_response(
                _Response(body, point_xyzs=[], point_intensities=[]), body
            )

    def test_factory_requires_explicit_scene_epoch(self):
        from adapters.nurec_260_client import build_nurec_260_handler
        from adapters.nurec_multimodal import NuRecMultimodalError

        with self.assertRaisesRegex(NuRecMultimodalError, "scene_start_us"):
            build_nurec_260_handler(
                {"nurec_runtime": {"runtime_scene_id": "scene-0061"}},
                SimpleNamespace(),
            )

    def test_runtime_inventory_reports_service_identity_and_lidar_boundary(self):
        inventory = self._client(_InventoryStub()).query_runtime_inventory()

        self.assertEqual(inventory["status"], "capability_only")
        self.assertEqual(inventory["renderer"]["version_id"], "26.04")
        self.assertEqual(inventory["available_scene_ids"], ["scene-0061", "scene-0062"])
        self.assertEqual(inventory["cameras"][0]["logical_id"], "camera_front")
        self.assertEqual(
            inventory["lidar"]["supported_device_types"],
            ["PANDAR128", "AT128"],
        )
        self.assertFalse(inventory["lidar"]["render_verified"])


if __name__ == "__main__":
    unittest.main()
