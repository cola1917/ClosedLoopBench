import unittest

from tests.test_nurec_multimodal import _frame


def _encoder(payload):
    return {
        "wire_request": {
            "sensor": payload["sensor"]["sensor_id"],
            "dynamic_objects": payload["dynamic_objects"],
        },
        "frame_id": payload["frame_id"],
        "modality": payload["modality"],
        "dynamic_object_sha256": payload["dynamic_object_sha256"],
    }


class NuRecGrpcDispatchTests(unittest.TestCase):
    def test_dispatches_rgb_and_lidar_through_version_specific_boundary(self):
        from adapters.nurec_grpc_dispatch import dispatch_nurec_multimodal_frame

        calls = []

        def rpc(request):
            calls.append(request)
            return b"rendered"

        evidence = dispatch_nurec_multimodal_frame(
            _frame(),
            encode_rgb=_encoder,
            encode_lidar=_encoder,
            render_rgb=rpc,
            render_lidar=rpc,
        )

        self.assertEqual(evidence["status"], "passed")
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0]["dynamic_objects"], calls[1]["dynamic_objects"])
        self.assertEqual(
            evidence["dispatch"]["dynamic_object_verification"],
            "encoder_echo_checked_before_rpc",
        )

    def test_encoder_cannot_silently_drop_dynamic_objects(self):
        from adapters.nurec_grpc_dispatch import dispatch_nurec_multimodal_frame

        def bad_encoder(payload):
            encoded = _encoder(payload)
            encoded["dynamic_object_sha256"] = "0" * 64
            return encoded

        evidence = dispatch_nurec_multimodal_frame(
            _frame(),
            encode_rgb=bad_encoder,
            encode_lidar=_encoder,
            render_rgb=lambda request: b"rgb",
            render_lidar=lambda request: b"lidar",
        )

        self.assertEqual(evidence["status"], "failed")
        self.assertTrue(any("rpc_status_not_ok" in issue for issue in evidence["issues"]))
        self.assertEqual(evidence["modalities"]["lidar"]["passed_count"], 1)

    def test_rpc_failure_is_recorded_without_hiding_other_modality(self):
        from adapters.nurec_grpc_dispatch import dispatch_nurec_multimodal_frame

        def fail(request):
            raise RuntimeError("server unavailable")

        evidence = dispatch_nurec_multimodal_frame(
            _frame(),
            encode_rgb=_encoder,
            encode_lidar=_encoder,
            render_rgb=fail,
            render_lidar=lambda request: b"lidar",
        )

        self.assertEqual(evidence["status"], "failed")
        self.assertEqual(evidence["modalities"]["rgb"]["passed_count"], 0)
        self.assertEqual(evidence["modalities"]["lidar"]["passed_count"], 1)


if __name__ == "__main__":
    unittest.main()
