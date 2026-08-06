import unittest


class DerivedBindingTests(unittest.TestCase):
    def test_corrected_reconstructed_mode_labels_keep_same_physical_frame(self):
        from runners.bind_open_loop_transfuserpp_derived import _validate_source_frame_binding

        source = {
            "source_kind": "neural_scene_bridge_multimodal_20fps",
            "source_frame_index": 0,
            "source_timestamp_us": 1000,
            "source_time_sec": 0.0,
            "delta_us": 0,
            "ir_frame_id": 0,
            "ir_timestamp_sec": 0.0,
            "rgb_mode": "original",
            "lidar_mode": "original",
            "rgb_source_sha256": "a" * 64,
            "lidar_source_sha256": "b" * 64,
            "rgb_materialized_sha256": "c" * 64,
            "lidar_materialized_sha256": "d" * 64,
        }
        target = dict(source)
        target.update({"rgb_mode": "reconstructed", "lidar_mode": "reconstructed"})

        _validate_source_frame_binding(source, target, index=0)

    def test_source_frame_payload_hash_mismatch_is_rejected(self):
        from runners.bind_open_loop_transfuserpp_derived import (
            DerivedBindingError,
            _validate_source_frame_binding,
        )

        source = {
            "source_kind": "neural_scene_bridge_multimodal_20fps",
            "source_frame_index": 0,
            "source_timestamp_us": 1000,
            "source_time_sec": 0.0,
            "delta_us": 0,
            "ir_frame_id": 0,
            "ir_timestamp_sec": 0.0,
            "rgb_source_sha256": "a" * 64,
            "lidar_source_sha256": "b" * 64,
            "rgb_materialized_sha256": "c" * 64,
            "lidar_materialized_sha256": "d" * 64,
        }
        target = dict(source)
        target["lidar_materialized_sha256"] = "e" * 64

        with self.assertRaisesRegex(DerivedBindingError, "lidar_materialized_sha256"):
            _validate_source_frame_binding(source, target, index=0)


if __name__ == "__main__":
    unittest.main()
