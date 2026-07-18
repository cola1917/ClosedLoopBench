import json
from pathlib import Path
import tempfile
import unittest

from adapters.nurec_multimodal import NuRecMultimodalError
from runners.probe_nurec_260_runtime import query_runtime


class _Client:
    def __init__(self, evidence):
        self.evidence = evidence
        self.closed = False

    def query_runtime_inventory(self):
        return {
            "schema_version": "nurec_260_runtime_inventory.v1",
            "lidar": {
                "supported_device_types": ["PANDAR128", "AT128"],
                "render_verified": False,
            },
            "status": "capability_only",
        }

    def dispatch_frame(self, _frame):
        return self.evidence

    def close(self):
        self.closed = True


def _evidence(*, lidar_points=2048, status="passed"):
    return {
        "schema_version": "nurec_multimodal_evidence.v1",
        "frame_id": 38,
        "dynamic_object_sha256": "d" * 64,
        "records": [
            {
                "modality": "rgb",
                "response_metadata": {
                    "width": 1600,
                    "height": 900,
                    "encoding": "jpeg",
                },
            },
            {
                "modality": "lidar",
                "response_metadata": {
                    "point_count": lidar_points,
                    "encoding": "float_xyz_intensity",
                },
            },
        ],
        "issues": [] if status == "passed" else ["lidar:empty"],
        "status": status,
    }


class ProbeNuRec260RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.root = Path(self.tempdir.name)
        self.config = self.root / "config.json"
        self.config.write_text(json.dumps({"nurec_runtime": {}}), encoding="utf-8")
        self.frame = self.root / "frame.json"
        self.frame.write_text(
            json.dumps(
                {
                    "modalities": {
                        "lidar": {
                            "requests": [
                                {
                                    "sensor": {
                                        "parameters": {"device_type": "PANDAR128"}
                                    }
                                }
                            ]
                        }
                    }
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self):
        self.tempdir.cleanup()

    def test_capability_query_is_not_render_verified(self):
        client = _Client(_evidence())
        result = query_runtime(self.config, client_factory=lambda _config: client)

        self.assertEqual(result["status"], "capability_only")
        self.assertFalse(result["lidar"]["render_verified"])
        self.assertTrue(client.closed)

    def test_real_frame_promotes_inventory_only_with_nonempty_lidar(self):
        client = _Client(_evidence(lidar_points=2048))
        result = query_runtime(
            self.config,
            self.frame,
            require_renderable_lidar=True,
            client_factory=lambda _config: client,
        )

        self.assertEqual(result["status"], "passed")
        self.assertTrue(result["lidar"]["render_verified"])
        self.assertEqual(result["lidar"]["probe_point_counts"], [2048])
        self.assertEqual(result["render_probe"]["rgb_record_count"], 1)
        self.assertTrue(client.closed)

    def test_empty_lidar_and_missing_probe_fail_closed(self):
        with self.assertRaisesRegex(NuRecMultimodalError, "requires --probe-frame"):
            query_runtime(
                self.config,
                require_renderable_lidar=True,
                client_factory=lambda _config: _Client(_evidence()),
            )

        client = _Client(_evidence(lidar_points=0, status="failed"))
        with self.assertRaisesRegex(NuRecMultimodalError, "renderability probe failed"):
            query_runtime(
                self.config,
                self.frame,
                require_renderable_lidar=True,
                client_factory=lambda _config: client,
            )
        self.assertTrue(client.closed)


if __name__ == "__main__":
    unittest.main()
