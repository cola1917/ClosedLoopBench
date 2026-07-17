import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from adapters.nurec_multimodal import (
    build_nurec_multimodal_evidence,
    materialize_nurec_rpc_requests,
)
from tests.test_actor_binding import VEHICLE_TRACK
from tests.test_nurec_multimodal import _frame


def _moved_frame(delta=1.0):
    frame = copy.deepcopy(_frame())
    actor = frame["shared_dynamic_objects"][0]
    for endpoint in ("start", "end"):
        actor["pose_pair"][endpoint]["position_m"]["x"] += delta
    digest = hashlib.sha256(
        json.dumps(
            frame["shared_dynamic_objects"],
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
        ).encode("utf-8")
    ).hexdigest()
    frame["shared_dynamic_object_sha256"] = digest
    for modality in ("rgb", "lidar"):
        for request in frame["modalities"][modality]["requests"]:
            request["dynamic_object_sha256"] = digest
    return frame


def _dispatcher(*, unchanged_modality=None, unstable_modality=None):
    call_count = 0

    def dispatch(frame):
        nonlocal call_count
        call_count += 1
        responses = []
        for payload in materialize_nurec_rpc_requests(frame):
            source = (
                f"fixed:{payload['modality']}"
                if payload["modality"] == unchanged_modality
                else f"unstable:{call_count}:{payload['modality']}"
                if payload["modality"] == unstable_modality
                else f"{payload['modality']}:{payload['dynamic_object_sha256']}"
            )
            responses.append(
                {
                    "request_id": payload["request_id"],
                    "status": "ok",
                    "frame_id": payload["frame_id"],
                    "dynamic_object_sha256": payload["dynamic_object_sha256"],
                    "payload_sha256": hashlib.sha256(source.encode()).hexdigest(),
                    "latency_ms": 1.0,
                }
            )
        return build_nurec_multimodal_evidence(frame, responses)

    return dispatch


class NuRecPoseProbeTests(unittest.TestCase):
    def test_requires_both_rgb_and_lidar_render_content_to_change(self):
        from adapters.nurec_pose_probe import run_nurec_dynamic_pose_ab_probe

        report = run_nurec_dynamic_pose_ab_probe(
            VEHICLE_TRACK,
            _frame(),
            _moved_frame(),
            dispatch_frame=_dispatcher(),
        )

        self.assertEqual(report["status"], "passed")
        self.assertAlmostEqual(report["probe"]["pose_delta_m"], 1.0)
        self.assertNotEqual(
            report["probe"]["baseline_dynamic_object_sha256"],
            report["probe"]["dynamic_object_sha256"],
        )
        for modality in ("rgb", "lidar"):
            evidence = report["probe"]["modalities"][modality]
            self.assertEqual(evidence["status"], "passed")
            self.assertTrue(evidence["baseline_repeatable"])
            self.assertTrue(evidence["content_changed"])
            self.assertNotEqual(
                evidence["baseline_payload_sha256"],
                evidence["moved_payload_sha256"],
            )

    def test_unchanged_lidar_output_fails_instead_of_promoting_track(self):
        from adapters.nurec_pose_probe import run_nurec_dynamic_pose_ab_probe

        report = run_nurec_dynamic_pose_ab_probe(
            VEHICLE_TRACK,
            _frame(),
            _moved_frame(),
            dispatch_frame=_dispatcher(unchanged_modality="lidar"),
        )

        self.assertEqual(report["status"], "failed")
        self.assertIn("lidar_render_unchanged", report["issues"])
        self.assertFalse(report["probe"]["modalities"]["lidar"]["content_changed"])

    def test_unrepeatable_rgb_baseline_fails_instead_of_claiming_actor_effect(self):
        from adapters.nurec_pose_probe import run_nurec_dynamic_pose_ab_probe

        report = run_nurec_dynamic_pose_ab_probe(
            VEHICLE_TRACK,
            _frame(),
            _moved_frame(),
            dispatch_frame=_dispatcher(unstable_modality="rgb"),
        )

        self.assertEqual(report["status"], "failed")
        self.assertIn("rgb_baseline_unrepeatable", report["issues"])
        self.assertFalse(report["probe"]["modalities"]["rgb"]["baseline_repeatable"])

    def test_probe_result_is_sufficient_for_verified_runtime_inventory(self):
        from adapters.nurec_inventory import build_nurec_runtime_track_inventory
        from adapters.nurec_pose_probe import run_nurec_dynamic_pose_ab_probe

        report = run_nurec_dynamic_pose_ab_probe(
            VEHICLE_TRACK,
            _frame(),
            _moved_frame(),
            dispatch_frame=_dispatcher(),
        )
        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "last.usdz"
            artifact.write_bytes(b"usdz")
            inventory = build_nurec_runtime_track_inventory(
                {
                    VEHICLE_TRACK: SimpleNamespace(
                        actor_inst=SimpleNamespace(id=101, type_id="vehicle.car")
                    )
                },
                artifact_path=artifact,
                renderer_version="26.04",
                probe_results={VEHICLE_TRACK: report["probe"]},
            )

        self.assertTrue(inventory["tracks"][0]["dynamic_object_pose_verified"])

    def test_rejects_probe_that_changes_time_or_has_too_small_pose_delta(self):
        from adapters.nurec_multimodal import NuRecMultimodalError
        from adapters.nurec_pose_probe import run_nurec_dynamic_pose_ab_probe

        changed_time = _moved_frame()
        changed_time["simulation_time_sec"] += 0.1
        with self.assertRaisesRegex(NuRecMultimodalError, "simulation_time_sec"):
            run_nurec_dynamic_pose_ab_probe(
                VEHICLE_TRACK,
                _frame(),
                changed_time,
                dispatch_frame=_dispatcher(),
            )

        with self.assertRaisesRegex(NuRecMultimodalError, "at least 0.05"):
            run_nurec_dynamic_pose_ab_probe(
                VEHICLE_TRACK,
                _frame(),
                _moved_frame(delta=0.01),
                dispatch_frame=_dispatcher(),
            )


if __name__ == "__main__":
    unittest.main()
