from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

from runners.build_scene0061_video_manifest import main as video_cli
from runtime.scene0061_video_manifest import (
    VIDEO_CAPABILITIES,
    VideoManifestError,
    build_scene0061_video_manifest,
    validate_scene0061_video_manifest,
)


LOCAL_PATHS = [
    "handoff/scene0061_formal40k_v1_six_view_centered.mp4",
    "formal_acceptance/dual_window.formal40k_v4/frame_00038.baseline.carla.png",
    "formal_acceptance/dual_window.formal40k_v4/actor_mapping.json",
    "formal_acceptance/dual_window.formal40k_v4/dual_window_report.json",
    "formal_acceptance/dual_window.formal40k_v4/frame_00038.baseline.nurec_grid.png",
    "formal_acceptance/dual_window.formal40k_v4/frame_00038.moved.nurec_grid.png",
    "diagnostics/lidar-probes/replay.formal40k_v1.vehicle_pose_probe.v2.json",
]


def _populate_local_evidence(root: Path) -> None:
    for relative in LOCAL_PATHS:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"fixture")


class Scene0061VideoManifestTests(unittest.TestCase):
    def test_build_has_all_capabilities_and_an_honest_remote_queue(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _populate_local_evidence(root)
            manifest = build_scene0061_video_manifest(
                root, created_at="2026-07-19T00:00:00Z"
            )

        self.assertEqual(
            {shot["capability"] for shot in manifest["shots"]}, VIDEO_CAPABILITIES
        )
        by_capability = {shot["capability"]: shot for shot in manifest["shots"]}
        self.assertEqual(by_capability["original_replay"]["current_availability"], "available")
        self.assertEqual(by_capability["lidar_inset"]["current_availability"], "partial")
        self.assertTrue(by_capability["lidar_inset"]["remote_capture_required"])
        self.assertEqual(
            by_capability["black_hole_quality_stress"]["evidence_classification"],
            "quality_stress",
        )

    def test_validator_rejects_missing_capability_and_availability_overclaim(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _populate_local_evidence(root)
            manifest = build_scene0061_video_manifest(root)
            missing = copy.deepcopy(manifest)
            missing["shots"].pop()
            with self.assertRaisesRegex(VideoManifestError, "missing capabilities"):
                validate_scene0061_video_manifest(missing, evidence_root=root)

            overclaim = copy.deepcopy(manifest)
            shot = next(
                item for item in overclaim["shots"] if item["capability"] == "lead_slowdown_hard_brake"
            )
            shot["current_availability"] = "available"
            with self.assertRaisesRegex(VideoManifestError, "does not match filesystem"):
                validate_scene0061_video_manifest(overclaim, evidence_root=root)

    def test_validator_rejects_non_stress_black_hole_and_incomplete_local_shot(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _populate_local_evidence(root)
            manifest = build_scene0061_video_manifest(root)
            black_hole = next(
                shot for shot in manifest["shots"] if shot["capability"] == "black_hole_quality_stress"
            )
            black_hole["evidence_classification"] = "perception_eligible"
            with self.assertRaisesRegex(VideoManifestError, "black-hole"):
                validate_scene0061_video_manifest(manifest, evidence_root=root)

            manifest = build_scene0061_video_manifest(root)
            lidar = next(shot for shot in manifest["shots"] if shot["capability"] == "lidar_inset")
            lidar["remote_capture_required"] = False
            manifest["remote_capture_queue"].remove(lidar["shot_id"])
            with self.assertRaisesRegex(VideoManifestError, "incomplete evidence"):
                validate_scene0061_video_manifest(manifest, evidence_root=root)

    def test_cli_build_and_validate(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _populate_local_evidence(root)
            output = root / "manifest.json"
            self.assertEqual(
                video_cli(
                    [
                        "--evidence-root",
                        str(root),
                        "--output",
                        str(output),
                        "--created-at",
                        "2026-07-19T00:00:00Z",
                    ]
                ),
                0,
            )
            self.assertEqual(
                video_cli(
                    [
                        "--evidence-root",
                        str(root),
                        "--validate",
                        str(output),
                    ]
                ),
                0,
            )
            manifest = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(manifest["status"], "capture_plan")


if __name__ == "__main__":
    unittest.main()
