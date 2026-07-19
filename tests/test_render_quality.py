from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from runners.evaluate_render_quality import main as quality_cli
from runtime.render_quality import RenderQualityError, evaluate_render_quality


ARTIFACT_SHA = "69e2c36e31e113f9ad66968a0a0a4243c7989dae5582a91a43d150051eaf98b4"


def _save(path: Path, value: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(value.astype(np.uint8)).save(path)


def _request(mask: bool = True, edit_kind: str = "light_vehicle_edit") -> dict:
    camera = {
        "camera_name": "camera_front",
        "baseline_frames": ["baseline_0.png", "baseline_1.png"],
        "edited_frames": ["edited_0.png", "edited_1.png"],
        "mask_provenance": {
            "kind": "synthetic_test_mask" if mask else "none",
            "reliable": mask,
            "source": "unit test" if mask else "no actor mask exists",
            "limitations": [] if mask else ["ROI metrics cannot be measured"],
        },
    }
    if mask:
        camera["actor_masks"] = ["mask.png", "mask.png"]
    return {
        "schema_version": "render_quality_evaluation_request.v1",
        "scene_id": "scene-0061",
        "case_id": "S3_lead_longitudinal_shift",
        "edit_kind": edit_kind,
        "artifact": {"path": "formal/last.usdz", "sha256": ARTIFACT_SHA, "immutable": True},
        "remote_validation_required": True,
        "rgb_lidar_actor_change": {
            "rgb_actor_changed": True,
            "lidar_actor_changed": True,
            "evidence_paths": ["pose_probe.json"],
        },
        "cameras": [camera],
    }


def _fixtures(root: Path, *, black_edit: bool = False) -> None:
    baseline = np.full((24, 32, 3), 120, dtype=np.uint8)
    edited = baseline.copy()
    edited[8:16, 10:22, :] = 0 if black_edit else np.array([150, 105, 90])
    mask = np.zeros((24, 32), dtype=np.uint8)
    mask[8:16, 10:22] = 255
    for index in range(2):
        _save(root / f"baseline_{index}.png", baseline)
        _save(root / f"edited_{index}.png", edited)
    _save(root / "mask.png", mask)


class RenderQualityTests(unittest.TestCase):
    def test_reliable_roi_and_multimodal_change_can_be_perception_eligible(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)
            report = evaluate_render_quality(_request(), base_dir=root)

        self.assertEqual(report["status"], "offline_quality_evaluation")
        self.assertEqual(report["evidence_classification"], "perception_eligible")
        camera = report["cameras"][0]
        self.assertTrue(camera["metrics"]["actor_roi_hole_ratio"]["available"])
        self.assertGreater(camera["metrics"]["edited_region_change"]["value"], 0)
        self.assertEqual(report["rgb_lidar_actor_change_consistency"]["status"], "passed")

    def test_no_reliable_mask_is_explicitly_control_only(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)
            report = evaluate_render_quality(_request(mask=False), base_dir=root)

        self.assertEqual(report["evidence_classification"], "control_only")
        metric = report["cameras"][0]["metrics"]["actor_roi_hole_ratio"]
        self.assertFalse(metric["available"])
        self.assertIn("no reliable actor mask", metric["reason"])

    def test_vehicle_removal_is_quality_stress_and_black_hole_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)
            stress = evaluate_render_quality(
                _request(edit_kind="vehicle_removal"), base_dir=root
            )
            _fixtures(root, black_edit=True)
            rejected = evaluate_render_quality(
                _request(edit_kind="vehicle_removal"), base_dir=root
            )

        self.assertEqual(stress["evidence_classification"], "quality_stress")
        self.assertEqual(rejected["evidence_classification"], "rejected")
        self.assertGreater(
            rejected["cameras"][0]["metrics"]["actor_roi_hole_ratio"]["value"],
            0.25,
        )

    def test_rgb_lidar_mismatch_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)
            request = _request()
            request["rgb_lidar_actor_change"]["lidar_actor_changed"] = False
            report = evaluate_render_quality(request, base_dir=root)

        self.assertEqual(report["evidence_classification"], "rejected")
        self.assertEqual(report["rgb_lidar_actor_change_consistency"]["status"], "failed")

    def test_harmonizer_never_upgrades_source_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)
            request = _request()
            request["harmonizer_applied"] = True
            request["source_evidence_classification"] = "quality_stress"
            report = evaluate_render_quality(request, base_dir=root)

        self.assertEqual(report["evidence_classification"], "quality_stress")
        self.assertIn(
            "Harmonizer cannot upgrade the source evidence classification",
            report["classification_reasons"],
        )

    def test_missing_image_and_false_reliable_mask_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            request = _request(mask=False)
            with self.assertRaisesRegex(RenderQualityError, "image does not exist"):
                evaluate_render_quality(request, base_dir=root)

            _fixtures(root)
            request = _request(mask=False)
            request["cameras"][0]["mask_provenance"]["reliable"] = True
            with self.assertRaisesRegex(RenderQualityError, "requires actor_masks"):
                evaluate_render_quality(request, base_dir=root)

    def test_invalid_threshold_contract_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)
            request = _request()
            request["thresholds"] = {
                "max_actor_hole_ratio": 0.5,
                "reject_actor_hole_ratio": 0.1,
            }
            with self.assertRaisesRegex(RenderQualityError, "must be >="):
                evaluate_render_quality(request, base_dir=root)

    def test_cli_writes_once_and_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)
            request_path = root / "request.json"
            output = root / "report.json"
            request_path.write_text(json.dumps(_request()), encoding="utf-8")
            self.assertEqual(
                quality_cli(
                    [
                        "--request",
                        str(request_path),
                        "--base-dir",
                        str(root),
                        "--output",
                        str(output),
                    ]
                ),
                0,
            )
            self.assertEqual(json.loads(output.read_text())["status"], "offline_quality_evaluation")
            with self.assertRaises(SystemExit):
                quality_cli(
                    [
                        "--request",
                        str(request_path),
                        "--base-dir",
                        str(root),
                        "--output",
                        str(output),
                    ]
                )


if __name__ == "__main__":
    unittest.main()
