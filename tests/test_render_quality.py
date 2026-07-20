from __future__ import annotations

import json
import hashlib
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


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _ref(path: Path, *, kind: str, encoding: str) -> dict:
    return {
        "path": str(path),
        "sha256": _sha(path),
        "size_bytes": path.stat().st_size,
        "kind": kind,
        "encoding": encoding,
    }


def _write_change_source(root: Path, *, case_id: str) -> dict:
    rgb_baseline = [root / f"baseline_{index}.png" for index in range(2)]
    rgb_edited = [root / f"edited_{index}.png" for index in range(2)]
    lidar_baseline = [root / f"baseline_{index}.bin" for index in range(2)]
    lidar_edited = [root / f"edited_{index}.bin" for index in range(2)]
    source = {
        "schema_version": "rgb_lidar_actor_change_source_report.v1",
        "status": "passed",
        "experiment": {
            "scene_id": "scene-0061",
            "case_id": case_id,
            "artifact_sha256": ARTIFACT_SHA,
        },
        "target_track_id": "vehicle-track",
        "frame_range": {
            phase: {
                "start_frame_id": 10,
                "end_frame_id": 11,
                "frame_count": 2,
                "start_timestamp_sec": 1.0,
                "end_timestamp_sec": 1.05,
            }
            for phase in ("baseline", "edited")
        },
        "payloads": {
            "rgb": {
                "baseline": [_ref(path, kind="rgb", encoding="png") for path in rgb_baseline],
                "edited": [_ref(path, kind="rgb", encoding="png") for path in rgb_edited],
            },
            "lidar": {
                "baseline": [
                    _ref(path, kind="lidar", encoding="float32_xyzi_little_endian")
                    for path in lidar_baseline
                ],
                "edited": [
                    _ref(path, kind="lidar", encoding="float32_xyzi_little_endian")
                    for path in lidar_edited
                ],
            },
        },
    }
    source["change_flags"] = {
        "rgb_actor_changed": [item["sha256"] for item in source["payloads"]["rgb"]["baseline"]]
        != [item["sha256"] for item in source["payloads"]["rgb"]["edited"]],
        "lidar_actor_changed": [item["sha256"] for item in source["payloads"]["lidar"]["baseline"]]
        != [item["sha256"] for item in source["payloads"]["lidar"]["edited"]],
    }
    path = root / "rgb_lidar_actor_change.source_report.json"
    path.write_text(json.dumps(source, allow_nan=False), encoding="utf-8")
    return {
        "path": str(path),
        "sha256": _sha(path),
        "size_bytes": path.stat().st_size,
        "schema_version": source["schema_version"],
        "status": source["status"],
    }


def _request(root: Path, mask: bool = True, edit_kind: str = "light_vehicle_edit") -> dict:
    case_id = "S3_lead_longitudinal_shift"
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
        "case_id": case_id,
        "target_track_id": "vehicle-track",
        "edit_kind": edit_kind,
        "artifact": {"path": "formal/last.usdz", "sha256": ARTIFACT_SHA, "immutable": True},
        "remote_validation_required": True,
        "rgb_lidar_actor_change": {
            "source_report_ref": _write_change_source(root, case_id=case_id),
        },
        "cameras": [camera],
    }


def _fixtures(
    root: Path, *, black_edit: bool = False, actor_change: bool = True
) -> None:
    baseline = np.full((24, 32, 3), 120, dtype=np.uint8)
    edited = baseline.copy()
    if actor_change:
        edited[8:16, 10:22, :] = 0 if black_edit else np.array([150, 105, 90])
    mask = np.zeros((24, 32), dtype=np.uint8)
    mask[8:16, 10:22] = 255
    for index in range(2):
        _save(root / f"baseline_{index}.png", baseline)
        _save(root / f"edited_{index}.png", edited)
        (root / f"baseline_{index}.bin").write_bytes(
            np.asarray([[1.0, 0.0, 0.0, 0.5]], dtype="<f4").tobytes()
        )
        (root / f"edited_{index}.bin").write_bytes(
            np.asarray(
                [[2.0 if actor_change else 1.0, 0.0, 0.0, 0.5]], dtype="<f4"
            ).tobytes()
        )
    _save(root / "mask.png", mask)


class RenderQualityTests(unittest.TestCase):
    def test_reliable_roi_and_multimodal_change_can_be_perception_eligible(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)
            report = evaluate_render_quality(_request(root), base_dir=root)

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
            report = evaluate_render_quality(_request(root, mask=False), base_dir=root)

        self.assertEqual(report["evidence_classification"], "control_only")
        metric = report["cameras"][0]["metrics"]["actor_roi_hole_ratio"]
        self.assertFalse(metric["available"])
        self.assertIn("no reliable actor mask", metric["reason"])

    def test_vehicle_removal_is_quality_stress_and_black_hole_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)
            stress = evaluate_render_quality(
                _request(root, edit_kind="vehicle_removal"), base_dir=root
            )
            _fixtures(root, black_edit=True)
            rejected = evaluate_render_quality(
                _request(root, edit_kind="vehicle_removal"), base_dir=root
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
            for index in range(2):
                (root / f"edited_{index}.bin").write_bytes(
                    (root / f"baseline_{index}.bin").read_bytes()
                )
            request = _request(root)
            report = evaluate_render_quality(request, base_dir=root)

        self.assertEqual(report["evidence_classification"], "rejected")
        self.assertEqual(report["rgb_lidar_actor_change_consistency"]["status"], "failed")

    def test_structured_change_evidence_forged_missing_and_mutated_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)

            forged = _request(root)
            forged_ref = forged["rgb_lidar_actor_change"]["source_report_ref"]
            source_path = Path(forged_ref["path"])
            source = json.loads(source_path.read_text(encoding="utf-8"))
            source["change_flags"]["lidar_actor_changed"] = False
            source_path.write_text(json.dumps(source, allow_nan=False), encoding="utf-8")
            forged_ref["sha256"] = _sha(source_path)
            forged_ref["size_bytes"] = source_path.stat().st_size
            with self.assertRaisesRegex(RenderQualityError, "conflicts with payload hashes"):
                evaluate_render_quality(forged, base_dir=root)

            _fixtures(root)
            missing = _request(root)
            Path(missing["rgb_lidar_actor_change"]["source_report_ref"]["path"]).unlink()
            with self.assertRaisesRegex(RenderQualityError, "file_missing"):
                evaluate_render_quality(missing, base_dir=root)

            _fixtures(root)
            mutated = _request(root)
            (root / "edited_0.bin").write_bytes(
                np.asarray([[99.0, 0.0, 0.0, 0.5]], dtype="<f4").tobytes()
            )
            with self.assertRaisesRegex(RenderQualityError, "sha256_mismatch"):
                evaluate_render_quality(mutated, base_dir=root)

    def test_original_replay_consistency_requires_unchanged_rgb_and_lidar(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root, actor_change=False)
            request = _request(root, edit_kind="original_replay")
            report = evaluate_render_quality(request, base_dir=root)

        self.assertEqual(
            report["rgb_lidar_actor_change_consistency"]["status"], "passed"
        )
        self.assertFalse(
            report["rgb_lidar_actor_change_consistency"]["expected_actor_change"]
        )

    def test_harmonizer_never_upgrades_source_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)
            request = _request(root)
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
            _fixtures(root)
            request = _request(root, mask=False)
            (root / "baseline_0.png").unlink()
            with self.assertRaisesRegex(RenderQualityError, "image does not exist"):
                evaluate_render_quality(request, base_dir=root)

            _fixtures(root)
            request = _request(root, mask=False)
            request["cameras"][0]["mask_provenance"]["reliable"] = True
            with self.assertRaisesRegex(RenderQualityError, "requires actor_masks"):
                evaluate_render_quality(request, base_dir=root)

    def test_invalid_threshold_contract_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            _fixtures(root)
            request = _request(root)
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
            request_path.write_text(json.dumps(_request(root)), encoding="utf-8")
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
