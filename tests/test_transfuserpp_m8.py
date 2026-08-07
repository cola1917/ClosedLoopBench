import hashlib
import json
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

from tests.test_transfuserpp_intermediate import _record


class TransFuserPPM8Tests(unittest.TestCase):
    def _scenario_ir(self) -> dict:
        return {
            "schema_version": "scenario_ir.v1",
            "scenario_id": "cc8c0bf57f984915a77078b10eb33198",
            "source": {"dataset": "nuscenes", "scene_name": "scene-0061"},
            "coordinate_frame": {
                "name": "scene_local_ego_start",
                "x_axis": "initial_ego_forward",
                "y_axis": "initial_ego_left",
            },
            "ego": {
                "reference_trajectory": [
                    {"t_sec": 0.0, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "speed_mps": 8.0},
                    {"t_sec": 0.5, "x": 4.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "speed_mps": 8.0},
                ]
            },
            "actors": [
                {
                    "actor_id": "vehicle-1",
                    "source_track_id": "vehicle-1",
                    "type": "vehicle",
                    "dimensions": {"length": 4.0, "width": 1.8, "height": 1.5},
                    "reference_trajectory": [
                        {"t_sec": 0.0, "x": 5.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "speed_mps": 0.0},
                        {"t_sec": 0.5, "x": 5.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "speed_mps": 0.0},
                    ],
                }
            ],
        }

    def _records(self, root: Path, scenario_sha: str) -> list[dict]:
        rows = []
        for frame_id, timestamp, ego_x in ((0, 0.0, 0.0), (1, 0.5, 4.0)):
            record = _record(
                root,
                frame_id=frame_id,
                timestamp=timestamp,
                case_id="S0_original_replay",
                labels=__import__("numpy").zeros((16, 16), dtype="uint8"),
                target_speed=8.0,
                brake=False,
            )
            record["experiment"]["scene_version"] = "open-loop-exchange-v1"
            record["experiment"]["scenario_ir_sha256"] = scenario_sha
            record["inputs"]["ego_pose"] = {"x": ego_x, "y": 0.0, "yaw": 0.0}
            rows.append(record)
        return rows

    def test_raw_gt_binds_trajectory_speed_and_dynamic_bev_metrics(self):
        from metrics.transfuserpp_m8 import evaluate_m8_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario_ir = self._scenario_ir()
            scenario_path = root / "scene_ir.json"
            scenario_path.write_text(json.dumps(scenario_ir), encoding="utf-8")
            scenario_sha = hashlib.sha256(scenario_path.read_bytes()).hexdigest()
            rows = self._records(root, scenario_sha)
            report = evaluate_m8_intermediate_trace(
                rows,
                scenario_ir=scenario_ir,
                scenario_ir_path=scenario_path,
                expected_scenario_ir_sha256=scenario_sha,
                expected_frame_count=2,
            )

        self.assertEqual(report["status"], "evaluated")
        self.assertTrue(report["input_binding"]["raw_input_source"])
        self.assertFalse(report["input_binding"]["reconstruction_input_used"])
        self.assertEqual(report["ground_truth_binding"]["dataset"], "nuscenes")
        self.assertEqual(report["ground_truth_binding"]["scenario_ir_actor_count"], 1)
        self.assertEqual(
            report["ground_truth_binding"]["evaluated_dynamic_actor_track_count"], 1
        )
        self.assertEqual(report["waypoints"]["status"], "evaluated")
        self.assertEqual(report["route_checkpoints"]["status"], "evaluated")
        self.assertEqual(report["target_speed"]["status"], "evaluated")
        self.assertEqual(report["dynamic_bev"]["status"], "evaluated")
        self.assertEqual(report["depth"]["status"], "unavailable")

    def test_reconstructed_input_is_rejected_even_with_raw_gt(self):
        from metrics.transfuserpp_m8 import evaluate_m8_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario_ir = self._scenario_ir()
            scenario_path = root / "scene_ir.json"
            scenario_path.write_text(json.dumps(scenario_ir), encoding="utf-8")
            scenario_sha = hashlib.sha256(scenario_path.read_bytes()).hexdigest()
            rows = self._records(root, scenario_sha)
            for row in rows:
                row["experiment"]["scene_version"] = "open-loop-nurec-stage-b-v1"
            report = evaluate_m8_intermediate_trace(
                rows,
                scenario_ir=scenario_ir,
                scenario_ir_path=scenario_path,
                expected_scenario_ir_sha256=scenario_sha,
                expected_frame_count=2,
            )

        self.assertEqual(report["status"], "failed")
        self.assertEqual(
            report["input_binding"]["observed_source"],
            "nurec_stage_b_6cam_rgb_lidar",
        )
        self.assertIn("m8_reconstructed_or_harmonized_input_forbidden", report["fail_closed_reasons"])

    def test_gt_hash_mismatch_is_fail_closed(self):
        from metrics.transfuserpp_m8 import evaluate_m8_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario_ir = self._scenario_ir()
            scenario_path = root / "scene_ir.json"
            scenario_path.write_text(json.dumps(scenario_ir), encoding="utf-8")
            scenario_sha = hashlib.sha256(scenario_path.read_bytes()).hexdigest()
            report = evaluate_m8_intermediate_trace(
                self._records(root, scenario_sha),
                scenario_ir=scenario_ir,
                scenario_ir_path=scenario_path,
                expected_scenario_ir_sha256="0" * 64,
                expected_frame_count=2,
            )

        self.assertEqual(report["status"], "failed")
        self.assertIn("m8_scenario_ir_sha256_mismatch", report["fail_closed_reasons"])

    def test_bbox_metrics_use_oriented_geometry_and_score_ordered_ap(self):
        from metrics.transfuserpp_m8 import _oriented_box_iou, evaluate_m8_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario_ir = self._scenario_ir()
            scenario_path = root / "scene_ir.json"
            scenario_path.write_text(json.dumps(scenario_ir), encoding="utf-8")
            scenario_sha = hashlib.sha256(scenario_path.read_bytes()).hexdigest()
            rows = self._records(root, scenario_sha)
            rows[0]["outputs"]["bounding_boxes_ego"] = [
                [5.0, 0.0, 2.0, 0.9, 0.0, 0.0, 0.0, 0.0, 0.9],
                [1.0, 0.0, 2.0, 0.9, 0.0, 0.0, 0.0, 0.0, 0.2],
            ]
            rows[1]["outputs"]["bounding_boxes_ego"] = [
                [1.0, 0.0, 2.0, 0.9, 0.0, 0.0, 0.0, 0.0, 0.95],
            ]
            report = evaluate_m8_intermediate_trace(
                rows,
                scenario_ir=scenario_ir,
                scenario_ir_path=scenario_path,
                expected_scenario_ir_sha256=scenario_sha,
                expected_frame_count=2,
            )

        self.assertEqual(report["status"], "evaluated")
        metrics = report["dynamic_bev"]
        self.assertEqual(metrics["metric_scope"], "dynamic_actor_oriented_bev_bbox")
        self.assertEqual(metrics["ground_truth_actor_sample_count_in_model_window"], 2)
        detection = metrics["box_detection"]
        self.assertEqual(detection["tp"], 2)
        self.assertEqual(detection["fp"], 1)
        self.assertEqual(detection["fn"], 0)
        self.assertAlmostEqual(detection["precision"], 2 / 3)
        self.assertAlmostEqual(detection["recall"], 1.0)
        self.assertIsNotNone(detection["mAP50"])
        self.assertGreater(detection["mAP50"], 0.0)
        self.assertEqual(detection["per_class"]["vehicle"]["size_error_m"]["count"], 2)
        self.assertEqual(detection["per_class"]["vehicle"]["yaw_error_deg"]["count"], 2)
        self.assertAlmostEqual(
            _oriented_box_iou(
                {
                    "center_ego_m": [0.0, 0.0],
                    "length_m": 4.0,
                    "width_m": 2.0,
                    "yaw_rad": 0.0,
                },
                {
                    "x": 0.0,
                    "y": 0.0,
                    "half_length": 2.0,
                    "half_width": 1.0,
                    "yaw_rad": 0.0,
                },
            ),
            1.0,
        )

    def test_bbox_ground_truth_outside_model_window_is_excluded(self):
        from metrics.transfuserpp_m8 import evaluate_m8_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario_ir = self._scenario_ir()
            scenario_ir["actors"][0]["reference_trajectory"] = [
                {"t_sec": 0.0, "x": 100.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "speed_mps": 0.0},
                {"t_sec": 0.5, "x": 100.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "speed_mps": 0.0},
            ]
            scenario_path = root / "scene_ir.json"
            scenario_path.write_text(json.dumps(scenario_ir), encoding="utf-8")
            scenario_sha = hashlib.sha256(scenario_path.read_bytes()).hexdigest()
            report = evaluate_m8_intermediate_trace(
                self._records(root, scenario_sha),
                scenario_ir=scenario_ir,
                scenario_ir_path=scenario_path,
                expected_scenario_ir_sha256=scenario_sha,
                expected_frame_count=2,
            )

        metrics = report["dynamic_bev"]
        self.assertEqual(metrics["full_scene_ground_truth_actor_sample_count"], 2)
        self.assertEqual(metrics["ground_truth_actor_sample_count_in_model_window"], 0)
        self.assertEqual(metrics["ground_truth_actor_sample_count_excluded_outside_model_window"], 2)
        self.assertEqual(metrics["box_detection"]["fn"], 0)

    def test_formal_bbox_fails_closed_without_actor_manifest(self):
        from metrics.transfuserpp_m8 import evaluate_m8_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario_ir = self._scenario_ir()
            scenario_path = root / "scene_ir.json"
            scenario_path.write_text(json.dumps(scenario_ir), encoding="utf-8")
            scenario_sha = hashlib.sha256(scenario_path.read_bytes()).hexdigest()
            report = evaluate_m8_intermediate_trace(
                self._records(root, scenario_sha),
                scenario_ir=scenario_ir,
                scenario_ir_path=scenario_path,
                expected_scenario_ir_sha256=scenario_sha,
                expected_frame_count=2,
                require_actor_manifest=True,
            )

        self.assertEqual(report["status"], "failed")
        self.assertIn(
            "m8_actor_manifest_required_for_formal_bbox",
            report["fail_closed_reasons"],
        )
        self.assertEqual(report["dynamic_bev"]["status"], "unavailable")

    def test_formal_bbox_uses_manifest_frame_binding_and_real_geometry(self):
        from adapters.open_loop_bbox_binding import canonical_sha256, manifest_content_sha256
        from metrics.transfuserpp_m8 import evaluate_m8_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario_ir = self._scenario_ir()
            scenario_path = root / "scene_ir.json"
            scenario_path.write_text(json.dumps(scenario_ir), encoding="utf-8")
            scenario_sha = hashlib.sha256(scenario_path.read_bytes()).hexdigest()
            dimensions = {"length": 4.0, "width": 1.8, "height": 1.5}
            trajectory = deepcopy(scenario_ir["actors"][0]["reference_trajectory"])
            objects = []
            frames = []
            for frame_id, state in enumerate(trajectory):
                pose = {
                    "x": float(state["x"]),
                    "y": float(state["y"]),
                    "z": float(state["z"]),
                    "yaw": float(state["yaw"]),
                }
                dynamic_object = {
                    "actor_id": "vehicle-1",
                    "track_id": "vehicle-1",
                    "actor_type": "vehicle",
                    "dimensions_m": deepcopy(dimensions),
                    "pose_source": "scenario_ir_actor_reference_trajectory",
                    "pose_reference": "actor_bbox_center",
                    "pose_pair": {"start": pose, "end": pose},
                }
                objects = [dynamic_object]
                pose_payload = [
                    {
                        "track_id": "vehicle-1",
                        "actor_type": "vehicle",
                        "dimensions_m": dimensions,
                        "pose_pair": {"start": pose, "end": pose},
                    }
                ]
                frames.append(
                    {
                        "frame_id": frame_id,
                        "timestamp_sec": float(state["t_sec"]),
                        "ego_timestamp_us": frame_id * 500000,
                        "active_actor_ids": ["vehicle-1"],
                        "gt_active_actor_ids": ["vehicle-1"],
                        "active_actor_set_sha256": canonical_sha256(["vehicle-1"]),
                        "pose_digest": canonical_sha256(pose_payload),
                        "dynamic_object_sha256": canonical_sha256(objects),
                        "usdz_track_bindings": [],
                        "dynamic_objects": deepcopy(objects),
                    }
                )
            manifest = {
                "schema_version": "open_loop_bbox_actor_manifest.v1",
                "scene_id": scenario_ir["scenario_id"],
                "scenario_id": scenario_ir["scenario_id"],
                "scenario_ir": {"path": str(scenario_path), "sha256": scenario_sha},
                "usdz": {"path": str(root / "scene.usdz"), "sha256": "d" * 64},
                "actors": [
                    {
                        "actor_id": "vehicle-1",
                        "source_track_id": "vehicle-1",
                        "actor_type": "vehicle",
                        "dimensions_m": dimensions,
                        "ir_trajectory": trajectory,
                        "ir_active_window_sec": {
                            "first": float(trajectory[0]["t_sec"]),
                            "last": float(trajectory[-1]["t_sec"]),
                        },
                    }
                ],
                "frames": frames,
                "summary": {
                    "actor_count": 1,
                    "vehicle_count": 1,
                    "pedestrian_count": 0,
                    "frame_count": 2,
                    "source_ir_actor_count": 1,
                },
            }
            manifest["manifest_sha256"] = manifest_content_sha256(manifest)
            manifest["manifest_file_sha256"] = "e" * 64
            rows = self._records(root, scenario_sha)
            for frame_id, row in enumerate(rows):
                frame = frames[frame_id]
                row["provenance"]["actor_manifest"] = {
                    "schema_version": "open_loop_bbox_dynamic_provenance.v1",
                    "actor_manifest_sha256": manifest["manifest_sha256"],
                    "actor_manifest_file_sha256": manifest["manifest_file_sha256"],
                    "frame_id": frame_id,
                    "active_actor_ids": ["vehicle-1"],
                    "active_actor_set_sha256": frame["active_actor_set_sha256"],
                    "pose_digest": frame["pose_digest"],
                    "manifest_dynamic_object_sha256": frame["dynamic_object_sha256"],
                }
                row["synchronization"]["dynamic_object_sha256"] = frame[
                    "dynamic_object_sha256"
                ]
            rows[1]["outputs"]["bounding_boxes_ego"][0][0] = 1.0
            report = evaluate_m8_intermediate_trace(
                rows,
                scenario_ir=scenario_ir,
                scenario_ir_path=scenario_path,
                expected_scenario_ir_sha256=scenario_sha,
                expected_frame_count=2,
                actor_manifest=manifest,
                require_actor_manifest=True,
            )

        self.assertEqual(report["status"], "evaluated")
        self.assertEqual(report["ground_truth_binding"]["evaluated_dynamic_actor_track_count"], 1)
        self.assertEqual(report["ground_truth_binding"]["actor_manifest"]["status"], "bound")
        self.assertEqual(report["dynamic_bev"]["box_detection"]["tp"], 2)
        self.assertEqual(report["dynamic_bev"]["box_detection"]["fn"], 0)


if __name__ == "__main__":
    unittest.main()
