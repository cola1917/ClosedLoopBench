import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image


HASH = "d" * 64
def _calibration():
    from agents.transfuserpp_contract import camera_adaptation_contract

    return {
    "camera_sensor_id": "camera_front",
    "camera_sensor_to_ego": [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 2.5,
        0.0, 0.0, 0.0, 1.0,
    ],
    "camera_sensor_to_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
    "camera_adaptation": camera_adaptation_contract(),
    "lidar_sensor_id": "lidar_top",
    "lidar_sensor_to_ego": [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 2.5,
        0.0, 0.0, 0.0, 1.0,
    ],
    }


CALIBRATION = _calibration()


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _record(root, *, frame_id, timestamp, case_id, labels, target_speed, brake):
    if case_id == "S2_lead_hard_brake":
        track_id = "c1958768d48640948f6053d04cffd35b"
        actor_type = "vehicle"
        class_id = 0.0
        bev_class = 9
    elif case_id == "S4_pedestrian_early_crossing":
        track_id = "71603dd1a2ba4e9daf095535e38310ac"
        actor_type = "pedestrian"
        class_id = 1.0
        bev_class = 10
    else:
        track_id = "baseline-track"
        actor_type = "vehicle"
        class_id = 0.0
        bev_class = 9
    if target_speed <= 0.0:
        target_speed_bins = [0.0, 5.0]
        target_speed_probabilities = [1.0, 0.0]
    else:
        target_speed_bins = [0.0, float(target_speed)]
        target_speed_probabilities = [0.0, 1.0]
    dense = root / f"{case_id}.{frame_id}.npz"
    np.savez_compressed(
        dense,
        bev_semantic_labels=labels,
        perspective_semantic_labels=np.zeros((4, 4), dtype=np.uint8),
        depth=np.zeros((1, 4, 4), dtype=np.float32),
        target_speed_probabilities=np.asarray(target_speed_probabilities, dtype=np.float32),
    )
    rgb = root / f"{case_id}.{frame_id}.jpg"
    Image.new("RGB", (64, 32), (80, 100, 120)).save(rgb)
    lidar = root / f"{case_id}.{frame_id}.bin"
    lidar.write_bytes(np.asarray([[1.0, 0.0, 0.0, 0.5]], dtype="<f4").tobytes())
    return {
        "schema_version": "transfuserpp_intermediate_frame.v1",
        "algorithm_id": "transfuserpp_v5",
        "algorithm_version": "carla_garage.leaderboard_2.transfuser_v5",
        "frame_id": frame_id,
        "timestamp": timestamp,
        "identity": {
            "repo_sha256": HASH,
            "checkpoint_sha256": HASH,
            "model_config_sha256": HASH,
            "repo_revision": "e" * 40,
            "runtime_config_sha256": ("a" if case_id == "S0_original_replay" else "b") * 64,
            "carla_agents_sha256": HASH,
            "adapter_source_sha256": HASH,
            "container_image_digest": "sha256:" + HASH,
        },
        "experiment": {
            "scene_id": "cc8c0bf57f984915a77078b10eb33198",
            "scene_version": "formal40k-v1",
            "case_id": case_id,
            "seed": 41,
            "run_id": f"{case_id}-attempt-01",
            "artifact_sha256": HASH,
            "scene_package_sha256": HASH,
            "scenario_ir_sha256": HASH,
            "immutable_matrix_sha256": HASH,
            "source_run_config_sha256": HASH,
            "variant_config_sha256": ("a" if case_id == "S0_original_replay" else "b") * 64,
            "run_config_sha256": HASH,
        },
        "provenance": {
            "execution_mode": "remote_model_inference",
            "real_checkpoint_loaded": True,
        },
        "inputs": {
            "camera_front": {
                "path": str(rgb),
                "sha256": _sha(rgb),
                "byte_count": rgb.stat().st_size,
                "encoding": "jpeg",
                "coordinate_frame": "camera_optical",
            },
            "lidar_top": {
                "path": str(lidar),
                "sha256": _sha(lidar),
                "byte_count": lidar.stat().st_size,
                "encoding": "float32_xyzi_little_endian",
                "coordinate_frame": "sensor_local",
                "axis_convention": "carla_sensor",
                "sensor_to_ego": list(CALIBRATION["lidar_sensor_to_ego"]),
            },
            "calibration": dict(CALIBRATION),
            "camera_adaptation": __import__(
                "agents.transfuserpp_contract", fromlist=["camera_adaptation_evidence"]
            ).camera_adaptation_evidence(
                contract=CALIBRATION["camera_adaptation"],
                source_payload={
                    "sha256": _sha(rgb),
                    "byte_count": rgb.stat().st_size,
                },
                model_sensor_width=1024,
                model_sensor_height=512,
                center_crop_xyxy=[0, 25, 800, 425],
                model_crop_applied_by_upstream=True,
            ),
            "model_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
            "ego_pose_coordinate_frame": "closedloopbench_scene_x_forward_y_left_z_up",
            "ego_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        },
        "outputs": {
            "waypoints_ego_m": [[1.0, 0.0], [2.0, 0.0]],
            "route_checkpoints_ego_m": [[2.0, 0.0], [4.0, 0.0]],
            "target_speed_mps": target_speed,
            "target_speed_probabilities": target_speed_probabilities,
            "target_speed_bins_mps": target_speed_bins,
            "target_speed_selection_mode": "argmax",
            "target_speed_selected_index": int(
                max(
                    range(len(target_speed_probabilities)),
                    key=lambda index: target_speed_probabilities[index],
                )
            ),
            "target_speed_brake_uncertainty_threshold": 0.5,
            "bounding_boxes_ego": [[5.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, class_id, 0.9]],
            "control": {
                "throttle": 0.0 if brake else 0.3,
                "steer": 0.0,
                "brake": 1.0 if brake else 0.0,
                "hand_brake": False,
                "reverse": False,
            },
        },
        "actor_proxies": [
            {
                "actor_id": "lead",
                "track_id": track_id,
                "actor_type": actor_type,
                "center_ego_m": [5.0, 0.0],
                "extent_m": {"x": 2.0, "y": 1.0},
            }
        ],
        "dynamic_bev_proxy": {
            "class_mapping": {"vehicle": 9, "pedestrian": 10},
            "grid": {
                "min_x_m": -8.0,
                "max_x_m": 8.0,
                "min_y_m": -8.0,
                "max_y_m": 8.0,
                "height": 16,
                "width": 16,
                "row_axis": "ego_x_forward",
                "column_axis": "ego_y_right",
            },
            "actor_samples": [
                {
                    "center_class_match": True,
                    "actor_type": actor_type,
                    "track_id": track_id,
                    "center_in_bev_bounds": True,
                    "predicted_center_bev_class": bev_class,
                }
            ],
        },
        "dense_outputs": {
            "path": str(dense),
            "sha256": _sha(dense),
            "encoding": "numpy_npz",
            "required_keys": [
                "bev_semantic_labels",
                "perspective_semantic_labels",
                "depth",
                "target_speed_probabilities",
            ],
        },
        "latency_ms": {"inference": 20.0},
        "synchronization": {
            "frame_id": frame_id,
            "error_ms": 0.0,
            "dynamic_object_sha256": HASH,
        },
        "semantics": {
            "occupancy_evaluation": "dynamic_bev_proxy_only",
            "full_3d_occupancy_ground_truth_available": False,
        },
    }


def _formal_quality_report(root: Path, experiment: dict) -> dict:
    from runtime.render_quality import evaluate_render_quality

    cameras = []
    baseline = np.full((450, 800, 3), 120, dtype=np.uint8)
    mask = np.zeros((450, 800), dtype=np.uint8)
    mask[180:270, 350:450] = 255
    for camera_name in (
        "camera_front",
        "camera_front_left",
        "camera_front_right",
        "camera_back",
        "camera_back_left",
        "camera_back_right",
    ):
        baseline_paths = []
        edited_paths = []
        mask_paths = []
        for frame_index in range(2):
            baseline_path = root / f"{camera_name}.baseline.{frame_index}.png"
            edited_path = root / f"{camera_name}.edited.{frame_index}.png"
            mask_path = root / f"{camera_name}.mask.{frame_index}.png"
            Image.fromarray(baseline).save(baseline_path)
            Image.fromarray(baseline).save(edited_path)
            Image.fromarray(mask).save(mask_path)
            baseline_paths.append(str(baseline_path))
            edited_paths.append(str(edited_path))
            mask_paths.append(str(mask_path))
        cameras.append(
            {
                "camera_name": camera_name,
                "baseline_frames": baseline_paths,
                "edited_frames": edited_paths,
                "actor_masks": mask_paths,
                "mask_provenance": {
                    "kind": "nurec_track_projection",
                    "reliable": True,
                    "source": "actor_mapping.json + calibrated projection",
                    "limitations": [],
                },
            }
        )
    lidar_baseline = []
    lidar_edited = []
    for frame_index in range(2):
        baseline_path = root / f"quality.lidar.baseline.{frame_index}.bin"
        edited_path = root / f"quality.lidar.edited.{frame_index}.bin"
        payload = np.asarray([[1.0, 0.0, 0.0, 0.5]], dtype="<f4").tobytes()
        baseline_path.write_bytes(payload)
        edited_path.write_bytes(payload)
        lidar_baseline.append(baseline_path)
        lidar_edited.append(edited_path)

    def payload_ref(path: Path, *, kind: str, encoding: str) -> dict:
        return {
            "path": str(path),
            "sha256": _sha(path),
            "size_bytes": path.stat().st_size,
            "kind": kind,
            "encoding": encoding,
        }

    front = next(camera for camera in cameras if camera["camera_name"] == "camera_front")
    target_track_id = "c1958768d48640948f6053d04cffd35b"
    consistency_source = {
        "schema_version": "rgb_lidar_actor_change_source_report.v1",
        "status": "passed",
        "experiment": dict(experiment),
        "target_track_id": target_track_id,
        "frame_range": {
            phase: {
                "start_frame_id": 1,
                "end_frame_id": 2,
                "frame_count": 2,
                "start_timestamp_sec": 0.05,
                "end_timestamp_sec": 0.10,
            }
            for phase in ("baseline", "edited")
        },
        "payloads": {
            "rgb": {
                "baseline": [
                    payload_ref(Path(path), kind="rgb", encoding="png")
                    for path in front["baseline_frames"]
                ],
                "edited": [
                    payload_ref(Path(path), kind="rgb", encoding="png")
                    for path in front["edited_frames"]
                ],
            },
            "lidar": {
                "baseline": [
                    payload_ref(path, kind="lidar", encoding="float32_xyzi_little_endian")
                    for path in lidar_baseline
                ],
                "edited": [
                    payload_ref(path, kind="lidar", encoding="float32_xyzi_little_endian")
                    for path in lidar_edited
                ],
            },
        },
        "change_flags": {
            "rgb_actor_changed": False,
            "lidar_actor_changed": False,
        },
    }
    consistency_path = root / "rgb_lidar_same_frame_contract.json"
    consistency_path.write_text(
        json.dumps(consistency_source, allow_nan=False), encoding="utf-8"
    )
    report = evaluate_render_quality(
        {
            "schema_version": "render_quality_evaluation_request.v1",
            "scene_id": experiment["scene_id"],
            "case_id": experiment["case_id"],
            "edit_kind": "original_replay",
            "artifact": {
                "path": "formal/last.usdz",
                "sha256": experiment["artifact_sha256"],
                "immutable": True,
            },
            "experiment": dict(experiment),
            "target_track_id": target_track_id,
            "remote_validation_required": True,
            "rgb_lidar_actor_change": {
                "source_report_ref": {
                    "path": str(consistency_path),
                    "sha256": _sha(consistency_path),
                    "size_bytes": consistency_path.stat().st_size,
                    "schema_version": consistency_source["schema_version"],
                    "status": consistency_source["status"],
                },
            },
            "cameras": cameras,
        }
    )
    return _bind_quality_report(root, report)


def _bind_quality_report(
    root: Path, report: dict, name: str = "render_quality_report.json"
) -> dict:
    report = dict(report)
    report.pop("_bound_report_ref", None)
    quality_path = root / name
    quality_path.write_text(
        json.dumps(report, allow_nan=False), encoding="utf-8"
    )
    report["_bound_report_ref"] = {
        "path": str(quality_path),
        "sha256": _sha(quality_path),
        "size_bytes": quality_path.stat().st_size,
    }
    return report


class TransFuserPPIntermediateTests(unittest.TestCase):
    def test_runtime_dense_writer_uses_evaluator_contract_keys(self):
        from agents.transfuserpp_runtime import TransFuserPPModelRuntime

        with tempfile.TemporaryDirectory() as directory:
            runtime = TransFuserPPModelRuntime.__new__(TransFuserPPModelRuntime)
            runtime.output_dir = Path(directory)
            runtime.np = np
            path = runtime._write_dense_outputs(
                1,
                bev_semantic_labels=np.zeros((2, 2), dtype=np.uint8),
                perspective_semantic_labels=np.zeros((2, 2), dtype=np.uint8),
                depth=np.zeros((1, 2, 2), dtype=np.float32),
                target_speed_probabilities=np.asarray([1.0], dtype=np.float32),
            )
            with np.load(path) as dense:
                self.assertEqual(
                    set(dense.files),
                    {
                        "bev_semantic_labels",
                        "perspective_semantic_labels",
                        "depth",
                        "target_speed_probabilities",
                    },
                )

    def test_trace_evaluation_separates_proxy_from_full_occ(self):
        from metrics.transfuserpp_intermediate import evaluate_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = _record(
                root,
                frame_id=1,
                timestamp=0.05,
                case_id="S0_original_replay",
                labels=np.zeros((16, 16), dtype=np.uint8),
                target_speed=4.0,
                brake=False,
            )
            quality_report = _formal_quality_report(root, record["experiment"])
            result = evaluate_intermediate_trace(
                [record],
                render_quality_report=quality_report,
            )
        self.assertEqual(result["status"], "evaluated")
        self.assertEqual(result["evidence_classification"], "perception_eligible")
        self.assertEqual(result["full_3d_occupancy"]["status"], "unavailable")
        self.assertEqual(result["dynamic_bev_proxy"]["box_detection"]["recall"], 1.0)

    def test_bare_perception_classification_cannot_upgrade_trace(self):
        from metrics.transfuserpp_intermediate import evaluate_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            record = _record(
                Path(directory),
                frame_id=1,
                timestamp=0.05,
                case_id="S0_original_replay",
                labels=np.zeros((16, 16), dtype=np.uint8),
                target_speed=4.0,
                brake=False,
            )
            result = evaluate_intermediate_trace(
                [record], render_quality_classification="perception_eligible"
            )
        self.assertEqual(result["status"], "failed")
        self.assertNotEqual(result["evidence_classification"], "perception_eligible")

    def test_unbound_or_incomplete_quality_report_cannot_upgrade_trace(self):
        from metrics.transfuserpp_intermediate import evaluate_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            record = _record(
                Path(directory),
                frame_id=1,
                timestamp=0.05,
                case_id="S0_original_replay",
                labels=np.zeros((16, 16), dtype=np.uint8),
                target_speed=4.0,
                brake=False,
            )
            result = evaluate_intermediate_trace(
                [record],
                render_quality_report={
                    "schema_version": "render_quality_report.v1",
                    "scene_id": "cc8c0bf57f984915a77078b10eb33198",
                    "case_id": "S0_original_replay",
                    "artifact": {"sha256": HASH},
                    "evidence_classification": "perception_eligible",
                },
            )
        self.assertEqual(result["status"], "failed")
        self.assertIn(
            "render_quality_report_file_binding_required",
            result["fail_closed_reasons"],
        )

    def test_empty_formal_camera_metrics_fail_closed(self):
        from metrics.transfuserpp_intermediate import evaluate_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = _record(
                root,
                frame_id=1,
                timestamp=0.05,
                case_id="S0_original_replay",
                labels=np.zeros((16, 16), dtype=np.uint8),
                target_speed=4.0,
                brake=False,
            )
            quality_report = _formal_quality_report(root, record["experiment"])
            quality_report["cameras"][0]["metrics"] = {}
            quality_report = _bind_quality_report(
                root, quality_report, "empty_metrics.render_quality_report.json"
            )
            result = evaluate_intermediate_trace(
                [record], render_quality_report=quality_report
            )
        self.assertEqual(result["status"], "failed")
        self.assertNotEqual(result["evidence_classification"], "perception_eligible")
        self.assertTrue(
            any(
                reason.startswith("render_quality_camera_metrics_empty:camera_front")
                for reason in result["fail_closed_reasons"]
            )
        )

    def test_formal_quality_identity_and_input_hash_fail_closed(self):
        from metrics.transfuserpp_intermediate import evaluate_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = _record(
                root,
                frame_id=1,
                timestamp=0.05,
                case_id="S0_original_replay",
                labels=np.zeros((16, 16), dtype=np.uint8),
                target_speed=4.0,
                brake=False,
            )
            identity_report = _formal_quality_report(root, record["experiment"])
            identity_report["experiment"]["seed"] = 99
            identity_report = _bind_quality_report(
                root, identity_report, "wrong_identity.render_quality_report.json"
            )
            identity_result = evaluate_intermediate_trace(
                [record], render_quality_report=identity_report
            )

            hash_report = _formal_quality_report(root, record["experiment"])
            edited_ref = hash_report["cameras"][0]["inputs"]["edited_frames"][0]
            Image.new("RGB", (800, 450), (1, 2, 3)).save(edited_ref["path"])
            hash_report = _bind_quality_report(
                root, hash_report, "stale_input_hash.render_quality_report.json"
            )
            hash_result = evaluate_intermediate_trace(
                [record], render_quality_report=hash_report
            )

        self.assertIn(
            "render_quality_experiment_identity_mismatch:seed",
            identity_result["fail_closed_reasons"],
        )
        self.assertTrue(
            any(
                "render_quality_input_ref_invalid:camera_front:edited_frames:0:"
                in reason
                for reason in hash_result["fail_closed_reasons"]
            )
        )

    def test_consistency_evidence_and_harmonizer_never_upgrade_fail_closed(self):
        from metrics.transfuserpp_intermediate import evaluate_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = _record(
                root,
                frame_id=1,
                timestamp=0.05,
                case_id="S0_original_replay",
                labels=np.zeros((16, 16), dtype=np.uint8),
                target_speed=4.0,
                brake=False,
            )
            quality_report = _formal_quality_report(root, record["experiment"])
            quality_report["rgb_lidar_actor_change_consistency"]["source_report_ref"] = None
            quality_report["harmonizer"] = {
                "applied": True,
                "source_evidence_classification": "quality_stress",
                "policy": "never_upgrade_source_evidence",
            }
            quality_report = _bind_quality_report(
                root, quality_report, "invalid_harmonizer.render_quality_report.json"
            )
            result = evaluate_intermediate_trace(
                [record], render_quality_report=quality_report
            )
        self.assertTrue(
            any(
                reason.startswith("render_quality_rgb_lidar_source_report_invalid:")
                for reason in result["fail_closed_reasons"]
            )
        )
        self.assertIn(
            "render_quality_harmonizer_illegal_upgrade",
            result["fail_closed_reasons"],
        )

    def test_formal_gate_rechecks_mutated_consistency_source_report(self):
        from metrics.transfuserpp_intermediate import evaluate_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = _record(
                root,
                frame_id=1,
                timestamp=0.05,
                case_id="S0_original_replay",
                labels=np.zeros((16, 16), dtype=np.uint8),
                target_speed=4.0,
                brake=False,
            )
            quality_report = _formal_quality_report(root, record["experiment"])
            source_ref = quality_report["rgb_lidar_actor_change_consistency"][
                "source_report_ref"
            ]
            source_path = Path(source_ref["path"])
            source_path.write_text(
                source_path.read_text(encoding="utf-8") + "\n", encoding="utf-8"
            )
            quality_report = _bind_quality_report(
                root, quality_report, "mutated_consistency.render_quality_report.json"
            )
            result = evaluate_intermediate_trace(
                [record], render_quality_report=quality_report
            )
        self.assertEqual(result["status"], "failed")
        self.assertTrue(
            any(
                reason.startswith("render_quality_rgb_lidar_source_report_invalid:")
                for reason in result["fail_closed_reasons"]
            )
        )

    def test_counterfactual_comparison_tracks_bev_planning_and_brake_chain(self):
        from metrics.transfuserpp_intermediate import compare_counterfactual_traces

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline_labels = np.zeros((16, 16), dtype=np.uint8)
            edited_labels = baseline_labels.copy()
            edited_labels[11:16, 6:11] = 9
            baseline_pre = _record(
                root,
                frame_id=1,
                timestamp=0.95,
                case_id="S0_original_replay",
                labels=baseline_labels,
                target_speed=6.0,
                brake=False,
            )
            baseline_post = _record(
                root,
                frame_id=2,
                timestamp=1.05,
                case_id="S0_original_replay",
                labels=baseline_labels,
                target_speed=6.0,
                brake=False,
            )
            edited_pre = _record(
                root,
                frame_id=101,
                timestamp=0.95,
                case_id="S2_lead_hard_brake",
                labels=baseline_labels,
                target_speed=6.0,
                brake=False,
            )
            edited_post = _record(
                root,
                frame_id=102,
                timestamp=1.05,
                case_id="S2_lead_hard_brake",
                labels=edited_labels,
                target_speed=0.0,
                brake=True,
            )
            result = compare_counterfactual_traces(
                [baseline_pre, baseline_post],
                [edited_pre, edited_post],
                event_timestamp=1.0,
                expected_case_id="S2_lead_hard_brake",
            )
        self.assertEqual(result["status"], "evaluated")
        self.assertTrue(result["causal_chain"]["target_speed_changed"])
        self.assertTrue(result["causal_chain"]["control_braked_after_event"])
        self.assertIsNotNone(result["bev_change"]["edited_region_change_ratio"])

    def test_pedestrian_response_can_pass_with_speed_reduction_without_hard_brake(self):
        from metrics.transfuserpp_intermediate import compare_counterfactual_traces

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline_labels = np.zeros((16, 16), dtype=np.uint8)
            edited_labels = baseline_labels.copy()
            edited_labels[11:16, 6:11] = 10
            baseline = [
                _record(
                    root,
                    frame_id=index,
                    timestamp=timestamp,
                    case_id="S0_original_replay",
                    labels=baseline_labels,
                    target_speed=5.0,
                    brake=False,
                )
                for index, timestamp in ((1, 0.95), (2, 1.05))
            ]
            edited = [
                _record(
                    root,
                    frame_id=index,
                    timestamp=timestamp,
                    case_id="S4_pedestrian_early_crossing",
                    labels=baseline_labels if timestamp < 1.0 else edited_labels,
                    target_speed=5.0 if timestamp < 1.0 else 3.0,
                    brake=False,
                )
                for index, timestamp in ((101, 0.95), (102, 1.05))
            ]
            edited[1]["outputs"]["control"]["throttle"] = 0.1
            result = compare_counterfactual_traces(
                baseline,
                edited,
                event_timestamp=1.0,
                expected_case_id="S4_pedestrian_early_crossing",
            )
        self.assertEqual(result["status"], "evaluated")
        self.assertIsNone(result["brake_response"]["edited_response_latency_sec"])
        self.assertIsNotNone(
            result["brake_response"]["counterfactual_response_latency_sec"]
        )

    def test_counterfactual_resolves_separate_baseline_and_edited_bundle_roots(self):
        from metrics.transfuserpp_intermediate import compare_counterfactual_traces

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline_root = root / "S0"
            edited_root = root / "S2"
            baseline_root.mkdir()
            edited_root.mkdir()
            labels = np.zeros((16, 16), dtype=np.uint8)
            edited_labels = labels.copy()
            edited_labels[11:16, 6:11] = 9
            baseline = [
                _record(
                    baseline_root,
                    frame_id=index,
                    timestamp=timestamp,
                    case_id="S0_original_replay",
                    labels=labels,
                    target_speed=5.0,
                    brake=False,
                )
                for index, timestamp in ((1, 0.95), (2, 1.05))
            ]
            edited = [
                _record(
                    edited_root,
                    frame_id=index,
                    timestamp=timestamp,
                    case_id="S2_lead_hard_brake",
                    labels=labels if timestamp < 1.0 else edited_labels,
                    target_speed=5.0 if timestamp < 1.0 else 0.0,
                    brake=timestamp >= 1.0,
                )
                for index, timestamp in ((101, 0.95), (102, 1.05))
            ]
            for records in (baseline, edited):
                for record in records:
                    for name in ("camera_front", "lidar_top"):
                        reference = record["inputs"][name]
                        reference["relative_path"] = Path(reference["path"]).name
                        reference["path"] = f"/sim-data/{name}.container-only"
                    dense = record["dense_outputs"]
                    dense["relative_path"] = Path(dense["path"]).name
                    dense["path"] = "/sim-data/dense.container-only.npz"
            result = compare_counterfactual_traces(
                baseline,
                edited,
                event_timestamp=1.0,
                expected_case_id="S2_lead_hard_brake",
                evidence_root=baseline_root,
                edited_evidence_root=edited_root,
            )
        self.assertEqual(result["status"], "evaluated")

    def test_visualizer_writes_derived_panel_without_modifying_rgb(self):
        from runtime.transfuserpp_visualization import render_intermediate_debug_frame

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = _record(
                root,
                frame_id=2,
                timestamp=0.1,
                case_id="S0_original_replay",
                labels=np.zeros((16, 16), dtype=np.uint8),
                target_speed=4.0,
                brake=False,
            )
            source_path = Path(record["inputs"]["camera_front"]["path"])
            before = _sha(source_path)
            record["inputs"]["camera_front"].update(
                path="/sim-data/container-only.jpg",
                relative_path=source_path.name,
            )
            record["dense_outputs"].update(
                path="/sim-data/container-only.npz",
                relative_path=Path(record["dense_outputs"]["path"]).name,
            )
            output = render_intermediate_debug_frame(
                record, root / "panel.png", evidence_root=root
            )
            self.assertTrue(output.is_file())
            self.assertEqual(before, _sha(root / record["inputs"]["camera_front"]["relative_path"]))

    def test_evaluator_resolves_container_relative_evidence_on_host(self):
        from metrics.transfuserpp_intermediate import evaluate_intermediate_trace

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = _record(
                root,
                frame_id=1,
                timestamp=0.05,
                case_id="S0_original_replay",
                labels=np.zeros((16, 16), dtype=np.uint8),
                target_speed=4.0,
                brake=False,
            )
            for name in ("camera_front", "lidar_top"):
                reference = record["inputs"][name]
                reference["relative_path"] = Path(reference["path"]).name
                reference["path"] = f"/sim-data/{name}.container-only"
            dense = record["dense_outputs"]
            dense["relative_path"] = Path(dense["path"]).name
            dense["path"] = "/sim-data/frame.container-only.npz"
            result = evaluate_intermediate_trace(
                [record], evidence_root=root
            )
        self.assertEqual(result["status"], "evaluated")


if __name__ == "__main__":
    unittest.main()
