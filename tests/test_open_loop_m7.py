import json
import tempfile
import unittest
from pathlib import Path


PINNED_IMAGE = "sha256:d5f814b8aab88bbef08e70a4f915771658221190d2501e2894815977d3db6394"
HASH = "a" * 64


def _identity():
    return {
        "repo_sha256": HASH,
        "checkpoint_sha256": HASH,
        "model_config_sha256": HASH,
        "repo_revision": "b" * 40,
        "carla_agents_sha256": HASH,
        "adapter_source_sha256": HASH,
        "container_image_digest": PINNED_IMAGE,
    }


def _report(seed: int, ade: float) -> dict:
    return {
        "schema_version": "open_loop_multimodal_report.v1",
        "scene_id": "scene-0061",
        "scenario_id": "scenario-0061",
        "scene_version": "open-loop-nurec-stage-b-v1",
        "execution_status": "completed",
        "evidence_classification": "open_loop_multimodal",
        "real_tfpp_checkpoint_loaded": True,
        "real_carla_stage_b_open_loop": True,
        "sensor_source": "nurec_stage_b_6cam_rgb_lidar",
        "ego_pose_source": "scenario_ir_reference_trajectory",
        "control_affects_next_ego_pose": False,
        "claims_m8": False,
        "claims_m9": False,
        "metrics": {
            "ade_m": ade,
            "fde_m": ade + 1.0,
            "lateral_error_p95_m": ade / 2.0,
            "heading_error_p95_deg": None,
            "prediction_point_count": 390,
            "collision_proxy_count": 0,
            "latency_ms": {
                "count": 39,
                "mean_ms": 150.0 + seed,
                "p95_ms": 180.0 + seed,
                "max_ms": 220.0 + seed,
            },
        },
        "frame_sync": {
            "source_frame_count": 39,
            "prediction_frame_count": 39,
            "matched_frame_count": 39,
            "dropped_frame_count": 0,
            "frame_mismatch_count": 0,
            "scored_frame_mismatch_count": 0,
        },
        "tfpp": {"intermediate_count": 39, "fallback_count": 0},
        "nurec": {
            "frame_count": 39,
            "camera_count": 6,
            "lidar_count": 1,
            "dynamic_actor_creation": False,
            "dynamic_object_count": 0,
            "all_frames_rgb6_passed": True,
            "all_frames_lidar_passed": True,
            "all_frames_raw_normalized_lidar_verified": True,
        },
        "runtime_manifest": {
            "execution_status": "prepared",
            "real_checkpoint_loaded": True,
            "identity": _identity(),
        },
        "plugin_identity": {"real_checkpoint_loaded": True},
        "run_id": f"scene0061-m7-seed-{seed}",
        "runtime_config_path": None,
        "observation_trace_path": None,
        "experiment": {
            "scene_id": "scenario-0061",
            "scene_version": "open-loop-nurec-stage-b-v1",
            "case_id": "S0_original_replay",
            "seed": seed,
            "artifact_sha256": HASH,
            "scene_package_sha256": HASH,
            "scenario_ir_sha256": HASH,
            "immutable_matrix_sha256": HASH,
            "source_run_config_sha256": HASH,
            "variant_config_sha256": f"{seed:064x}",
            "run_config_sha256": f"{seed + 100:064x}",
        },
    }


def _evaluation(seed: int) -> dict:
    return {
        "schema_version": "transfuserpp_intermediate_evaluation.v1",
        "status": "evaluated",
        "evidence_classification": "control_only",
        "frame_count": 39,
        "experiment": {
            "scene_id": "scenario-0061",
            "scene_version": "open-loop-nurec-stage-b-v1",
            "case_id": "S0_original_replay",
            "seed": seed,
            "scenario_ir_sha256": HASH,
        },
    }


class OpenLoopM7Tests(unittest.TestCase):
    def test_triplicate_aggregate_has_population_mean_and_variance(self):
        from metrics.open_loop_m7 import aggregate_open_loop_m7

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reports = []
            report_paths = []
            evaluations = []
            evaluation_paths = []
            for seed, ade in zip((41, 43, 47), (4.0, 6.0, 8.0)):
                report = _report(seed, ade)
                report_path = root / f"report-{seed}.json"
                report_path.write_text(json.dumps(report), encoding="utf-8")
                reports.append(report)
                report_paths.append(report_path)
                evaluation = _evaluation(seed)
                evaluation_path = root / f"intermediate-{seed}.json"
                evaluation_path.write_text(json.dumps(evaluation), encoding="utf-8")
                evaluations.append(evaluation)
                evaluation_paths.append(evaluation_path)

            result = aggregate_open_loop_m7(
                reports,
                report_paths=report_paths,
                intermediate_evaluations=evaluations,
                intermediate_evaluation_paths=evaluation_paths,
            )

        self.assertEqual(result["acceptance_status"], "passed")
        self.assertEqual(result["seeds"], [41, 43, 47])
        summary = result["metrics"]["summary"]["metrics.ade_m"]
        self.assertEqual(summary["mean"], 6.0)
        self.assertAlmostEqual(summary["variance"], 8.0 / 3.0)
        self.assertEqual(summary["available_count"], 3)

    def test_incomplete_or_fallback_seed_is_rejected(self):
        from metrics.open_loop_m7 import OpenLoopM7Error, aggregate_open_loop_m7

        reports = [_report(seed, 5.0) for seed in (41, 43, 47)]
        reports[1]["tfpp"]["fallback_count"] = 1
        evaluations = [_evaluation(seed) for seed in (41, 43, 47)]
        with self.assertRaisesRegex(OpenLoopM7Error, "fallback_count"):
            aggregate_open_loop_m7(reports, intermediate_evaluations=evaluations)


if __name__ == "__main__":
    unittest.main()
