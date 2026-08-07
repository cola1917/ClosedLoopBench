import unittest


def _scenario_ir():
    return {
        "schema_version": "scenario_ir.v1",
        "scenario_id": "scene-metrics-test",
        "source": {"scene_name": "scene-metrics", "version": "v1", "scene_token": "scene-metrics-test"},
        "ego": {
            "reference_trajectory": [
                {"t_sec": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
                {"t_sec": 1.0, "x": 10.0, "y": 0.0, "yaw": 0.0},
                {"t_sec": 2.0, "x": 20.0, "y": 0.0, "yaw": 0.0},
            ]
        },
        "actors": [
            {
                "actor_id": "blocking-vehicle",
                "dimensions": {"length": 4.0, "width": 2.0},
                "reference_trajectory": [
                    {"t_sec": 0.0, "x": 50.0, "y": 50.0, "yaw": 0.0},
                    {"t_sec": 2.0, "x": 50.0, "y": 50.0, "yaw": 0.0},
                ],
            }
        ],
    }


class OpenLoopMetricsTests(unittest.TestCase):
    def test_scores_ade_fde_lateral_heading_and_latency(self):
        from metrics.open_loop import score_open_loop_predictions

        report = score_open_loop_predictions(
            _scenario_ir(),
            {
                "frames": [
                    {
                        "frame_id": 0,
                        "observation_frame_id": 0,
                        "inference_ms": 4.0,
                        "predicted_waypoints": [
                            {"horizon_sec": 1.0, "x": 11.0, "y": 2.0, "yaw": 5.0},
                            {"horizon_sec": 2.0, "x": 21.0, "y": 2.0, "yaw": 5.0},
                        ],
                    }
                ]
            },
        )

        self.assertEqual(report["execution_status"], "completed")
        self.assertEqual(report["evidence_classification"], "open_loop_multimodal")
        self.assertAlmostEqual(report["metrics"]["ade_m"], 5.0**0.5)
        self.assertAlmostEqual(report["metrics"]["fde_m"], 5.0**0.5)
        self.assertAlmostEqual(report["metrics"]["lateral_error_p95_m"], 2.0)
        self.assertAlmostEqual(report["metrics"]["heading_error_p95_deg"], 5.0)
        self.assertEqual(report["metrics"]["latency_ms"]["mean_ms"], 4.0)
        self.assertEqual(report["frame_sync"]["scored_frame_mismatch_count"], 0)

    def test_frame_mismatch_and_collision_proxy_are_explicit(self):
        from metrics.open_loop import score_open_loop_predictions

        scenario = _scenario_ir()
        scenario["actors"][0]["reference_trajectory"] = [
            {"t_sec": 0.0, "x": 10.0, "y": 0.0, "yaw": 0.0},
            {"t_sec": 2.0, "x": 20.0, "y": 0.0, "yaw": 0.0},
        ]
        report = score_open_loop_predictions(
            scenario,
            {
                "frames": [
                    {
                        "frame_id": 0,
                        "observation_frame_id": 4,
                        "predicted_waypoints": [{"horizon_sec": 1.0, "x": 10.0, "y": 0.0}],
                    }
                ]
            },
        )

        self.assertEqual(report["execution_status"], "failed")
        self.assertEqual(report["frame_sync"]["frame_mismatch_count"], 1)
        self.assertEqual(report["metrics"]["collision_proxy_count"], 0)

    def test_report_fail_closed_flags_cannot_be_relabelled(self):
        from metrics.open_loop import validate_open_loop_report

        report = {
            "schema_version": "open_loop_multimodal_report.v1",
            "scene_id": "scene",
            "scenario_id": "scenario",
            "execution_status": "completed",
            "evidence_classification": "open_loop_multimodal",
            "ego_pose_source": "scenario_ir_reference_trajectory",
            "control_affects_next_ego_pose": True,
            "claims_m8": False,
            "claims_m9": False,
            "metrics": {},
            "frame_sync": {"scored_frame_mismatch_count": 0},
        }
        with self.assertRaisesRegex(ValueError, "control_affects_next_ego_pose"):
            validate_open_loop_report(report)


if __name__ == "__main__":
    unittest.main()
