import copy
import unittest


def _config():
    trajectory = [
        {"t_sec": float(index), "x": float(index * 5), "y": 0.0, "speed_mps": 5.0}
        for index in range(6)
    ]
    return {
        "scenario_id": "scene",
        "experiment": {
            "scene_id": "cc8c0bf57f984915a77078b10eb33198",
            "scene_version": "formal40k-v1",
        },
        "carla": {},
        "actors": [
            {
                "actor_id": "lead",
                "actor_type": "vehicle",
                "binding": {"nurec_track_id": "c1958768d48640948f6053d04cffd35b"},
                "reference_trajectory": copy.deepcopy(trajectory),
            },
            {
                "actor_id": "walker",
                "actor_type": "pedestrian",
                "binding": {"nurec_track_id": "71603dd1a2ba4e9daf095535e38310ac"},
                "reference_trajectory": [
                    {"t_sec": float(index + 2), "x": 0.0, "y": float(index), "speed_mps": 1.0}
                    for index in range(6)
                ],
            },
        ],
    }


class Scene0061TransFuserPPVariantTests(unittest.TestCase):
    def test_freezes_case_specific_actor_control_modes(self):
        from runtime.scene0061_variants import build_scene0061_variant

        expected = {
            "S0_original_replay": ("replay", "replay"),
            "S2_lead_hard_brake": ("scripted", "replay"),
            "S4_pedestrian_early_crossing": ("replay", "scripted"),
        }
        event_times = {
            "S0_original_replay": None,
            "S2_lead_hard_brake": 2.0,
            "S4_pedestrian_early_crossing": 4.0,
        }
        for case_id, modes in expected.items():
            with self.subTest(case_id=case_id):
                source = _config()
                source["actors"][0]["closed_loop"] = {"ego_responsive": True}
                variant, delta = build_scene0061_variant(
                    source,
                    case_id=case_id,
                    seed=41,
                    event_timestamp_sec=event_times[case_id],
                )
                self.assertEqual(
                    tuple(actor["effective_control_mode"] for actor in variant["actors"]),
                    modes,
                )
                self.assertEqual(
                    tuple(actor["closed_loop"]["ego_responsive"] for actor in variant["actors"]),
                    tuple(mode == "scripted" for mode in modes),
                )
                self.assertEqual(
                    delta["effective_actor_control_modes"],
                    variant["actor_control_contract"]["effective_modes_by_track"],
                )
                for actor, row in zip(
                    variant["actors"], variant["actor_control_contract"]["actors"]
                ):
                    self.assertEqual(
                        row["runner_executor"],
                        actor["control_mode_contract"]["runner_executor"],
                    )

    def test_hard_brake_retimes_only_along_source_corridor(self):
        from runtime.scene0061_variants import build_scene0061_variant

        base = _config()
        variant, delta = build_scene0061_variant(
            base,
            case_id="S2_lead_hard_brake",
            seed=41,
            event_timestamp_sec=2.0,
        )
        edited = variant["actors"][0]["reference_trajectory"]
        self.assertTrue(all(point["y"] == 0.0 for point in edited))
        self.assertGreater(delta["longitudinal_lag_m"], 0.0)
        self.assertFalse(delta["path_geometry_changed"])
        self.assertEqual(base["actors"][0]["reference_trajectory"][3]["x"], 15.0)

    def test_pedestrian_early_crossing_changes_time_not_corridor(self):
        from runtime.scene0061_variants import build_scene0061_variant

        base = _config()
        variant, delta = build_scene0061_variant(
            base,
            case_id="S4_pedestrian_early_crossing",
            seed=42,
            event_timestamp_sec=4.0,
        )
        source = base["actors"][1]["reference_trajectory"]
        edited = variant["actors"][1]["reference_trajectory"]
        self.assertEqual([point["y"] for point in source], [point["y"] for point in edited])
        self.assertEqual(edited[0]["t_sec"], source[0]["t_sec"])
        self.assertEqual(edited[2]["t_sec"], source[2]["t_sec"] - 1.0)
        self.assertTrue(delta["pre_intervention_trajectory_unchanged"])
        self.assertEqual(delta["corridor"], "source_reference_corridor")
        self.assertGreaterEqual(edited[0]["t_sec"], 0.0)
        self.assertEqual(delta["event_timestamp_sec"], 2.0)
        event = variant["counterfactual_event_evidence"]
        self.assertEqual(
            event["event_kind"],
            "baseline_pedestrian_source_corridor_crossing_anchor",
        )
        self.assertEqual(
            event["source_actor_track_id"],
            "71603dd1a2ba4e9daf095535e38310ac",
        )
        self.assertEqual(event["requested_event_timestamp_sec"], 4.0)
        self.assertEqual(event["source_event_pose"], {"t_sec": 4.0, "x": 0.0, "y": 2.0})
        self.assertFalse(event["source_geometry"]["free_space_geometry_used"])
        self.assertEqual(len(event["source_trajectory_sha256"]), 64)

    def test_rejects_negative_seed_and_incomplete_brake_window(self):
        from runtime.scene0061_variants import (
            Scene0061VariantError,
            build_scene0061_variant,
        )

        with self.assertRaisesRegex(Scene0061VariantError, "non-negative"):
            build_scene0061_variant(
                _config(), case_id="S0_original_replay", seed=-1
            )
        with self.assertRaisesRegex(Scene0061VariantError, "complete hard-brake"):
            build_scene0061_variant(
                _config(),
                case_id="S2_lead_hard_brake",
                seed=41,
                event_timestamp_sec=4.5,
            )


if __name__ == "__main__":
    unittest.main()
