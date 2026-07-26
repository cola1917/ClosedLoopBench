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
        "evaluation": {
            "criteria": [
                {"name": "min_ttc", "metric": "min_ttc", "op": ">=", "value": 1.0}
            ]
        },
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
                    {
                        "t_sec": float(index + 2),
                        "x": 0.0,
                        "y": float(index),
                        "speed_mps": 1.0,
                    }
                    for index in range(6)
                ],
            },
        ],
    }


class KpiGateTranslationTests(unittest.TestCase):
    def test_threshold_and_availability_entries_translate(self):
        from runtime.scene0061_variants import kpi_gate_criteria

        criteria = kpi_gate_criteria(
            ["collision_count==0", "route_progress>=0.95", "min_ttc_available"]
        )
        self.assertEqual(
            criteria,
            [
                {
                    "name": "collision_count",
                    "metric": "collision_count",
                    "op": "==",
                    "value": 0,
                },
                {
                    "name": "route_progress",
                    "metric": "route_progress",
                    "op": ">=",
                    "value": 0.95,
                },
                {
                    "name": "min_ttc_available",
                    "metric": "min_ttc",
                    "op": "available",
                    "value": True,
                },
            ],
        )

    def test_unsupported_entry_is_rejected(self):
        from runtime.scene0061_variants import (
            Scene0061VariantError,
            kpi_gate_criteria,
        )

        with self.assertRaises(Scene0061VariantError):
            kpi_gate_criteria(["route_progress approximately 1"])

    def test_variant_replaces_base_evaluation_with_case_gate(self):
        from runtime.scene0061_variants import build_scene0061_variant

        variant, _ = build_scene0061_variant(
            _config(),
            case_id="S2_lead_hard_brake",
            seed=41,
            event_timestamp_sec=2.0,
        )
        ops = {
            criterion["metric"]: criterion["op"]
            for criterion in variant["evaluation"]["criteria"]
        }
        # S2 deliberately produces a low TTC; the base min_ttc>=1.0 threshold
        # must be replaced by the case's availability gate.
        self.assertEqual(ops.get("min_ttc"), "available")
        self.assertEqual(ops.get("collision_count"), "==")
        self.assertEqual(ops.get("route_progress"), ">=")
        self.assertEqual(
            variant["evaluation_provenance"]["case_id"], "S2_lead_hard_brake"
        )
        self.assertIn(
            "min_ttc_available", variant["evaluation_provenance"]["kpi_gate"]
        )


class AvailabilityCriterionTests(unittest.TestCase):
    def _summary_rows(self, min_ttc):
        summary = {
            "collision_count": 0,
            "route_progress": 1.0,
            "min_ttc": min_ttc,
        }
        rows = [
            {
                "collision": False,
                "route_progress": 1.0,
                **({"min_ttc": min_ttc} if min_ttc is not None else {}),
            }
        ]
        return summary, rows

    def _run_config(self):
        return {
            "evaluation": {
                "criteria": [
                    {
                        "name": "min_ttc_available",
                        "metric": "min_ttc",
                        "op": "available",
                        "value": True,
                    }
                ]
            }
        }

    def test_available_metric_passes_even_below_nominal_threshold(self):
        from metrics.criteria import evaluate_report

        summary, rows = self._summary_rows(0.7)
        result = evaluate_report(
            self._run_config(), summary, rows, "interactive_closed_loop"
        )
        self.assertEqual(result["overall_result"], "pass")
        self.assertEqual(result["criteria"][0]["result"], "pass")

    def test_missing_metric_fails_availability_gate(self):
        from metrics.criteria import evaluate_report

        summary, rows = self._summary_rows(None)
        result = evaluate_report(
            self._run_config(), summary, rows, "interactive_closed_loop"
        )
        self.assertEqual(result["overall_result"], "fail")
        self.assertEqual(result["criteria"][0]["result"], "fail")


if __name__ == "__main__":
    unittest.main()
