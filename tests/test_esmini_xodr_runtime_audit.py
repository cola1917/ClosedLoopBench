from __future__ import annotations

import unittest


class EsminiXodrRuntimeAuditTests(unittest.TestCase):
    def test_parse_odrplot_lanes_collects_unique_road_ids(self) -> None:
        from adapters.esmini_xodr_runtime_audit import parse_odrplot_lanes

        road_ids, sample_count = parse_odrplot_lanes(
            "Created output.csv\n"
            "lane, 1, 0, -1, driving\n"
            "lane, 1, 0, -1, driving\n"
            "lane, 2001, 0, -1, driving\n"
        )

        self.assertEqual(road_ids, {"1", "2001"})
        self.assertEqual(sample_count, 3)

    def test_missing_materialized_road_fails_closed(self) -> None:
        from adapters.esmini_xodr_runtime_audit import evaluate_odrplot_result

        report = evaluate_odrplot_result(
            road_ids={"1", "2"},
            sampled_road_ids={"1"},
            lane_sample_count=1,
            returncode=0,
            command=["odrplot"],
            stdout="",
            stderr="",
            artifact_sha256="a" * 64,
            expected_sha256="a" * 64,
        )

        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["missing_road_ids"], ["2"])

    def test_all_materialized_roads_and_hash_pass(self) -> None:
        from adapters.esmini_xodr_runtime_audit import evaluate_odrplot_result

        report = evaluate_odrplot_result(
            road_ids={"1", "2"},
            sampled_road_ids={"1", "2"},
            lane_sample_count=4,
            returncode=0,
            command=["odrplot"],
            stdout="",
            stderr="",
            artifact_sha256="b" * 64,
            expected_sha256="b" * 64,
        )

        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["sampled_road_count"], 2)


if __name__ == "__main__":
    unittest.main()
