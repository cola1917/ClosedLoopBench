from __future__ import annotations

import unittest
import tempfile
from pathlib import Path


def _waypoint(road: int, lane: int, x: float, *, junction: bool = False) -> dict:
    return {
        "location": {"x": x, "y": 0.0, "z": 0.0},
        "road_id": road,
        "section_id": 0,
        "lane_id": lane,
        "is_driving_lane": True,
        "lane_width_m": 3.6,
        "is_junction": junction,
    }


class CarlaXodrRuntimeAuditTests(unittest.TestCase):
    def test_waypoint_continuity_lane_membership_and_branch_pass(self) -> None:
        from adapters.carla_xodr_runtime_audit import audit_waypoint_samples

        first = _waypoint(1, -1, 0.0)
        second = _waypoint(1, -1, 5.0)
        junction = _waypoint(2, -1, 10.0, junction=True)
        report = audit_waypoint_samples(
            [
                {
                    "index": 0,
                    "expected": {"x": 0.0, "y": 0.0},
                    "waypoint": first,
                    "next_waypoints": [second],
                    "step_distance_m": 5.0,
                },
                {
                    "index": 1,
                    "expected": {"x": 5.0, "y": 0.0},
                    "waypoint": second,
                    "next_waypoints": [junction],
                    "step_distance_m": 5.0,
                },
                {
                    "index": 2,
                    "expected": {"x": 10.0, "y": 0.0},
                    "waypoint": junction,
                    "next_waypoints": [],
                    "step_distance_m": 0.0,
                },
            ]
        )
        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["lane_membership"]["inside_lane_fraction"], 1.0)
        self.assertEqual(report["waypoint_continuity"]["transition_count"], 2)
        self.assertEqual(report["route_branch"]["transition_count"], 1)

    def test_lane_and_out_of_junction_transition_fail_closed(self) -> None:
        from adapters.carla_xodr_runtime_audit import audit_waypoint_samples

        first = _waypoint(1, -1, 0.0)
        second = _waypoint(2, -1, 5.0)
        second["location"]["y"] = 3.0
        report = audit_waypoint_samples(
            [
                {
                    "index": 0,
                    "expected": {"x": 0.0, "y": 0.0},
                    "waypoint": first,
                    "next_waypoints": [second],
                    "step_distance_m": 5.0,
                },
                {
                    "index": 1,
                    "expected": {"x": 5.0, "y": 0.0},
                    "waypoint": second,
                    "next_waypoints": [],
                    "step_distance_m": 0.0,
                },
            ]
        )
        self.assertEqual(report["status"], "failed")
        self.assertIn("outside_lane_width", report["lane_membership"]["failures"][0]["issues"])
        self.assertEqual(report["route_branch"]["status"], "failed")
        self.assertEqual(
            report["route_branch"]["failures"][0]["issue"],
            "road_or_lane_change_outside_junction",
        )

    def test_route_contract_rejects_waypoint_on_unrelated_road(self) -> None:
        from adapters.carla_xodr_runtime_audit import audit_waypoint_samples

        first = _waypoint(1, -1, 0.0)
        unrelated = _waypoint(99, -1, 5.0)
        report = audit_waypoint_samples(
            [
                {
                    "index": 0,
                    "expected": {"x": 0.0, "y": 0.0},
                    "waypoint": first,
                    "next_waypoints": [unrelated],
                    "step_distance_m": 5.0,
                },
                {
                    "index": 1,
                    "expected": {"x": 5.0, "y": 0.0},
                    "waypoint": unrelated,
                    "next_waypoints": [],
                    "step_distance_m": 0.0,
                },
            ],
            route_contract={
                "road_sequence": ["1", "2"],
                "transition_edges": [
                    {
                        "from_road_id": "1",
                        "to_road_id": "2",
                        "through_junction": True,
                    }
                ],
            },
        )

        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["route_topology"]["status"], "failed")
        self.assertIn(
            "waypoint_road_not_in_declared_route",
            report["route_topology"]["failures"][0]["issues"],
        )

    def test_route_contract_allows_sparse_sample_across_declared_junction(self) -> None:
        from adapters.carla_xodr_runtime_audit import audit_waypoint_samples

        first = _waypoint(1, -1, 0.0)
        last = _waypoint(3, -1, 10.0)
        report = audit_waypoint_samples(
            [
                {
                    "index": 0,
                    "expected": {"x": 0.0, "y": 0.0},
                    "waypoint": first,
                    "next_waypoints": [last],
                    "step_distance_m": 10.0,
                },
                {
                    "index": 1,
                    "expected": {"x": 10.0, "y": 0.0},
                    "waypoint": last,
                    "next_waypoints": [],
                    "step_distance_m": 0.0,
                },
            ],
            route_contract={
                "road_sequence": ["1", "2", "3"],
                "transition_edges": [
                    {
                        "from_road_id": "1",
                        "to_road_id": "2",
                        "through_junction": True,
                    },
                    {
                        "from_road_id": "2",
                        "to_road_id": "3",
                        "through_junction": True,
                    },
                ],
            },
        )

        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["route_topology"]["status"], "passed")
        self.assertEqual(report["route_topology"]["observed_road_sequence"], ["1", "3"])

    def test_load_route_contract_expands_junction_connector(self) -> None:
        from adapters.carla_xodr_runtime_audit import load_route_topology_contract

        xodr = """
        <OpenDRIVE>
          <road id="1" name="inferred_route_source_gap" length="1" junction="-1">
            <link><successor elementType="junction" elementId="7" contactPoint="end"/></link>
            <userData><property name="route_path_order" value="1"/></userData>
          </road>
          <road id="2" name="nuscenes_lane_target" length="1" junction="-1">
            <link><predecessor elementType="junction" elementId="7" contactPoint="start"/></link>
            <userData><property name="route_path_order" value="2"/></userData>
          </road>
          <road id="100" name="inferred_connector_route" length="1" junction="7">
            <link>
              <predecessor elementType="road" elementId="1" contactPoint="end"/>
              <successor elementType="road" elementId="2" contactPoint="start"/>
            </link>
          </road>
          <junction id="7">
            <connection id="1" incomingRoad="1" connectingRoad="100" contactPoint="start">
              <laneLink from="-1" to="-1"/>
            </connection>
          </junction>
        </OpenDRIVE>
        """
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "road.xodr"
            path.write_text(xodr, encoding="utf-8")
            contract = load_route_topology_contract(path)

        self.assertEqual(contract["route_path_road_ids"], ["1", "2"])
        self.assertEqual(contract["road_sequence"], ["1", "100", "2"])
        self.assertTrue(contract["transition_edges"][0]["through_junction"])


if __name__ == "__main__":
    unittest.main()
