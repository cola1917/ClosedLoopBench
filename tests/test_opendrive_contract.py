from __future__ import annotations

import hashlib
import tempfile
import unittest
import json
from pathlib import Path


def _valid_junction_xodr() -> str:
    return (
        "<OpenDRIVE>"
        "<road id='1' name='nuscenes_lane_in' length='10' junction='-1'>"
        "<link><successor elementType='junction' elementId='1' contactPoint='end'/></link>"
        "<planView><geometry x='0' y='0' hdg='0' length='10'><line/></geometry></planView>"
        "</road>"
        "<road id='2' name='nuscenes_lane_out' length='10' junction='-1'>"
        "<link><predecessor elementType='junction' elementId='1' contactPoint='start'/></link>"
        "<planView><geometry x='11' y='0' hdg='0' length='10'><line/></geometry></planView>"
        "</road>"
        "<road id='1001' name='inferred_connector_1_in_to_out' length='1' junction='1'>"
        "<link><predecessor elementType='road' elementId='1' contactPoint='end'/>"
        "<successor elementType='road' elementId='2' contactPoint='start'/></link>"
        "<planView><geometry x='10' y='0' hdg='0' length='1'><line/></geometry></planView>"
        "</road>"
        "<junction id='1'><connection id='1' incomingRoad='1' connectingRoad='1001' contactPoint='start'>"
        "<laneLink from='-1' to='-1'/></connection></junction>"
        "</OpenDRIVE>"
    )


class OpenDriveContractTests(unittest.TestCase):
    def _write(self, xml: str) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "road.xodr"
        path.write_text(xml, encoding="utf-8")
        return path

    def test_valid_branching_connector_is_traversable(self) -> None:
        from adapters.opendrive_contract import validate_topology_artifact

        path = self._write(_valid_junction_xodr())
        expected_sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        summary = validate_topology_artifact(
            path,
            expected_sha256=expected_sha256,
            expected_ego_corridor_count=0,
            require_map_topology=True,
            require_junction_topology=True,
        )
        self.assertEqual(summary["road_count"], 3)
        self.assertEqual(summary["junction_connection_count"], 1)
        self.assertEqual(summary["network_component_count"], 1)
        self.assertEqual(summary["largest_network_component_road_count"], 3)
        self.assertEqual(summary["network_connectivity_status"], "connected")
        self.assertEqual(summary["warnings"], [])
        self.assertEqual(summary["artifact_sha256"], expected_sha256)
        self.assertEqual(summary["expected_artifact_sha256"], expected_sha256)
        self.assertEqual(summary["status"], "passed")

    def test_expected_artifact_sha256_rejects_a_different_multi_road_file(self) -> None:
        from adapters.opendrive_contract import OpenDriveContractError, validate_topology_artifact

        with self.assertRaisesRegex(OpenDriveContractError, "SHA-256 mismatch"):
            validate_topology_artifact(
                self._write(_valid_junction_xodr()),
                expected_sha256="0" * 64,
                expected_ego_corridor_count=0,
                require_map_topology=True,
                require_junction_topology=True,
            )

    def test_missing_connector_successor_fails_closed(self) -> None:
        from adapters.opendrive_contract import OpenDriveContractError, validate_topology_artifact

        xml = _valid_junction_xodr().replace(
            "<successor elementType='road' elementId='2' contactPoint='start'/>",
            "",
        )
        with self.assertRaisesRegex(OpenDriveContractError, "outgoing road"):
            validate_topology_artifact(
                self._write(xml),
                expected_ego_corridor_count=0,
                require_map_topology=True,
                require_junction_topology=True,
            )

    def test_corridor_only_artifact_is_not_a_map(self) -> None:
        from adapters.opendrive_contract import OpenDriveContractError, validate_topology_artifact

        xml = (
            "<OpenDRIVE><road id='1000' name='ego_route_corridor' length='1' junction='-1'>"
            "<planView><geometry x='0' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "</road></OpenDRIVE>"
        )
        with self.assertRaisesRegex(OpenDriveContractError, "single-road"):
            validate_topology_artifact(
                self._write(xml),
                expected_ego_corridor_count=1,
                require_map_topology=True,
            )

    def test_disconnected_network_can_be_required_to_fail(self) -> None:
        from adapters.opendrive_contract import OpenDriveContractError, validate_topology_artifact

        xml = (
            "<OpenDRIVE>"
            "<road id='1' name='nuscenes_lane_a' length='1' junction='-1'>"
            "<planView><geometry x='0' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "</road>"
            "<road id='2' name='nuscenes_lane_b' length='1' junction='-1'>"
            "<planView><geometry x='10' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "</road>"
            "</OpenDRIVE>"
        )
        with self.assertRaisesRegex(OpenDriveContractError, "at most 1 component"):
            validate_topology_artifact(
                self._write(xml),
                expected_ego_corridor_count=0,
                require_map_topology=True,
                max_network_components=1,
            )

    def test_isolated_map_lane_requires_explicit_boundary_metadata(self) -> None:
        from adapters.opendrive_contract import OpenDriveContractError, validate_topology_artifact

        xml = (
            "<OpenDRIVE>"
            "<road id='1' name='nuscenes_lane_a' length='1' junction='-1'>"
            "<planView><geometry x='0' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "</road>"
            "<road id='2' name='nuscenes_lane_b' length='1' junction='-1'>"
            "<planView><geometry x='10' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "</road>"
            "</OpenDRIVE>"
        )
        with self.assertRaisesRegex(OpenDriveContractError, "boundary topology audit"):
            validate_topology_artifact(
                self._write(xml),
                expected_ego_corridor_count=0,
                require_map_topology=True,
                require_boundary_audit=True,
            )

    def test_isolated_map_lane_can_be_declared_source_boundary(self) -> None:
        from adapters.opendrive_contract import validate_topology_artifact

        xml = (
            "<OpenDRIVE>"
            "<road id='1' name='nuscenes_lane_a' length='1' junction='-1'>"
            "<planView><geometry x='0' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "<userData><property name='topology_boundary' value='true'/></userData>"
            "</road>"
            "<road id='2' name='nuscenes_lane_b' length='1' junction='-1'>"
            "<planView><geometry x='10' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "<userData><property name='topology_boundary' value='true'/></userData>"
            "</road>"
            "</OpenDRIVE>"
        )
        summary = validate_topology_artifact(
            self._write(xml),
            expected_ego_corridor_count=0,
            require_map_topology=False,
            require_boundary_audit=True,
        )
        self.assertEqual(summary["isolated_map_lane_boundary_count"], 2)
        self.assertEqual(summary["isolated_map_lane_unclassified_count"], 0)
        self.assertEqual(summary["isolated_map_lane_boundary_status"], "passed")

    def test_route_chain_must_cover_ego_trajectory(self) -> None:
        from adapters.opendrive_contract import validate_topology_artifact

        xml = (
            "<OpenDRIVE>"
            "<road id='1' name='nuscenes_lane_a' length='1' junction='-1'>"
            "<planView><geometry x='0' y='2' hdg='0' length='1'><line/></geometry></planView>"
            "</road>"
            "<road id='2' name='nuscenes_lane_b' length='1' junction='-1'>"
            "<planView><geometry x='10' y='2' hdg='0' length='1'><line/></geometry></planView>"
            "</road>"
            "<road id='2001' name='inferred_route_test_a' length='5' junction='-1'>"
            "<link><successor elementType='road' elementId='2002' contactPoint='start'/></link>"
            "<planView><geometry x='0' y='0' hdg='0' length='5'><line/></geometry></planView>"
            "</road>"
            "<road id='2002' name='inferred_route_test_b' length='5' junction='-1'>"
            "<link><predecessor elementType='road' elementId='2001' contactPoint='end'/></link>"
            "<planView><geometry x='5' y='0' hdg='0' length='5'><line/></geometry></planView>"
            "</road>"
            "</OpenDRIVE>"
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xodr = root / "road.xodr"
            scenario_ir = root / "scene_ir.json"
            xodr.write_text(xml, encoding="utf-8")
            scenario_ir.write_text(
                json.dumps(
                    {
                        "coordinate_frame": {"units": {"yaw": "degree"}},
                        "ego": {
                            "reference_trajectory": [
                                {"x": 0.0, "y": 0.0, "yaw": 0.0},
                                {"x": 5.0, "y": 0.0, "yaw": 0.0},
                                {"x": 10.0, "y": 0.0, "yaw": 0.0},
                            ]
                        },
                    }
                ),
                encoding="utf-8",
            )
            summary = validate_topology_artifact(
                xodr,
                scenario_ir_path=scenario_ir,
                require_map_topology=True,
                require_route_chain=True,
                require_ego_route_coverage=True,
            )
        self.assertEqual(summary["route_chain_status"], "passed")
        self.assertEqual(summary["route_chain_road_ids"], ["2001", "2002"])
        self.assertEqual(summary["ego_route_coverage_status"], "passed")
        self.assertEqual(summary["ego_route_missing_sample_count"], 0)

    def test_route_chain_can_traverse_junction_and_join_map_graph(self) -> None:
        from adapters.opendrive_contract import validate_topology_artifact

        xml = (
            "<OpenDRIVE>"
            "<road id='1' name='nuscenes_lane_in' length='10' junction='-1'>"
            "<link><successor elementType='junction' elementId='1' contactPoint='end'/></link>"
            "<planView><geometry x='0' y='0' hdg='0' length='10'><line/></geometry></planView>"
            "</road>"
            "<road id='2' name='nuscenes_lane_out' length='10' junction='-1'>"
            "<link><predecessor elementType='junction' elementId='1' contactPoint='start'/></link>"
            "<planView><geometry x='11' y='0' hdg='0' length='10'><line/></geometry></planView>"
            "</road>"
            "<road id='1001' name='inferred_connector_1_map' length='1' junction='1'>"
            "<link><predecessor elementType='road' elementId='1' contactPoint='end'/>"
            "<successor elementType='road' elementId='2' contactPoint='start'/></link>"
            "<planView><geometry x='10' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "</road>"
            "<road id='2001' name='inferred_route_a' length='5' junction='-1'>"
            "<link><successor elementType='junction' elementId='1' contactPoint='end'/></link>"
            "<planView><geometry x='0' y='0' hdg='0' length='5'><line/></geometry></planView>"
            "</road>"
            "<road id='2002' name='inferred_route_b' length='5' junction='-1'>"
            "<link><predecessor elementType='junction' elementId='1' contactPoint='start'/>"
            "<successor elementType='junction' elementId='1' contactPoint='end'/></link>"
            "<planView><geometry x='5' y='0' hdg='0' length='5'><line/></geometry></planView>"
            "</road>"
            "<road id='1002' name='inferred_connector_route_chain' length='0.1' junction='1'>"
            "<link><predecessor elementType='road' elementId='2001' contactPoint='end'/>"
            "<successor elementType='road' elementId='2002' contactPoint='start'/></link>"
            "<planView><geometry x='5' y='0' hdg='0' length='0.1'><line/></geometry></planView>"
            "</road>"
            "<road id='1003' name='inferred_connector_route_to_map' length='1' junction='1'>"
            "<link><predecessor elementType='road' elementId='2002' contactPoint='end'/>"
            "<successor elementType='road' elementId='2' contactPoint='start'/></link>"
            "<planView><geometry x='10' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "</road>"
            "<junction id='1'>"
            "<connection id='1' incomingRoad='1' connectingRoad='1001' contactPoint='start'><laneLink from='-1' to='-1'/></connection>"
            "<connection id='2' incomingRoad='2001' connectingRoad='1002' contactPoint='start'><laneLink from='-1' to='-1'/></connection>"
            "<connection id='3' incomingRoad='2002' connectingRoad='1003' contactPoint='start'><laneLink from='-1' to='-1'/></connection>"
            "</junction>"
            "</OpenDRIVE>"
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xodr = root / "road.xodr"
            scenario_ir = root / "scene_ir.json"
            xodr.write_text(xml, encoding="utf-8")
            scenario_ir.write_text(
                json.dumps(
                    {
                        "coordinate_frame": {"units": {"yaw": "degree"}},
                        "ego": {
                            "reference_trajectory": [
                                {"x": 0.0, "y": 0.0, "yaw": 0.0},
                                {"x": 5.0, "y": 0.0, "yaw": 0.0},
                                {"x": 10.0, "y": 0.0, "yaw": 0.0},
                            ]
                        },
                    }
                ),
                encoding="utf-8",
            )
            summary = validate_topology_artifact(
                xodr,
                scenario_ir_path=scenario_ir,
                require_map_topology=True,
                require_junction_topology=True,
                require_route_chain=True,
                require_route_map_integration=True,
                require_ego_route_coverage=True,
            )
        self.assertEqual(summary["route_chain_status"], "passed")
        self.assertEqual(summary["route_chain_link_count"], 1)
        self.assertEqual(summary["route_map_integration_status"], "passed")
        self.assertEqual(summary["route_map_junction_connection_count"], 1)
        self.assertEqual(summary["route_map_linked_route_road_count"], 1)
        self.assertEqual(summary["route_map_linked_route_road_ratio"], 0.5)
        self.assertEqual(summary["route_to_route_junction_connection_count"], 1)

    def test_route_source_audit_requires_explicit_synthetic_metadata(self) -> None:
        from adapters.opendrive_contract import OpenDriveContractError, validate_topology_artifact

        xml = (
            "<OpenDRIVE>"
            "<road id='1' name='nuscenes_lane_in' length='1' junction='-1'>"
            "<planView><geometry x='0' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "</road>"
            "<road id='2001' name='inferred_route_gap' length='1' junction='-1'>"
            "<planView><geometry x='0' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "</road>"
            "</OpenDRIVE>"
        )
        with self.assertRaisesRegex(OpenDriveContractError, "source-gap metadata"):
            validate_topology_artifact(
                self._write(xml),
                require_map_topology=False,
                require_route_source_audit=True,
            )

    def test_declared_mixed_route_path_validates_map_connector_order(self) -> None:
        from adapters.opendrive_contract import validate_topology_artifact

        xml = (
            "<OpenDRIVE>"
            "<road id='1' name='nuscenes_lane_in' length='10' junction='-1'>"
            "<link><successor elementType='junction' elementId='1' contactPoint='end'/></link>"
            "<planView><geometry x='0' y='0' hdg='0' length='10'><line/></geometry></planView>"
            "<userData><property name='route_path_order' value='1'/>"
            "<property name='route_path_kind' value='map_lane'/></userData>"
            "</road>"
            "<road id='1001' name='inferred_connector_1_in_to_out' length='1' junction='1'>"
            "<link><predecessor elementType='road' elementId='1' contactPoint='end'/>"
            "<successor elementType='road' elementId='2' contactPoint='start'/></link>"
            "<planView><geometry x='10' y='0' hdg='0' length='1'><line/></geometry></planView>"
            "<userData><property name='route_path_order' value='2'/>"
            "<property name='route_path_kind' value='map_connector'/></userData>"
            "</road>"
            "<road id='2' name='nuscenes_lane_out' length='10' junction='-1'>"
            "<link><predecessor elementType='junction' elementId='1' contactPoint='start'/></link>"
            "<planView><geometry x='11' y='0' hdg='0' length='10'><line/></geometry></planView>"
            "<userData><property name='route_path_order' value='3'/>"
            "<property name='route_path_kind' value='map_lane'/></userData>"
            "</road>"
            "<junction id='1'><connection id='1' incomingRoad='1' connectingRoad='1001' contactPoint='start'>"
            "<laneLink from='-1' to='-1'/></connection></junction>"
            "</OpenDRIVE>"
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xodr = root / "road.xodr"
            scenario_ir = root / "scene_ir.json"
            xodr.write_text(xml, encoding="utf-8")
            scenario_ir.write_text(
                json.dumps(
                    {
                        "coordinate_frame": {"units": {"yaw": "degree"}},
                        "ego": {
                            "reference_trajectory": [
                                {"x": 0.0, "y": 0.0, "yaw": 0.0},
                                {"x": 10.0, "y": 0.0, "yaw": 0.0},
                                {"x": 21.0, "y": 0.0, "yaw": 0.0},
                            ]
                        },
                    }
                ),
                encoding="utf-8",
            )
            summary = validate_topology_artifact(
                xodr,
                scenario_ir_path=scenario_ir,
                require_map_topology=True,
                require_junction_topology=True,
                require_route_chain=True,
                require_route_map_integration=True,
                require_ego_route_coverage=True,
            )
        self.assertEqual(summary["route_chain_status"], "passed")
        self.assertEqual(summary["route_chain_road_ids"], ["1", "1001", "2"])
        self.assertEqual(summary["route_map_geometry_road_count"], 3)
        self.assertEqual(summary["route_source_gap_road_count_in_path"], 0)


if __name__ == "__main__":
    unittest.main()
