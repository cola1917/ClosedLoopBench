from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET


def _lane_fixture(prefix: str, x0: float, x1: float, y: float) -> tuple[dict, list[dict], list[dict], list[dict]]:
    node_tokens = [f"{prefix}{index}" for index in range(4)]
    nodes = [
        {"token": node_tokens[0], "x": x0, "y": y - 2.0},
        {"token": node_tokens[1], "x": x1, "y": y - 2.0},
        {"token": node_tokens[2], "x": x1, "y": y + 2.0},
        {"token": node_tokens[3], "x": x0, "y": y + 2.0},
    ]
    from_token, to_token = f"{prefix}-from", f"{prefix}-to"
    lines = [
        {"token": from_token, "node_tokens": [node_tokens[0], node_tokens[3]]},
        {"token": to_token, "node_tokens": [node_tokens[1], node_tokens[2]]},
    ]
    polygon_token = f"{prefix}-polygon"
    polygons = [{"token": polygon_token, "exterior_node_tokens": node_tokens, "holes": []}]
    lane = {
        "token": f"{prefix}-lane",
        "polygon_token": polygon_token,
        "from_edge_line_token": from_token,
        "to_edge_line_token": to_token,
        "lane_type": "CAR",
    }
    return lane, nodes, lines, polygons


def _fixture() -> tuple[dict, dict]:
    lanes = []
    nodes = []
    lines = []
    polygons = []
    for args in (("a", 0.0, 10.0, 0.0), ("b", 14.0, 24.0, 0.0), ("c", 14.0, 24.0, 3.0)):
        lane, lane_nodes, lane_lines, lane_polygons = _lane_fixture(*args)
        lanes.append(lane)
        nodes.extend(lane_nodes)
        lines.extend(lane_lines)
        polygons.extend(lane_polygons)
    scenario_ir = {
        "scenario_id": "topology-test",
        "coordinate_frame": {
            "origin_global_translation": [0.0, 0.0, 0.0],
            "origin_global_yaw_deg": 0.0,
        },
        "ego": {
            "reference_trajectory": [
                {"x": 0.0, "y": 0.0, "yaw": 0.0},
                {"x": 24.0, "y": 0.0, "yaw": 0.0},
            ]
        },
        "actors": [],
    }
    return scenario_ir, {"node": nodes, "line": lines, "polygon": polygons, "lane": lanes}


class NuScenesTopologyOpenDriveTests(unittest.TestCase):
    def test_block_pair_transitions_use_one_to_one_lane_matching(self):
        from adapters.nuscenes_topology_opendrive import (
            _prune_block_pair_transitions,
        )

        transitions = [
            {
                "incoming": incoming,
                "outgoing": outgoing,
                "incoming_block": "block-a",
                "outgoing_block": "block-b",
                "distance_m": distance,
                "turn_deg": 0.0,
            }
            for incoming, outgoing, distance in (
                ("in-1", "out-1", 1.0),
                ("in-1", "out-2", 5.0),
                ("in-2", "out-1", 4.0),
                ("in-2", "out-2", 1.0),
            )
        ]

        result = _prune_block_pair_transitions(transitions)

        self.assertEqual(
            {(item["incoming"], item["outgoing"]) for item in result},
            {("in-1", "out-1"), ("in-2", "out-2")},
        )

    def test_boundary_extension_adds_only_a_compatible_unselected_lane(self):
        from adapters.nuscenes_topology_opendrive import _extend_selected_boundary_lanes

        selected = {
            "token": "selected",
            "points": [(0.0, 0.0), (10.0, 0.0)],
            "source_road_block_token": "block-a",
        }
        compatible = {
            "token": "compatible",
            "points": [(10.0, 0.0), (20.0, 0.0)],
            "source_road_block_token": "block-b",
        }
        reverse_parallel = {
            "token": "reverse-parallel",
            "points": [(10.0, 3.0), (0.0, 3.0)],
            "source_road_block_token": "block-c",
        }

        additions = _extend_selected_boundary_lanes(
            [selected],
            [selected, compatible, reverse_parallel],
            selected_tokens={"selected"},
            connection_tolerance_m=8.0,
            max_turn_deg=135.0,
        )

        self.assertEqual([lane["token"] for lane in additions], ["compatible"])

    def test_same_source_road_block_lanes_are_not_connected(self):
        from adapters.nuscenes_topology_opendrive import _infer_transitions

        lanes = [
            {
                "token": "incoming",
                "points": [(0.0, 0.0), (10.0, 0.0)],
                "width": 3.5,
                "source_road_block_token": "block-a",
            },
            {
                "token": "parallel",
                "points": [(10.0, 3.0), (20.0, 3.0)],
                "width": 3.5,
                "source_road_block_token": "block-a",
            },
            {
                "token": "continuation",
                "points": [(10.0, 0.0), (20.0, 0.0)],
                "width": 3.5,
                "source_road_block_token": "block-b",
            },
        ]

        transitions = _infer_transitions(
            lanes,
            connection_tolerance_m=5.0,
            boundary_connection_tolerance_m=None,
            boundary_region_tolerance_m=5.0,
            boundary_regions=[],
            max_turn_deg=135.0,
        )

        pairs = {(item["incoming"], item["outgoing"]) for item in transitions}
        self.assertIn(("incoming", "continuation"), pairs)
        self.assertNotIn(("incoming", "parallel"), pairs)

    def test_long_transition_requires_source_evidence(self):
        from adapters.nuscenes_topology_opendrive import _infer_transitions

        lanes = [
            {
                "token": "incoming",
                "points": [(0.0, 0.0), (10.0, 0.0)],
                "width": 3.5,
                "source_road_block_token": "block-a",
            },
            {
                "token": "outgoing",
                "points": [(22.0, 0.0), (32.0, 0.0)],
                "width": 3.5,
                "source_road_block_token": "block-b",
            },
        ]

        transitions = _infer_transitions(
            lanes,
            connection_tolerance_m=20.0,
            boundary_connection_tolerance_m=20.0,
            boundary_region_tolerance_m=5.0,
            boundary_regions=[],
            max_turn_deg=135.0,
        )

        self.assertEqual(transitions, [])

    def test_long_transition_accepts_shared_source_edge_line(self):
        from adapters.nuscenes_topology_opendrive import _infer_transitions

        lanes = [
            {
                "token": "incoming",
                "points": [(0.0, 0.0), (10.0, 0.0)],
                "width": 3.5,
                "source_road_block_token": "block-a",
                "source_to_edge_line_token": "shared-edge",
                "source_to_edge_node_tokens": ("shared-node",),
            },
            {
                "token": "outgoing",
                "points": [(22.0, 0.0), (32.0, 0.0)],
                "width": 3.5,
                "source_road_block_token": "block-b",
                "source_from_edge_line_token": "shared-edge",
                "source_from_edge_node_tokens": ("shared-node",),
            },
        ]

        transitions = _infer_transitions(
            lanes,
            connection_tolerance_m=20.0,
            boundary_connection_tolerance_m=20.0,
            boundary_region_tolerance_m=5.0,
            boundary_regions=[],
            max_turn_deg=135.0,
        )

        self.assertEqual(len(transitions), 1)
        self.assertEqual(
            transitions[0]["source_evidence"], "source_edge_line_continuity"
        )

    def test_long_transition_accepts_source_intersection_evidence(self):
        from adapters.nuscenes_topology_opendrive import _infer_transitions

        lanes = [
            {
                "token": "incoming",
                "points": [(0.0, 0.0), (10.0, 0.0)],
                "width": 3.5,
                "source_road_block_token": "block-a",
            },
            {
                "token": "outgoing",
                "points": [(22.0, 0.0), (32.0, 0.0)],
                "width": 3.5,
                "source_road_block_token": "block-b",
            },
        ]

        transitions = _infer_transitions(
            lanes,
            connection_tolerance_m=20.0,
            boundary_connection_tolerance_m=20.0,
            boundary_region_tolerance_m=5.0,
            boundary_regions=[
                [(14.0, -2.0), (18.0, -2.0), (18.0, 2.0), (14.0, 2.0)]
            ],
            max_turn_deg=135.0,
        )

        self.assertEqual(len(transitions), 1)
        self.assertEqual(
            transitions[0]["source_evidence"], "source_intersection_region"
        )

    def test_transition_clusters_do_not_bridge_source_intersections(self):
        from adapters.nuscenes_topology_opendrive import _cluster_transitions

        def transition(x: float, region: int) -> dict:
            return {
                "incoming_end": (x, 0.0),
                "outgoing_start": (x + 0.1, 0.0),
                "source_intersection_index": region,
            }

        groups = _cluster_transitions(
            [transition(0.0, 1), transition(1.0, 2), transition(2.0, 1)],
            junction_tolerance_m=12.0,
        )

        self.assertEqual(sorted(len(group) for group in groups), [1, 2])

    def test_direct_transition_uses_numeric_road_ids(self):
        from adapters.nuscenes_topology_opendrive import build_topology_opendrive_xml

        scenario_ir, map_data = _fixture()
        map_data["lane"] = map_data["lane"][:2]
        for node in map_data["node"]:
            if node["token"] in {"b0", "b3"}:
                node["x"] = 10.0
        root = ET.fromstring(
            build_topology_opendrive_xml(
                scenario_ir,
                map_data,
                radius_m=10.0,
                connection_tolerance_m=5.0,
                junction_tolerance_m=8.0,
            )
        )
        roads = root.findall("road")
        self.assertEqual(len(root.findall("junction")), 0)
        self.assertEqual(roads[0].find("link/successor").attrib["elementId"], "2")
        self.assertEqual(roads[1].find("link/predecessor").attrib["elementId"], "1")
        self.assertEqual(roads[0].find("lanes/laneSection/right/lane/link/successor").attrib["id"], "-1")

    def test_infers_junction_and_connector_roads(self):
        from adapters.nuscenes_topology_opendrive import build_topology_opendrive_xml

        scenario_ir, map_data = _fixture()
        root = ET.fromstring(
            build_topology_opendrive_xml(
                scenario_ir,
                map_data,
                radius_m=10.0,
                connection_tolerance_m=5.0,
                junction_tolerance_m=8.0,
            )
        )
        roads = root.findall("road")
        junctions = root.findall("junction")
        self.assertEqual(len(roads), 5)
        self.assertEqual(len(junctions), 1)
        self.assertEqual(len(junctions[0].findall("connection")), 2)
        self.assertEqual(sum(road.attrib["junction"] != "-1" for road in roads), 2)
        road_ids = {road.attrib["id"] for road in roads}
        connector_ids = {
            road.attrib["id"] for road in roads if road.attrib["junction"] != "-1"
        }
        for connection in junctions[0].findall("connection"):
            self.assertIn(connection.attrib["incomingRoad"], road_ids)
            self.assertIn(connection.attrib["connectingRoad"], road_ids)
            self.assertIn(connection.attrib["connectingRoad"], connector_ids)
            self.assertIsNotNone(connection.find("laneLink"))
        # A junction road carries one explicit predecessor/successor movement;
        # the incoming/outgoing map roads point at the junction so branching
        # does not overwrite a single road link slot.
        self.assertEqual(
            sum(
                road.find("link/predecessor") is not None
                and road.find("link/successor") is not None
                for road in roads
                if road.attrib["id"] in connector_ids
            ),
            2,
        )
        self.assertEqual(
            roads[0].find("link/successor").attrib,
            {"elementType": "junction", "elementId": "1", "contactPoint": "end"},
        )
        connector_evidence = [
            property_node.attrib["value"]
            for road in roads
            if road.attrib["id"] in connector_ids
            for property_node in road.findall("./userData/property")
            if property_node.attrib.get("name") == "topology_evidence"
        ]
        self.assertEqual(connector_evidence, ["endpoint_heading", "endpoint_heading"])

    def test_unique_connector_gets_reciprocal_road_links(self):
        from adapters.nuscenes_topology_opendrive import build_topology_opendrive_xml

        scenario_ir, map_data = _fixture()
        map_data["lane"] = [map_data["lane"][0], map_data["lane"][2]]
        for node in map_data["node"]:
            if node["token"] in {"c0", "c3"}:
                node["x"] = 10.0
        root = ET.fromstring(
            build_topology_opendrive_xml(
                scenario_ir,
                map_data,
                radius_m=10.0,
                connection_tolerance_m=5.0,
                junction_tolerance_m=8.0,
            )
        )
        connector = root.find("road[@junction='1']")
        self.assertIsNotNone(connector)
        self.assertIsNotNone(connector.find("link/predecessor"))
        self.assertIsNotNone(connector.find("link/successor"))
        incoming_id = connector.find("link/predecessor").attrib["elementId"]
        outgoing_id = connector.find("link/successor").attrib["elementId"]
        self.assertEqual(
            root.find(f"road[@id='{incoming_id}']/link/successor").attrib,
            {"elementType": "junction", "elementId": "1", "contactPoint": "end"},
        )
        self.assertEqual(
            root.find(f"road[@id='{outgoing_id}']/link/predecessor").attrib,
            {"elementType": "junction", "elementId": "1", "contactPoint": "start"},
        )

    def test_can_keep_multi_road_topology_and_add_ego_corridor(self):
        from adapters.nuscenes_topology_opendrive import build_topology_opendrive_xml

        scenario_ir, map_data = _fixture()
        root = ET.fromstring(
            build_topology_opendrive_xml(
                scenario_ir,
                map_data,
                radius_m=10.0,
                connection_tolerance_m=5.0,
                junction_tolerance_m=8.0,
                include_ego_corridor=True,
            )
        )
        self.assertEqual(len(root.findall("road")), 6)
        corridor = root.find("road[@name='ego_route_corridor']")
        self.assertIsNotNone(corridor)
        self.assertEqual(corridor.attrib["id"], "1000")
        self.assertEqual(len(root.findall("junction")), 1)

    def test_infers_source_labelled_route_roads_and_keeps_them_multi_road(self):
        from adapters.nuscenes_topology_opendrive import build_topology_opendrive_xml

        scenario_ir, map_data = _fixture()
        map_data["lane"] = map_data["lane"][:1]
        scenario_ir["ego"]["reference_trajectory"] = [
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
            {"x": 10.0, "y": 0.0, "yaw": 0.0},
            {"x": 12.0, "y": 0.0, "yaw": 0.0},
            {"x": 24.0, "y": 0.0, "yaw": 0.0},
        ]
        map_data["polygon"].append(
            {
                "token": "block-polygon",
                "exterior_node_tokens": [
                    "block-node-0",
                    "block-node-1",
                    "block-node-2",
                    "block-node-3",
                ],
                "holes": [],
            }
        )
        map_data["node"].extend(
            [
                {"token": "block-node-0", "x": 9.0, "y": -3.0},
                {"token": "block-node-1", "x": 15.0, "y": -3.0},
                {"token": "block-node-2", "x": 15.0, "y": 3.0},
                {"token": "block-node-3", "x": 9.0, "y": 3.0},
            ]
        )
        map_data["road_block"] = [
            {"token": "block-token", "polygon_token": "block-polygon"}
        ]

        root = ET.fromstring(
            build_topology_opendrive_xml(
                scenario_ir,
                map_data,
                radius_m=10.0,
                connection_tolerance_m=5.0,
                junction_tolerance_m=8.0,
                include_ego_corridor=False,
            )
        )
        inferred = [
            road
            for road in root.findall("road")
            if road.attrib["name"].startswith("inferred_route_")
        ]
        self.assertEqual(len(inferred), 1)
        self.assertIn("road_block_block-token", inferred[0].attrib["name"])
        self.assertEqual(root.findall("road[@name='ego_route_corridor']"), [])

    def test_inferred_route_chain_preserves_both_reciprocal_links(self):
        from adapters.nuscenes_topology_opendrive import build_topology_opendrive_xml

        scenario_ir, map_data = _fixture()
        map_data["lane"] = map_data["lane"][:1]
        scenario_ir["ego"]["reference_trajectory"] = [
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
            {"x": 10.0, "y": 0.0, "yaw": 0.0},
            {"x": 12.0, "y": 0.0, "yaw": 0.0},
            {"x": 14.0, "y": 0.0, "yaw": 0.0},
            {"x": 16.0, "y": 0.0, "yaw": 0.0},
            {"x": 24.0, "y": 0.0, "yaw": 0.0},
        ]
        nodes = [
            ("block-a-0", 9.0, -3.0),
            ("block-a-1", 13.0, -3.0),
            ("block-a-2", 13.0, 3.0),
            ("block-a-3", 9.0, 3.0),
            ("block-b-0", 13.0, -3.0),
            ("block-b-1", 17.0, -3.0),
            ("block-b-2", 17.0, 3.0),
            ("block-b-3", 13.0, 3.0),
        ]
        map_data["node"].extend(
            {"token": token, "x": x, "y": y} for token, x, y in nodes
        )
        map_data["polygon"].extend(
            {
                "token": polygon_token,
                "exterior_node_tokens": node_tokens,
                "holes": [],
            }
            for polygon_token, node_tokens in (
                (
                    "block-a-polygon",
                    ["block-a-0", "block-a-1", "block-a-2", "block-a-3"],
                ),
                (
                    "block-b-polygon",
                    ["block-b-0", "block-b-1", "block-b-2", "block-b-3"],
                ),
            )
        )
        map_data["road_block"] = [
            {"token": "block-a", "polygon_token": "block-a-polygon"},
            {"token": "block-b", "polygon_token": "block-b-polygon"},
        ]

        root = ET.fromstring(
            build_topology_opendrive_xml(
                scenario_ir,
                map_data,
                radius_m=10.0,
                connection_tolerance_m=5.0,
                junction_tolerance_m=8.0,
                include_ego_corridor=False,
            )
        )
        inferred = [
            road
            for road in root.findall("road")
            if road.attrib["name"].startswith("inferred_route_")
        ]
        self.assertEqual(len(inferred), 2)
        first, second = inferred
        self.assertEqual(
            first.find("link/successor").attrib["elementId"], second.attrib["id"]
        )
        self.assertEqual(
            second.find("link/predecessor").attrib["elementId"], first.attrib["id"]
        )


if __name__ == "__main__":
    unittest.main()
