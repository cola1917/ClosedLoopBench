import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch


def _scene_ir():
    return {
        "schema_version": "scenario_ir.v1",
        "scenario_id": "a" * 32,
        "source": {
            "dataset": "nuscenes",
            "scene_name": "scene-test",
            "scene_token": "a" * 32,
        },
        "coordinate_frame": {
            "name": "scene_local_ego_start",
            "handedness": "right",
            "x_axis": "initial_ego_forward",
            "y_axis": "initial_ego_left",
            "origin_global_translation": [0.0, 0.0, 0.0],
            "origin_global_rotation_wxyz": [1.0, 0.0, 0.0, 0.0],
            "origin_global_yaw_deg": 0.0,
            "transform": "local_xy = R(-origin_yaw) * (global_xy - origin_xy)",
            "units": {"position": "meter", "time": "second", "yaw": "degree"},
        },
        "map_context": {"location": "test-map"},
        "windows": {"event": {"start_sec": 0.0, "end_sec": 1.0}},
        "ego": {
            "initial_state": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "reference_trajectory": [
                {"t_sec": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
                {"t_sec": 1.0, "x": 10.0, "y": 0.0, "yaw": 0.0},
            ],
        },
        "actors": [],
    }


class NuScenesTopologyExchangeTests(unittest.TestCase):
    def test_exchange_rejects_corridor_only_or_single_map_artifact(self):
        from runners.build_nuscenes_topology_exchange import _validate_topology_artifact

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "single-road.xodr"
            path.write_text(
                "<OpenDRIVE>"
                "<road id='1000' name='ego_route_corridor' length='1' junction='-1'>"
                "<planView><geometry x='0' y='0' hdg='0' length='1'><line/></geometry></planView>"
                "</road></OpenDRIVE>",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "single-road"):
                _validate_topology_artifact(path, include_ego_corridor=True)

    def test_exchange_calls_topology_writer_and_preserves_multi_road_bundle(self):
        from runners.build_nuscenes_topology_exchange import (
            build_topology_exchange_from_scenario_ir,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario_ir = root / "scene_ir.json"
            scenario_ir.write_text(json.dumps(_scene_ir()), encoding="utf-8")

            def write_topology(_dataroot, output, **kwargs):
                output.write_text(
                    "<OpenDRIVE><road id='1' name='nuscenes_lane_a' length='10' junction='-1'>"
                    "<link><successor elementType='junction' elementId='1' contactPoint='end'/></link>"
                    "<planView><geometry x='0' y='0' hdg='0' length='10'><line/></geometry></planView>"
                    "</road><road id='2' name='nuscenes_lane_b' length='10' junction='-1'>"
                    "<link><predecessor elementType='junction' elementId='1' contactPoint='start'/></link>"
                    "<planView><geometry x='10' y='0' hdg='0' length='10'><line/></geometry></planView>"
                    "</road><road id='1001' name='inferred_connector_1' length='1' junction='1'>"
                    "<link><predecessor elementType='road' elementId='1' contactPoint='end'/>"
                    "<successor elementType='road' elementId='2' contactPoint='start'/></link>"
                    "<userData><property name='topology_evidence' value='endpoint_heading'/></userData>"
                    "<planView><geometry x='10' y='0' hdg='0' length='1'><line/></geometry></planView>"
                    "</road><junction id='1'><connection id='1' incomingRoad='1' connectingRoad='1001' contactPoint='start'><laneLink from='-1' to='-1'/></connection></junction></OpenDRIVE>",
                    encoding="utf-8",
                )
                return output

            def write_xosc(_scenario_ir, output, **kwargs):
                output.write_text("<OpenSCENARIO/>", encoding="utf-8")
                return output

            with patch(
                "runners.build_nuscenes_topology_exchange.write_nuscenes_topology_opendrive",
                side_effect=write_topology,
            ) as topology_writer, patch(
                "runners.build_nuscenes_topology_exchange.write_openscenario",
                side_effect=write_xosc,
            ):
                paths = build_topology_exchange_from_scenario_ir(
                    root / "nuscenes",
                    scenario_ir,
                    root / "bundle",
                    radius_m=50.0,
                    connection_tolerance_m=8.0,
                    junction_tolerance_m=12.0,
                    max_turn_deg=135.0,
                    include_ego_corridor=False,
                    include_route_inference=False,
                )

            self.assertEqual(topology_writer.call_args.kwargs["include_ego_corridor"], False)
            self.assertEqual(topology_writer.call_args.kwargs["radius_m"], 50.0)
            parsed = ET.parse(paths["opendrive"]).getroot()
            self.assertGreater(len(parsed.findall("road")), 1)
            self.assertEqual(len(parsed.findall("junction")), 1)
            package = json.loads(paths["scene_package"].read_text(encoding="utf-8"))
            self.assertEqual(package["map"]["source"], "nuscenes_topology_map_expansion")


if __name__ == "__main__":
    unittest.main()
