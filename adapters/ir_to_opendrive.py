from __future__ import annotations

from typing import Any

from adapters.route_aligned_opendrive import build_route_aligned_opendrive_xml


def build_minimal_opendrive_xml(scenario_ir: dict[str, Any]) -> str:
    """Build an explicitly named Ego-only control corridor.

    Scenario IR alone does not contain the nuScenes lane graph.  Keeping this
    fallback visibly scoped prevents it from being mistaken for the formal
    topology exchange map.
    """

    return build_route_aligned_opendrive_xml(
        scenario_ir,
        road_id=1000,
        lane_width_m=3.5,
        extension_m=0.0,
        sample_spacing_m=2.0,
        road_name="ego_route_corridor",
    )
