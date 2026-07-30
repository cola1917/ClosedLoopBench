from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.route_aligned_opendrive import build_route_aligned_opendrive_xml


def write_route_aligned_opendrive(
    scenario_ir_path: Path,
    output: Path,
    *,
    lane_width_m: float = 3.7,
    extension_m: float = 10.0,
    sample_spacing_m: float = 2.0,
) -> Path:
    scenario_ir = json.loads(scenario_ir_path.read_text(encoding="utf-8"))
    xml_text = build_route_aligned_opendrive_xml(
        scenario_ir,
        lane_width_m=lane_width_m,
        extension_m=extension_m,
        sample_spacing_m=sample_spacing_m,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xml_text, encoding="utf-8")
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Build a single-road Ego control corridor from Scenario IR; "
            "this is not a map topology export."
        )
    )
    parser.add_argument("--scenario-ir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--lane-width-m", type=float, default=3.7)
    parser.add_argument("--extension-m", type=float, default=10.0)
    parser.add_argument("--sample-spacing-m", type=float, default=2.0)
    args = parser.parse_args(argv)

    output = write_route_aligned_opendrive(
        args.scenario_ir,
        args.output,
        lane_width_m=args.lane_width_m,
        extension_m=args.extension_m,
        sample_spacing_m=args.sample_spacing_m,
    )
    print(
        json.dumps(
            {
                "opendrive": str(output.resolve()),
                "scope": "ego_control_corridor_only",
                "road_count": 1,
                "map_reconstruction": False,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
