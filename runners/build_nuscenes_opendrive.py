from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nuscenes_map_to_opendrive import build_local_opendrive_xml, load_nuscenes_map
from adapters.nuscenes_scene import build_scene_ir


def write_nuscenes_opendrive(
    dataroot: Path,
    output: Path,
    *,
    scene: str | None = None,
    scenario_ir_path: Path | None = None,
    version: str = "v1.0-mini",
    radius_m: float = 35.0,
) -> Path:
    if (scene is None) == (scenario_ir_path is None):
        raise ValueError("provide exactly one of scene or scenario_ir_path")
    scenario_ir = (
        build_scene_ir(dataroot, scene, version=version)
        if scene is not None
        else json.loads(scenario_ir_path.read_text(encoding="utf-8"))
    )
    location = scenario_ir.get("map_context", {}).get("location")
    if not location:
        raise ValueError("Scenario IR does not identify a nuScenes map location")
    map_data = load_nuscenes_map(dataroot, str(location))
    xml_text = build_local_opendrive_xml(scenario_ir, map_data, radius_m=radius_m)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xml_text, encoding="utf-8")
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Build a limited local OpenDRIVE 1.4 map from nuScenes HD Map polygons.")
    parser.add_argument("--dataroot", required=True, help="nuScenes root containing maps and metadata.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--scene", help="nuScenes scene name or token.")
    source.add_argument("--scenario-ir", help="Existing normalized Scenario IR JSON.")
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument("--radius-m", type=float, default=35.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)

    output = write_nuscenes_opendrive(
        Path(args.dataroot),
        Path(args.output),
        scene=args.scene,
        scenario_ir_path=Path(args.scenario_ir) if args.scenario_ir else None,
        version=args.version,
        radius_m=args.radius_m,
    )
    print(json.dumps({"opendrive": str(output), "scope": "local_limited"}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
