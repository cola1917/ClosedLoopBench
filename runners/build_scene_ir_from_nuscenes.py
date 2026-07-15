from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nuscenes_scene import build_scene_ir


def write_scene_ir(dataroot: Path, version: str, scene: str, output: Path) -> Path:
    scenario_ir = build_scene_ir(dataroot, scene, version=version)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(scenario_ir, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Build normalized Scenario IR from nuScenes JSON tables.")
    parser.add_argument("--dataroot", required=True, help="nuScenes root containing the version directory.")
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument("--scene", required=True, help="Scene name (for example scene-0061) or token.")
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)

    output = write_scene_ir(Path(args.dataroot), args.version, args.scene, Path(args.output))
    print(json.dumps({"scenario_ir": str(output)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
