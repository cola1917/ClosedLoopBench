from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.ir_to_openscenario import build_openscenario_xml


def write_openscenario(scenario_ir_path: Path, output: Path, *, road_file: str = "road.xodr") -> Path:
    scenario_ir = json.loads(scenario_ir_path.read_text(encoding="utf-8"))
    xml_text = build_openscenario_xml(scenario_ir, road_file=road_file)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xml_text, encoding="utf-8")
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Build OpenSCENARIO MVP export from Scenario IR.")
    parser.add_argument("--scenario-ir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--road-file", default="road.xodr")
    args = parser.parse_args(argv)

    output = write_openscenario(Path(args.scenario_ir), Path(args.output), road_file=args.road_file)
    print(json.dumps({"openscenario": str(output)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
