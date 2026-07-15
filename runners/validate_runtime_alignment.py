from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.runtime_alignment import (
    promote_runtime_validated_package,
    validate_runtime_alignment,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate measured NuRec/simulator landmarks against sim_from_log_transform."
    )
    parser.add_argument("--scene-package", required=True, type=Path)
    parser.add_argument("--observations", required=True, type=Path)
    parser.add_argument("--evidence-output", required=True, type=Path)
    parser.add_argument("--promoted-package-output", type=Path)
    parser.add_argument("--horizontal-threshold-m", type=float, default=0.25)
    parser.add_argument("--vertical-threshold-m", type=float, default=0.25)
    parser.add_argument("--yaw-threshold-deg", type=float, default=2.0)
    args = parser.parse_args(argv)
    load = lambda path: json.loads(path.read_text(encoding="utf-8"))
    package = load(args.scene_package)
    evidence = validate_runtime_alignment(
        package,
        load(args.observations),
        horizontal_threshold_m=args.horizontal_threshold_m,
        vertical_threshold_m=args.vertical_threshold_m,
        yaw_threshold_deg=args.yaw_threshold_deg,
    )
    args.evidence_output.parent.mkdir(parents=True, exist_ok=True)
    args.evidence_output.write_text(
        json.dumps(evidence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    if evidence["status"] != "passed":
        return 2
    if args.promoted_package_output:
        promoted = promote_runtime_validated_package(
            package,
            evidence,
            evidence_path=args.evidence_output.name,
        )
        args.promoted_package_output.write_text(
            json.dumps(promoted, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
