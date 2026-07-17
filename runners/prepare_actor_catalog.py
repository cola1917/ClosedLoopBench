from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.actor_catalog import rank_actor_candidates, repair_scenario_actor_catalog
from adapters.nuscenes_scene import build_scene_ir


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Audit the full nuScenes actor catalog and repair missing event/selected actors."
    )
    parser.add_argument("--dataroot", required=True, type=Path)
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument("--scene", required=True)
    parser.add_argument("--scenario-ir", type=Path, help="Existing mined Scenario IR to repair.")
    parser.add_argument("--include-actor", action="append", default=[])
    parser.add_argument("--output", type=Path, help="Repaired Scenario IR output.")
    parser.add_argument("--audit-output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        full = build_scene_ir(args.dataroot, args.scene, version=args.version)
        audit = rank_actor_candidates(full)
        repaired = None
        if args.scenario_ir is not None:
            source = json.loads(args.scenario_ir.read_text(encoding="utf-8"))
            repaired = repair_scenario_actor_catalog(
                source,
                full,
                additional_actor_ids=args.include_actor,
            )
            if args.output is None:
                parser.error("--output is required with --scenario-ir")
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(repaired, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        elif args.output is not None:
            parser.error("--output requires --scenario-ir")
        args.audit_output.parent.mkdir(parents=True, exist_ok=True)
        args.audit_output.write_text(
            json.dumps(audit, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(
        json.dumps(
            {
                "status": "completed",
                "scene_id": audit["scene_id"],
                "audit_output": str(args.audit_output),
                "repaired_output": str(args.output) if repaired is not None else None,
                "recommendations": audit["recommendations"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
