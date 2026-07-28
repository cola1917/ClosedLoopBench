from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_inventory import NuRecInventoryError, audit_registry_ncore_dynamic_closure


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Fail closed unless the full CARLA registry and NCore dynamic tracks match exactly."
    )
    parser.add_argument("--scene-object-registry", required=True, type=Path)
    parser.add_argument("--ncore-track-audit", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--require-ready", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(f"refusing to overwrite dynamic closure audit: {args.output}")
        registry = json.loads(args.scene_object_registry.read_text(encoding="utf-8"))
        ncore_track_audit = json.loads(args.ncore_track_audit.read_text(encoding="utf-8"))
        audit = audit_registry_ncore_dynamic_closure(registry, ncore_track_audit)
        if args.require_ready and audit["status"] != "passed":
            raise NuRecInventoryError("NCore dynamic track closure is not ready for M8")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, NuRecInventoryError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": audit["status"], "summary": audit["summary"]}, ensure_ascii=False))
    return 0 if audit["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
