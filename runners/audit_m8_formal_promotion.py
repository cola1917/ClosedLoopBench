from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.m8_promotion import evaluate_m8_promotion


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Promote a NuRec candidate only after candidate smoke and all four M8 streams pass."
    )
    parser.add_argument("--smoke-report", required=True, type=Path)
    parser.add_argument("--m8-audit-summary", required=True, type=Path)
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--scene-id")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"refusing to overwrite immutable promotion evidence: {args.output}")
    try:
        smoke = _load(args.smoke_report)
        summary = _load(args.m8_audit_summary)
        report = evaluate_m8_promotion(
            smoke,
            summary,
            artifact_path=args.artifact,
            scene_id=args.scene_id,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": report["status"], "output": str(args.output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 2


def _load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


if __name__ == "__main__":
    raise SystemExit(main())
