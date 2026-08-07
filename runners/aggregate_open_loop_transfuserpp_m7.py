"""Freeze the real TransFuser++ scene-0061 M7 triplicate acceptance report."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from agents.plugin_contract import strict_json_loads
from metrics.open_loop_m7 import OpenLoopM7Error, aggregate_open_loop_m7


def _load_json(path: Path) -> dict[str, Any]:
    value = strict_json_loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise OpenLoopM7Error(f"JSON artifact must be an object: {path}")
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--report",
        action="append",
        required=True,
        type=Path,
        help="One completed per-seed open_loop_multimodal_report.v1; repeat three times.",
    )
    parser.add_argument(
        "--intermediate-evaluation",
        action="append",
        required=True,
        type=Path,
        help="One evaluated intermediate report per seed; repeat three times.",
    )
    parser.add_argument(
        "--evidence-root",
        type=Path,
        required=True,
        help="Host directory mounted as /sim-data during formal runs.",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)

    try:
        if args.output.exists():
            raise OpenLoopM7Error(f"refusing to overwrite existing output: {args.output}")
        reports = [_load_json(path) for path in args.report]
        evaluations = [_load_json(path) for path in args.intermediate_evaluation]
        result = aggregate_open_loop_m7(
            reports,
            report_paths=args.report,
            intermediate_evaluations=evaluations,
            intermediate_evaluation_paths=args.intermediate_evaluation,
            evidence_root=args.evidence_root,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (OSError, TypeError, ValueError, json.JSONDecodeError, OpenLoopM7Error) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2

    print(
        json.dumps(
            {
                "status": result["acceptance_status"],
                "output": str(args.output),
                "seeds": result["seeds"],
                "schema_version": result["schema_version"],
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
