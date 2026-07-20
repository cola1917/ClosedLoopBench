from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

from agents.plugin_contract import strict_json_loads
from metrics.transfuserpp_intermediate import (
    TransFuserPPIntermediateError,
    compare_counterfactual_traces,
    evaluate_intermediate_trace,
)


def _load_records(path: Path) -> list[dict[str, Any]]:
    if path.is_dir():
        paths = sorted(path.glob("*.intermediate.json"))
        return [strict_json_loads(item.read_text(encoding="utf-8")) for item in paths]
    if path.suffix.lower() == ".jsonl":
        return [
            strict_json_loads(line)
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    value = strict_json_loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, list) else [value]


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Evaluate real TransFuser++ intermediate traces without claiming full Occ3D."
    )
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--edited-trace", type=Path)
    parser.add_argument("--event-timestamp", type=float)
    parser.add_argument("--expected-case-id")
    parser.add_argument(
        "--evidence-root",
        type=Path,
        help="Host path mounted as /sim-data for --trace.",
    )
    parser.add_argument(
        "--edited-evidence-root",
        type=Path,
        help="Host path mounted as /sim-data for --edited-trace; defaults to --evidence-root.",
    )
    parser.add_argument(
        "--render-quality-report",
        type=Path,
        help="Bound render_quality_report.v1; a bare classification cannot grant perception eligibility.",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            parser.error(f"refusing to overwrite existing output: {args.output}")
        baseline = _load_records(args.trace)
        if args.edited_trace:
            report = compare_counterfactual_traces(
                baseline,
                _load_records(args.edited_trace),
                event_timestamp=args.event_timestamp,
                expected_case_id=args.expected_case_id,
                evidence_root=args.evidence_root,
                edited_evidence_root=args.edited_evidence_root,
            )
        else:
            quality_report = None
            if args.render_quality_report:
                report_path = args.render_quality_report.resolve()
                quality_report = strict_json_loads(
                    report_path.read_text(encoding="utf-8")
                )
                quality_report["_bound_report_ref"] = {
                    "path": str(report_path),
                    "sha256": _file_sha256(report_path),
                    "size_bytes": report_path.stat().st_size,
                }
            report = evaluate_intermediate_trace(
                baseline,
                render_quality_report=quality_report,
                evidence_root=args.evidence_root,
            )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, ensure_ascii=False, sort_keys=True))
        return 0 if report["status"] == "evaluated" else 2
    except (OSError, ValueError, TransFuserPPIntermediateError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
