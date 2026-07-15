from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.evaluation_protocol import build_evaluation_result_message
from adapters.shared_message_store import reference_existing_artifact


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Build an evaluation.run.result protocol message.")
    parser.add_argument("--request", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--exchange-root", required=True)
    parser.add_argument("--started-at", required=True)
    parser.add_argument("--finished-at", required=True)
    parser.add_argument("--producer-version", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    request = json.loads(Path(args.request).read_text(encoding="utf-8"))
    report_path = Path(args.report)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    reference = reference_existing_artifact(
        Path(args.exchange_root),
        report_path,
        role="closed_loop_report",
        media_type="application/json",
        content_schema="closed_loop_report.mvp.v0",
    )
    result = build_evaluation_result_message(
        request,
        report,
        report_reference=reference,
        started_at=args.started_at,
        finished_at=args.finished_at,
        producer={
            "project": "ClosedLoopBench",
            "component": "evaluation-result-adapter",
            "version": args.producer_version,
        },
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "written", "output": str(output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
