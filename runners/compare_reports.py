from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from metrics.comparison import compare_closed_loop_reports


def compare_report_files(report_paths: list[Path], output: Path) -> Path:
    reports = [json.loads(path.read_text(encoding="utf-8")) for path in report_paths]
    comparison = compare_closed_loop_reports(reports)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(comparison, ensure_ascii=False, indent=2), encoding="utf-8")
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Compare closed-loop reports by algorithm and ODD.")
    parser.add_argument("reports", nargs="+", help="Closed-loop report JSON files.")
    parser.add_argument("--output", required=True, help="Comparison JSON output path.")
    args = parser.parse_args(argv)
    written = compare_report_files([Path(path) for path in args.reports], Path(args.output))
    print(str(written))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
