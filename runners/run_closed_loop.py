from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from metrics.report import build_closed_loop_report


def write_closed_loop_report(
    run_config_path: Path,
    output: Path | None = None,
    *,
    dry_run: bool = True,
) -> Path:
    if not dry_run:
        raise NotImplementedError("CARLA execution handoff is not implemented in the MVP runner skeleton.")

    run_config = json.loads(run_config_path.read_text(encoding="utf-8"))
    report = build_closed_loop_report(run_config, status="not_run")
    report["artifacts"]["run_config"] = str(run_config_path)

    output_path = output or run_config_path.with_name("closed_loop_report.json")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return output_path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a ClosedLoopBench CARLA config. Defaults to a no-CARLA dry run."
    )
    parser.add_argument("--run-config", required=True, help="Path to carla_run_config.json.")
    parser.add_argument("--output", default=None, help="Path to write closed_loop_report.json.")
    parser.add_argument(
        "--execute-carla",
        action="store_true",
        help="Reserved handoff point for the future real CARLA runner.",
    )
    args = parser.parse_args(argv)

    output = write_closed_loop_report(
        Path(args.run_config),
        Path(args.output) if args.output else None,
        dry_run=not args.execute_carla,
    )
    print(json.dumps({"closed_loop_report": str(output)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
