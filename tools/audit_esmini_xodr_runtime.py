#!/usr/bin/env python3
"""Check that esmini materializes every road declared by an OpenDRIVE file."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.esmini_xodr_runtime_audit import (  # noqa: E402
    EsminiXodrAuditError,
    audit_xodr_with_odrplot,
)
from tools.esmini import find_esmini  # noqa: E402


def _find_odrplot(explicit: str | None) -> Path | None:
    if explicit:
        return Path(explicit).expanduser()
    env_value = os.environ.get("ODRPLOT_BIN")
    if env_value:
        return Path(env_value).expanduser()
    esmini = find_esmini()
    if esmini is None:
        return None
    candidates = (
        esmini.with_name("odrplot.exe"),
        esmini.with_name("odrplot"),
    )
    return next((candidate for candidate in candidates if candidate.is_file()), None)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Audit esmini OpenDRIVE sampling so a multi-road XML cannot silently "
            "materialize as a single road."
        )
    )
    parser.add_argument("--xodr", required=True, type=Path)
    parser.add_argument("--odrplot", type=Path)
    parser.add_argument("--expected-sha256")
    parser.add_argument("--sample-step-m", type=float, default=1.0)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    odrplot = _find_odrplot(str(args.odrplot) if args.odrplot else None)
    if odrplot is None or not odrplot.is_file():
        report = {
            "schema_version": "esmini_xodr_runtime_audit.v1",
            "status": "unavailable",
            "errors": ["odrplot executable was not found"],
        }
        _write_report(args.output, report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 2
    try:
        report = audit_xodr_with_odrplot(
            args.xodr,
            odrplot,
            sample_step_m=args.sample_step_m,
            expected_sha256=args.expected_sha256,
            timeout_sec=args.timeout_sec,
        )
    except (EsminiXodrAuditError, OSError, ValueError) as exc:
        report = {
            "schema_version": "esmini_xodr_runtime_audit.v1",
            "status": "failed",
            "errors": [str(exc)],
        }
        _write_report(args.output, report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1

    _write_report(args.output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "passed" else 1


def _write_report(output: Path | None, report: dict) -> None:
    if output is None:
        return
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
