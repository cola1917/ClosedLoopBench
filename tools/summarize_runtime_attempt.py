#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("attempt_dir", type=Path)
    args = parser.parse_args()
    root = args.attempt_dir
    report_path = root / "closed_loop_report.json"
    cleanup_path = root / "cleanup_audit.json"
    report = json.loads(report_path.read_text()) if report_path.exists() else {}
    cleanup = json.loads(cleanup_path.read_text()) if cleanup_path.exists() else {}
    summary = report.get("summary") or {}
    runtime = report.get("runtime") or {}
    artifacts = {}
    for path in sorted(root.iterdir()):
        if path.is_file():
            artifacts[path.name] = path.stat().st_size
    print(json.dumps({
        "attempt_dir": str(root.resolve()),
        "status": report.get("status"),
        "evaluation": (report.get("evaluation") or {}).get("overall_result"),
        "ticks": runtime.get("frame_trace_count"),
        "route_progress": summary.get("route_progress"),
        "collision_count": summary.get("collision_count"),
        "control_count": (runtime.get("ego_driver_diagnostics") or {}).get("control_count"),
        "driver": runtime.get("ego_driver"),
        "cleanup_succeeded": cleanup.get("succeeded"),
        "cleanup_action_count": len(cleanup.get("actions") or []),
        "artifacts_size_bytes": artifacts,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
