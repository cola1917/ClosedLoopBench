from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.actor_pose_audit import ActorPoseAuditError, audit_actor_pose_request


def _jsonl(path: Path) -> list[dict]:
    rows = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"{path}:{line_number} must be a JSON object")
        rows.append(value)
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build M7 CARLA/NuRec actor pose audit evidence.")
    parser.add_argument("--request-trace", required=True, type=Path)
    parser.add_argument("--actor-bindings", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--summary-output", type=Path)
    args = parser.parse_args(argv)
    try:
        bindings = json.loads(args.actor_bindings.read_text(encoding="utf-8"))
        expected = {
            str(item["actor_id"])
            for item in bindings.get("bindings") or []
            if isinstance(item, dict) and str(item.get("actor_id") or "")
        }
        rows = []
        for request in _jsonl(args.request_trace):
            rows.extend(audit_actor_pose_request(request, expected_actor_ids=expected))
        if not rows:
            raise ValueError("request trace contains no M7 pose rows")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            "".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in rows),
            encoding="utf-8",
        )
        summary = {
            "schema_version": "actor_pose_audit_summary.v1",
            "request_trace": str(args.request_trace.resolve()),
            "actor_bindings": str(args.actor_bindings.resolve()),
            "row_count": len(rows),
            "passed_count": sum(row["status"] == "passed" for row in rows),
            "not_applicable_count": sum(row["status"] == "not_applicable" for row in rows),
            "failed_count": sum(row["status"] == "failed" for row in rows),
            "status": (
                "passed"
                if any(row["status"] == "passed" for row in rows)
                and all(row["status"] != "failed" for row in rows)
                else "failed"
            ),
        }
        if args.summary_output:
            args.summary_output.parent.mkdir(parents=True, exist_ok=True)
            args.summary_output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (ActorPoseAuditError, OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
