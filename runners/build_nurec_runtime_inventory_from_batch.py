from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runners.build_nurec_runtime_inventory import build_inventory_from_files


def build_inventory_from_batch(
    *,
    actor_mapping: Path,
    artifact: Path,
    batch_summary: Path,
    renderer_version: str,
) -> dict[str, Any]:
    """Promote only passed A/A/B reports from one immutable batch manifest."""

    summary = _load_object(batch_summary)
    if summary.get("schema_version") != "nurec_runtime_track_probe_batch.v1":
        raise ValueError("batch summary must use nurec_runtime_track_probe_batch.v1")
    rows = summary.get("tracks")
    if not isinstance(rows, list) or not rows:
        raise ValueError("batch summary must contain tracks")
    root = batch_summary.parent
    reports = []
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("batch track entries must be objects")
        if row.get("status") != "passed":
            continue
        report = row.get("report")
        if not isinstance(report, str) or not report:
            raise ValueError("passed batch track has no report path")
        report_path = (root / report).resolve()
        if root.resolve() not in report_path.parents:
            raise ValueError("batch report path escapes its output directory")
        reports.append(report_path)
    if not reports:
        raise ValueError("batch contains no passed reports")
    return build_inventory_from_files(
        actor_mapping,
        artifact,
        reports,
        renderer_version=renderer_version,
    )


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build a NuRec runtime inventory from passed batch pose probes."
    )
    parser.add_argument("--actor-mapping", required=True, type=Path)
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--batch-summary", required=True, type=Path)
    parser.add_argument("--renderer-version", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"refusing to overwrite inventory: {args.output}")
    try:
        result = build_inventory_from_batch(
            actor_mapping=args.actor_mapping,
            artifact=args.artifact,
            batch_summary=args.batch_summary,
            renderer_version=args.renderer_version,
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["summary"], ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
