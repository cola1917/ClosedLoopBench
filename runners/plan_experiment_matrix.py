from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runtime.experiment_matrix import build_experiment_plan, evaluate_experiment_coverage


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Plan or audit an algorithm x ODD x seed matrix.")
    parser.add_argument("--matrix")
    parser.add_argument("--plan")
    parser.add_argument("--reports", nargs="*")
    parser.add_argument("--output", required=True)
    parser.add_argument("--created-at")
    args = parser.parse_args(argv)
    if bool(args.matrix) == bool(args.plan):
        parser.error("provide exactly one of --matrix or --plan")
    if args.matrix:
        payload = build_experiment_plan(
            json.loads(Path(args.matrix).read_text(encoding="utf-8")),
            created_at=args.created_at,
        )
    else:
        plan = json.loads(Path(args.plan).read_text(encoding="utf-8"))
        reports = [json.loads(Path(path).read_text(encoding="utf-8")) for path in (args.reports or [])]
        payload = evaluate_experiment_coverage(plan, reports)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "written", "output": str(output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
