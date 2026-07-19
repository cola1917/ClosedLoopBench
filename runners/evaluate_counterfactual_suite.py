from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from metrics.counterfactual_suite import evaluate_counterfactual_suite


def _load_many(paths):
    return [json.loads(Path(path).read_text(encoding="utf-8")) for path in paths]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Fail-closed scene counterfactual coverage and comparison evaluator.")
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--reports", nargs="*", default=[])
    parser.add_argument("--quality-reports", nargs="*", default=[])
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    matrix = json.loads(Path(args.matrix).read_text(encoding="utf-8"))
    result = evaluate_counterfactual_suite(matrix, _load_many(args.reports), _load_many(args.quality_reports))
    output = Path(args.output)
    if output.exists():
        parser.error(f"refusing to overwrite existing output: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "written", "output": str(output), "ready": result["ready_for_formal_comparison"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
