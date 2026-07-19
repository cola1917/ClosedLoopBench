from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runtime.render_quality import evaluate_render_quality  # noqa: E402


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Evaluate paired NuRec RGB evidence without modifying source imagery."
    )
    parser.add_argument("--request", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--base-dir",
        help="Base directory for image paths (defaults to the request directory).",
    )
    args = parser.parse_args(argv)
    request_path = Path(args.request)
    output = Path(args.output)
    if output.exists():
        parser.error(f"refusing to overwrite existing output: {output}")
    request = json.loads(request_path.read_text(encoding="utf-8"))
    report = evaluate_render_quality(
        request,
        base_dir=Path(args.base_dir) if args.base_dir else request_path.parent,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "status": "written",
                "output": str(output),
                "evidence_classification": report["evidence_classification"],
                "remote_validation_required": report["remote_validation_required"],
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
