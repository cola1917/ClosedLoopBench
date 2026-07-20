from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from runtime.transfuserpp_visualization import load_and_render


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Render one TF++ intermediate frame without modifying raw NuRec RGB."
    )
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--evidence-root",
        type=Path,
        help="Host path mounted as /sim-data; resolves container-relative refs.",
    )
    args = parser.parse_args(argv)
    try:
        output = load_and_render(
            args.record, args.output, evidence_root=args.evidence_root
        )
        print(json.dumps({"status": "rendered", "output": str(output)}))
        return 0
    except (OSError, ValueError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
