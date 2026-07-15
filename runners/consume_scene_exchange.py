from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_exchange import consume_scene_version


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Resolve a complete, READY Scene Package version.")
    parser.add_argument("--exchange-root", required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--version", help="Defaults to the lexically latest ready version.")
    parser.add_argument("--output", help="Optional JSON file; stdout is always written.")
    args = parser.parse_args(argv)
    result = consume_scene_version(Path(args.exchange_root), args.scene_id, args.version)
    payload = json.dumps(result, indent=2) + "\n"
    if args.output:
        Path(args.output).write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
