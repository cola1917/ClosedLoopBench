from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_exchange import publish_scene_version


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Atomically publish a versioned Scene Package.")
    parser.add_argument("--bundle-dir", required=True)
    parser.add_argument("--exchange-root", required=True)
    parser.add_argument(
        "--scene-id",
        default=None,
        help="Canonical nuScenes scene token; defaults to scene_package.json scene_id.",
    )
    parser.add_argument("--version", required=True)
    args = parser.parse_args(argv)
    target = publish_scene_version(
        Path(args.bundle_dir), Path(args.exchange_root), args.scene_id, args.version
    )
    print(json.dumps({"published": str(target)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
