from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runtime.carla_probe import build_probe_config, probe_carla


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Probe a CARLA server without mutating the world.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--timeout-sec", default=2.0, type=float)
    parser.add_argument("--map", dest="map_name", default=None)
    args = parser.parse_args(argv)

    config = build_probe_config(
        host=args.host,
        port=args.port,
        timeout_sec=args.timeout_sec,
        map_name=args.map_name,
    )
    result = probe_carla(config)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("status") == "available" else 2


if __name__ == "__main__":
    raise SystemExit(main())
