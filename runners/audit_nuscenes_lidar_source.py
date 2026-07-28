#!/usr/bin/env python3
"""Produce a fail-closed raw nuScenes LiDAR support audit."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from adapters.nuscenes_lidar_source_audit import audit_nuscenes_lidar_source


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--version", default="v1.0-mini")
    args = parser.parse_args()
    registry = json.loads(args.registry.read_text(encoding="utf-8"))
    audit = audit_nuscenes_lidar_source(args.dataset_root, registry, version=args.version)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(audit["summary"], sort_keys=True))


if __name__ == "__main__":
    main()
