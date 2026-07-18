#!/usr/bin/env python3
"""Add loaded ``NurecScenario.actor_mapping`` export to NVIDIA's replay example."""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


BASE_CLI = '''    argparser.add_argument(
        "--overlap-log",
        default="",
        help="Optional JSON output for ego/actor bounding-box overlap diagnostics",
    )
    args = argparser.parse_args()
'''
PATCHED_CLI = '''    argparser.add_argument(
        "--overlap-log",
        default="",
        help="Optional JSON output for ego/actor bounding-box overlap diagnostics",
    )
    argparser.add_argument(
        "--actor-mapping-log",
        default="",
        help="Optional JSON output for loaded NuRec track/CARLA actor observations",
    )
    args = argparser.parse_args()
'''

BASE_STATE = '''        overlap_samples = []
        try:
'''
PATCHED_STATE = '''        overlap_samples = []
        actor_mapping_records = {}
        try:
'''

BASE_TICK = '''                scenario.tick()
                if args.overlap_log:
'''
PATCHED_TICK = '''                scenario.tick()
                if args.actor_mapping_log:
                    snapshot = client.get_world().get_snapshot()
                    scenario_time = scenario.seconds_since_start()
                    for track_id, nurec_actor in scenario.actor_mapping.items():
                        actor_inst = getattr(nurec_actor, "actor_inst", None)
                        if actor_inst is None:
                            continue
                        key = str(track_id)
                        row = actor_mapping_records.setdefault(
                            key,
                            {
                                "track_id": key,
                                "runtime_actor_id": actor_inst.id,
                                "runtime_type_id": actor_inst.type_id,
                                "first_frame": snapshot.frame,
                                "first_scenario_time_sec": scenario_time,
                            },
                        )
                        row["runtime_actor_id"] = actor_inst.id
                        row["runtime_type_id"] = actor_inst.type_id
                        row["last_frame"] = snapshot.frame
                        row["last_scenario_time_sec"] = scenario_time
                if args.overlap_log:
'''

BASE_FINALLY = '''        finally:
            if args.overlap_log:
'''
PATCHED_FINALLY = '''        finally:
            if args.actor_mapping_log:
                mapping_path = Path(args.actor_mapping_log)
                mapping_path.parent.mkdir(parents=True, exist_ok=True)
                mapping_path.write_text(
                    json.dumps(
                        {
                            "schema_version": "nurec_actor_mapping_observation.v1",
                            "source": "loaded_nurec_scenario.actor_mapping",
                            "usdz_path": str(Path(args.usdz_filename).resolve()),
                            "track_count": len(actor_mapping_records),
                            "tracks": [
                                actor_mapping_records[key]
                                for key in sorted(actor_mapping_records)
                            ],
                        },
                        indent=2,
                    )
                    + "\\n",
                    encoding="utf-8",
                )
            if args.overlap_log:
'''


def apply_patch(target: Path) -> str:
    original = target.read_text(encoding="utf-8")
    markers = (PATCHED_CLI, PATCHED_STATE, PATCHED_TICK, PATCHED_FINALLY)
    if all(marker in original for marker in markers):
        return "already_patched"

    patched = original
    changes = []
    for source, replacement, label in (
        (BASE_CLI, PATCHED_CLI, "cli"),
        (BASE_STATE, PATCHED_STATE, "state"),
        (BASE_TICK, PATCHED_TICK, "sampling"),
        (BASE_FINALLY, PATCHED_FINALLY, "report"),
    ):
        if replacement in patched:
            continue
        if source not in patched:
            raise RuntimeError(f"expected {label} block was not found")
        patched = patched.replace(source, replacement, 1)
        changes.append(label)

    if not changes:
        return "already_patched"
    backup = target.with_suffix(target.suffix + ".pre-actor-inventory")
    if not backup.exists():
        shutil.copy2(target, backup)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(patched, encoding="utf-8")
    os.replace(temporary, target)
    return "patched:" + ",".join(changes)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("target", type=Path)
    args = parser.parse_args()
    if not args.target.is_file():
        parser.error(f"target does not exist: {args.target}")
    try:
        print(apply_patch(args.target.resolve()))
    except (OSError, RuntimeError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
