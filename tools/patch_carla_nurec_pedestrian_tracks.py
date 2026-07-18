#!/usr/bin/env python3
"""Map NuRec v4 ``pedestrian`` tracks to CARLA walker actors.

CARLA 0.9.16's bundled NuRec loader only recognizes the older ``person``
label.  NuRec v4 scene packages use ``pedestrian``, causing every pedestrian
track to be skipped before ``NurecScenario.actor_mapping`` is populated.  This
patch keeps backward compatibility and makes the walker blueprint selection
explicit.  It deliberately does not synthesize mapping entries: a walker must
still spawn successfully to enter the live mapping.
"""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


OLD_ACTOR_FILTER_BLOCK = '''            if not (track.label in VEHICLE_LABELS or track.label == "person"):
                continue

            best_fit_blueprint = self.blueprint_library.get_best_fit_blueprint(
                track.dims, track.label != "person"
            )
'''

NEW_ACTOR_FILTER_BLOCK = '''            # NuRec v4 scene packages label walker tracks as
            # ``pedestrian``. Keep the legacy ``person`` spelling for older
            # packages, and make the vehicle/walker blueprint choice explicit.
            is_vehicle_track = track.label in VEHICLE_LABELS
            is_pedestrian_track = track.label in {"person", "pedestrian"}
            if not (is_vehicle_track or is_pedestrian_track):
                continue

            best_fit_blueprint = self.blueprint_library.get_best_fit_blueprint(
                track.dims, is_vehicle_track
            )
'''


def apply_patch(target: Path) -> str:
    original = target.read_text(encoding="utf-8")
    if NEW_ACTOR_FILTER_BLOCK in original:
        return "already_patched"
    if OLD_ACTOR_FILTER_BLOCK not in original:
        raise RuntimeError("expected actor label/blueprint block was not found")

    patched = original.replace(
        OLD_ACTOR_FILTER_BLOCK,
        NEW_ACTOR_FILTER_BLOCK,
        1,
    )
    backup = target.with_suffix(target.suffix + ".pre-pedestrian-label")
    if not backup.exists():
        shutil.copy2(target, backup)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(patched, encoding="utf-8")
    os.replace(temporary, target)
    return "patched:pedestrian-label"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("target", type=Path)
    args = parser.parse_args()
    target = args.target.resolve()
    if not target.is_file():
        parser.error(f"target does not exist: {target}")
    try:
        print(apply_patch(target))
    except (OSError, RuntimeError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
