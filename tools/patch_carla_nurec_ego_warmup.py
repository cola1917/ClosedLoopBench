#!/usr/bin/env python3
"""Disable replay ego physics before a slow NuRec renderer warm-up."""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


BASE_EGO_GUARD = '''        if not enable_physics:
            self.actors_to_disable_physics.append(self.actor_mapping[EGO_TRACK_ID])
'''
PATCHED_EGO_GUARD = '''        if not enable_physics:
            # The formal NuRec model can spend more than a minute warming its
            # first render.  Disable physics immediately so the ego cannot
            # fall below the generated OpenDRIVE world before the first tick.
            ego_instance.set_simulate_physics(False)
            self.actors_to_disable_physics.append(self.actor_mapping[EGO_TRACK_ID])
'''


def apply_patch(target: Path) -> str:
    original = target.read_text(encoding="utf-8")
    if PATCHED_EGO_GUARD in original:
        return "already_patched"
    if BASE_EGO_GUARD not in original:
        raise RuntimeError("expected delayed ego physics block was not found")
    patched = original.replace(BASE_EGO_GUARD, PATCHED_EGO_GUARD, 1)
    backup = target.with_suffix(target.suffix + ".pre-ego-warmup")
    if not backup.exists():
        shutil.copy2(target, backup)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(patched, encoding="utf-8")
    os.replace(temporary, target)
    return "patched:immediate-ego-physics-disable"


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
