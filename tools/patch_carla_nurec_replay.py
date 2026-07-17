#!/usr/bin/env python3
"""Make CARLA's NuRec example keep ego control opt-in.

The upstream example switches to its simple trajectory follower after one
second. Scene integration must first validate the recorded scene replay, so
controller takeover is exposed as ``--enable-ego-control`` and remains off by
default.
"""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


OLD_ARG_BLOCK = '''    argparser.add_argument(
        "--move-spectator", action="store_true", help="move spectator camera"
    )
    args = argparser.parse_args()
'''

NEW_ARG_BLOCK = '''    argparser.add_argument(
        "--move-spectator", action="store_true", help="move spectator camera"
    )
    argparser.add_argument(
        "--enable-ego-control",
        action="store_true",
        help="Enable the example trajectory follower after one second (default: replay only)",
    )
    args = argparser.parse_args()
'''

OLD_CONTROL_LINE = '''            should_apply_control = True
'''
NEW_CONTROL_LINE = '''            should_apply_control = args.enable_ego_control
'''

OLD_PARSE_LINE = '''    args = argparser.parse_args()
'''
NEW_PARSE_BLOCK = '''    argparser.add_argument(
        "--resolution-ratio",
        type=float,
        default=0.5,
        help="NuRec render resolution relative to source cameras (default: 0.5)",
    )
    args = argparser.parse_args()
'''

OLD_RATIO_LINE = '''                resolution_ratio=0.125,
'''
NEW_RATIO_LINE = '''                resolution_ratio=resolution_ratio,
'''

OLD_ADD_CAMERAS_LINE = '''            spectator, display = add_cameras(scenario, client, args.output_dir, args.saveimages )
'''
NEW_ADD_CAMERAS_LINE = '''            spectator, display = add_cameras(
                scenario,
                client,
                args.output_dir,
                args.saveimages,
                resolution_ratio=args.resolution_ratio,
            )
'''

CAMERA_CONFIG_ADD_CAMERAS_LINE = '''            spectator, display = add_cameras(
                scenario,
                client,
                args.output_dir,
                args.saveimages,
                resolution_ratio=args.resolution_ratio,
                camera_config_path=args.camera_config,
            )
'''


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("target", type=Path)
    args = parser.parse_args()
    target = args.target.resolve()
    text = target.read_text(encoding="utf-8")
    patched = text
    changes = []
    if OLD_ARG_BLOCK in patched:
        patched = patched.replace(OLD_ARG_BLOCK, NEW_ARG_BLOCK, 1)
        changes.append("cli")
    elif '"--enable-ego-control"' not in patched:
        raise RuntimeError("expected replay argument block was not found")
    if OLD_CONTROL_LINE in patched:
        patched = patched.replace(OLD_CONTROL_LINE, NEW_CONTROL_LINE, 1)
        changes.append("control-opt-in")
    elif NEW_CONTROL_LINE not in patched:
        raise RuntimeError("expected replay control line was not found")
    if '"--resolution-ratio"' not in patched:
        if OLD_PARSE_LINE not in patched:
            raise RuntimeError("expected argument parse line was not found")
        patched = patched.replace(OLD_PARSE_LINE, NEW_PARSE_BLOCK, 1)
        changes.append("resolution-cli")
    if OLD_RATIO_LINE in patched:
        patched = patched.replace(OLD_RATIO_LINE, NEW_RATIO_LINE, 1)
        changes.append("resolution-forward")
    elif NEW_RATIO_LINE not in patched:
        raise RuntimeError("expected NuRec resolution ratio line was not found")
    if CAMERA_CONFIG_ADD_CAMERAS_LINE in patched:
        pass
    elif OLD_ADD_CAMERAS_LINE in patched:
        patched = patched.replace(OLD_ADD_CAMERAS_LINE, NEW_ADD_CAMERAS_LINE, 1)
        changes.append("resolution-call")
    elif NEW_ADD_CAMERAS_LINE not in patched:
        raise RuntimeError("expected add_cameras call was not found")
    if not changes:
        print("already_patched")
        return 0
    backup = target.with_suffix(target.suffix + ".pre-closedloopbench")
    if not backup.exists():
        shutil.copy2(target, backup)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(patched, encoding="utf-8")
    os.replace(temporary, target)
    print("patched:" + ",".join(changes))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
