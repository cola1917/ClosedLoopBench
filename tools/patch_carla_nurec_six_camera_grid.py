#!/usr/bin/env python3
"""Make NVIDIA's NuRec pygame grid dynamic and safe for six cameras plus overhead."""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


OLD_CALLBACK = '''def make_camera_callback(display, camera_name, pygame_pos, saveimages: bool, output_dir: str = "data"):
    def callback(image):
        display.setImage(image, (3, 2), pygame_pos)
'''

NEW_CALLBACK = '''def make_camera_callback(
    display,
    camera_name,
    pygame_pos,
    saveimages: bool,
    output_dir: str = "data",
    pygame_dims=(3, 2),
):
    def callback(image):
        display.setImage(image, pygame_dims, pygame_pos)
'''

OLD_GRID = '''    grid_size =  (3, 2)
    grid_pos = (0, 0)
'''

NEW_GRID = '''    grid_columns = 3
    grid_rows = max(1, (len(camera_configs) + grid_columns - 1) // grid_columns)
    grid_size = (grid_columns, grid_rows)
    grid_pos = (0, 0)
'''

OLD_RICH_CALLBACK = '''                make_camera_callback(pygame_display, cameraname, grid_pos, saveimages, output_dir),
'''

NEW_RICH_CALLBACK = '''                make_camera_callback(
                    pygame_display,
                    cameraname,
                    grid_pos,
                    saveimages,
                    output_dir,
                    pygame_dims=grid_size,
                ),
'''

OLD_RECORDED_CALLBACK = '''                    camera_name,
                    grid_pos,
                    saveimages,
                    output_dir,
                ),
'''

NEW_RECORDED_CALLBACK = '''                    camera_name,
                    grid_pos,
                    saveimages,
                    output_dir,
                    pygame_dims=grid_size,
                ),
'''

OLD_ADVANCE = '''        grid_pos = (grid_pos[0] + 1, grid_pos[1])
        if grid_pos[0] >= grid_size[0]:
            grid_pos = (1, grid_pos[1] + 1)
'''

NEW_ADVANCE = '''        grid_pos = (grid_pos[0] + 1, grid_pos[1])
        if grid_pos[0] >= grid_size[0]:
            grid_pos = (0, grid_pos[1] + 1)
'''


def apply_patch(target: Path) -> str:
    original = target.read_text(encoding="utf-8")
    if all(
        marker in original
        for marker in (NEW_CALLBACK, NEW_GRID, NEW_RICH_CALLBACK, NEW_RECORDED_CALLBACK, NEW_ADVANCE)
    ):
        return "already_patched"
    replacements = (
        (OLD_CALLBACK, NEW_CALLBACK, "callback-dimensions"),
        (OLD_GRID, NEW_GRID, "dynamic-grid"),
        (OLD_RICH_CALLBACK, NEW_RICH_CALLBACK, "rich-camera-grid"),
        (OLD_RECORDED_CALLBACK, NEW_RECORDED_CALLBACK, "recorded-camera-grid"),
        (OLD_ADVANCE, NEW_ADVANCE, "grid-wrap"),
    )
    patched = original
    changes = []
    for old, new, name in replacements:
        if new in patched:
            continue
        if old not in patched:
            raise RuntimeError(f"expected {name} block was not found")
        patched = patched.replace(old, new, 1)
        changes.append(name)
    backup = target.with_suffix(target.suffix + ".pre-six-camera-grid")
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
    print(apply_patch(args.target.resolve()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
