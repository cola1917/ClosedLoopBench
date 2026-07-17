#!/usr/bin/env python3
"""Let CARLA's NuRec replay UI use scene-specific recorded cameras."""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


OLD_SIGNATURE = '''def add_cameras(
    scenario: NurecScenario, client: carla.Client, output_dir: str, saveimages: bool, resolution_ratio: float = 0.125
) -> Tuple[carla.Actor, PygameDisplay]:
'''

NEW_SIGNATURE = '''def add_cameras(
    scenario: NurecScenario,
    client: carla.Client,
    output_dir: str,
    saveimages: bool,
    resolution_ratio: float = 0.125,
    camera_config_path: str = "carla_example_camera_config.yaml",
) -> Tuple[carla.Actor, PygameDisplay]:
'''

OLD_CONFIG_OPEN = '''    with open("carla_example_camera_config.yaml", "r") as f:
        camera_configs = yaml.safe_load(f)
'''

NEW_CONFIG_OPEN = '''    with open(camera_config_path, "r", encoding="utf-8") as f:
        camera_configs = yaml.safe_load(f)
'''

OLD_SENSOR_BRANCH = '''        # Case 2: Simple CARLA sensor style
        elif "sensor" in cam_cfg:
'''

NEW_SENSOR_BRANCH = '''        # Case 2: A recorded NuRec camera with authoritative intrinsics
        # and rig extrinsics from the USDZ artifact.
        elif "nurec_camera" in cam_cfg:
            camera_name = cam_cfg["nurec_camera"]
            scenario.add_camera(
                camera_name,
                make_camera_callback(
                    pygame_display,
                    camera_name,
                    grid_pos,
                    saveimages,
                    output_dir,
                ),
                framerate=30,
                resolution_ratio=resolution_ratio,
            )
        # Case 3: Simple CARLA sensor style
        elif "sensor" in cam_cfg:
'''

OLD_CLI = '''    argparser.add_argument(
        "--resolution-ratio",
        type=float,
        default=0.5,
        help="NuRec render resolution relative to source cameras (default: 0.5)",
    )
    args = argparser.parse_args()
'''

NEW_CLI = '''    argparser.add_argument(
        "--resolution-ratio",
        type=float,
        default=0.5,
        help="NuRec render resolution relative to source cameras (default: 0.5)",
    )
    argparser.add_argument(
        "--camera-config",
        default="carla_example_camera_config.yaml",
        help="YAML camera grid config (default: bundled CARLA example)",
    )
    args = argparser.parse_args()
'''

CAMERA_OPTION_BLOCK = '''    argparser.add_argument(
        "--resolution-ratio",
        type=float,
        default=0.5,
        help="NuRec render resolution relative to source cameras (default: 0.5)",
    )
    argparser.add_argument(
        "--camera-config",
        default="carla_example_camera_config.yaml",
        help="YAML camera grid config (default: bundled CARLA example)",
    )
'''

OLD_CALL = '''            spectator, display = add_cameras(
                scenario,
                client,
                args.output_dir,
                args.saveimages,
                resolution_ratio=args.resolution_ratio,
            )
'''

NEW_CALL = '''            spectator, display = add_cameras(
                scenario,
                client,
                args.output_dir,
                args.saveimages,
                resolution_ratio=args.resolution_ratio,
                camera_config_path=args.camera_config,
            )
'''


def apply_patch(target: Path) -> str:
    original_text = target.read_text(encoding="utf-8")
    text = original_text
    deduplicated = text
    while CAMERA_OPTION_BLOCK + CAMERA_OPTION_BLOCK in deduplicated:
        deduplicated = deduplicated.replace(
            CAMERA_OPTION_BLOCK + CAMERA_OPTION_BLOCK,
            CAMERA_OPTION_BLOCK,
            1,
        )
    text = deduplicated
    if (
        NEW_SIGNATURE in text
        and NEW_CONFIG_OPEN in text
        and NEW_SENSOR_BRANCH in text
        and NEW_CLI in text
        and NEW_CALL in text
        and text == original_text
    ):
        return "already_patched"
    replacements = (
        (OLD_SIGNATURE, NEW_SIGNATURE, "signature"),
        (OLD_CONFIG_OPEN, NEW_CONFIG_OPEN, "config-path"),
        (OLD_SENSOR_BRANCH, NEW_SENSOR_BRANCH, "recorded-camera"),
        (OLD_CLI, NEW_CLI, "cli"),
        (OLD_CALL, NEW_CALL, "call"),
    )
    patched = text
    changes = []
    for old, new, name in replacements:
        if name == "cli" and '"--camera-config"' in patched:
            continue
        if new in patched:
            continue
        if old not in patched:
            raise RuntimeError(f"expected {name} block was not found")
        patched = patched.replace(old, new, 1)
        changes.append(name)

    if deduplicated != original_text:
        changes.insert(0, "cli-deduplicate")
    if not changes:
        return "already_patched"

    backup = target.with_suffix(target.suffix + ".pre-camera-config")
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
