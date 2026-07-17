#!/usr/bin/env python3
"""Patch CARLA 0.9.16's bundled NuRec integration for external OpenDRIVE.

NuRec exports produced by NeuralSceneBridge do not necessarily embed
``map.xodr``. ClosedLoopBench keeps the generated road as a separate artifact,
so the runtime accepts it through ``NUREC_XODR_PATH``. The bundled integration
also uses a 500 m OpenDRIVE chunk length that crashes the packaged CARLA 0.9.16
server for scene-0061; CARLA's version-specific defaults are stable.
"""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


OLD_XODR_BLOCK = '''                if "map.xodr" not in zip_ref.namelist():
                    available_files = zip_ref.namelist()
                    raise KeyError(
                        f"map.xodr not found in {nurec_file}. Available files: {available_files}"
                    )

                # Read the map.xodr file content
                with zip_ref.open("map.xodr") as xodr_file:
                    data = xodr_file.read().decode("utf-8")
'''

NEW_XODR_BLOCK = '''                if "map.xodr" not in zip_ref.namelist():
                    external_xodr = os.getenv("NUREC_XODR_PATH")
                    if not external_xodr:
                        available_files = zip_ref.namelist()
                        raise KeyError(
                            "map.xodr is not embedded and NUREC_XODR_PATH is unset. "
                            f"Available files: {available_files}"
                        )
                    external_path = os.path.realpath(external_xodr)
                    if not os.path.isfile(external_path):
                        raise FileNotFoundError(
                            f"NUREC_XODR_PATH does not exist: {external_path}"
                        )
                    logger.info(f"Loading external OpenDRIVE map: {external_path}")
                    with open(external_path, "r", encoding="utf-8") as xodr_file:
                        data = xodr_file.read()
                else:
                    # Read the embedded map.xodr file content.
                    with zip_ref.open("map.xodr") as xodr_file:
                        data = xodr_file.read().decode("utf-8")
'''

OLD_GENERATION_BLOCK = '''        world = self.client.generate_opendrive_world(
            data,
            carla.OpendriveGenerationParameters(
                vertex_distance=2.0,
                max_road_length=500.0,
                wall_height=0.0,
                additional_width=7.6,
                smooth_junctions=True,
                enable_mesh_visibility=True,
            ),
        )
'''

NEW_GENERATION_BLOCK = '''        # Use CARLA's version-specific defaults. The previous 500 m chunk
        # length crashes the packaged 0.9.16 server for scene-0061 after
        # navigation-mesh generation.
        world = self.client.generate_opendrive_world(
            data, carla.OpendriveGenerationParameters()
        )
'''


def apply_patch(target: Path) -> str:
    text = target.read_text(encoding="utf-8")
    already_patched = NEW_XODR_BLOCK in text and NEW_GENERATION_BLOCK in text
    if already_patched:
        return "already_patched"
    if OLD_XODR_BLOCK not in text:
        raise RuntimeError("expected map.xodr block was not found; refusing partial patch")
    if OLD_GENERATION_BLOCK not in text:
        raise RuntimeError("expected OpenDRIVE generation block was not found")

    backup = target.with_suffix(target.suffix + ".pre-closedloopbench")
    if not backup.exists():
        shutil.copy2(target, backup)
    patched = text.replace(OLD_XODR_BLOCK, NEW_XODR_BLOCK, 1).replace(
        OLD_GENERATION_BLOCK, NEW_GENERATION_BLOCK, 1
    )
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(patched, encoding="utf-8")
    os.replace(temporary, target)
    return "patched"


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
