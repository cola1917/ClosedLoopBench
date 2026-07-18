#!/usr/bin/env python3
"""Add an optional NuRec image subcommand to CARLA's container launcher.

The CARLA 0.9.16 example targets the standalone 0.2.0 gRPC image, whose
entrypoint is already the server.  NuRec 26.04 uses the same image for all
operations and requires the ``serve-grpc`` subcommand.  The
``NUREC_IMAGE_COMMAND`` environment variable bridges both layouts without
changing the legacy default.
"""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


OLD_IMPORT = "import subprocess\n"
NEW_IMPORT = "import subprocess\nimport shlex\n"

OLD_COMMAND = '''        cmd = [
            "docker",
            "run",
            "--env",
            f"CUDA_VISIBLE_DEVICES={visible_gpu_ids}",
            "--name",
            self.container_name,
            "--gpus",
            'all,"capabilities=compute,video,utility"',
            "--rm",
            "--net=host",
            "-v",
            f"{usdz_folder}:{usdz_folder}:ro",
            "--gpus=all",
            self.image,
            "--artifact-glob",
            f"{os.path.realpath(self.usdz_path)}",
            f"--port={self.final_port}",
            "--host=localhost",
            "--test-scenes-are-valid",
        ]
'''

PREVIOUS_COMMAND = '''        cmd = [
            "docker",
            "run",
            "--shm-size=64g",
            "--env",
            f"CUDA_VISIBLE_DEVICES={visible_gpu_ids}",
            "--name",
            self.container_name,
            "--gpus",
            'all,"capabilities=compute,video,utility"',
            "--rm",
            "--net=host",
            "-v",
            f"{usdz_folder}:{usdz_folder}:ro",
            "--gpus=all",
            self.image,
        ]
        image_command = os.getenv("NUREC_IMAGE_COMMAND", "").strip()
        if image_command:
            cmd.extend(shlex.split(image_command))
        cmd.extend([
            "--artifact-glob",
            f"{os.path.realpath(self.usdz_path)}",
            f"--port={self.final_port}",
            "--host=localhost",
            "--test-scenes-are-valid",
        ])
'''

NEW_COMMAND = '''        cmd = [
            "docker",
            "run",
            "--shm-size=64g",
            "--env",
            f"CUDA_VISIBLE_DEVICES={visible_gpu_ids}",
            "--name",
            self.container_name,
            "--gpus",
            'all,"capabilities=compute,video,utility"',
            "--rm",
            "--net=host",
            "-v",
            f"{usdz_folder}:{usdz_folder}:ro",
            "--gpus=all",
            self.image,
        ]
        image_command = os.getenv("NUREC_IMAGE_COMMAND", "").strip()
        if image_command:
            cmd.extend(shlex.split(image_command))
        server_args = os.getenv("NUREC_SERVER_ARGS", "").strip()
        if server_args:
            cmd.extend(shlex.split(server_args))
        cmd.extend([
            "--artifact-glob",
            f"{os.path.realpath(self.usdz_path)}",
            f"--port={self.final_port}",
            "--host=localhost",
            "--test-scenes-are-valid",
        ])
'''


def apply_patch(target: Path) -> str:
    text = target.read_text(encoding="utf-8")
    if NEW_COMMAND in text and NEW_IMPORT in text:
        return "already_patched"
    source_command = (
        PREVIOUS_COMMAND if PREVIOUS_COMMAND in text else OLD_COMMAND
    )
    if source_command not in text:
        raise RuntimeError("expected Docker command block was not found")
    if OLD_IMPORT not in text and NEW_IMPORT not in text:
        raise RuntimeError("expected subprocess import was not found")

    backup = target.with_suffix(target.suffix + ".pre-closedloopbench")
    if not backup.exists():
        shutil.copy2(target, backup)
    patched = text
    if NEW_IMPORT not in patched:
        patched = patched.replace(OLD_IMPORT, NEW_IMPORT, 1)
    patched = patched.replace(source_command, NEW_COMMAND, 1)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(patched, encoding="utf-8")
    os.replace(temporary, target)
    return "upgraded:server-args" if source_command == PREVIOUS_COMMAND else "patched"


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
