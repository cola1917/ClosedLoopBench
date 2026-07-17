#!/usr/bin/env python3
"""Make CARLA NuRec release callbacks and GPU decoder before Python exits."""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


OLD_INIT = '''        self.default_follow_path = False
'''
NEW_INIT = '''        self.default_follow_path = False
        self._tick_callback_ids: List[int] = []
        self._callback_world = None
'''

OLD_CALLBACKS = '''        world = self.client.get_world()
        world.on_tick(lambda snapshot: self.render(snapshot))
        world.on_tick(lambda snapshot: self.update(snapshot))

        self.tick()
'''
NEW_CALLBACKS = '''        world = self.client.get_world()
        self._callback_world = world
        self._tick_callback_ids = [
            world.on_tick(lambda snapshot: self.render(snapshot)),
            world.on_tick(lambda snapshot: self.update(snapshot)),
        ]

        self.tick()
'''

ANCHOR = '''    def _warm_cache(self) -> None:
'''
CLEANUP_METHODS = '''    def _shutdown_runtime(self) -> None:
        """Release CARLA callbacks and native gRPC/GPU objects deterministically."""
        self.running = False
        world = self._callback_world
        if world is not None:
            for callback_id in self._tick_callback_ids:
                try:
                    world.remove_on_tick(callback_id)
                except Exception as exc:
                    logger.warning(f"Could not remove world tick callback {callback_id}: {exc}")
        self._tick_callback_ids.clear()
        self._callback_world = None
        self.cameras.clear()

        renderer = self.renderer
        self.renderer = None
        if renderer is not None:
            channel = getattr(renderer, "channel", None)
            if channel is not None:
                channel.close()
            renderer.client_service = None
            # nvimgcodec owns CUDA/native resources; destroy it before interpreter
            # finalization, where teardown order is otherwise undefined.
            renderer.jpeg_decoder = None

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self._shutdown_runtime()
        except Exception as exc:
            logger.warning(f"NuRec runtime cleanup failed: {exc}")
        return super().__exit__(exc_type, exc_val, exc_tb)

'''


def apply_patch(target: Path) -> str:
    original = target.read_text(encoding="utf-8")
    if (
        NEW_INIT in original
        and NEW_CALLBACKS in original
        and CLEANUP_METHODS in original
    ):
        return "already_patched"
    patched = original
    changes = []
    for old, new, name in (
        (OLD_INIT, NEW_INIT, "state"),
        (OLD_CALLBACKS, NEW_CALLBACKS, "callbacks"),
        (ANCHOR, CLEANUP_METHODS + ANCHOR, "cleanup"),
    ):
        if new in patched:
            continue
        if old not in patched:
            raise RuntimeError(f"expected {name} block was not found")
        patched = patched.replace(old, new, 1)
        changes.append(name)
    backup = target.with_suffix(target.suffix + ".pre-cleanup")
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
