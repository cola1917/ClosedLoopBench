"""Dispatch the small, supported runner surface.

Existing ``runners/*.py`` paths remain compatibility entrypoints. This module
is the preferred command boundary for new documentation and development.
"""

from __future__ import annotations

import argparse
import json
from importlib import import_module
from typing import Sequence

from runners.runner_registry import CANONICAL_RUNNERS, canonical_runner, runner_inventory


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m runners",
        description="ClosedLoopBench canonical workflow commands.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("inventory", help="Print the runner surface inventory.")
    for spec in CANONICAL_RUNNERS:
        subparsers.add_parser(spec.command, help=spec.description)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args, forwarded = parser.parse_known_args(argv)
    if args.command == "inventory":
        print(json.dumps(runner_inventory(), ensure_ascii=False, indent=2, sort_keys=True))
        return 0

    spec = canonical_runner(args.command)
    module = import_module(spec.module)
    entrypoint = getattr(module, "main", None)
    if not callable(entrypoint):
        parser.error(f"canonical runner has no main(): {spec.module}")
    result = entrypoint(forwarded)
    return 0 if result is None else int(result)


if __name__ == "__main__":
    raise SystemExit(main())
