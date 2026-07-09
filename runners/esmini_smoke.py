from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.esmini import find_esmini


def build_esmini_command(esmini: Path, xosc: Path, *, headless: bool = True) -> list[str]:
    command = [str(esmini), "--osc", str(xosc)]
    if headless:
        command.append("--headless")
    return command


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Optionally smoke-test an OpenSCENARIO file with esmini.")
    parser.add_argument("--xosc", required=True)
    parser.add_argument("--log", default=None)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--no-headless", action="store_true")
    args = parser.parse_args(argv)

    esmini = find_esmini()
    if esmini is None:
        print(json.dumps({"status": "skipped", "reason": "esmini not found"}, ensure_ascii=False, indent=2))
        return 0

    command = build_esmini_command(esmini, Path(args.xosc), headless=not args.no_headless)
    if not args.execute:
        print(json.dumps({"status": "dry_run", "command": command}, ensure_ascii=False, indent=2))
        return 0

    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    payload = {
        "status": "passed" if completed.returncode == 0 else "failed",
        "returncode": completed.returncode,
        "command": command,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }
    if args.log:
        log_path = Path(args.log)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in payload.items() if k not in ("stdout", "stderr")}, ensure_ascii=False, indent=2))
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
