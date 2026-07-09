from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runtime.scenario_runner import run_scenario_runner


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Build or execute a ScenarioRunner OpenSCENARIO command.")
    parser.add_argument("--scenario-runner-root", required=True)
    parser.add_argument("--openscenario", required=True)
    parser.add_argument("--python", default="python")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--output", action="store_true")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--timeout", default=None)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)

    config = {
        "python": args.python,
        "scenario_runner_root": args.scenario_runner_root,
        "openscenario": args.openscenario,
        "host": args.host,
        "port": args.port,
        "output": args.output,
        "output_dir": args.output_dir,
        "timeout": args.timeout,
    }
    result = run_scenario_runner(config, dry_run=not args.execute)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("status") in {"planned", "completed"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
