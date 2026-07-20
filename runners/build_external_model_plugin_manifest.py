from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from agents.model_plugin_wrappers import (
    build_external_model_config_schema,
    build_external_model_runtime_manifest,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build a fail-closed TCP/TransFuser remote-binding manifest."
    )
    parser.add_argument(
        "--algorithm", choices=("tcp", "transfuser", "transfuserpp"), required=True
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            parser.error(f"refusing to overwrite existing output: {args.output}")
        config = json.loads(args.config.read_text(encoding="utf-8"))
        manifest = build_external_model_runtime_manifest(args.algorithm, config)
        manifest["config_schema"] = build_external_model_config_schema(args.algorithm)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
        print(json.dumps(manifest, sort_keys=True))
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"execution_status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
