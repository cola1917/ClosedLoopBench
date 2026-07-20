from __future__ import annotations

import argparse
import json
import sys
from copy import deepcopy
from pathlib import Path

from agents.plugin_contract import file_sha256, strict_json_loads
from runners.run_carla_acceptance_triplicate import (
    _validate_transfuserpp_cuda_evidence,
    _validate_transfuserpp_run_config_identity,
)


def bind_cuda_preflight(config: dict, evidence_path: Path) -> dict:
    _validate_transfuserpp_run_config_identity(config)
    if not evidence_path.is_file():
        raise ValueError("CUDA preflight evidence file is unavailable")
    candidate = deepcopy(config)
    candidate["algorithm_gpu_validation"] = {
        "status": "bound",
        "evidence_path": str(evidence_path.resolve()),
        "evidence_sha256": file_sha256(evidence_path),
    }
    _validate_transfuserpp_cuda_evidence(candidate)
    _validate_transfuserpp_run_config_identity(candidate)
    return candidate


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Bind a verified CUDA preflight report to a new formal TF++ run config."
    )
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--cuda-evidence", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise FileExistsError(f"refusing to overwrite: {args.output}")
        config = strict_json_loads(args.run_config.read_text(encoding="utf-8"))
        # Parse once independently so duplicate-key or malformed evidence fails
        # before its content-addressed binding is written.
        strict_json_loads(args.cuda_evidence.read_text(encoding="utf-8"))
        result = bind_cuda_preflight(config, args.cuda_evidence)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"status": "bound", "output": str(args.output)}, sort_keys=True))
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
