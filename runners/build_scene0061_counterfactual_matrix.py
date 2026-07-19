from __future__ import annotations

import argparse
from hashlib import sha256
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runtime.scene0061_counterfactual import (
    build_scene0061_counterfactual_matrix,
    validate_scene0061_counterfactual_matrix,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Build or validate the frozen scene-0061 counterfactual matrix.")
    parser.add_argument("--output")
    parser.add_argument("--validate")
    parser.add_argument("--validation-output")
    parser.add_argument("--created-at", default="2026-07-19T00:00:00Z")
    args = parser.parse_args(argv)
    if bool(args.output) == bool(args.validate):
        parser.error("provide exactly one of --output or --validate")
    if args.validate:
        path = Path(args.validate)
        matrix = json.loads(path.read_text(encoding="utf-8"))
        validate_scene0061_counterfactual_matrix(matrix)
        validation = {
            "schema_version": "scene_counterfactual_matrix_validation.v1",
            "status": "passed",
            "validation_class": "offline_conformance",
            "matrix_path": str(path),
            "matrix_file_sha256": sha256(path.read_bytes()).hexdigest(),
            "immutable_matrix_sha256": matrix["immutable_matrix_sha256"],
            "case_count": len(matrix["cases"]),
            "algorithm_count": len(matrix["algorithms"]),
            "seed_count": len(matrix["seeds"]),
            "expected_remote_run_count": len(matrix["cases"]) * len(matrix["algorithms"]) * len(matrix["seeds"]),
            "remote_validation_required": True,
            "limitations": [
                "Matrix validation does not execute CARLA or NuRec.",
                "No runtime acceptance result is claimed.",
            ],
        }
        if args.validation_output:
            validation_output = Path(args.validation_output)
            if validation_output.exists():
                parser.error(f"refusing to overwrite existing output: {validation_output}")
            validation_output.parent.mkdir(parents=True, exist_ok=True)
            validation_output.write_text(
                json.dumps(validation, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        print(json.dumps(validation, ensure_ascii=False))
        return 0
    matrix = build_scene0061_counterfactual_matrix(created_at=args.created_at)
    output = Path(args.output)
    if output.exists():
        parser.error(f"refusing to overwrite existing output: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(matrix, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "written", "output": str(output), "sha256": matrix["immutable_matrix_sha256"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
