from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from agents.transfuserpp_contract import (
    build_runtime_manifest,
    directory_snapshot_sha256,
    repository_snapshot_sha256,
    runtime_config_schema,
)
from agents.plugin_contract import file_sha256, strict_json_loads


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build or audit a fail-closed TransFuser++ v5 runtime binding."
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-prepared", action="store_true")
    parser.add_argument(
        "--print-identities",
        action="store_true",
        help="Print detected repo/checkpoint/config identities for binding; does not edit config.",
    )
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            parser.error(f"refusing to overwrite existing output: {args.output}")
        config = strict_json_loads(args.config.read_text(encoding="utf-8"))
        if args.print_identities:
            repo = Path(str(config.get("repo_path") or ""))
            checkpoint = Path(str(config.get("checkpoint_path") or ""))
            model_config = Path(str(config.get("model_config_path") or ""))
            carla_agents = Path(str(config.get("carla_agents_path") or ""))
            detected = {
                "repo_sha256": repository_snapshot_sha256(repo),
                "checkpoint_sha256": file_sha256(checkpoint) if checkpoint.is_file() else None,
                "model_config_sha256": file_sha256(model_config) if model_config.is_file() else None,
                "carla_agents_sha256": directory_snapshot_sha256(carla_agents),
            }
            print(json.dumps(detected, indent=2, sort_keys=True))
        manifest = build_runtime_manifest(config)
        manifest["config_schema"] = runtime_config_schema()
        container_files = {
            "dockerfile": PROJECT_ROOT / "docker" / "transfuserpp" / "Dockerfile",
            "requirements": PROJECT_ROOT
            / "docker"
            / "transfuserpp"
            / "requirements.runtime.txt",
            "compose": PROJECT_ROOT / "docker" / "compose.transfuserpp.yml",
            "entrypoint": PROJECT_ROOT / "docker" / "algorithm" / "entrypoint.sh",
        }
        manifest["container_contract"] = {
            "base_image": "ros:humble-ros-base-jammy",
            "python": "3.10",
            "gpu_required": True,
            "files": {
                name: {
                    "path": str(path.relative_to(PROJECT_ROOT)),
                    "sha256": file_sha256(path) if path.is_file() else None,
                }
                for name, path in container_files.items()
            },
            "external_repo_and_checkpoint_mounted_read_only": True,
            "carla_pythonapi_agents_mounted_read_only": True,
            "carla_service_in_compose": False,
        }
        missing_container_contract = [
            name for name, path in container_files.items() if not path.is_file()
        ]
        if missing_container_contract:
            manifest["problems"] = sorted(
                set(manifest.get("problems") or [])
                | {
                    f"container_contract_file_missing:{name}"
                    for name in missing_container_contract
                }
            )
            manifest["execution_status"] = "blocked"
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(manifest, ensure_ascii=False, sort_keys=True))
        return 2 if args.require_prepared and manifest["execution_status"] != "prepared" else 0
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
