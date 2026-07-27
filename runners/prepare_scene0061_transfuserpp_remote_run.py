from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path

from agents.plugin_contract import strict_json_loads
from runtime.scene0061_transfuserpp_remote import (
    Scene0061TransFuserPPRemoteError,
    prepare_scene0061_transfuserpp_remote_run,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build one fail-closed scene-0061 TF++ S0/S2/S4 remote run bundle."
    )
    parser.add_argument("--base-run-config", type=Path, required=True)
    parser.add_argument("--runtime-template", type=Path, required=True)
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--case-id", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--event-timestamp-sec", type=float)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output_dir.exists():
            parser.error(f"refusing to overwrite output directory: {args.output_dir}")
        base_run_config = strict_json_loads(
            args.base_run_config.read_text(encoding="utf-8")
        )
        # Freeze the sidecar actor binding-set FILE to the focused case's
        # control modes alongside the embedded per-actor contracts; the NuRec
        # handler cross-checks both and fails closed on any divergence.
        base_bindings_path = Path(
            str((base_run_config.get("nurec_runtime") or {}).get("actor_bindings") or "")
        )
        if not base_bindings_path.is_file():
            parser.error(
                f"base config actor_bindings file not found: {base_bindings_path}"
            )
        frozen_bindings_target = (
            args.output_dir / "runtime" / "actor_bindings.case-frozen.json"
        ).resolve()
        run_config, runtime_config, bundle, frozen_bindings = (
            prepare_scene0061_transfuserpp_remote_run(
                base_run_config,
                strict_json_loads(args.runtime_template.read_text(encoding="utf-8")),
                strict_json_loads(args.matrix.read_text(encoding="utf-8")),
                case_id=args.case_id,
                seed=args.seed,
                event_timestamp_sec=args.event_timestamp_sec,
                base_actor_bindings=strict_json_loads(
                    base_bindings_path.read_text(encoding="utf-8")
                ),
                actor_bindings_out_path=str(frozen_bindings_target),
            )
        )
        args.output_dir.mkdir(parents=True, exist_ok=False)
        outputs = {
            "carla_run_config.json": run_config,
            "runtime/transfuserpp.runtime.json": runtime_config,
            "remote_run_bundle.json": bundle,
        }
        for name, value in outputs.items():
            target = args.output_dir / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(
                json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                encoding="utf-8",
            )
        assert frozen_bindings is not None
        frozen_bindings_target.parent.mkdir(parents=True, exist_ok=True)
        frozen_bindings_target.write_bytes(
            base64.b64decode(frozen_bindings["file_bytes_b64"])
        )
        print(json.dumps(bundle, ensure_ascii=False, sort_keys=True))
        return 0
    except (
        OSError,
        ValueError,
        json.JSONDecodeError,
        Scene0061TransFuserPPRemoteError,
    ) as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
