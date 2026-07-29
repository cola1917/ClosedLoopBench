from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_candidate_config import NuRecCandidateConfigError, derive_candidate_config
from adapters.nurec_reconstruction_smoke import load_config


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Derive a bounded NuRec candidate config from passed M8 selections.")
    parser.add_argument("--source-config", required=True, type=Path)
    parser.add_argument("--scene-object-registry", required=True, type=Path)
    parser.add_argument("--render-selection", required=True, type=Path)
    parser.add_argument("--editable-quality-window-manifest", required=True, type=Path)
    parser.add_argument("--max-samples-per-epoch", type=int, default=1000)
    parser.add_argument("--max-epochs", type=int, default=1)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"refusing to overwrite immutable candidate config: {args.output}")
    try:
        result = derive_candidate_config(
            load_config(args.source_config),
            _load(args.scene_object_registry),
            _load(args.render_selection),
            _load(args.editable_quality_window_manifest),
            max_samples_per_epoch=args.max_samples_per_epoch,
            max_epochs=args.max_epochs,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        _dump(result, args.output)
    except (OSError, ValueError, json.JSONDecodeError, NuRecCandidateConfigError) as exc:
        parser.error(str(exc))
    print(json.dumps({"status": "passed", "output": str(args.output)}, ensure_ascii=False))
    return 0


def _load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _dump(value: dict, path: Path) -> None:
    try:
        import yaml
    except ImportError:
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    else:
        path.write_text(yaml.safe_dump(value, sort_keys=False, allow_unicode=False), encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
