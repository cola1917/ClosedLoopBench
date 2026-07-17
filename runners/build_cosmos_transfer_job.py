from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.cosmos_transfer import CosmosTransferError, build_cosmos_transfer_job


def _pairs(values: list[str], option: str) -> dict[str, str]:
    result = {}
    for value in values:
        key, separator, item = value.partition("=")
        if not separator or not key or not item or key in result:
            raise CosmosTransferError(f"{option} requires unique TYPE=VALUE entries")
        result[key] = item
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build an offline Cosmos Transfer2.5 job after multimodal acceptance."
    )
    parser.add_argument("--accepted-run", required=True)
    parser.add_argument("--rgb-video", required=True)
    parser.add_argument("--control", action="append", default=[], metavar="TYPE=MP4")
    parser.add_argument("--control-weight", action="append", default=[], metavar="TYPE=WEIGHT")
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--negative-prompt")
    parser.add_argument("--frame-count", required=True, type=int)
    parser.add_argument("--fps", required=True, type=float)
    parser.add_argument("--width", required=True, type=int)
    parser.add_argument("--height", required=True, type=int)
    parser.add_argument("--resolution", default="480", choices=("256", "480", "512", "720"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        controls = _pairs(args.control, "--control")
        weights = {
            key: float(value)
            for key, value in _pairs(args.control_weight, "--control-weight").items()
        }
        job = build_cosmos_transfer_job(
            args.accepted_run,
            args.rgb_video,
            controls,
            prompt=args.prompt,
            frame_count=args.frame_count,
            frames_per_sec=args.fps,
            width=args.width,
            height=args.height,
            resolution=args.resolution,
            seed=args.seed,
            negative_prompt=args.negative_prompt,
            control_weights=weights,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(job, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, CosmosTransferError) as exc:
        parser.error(str(exc))
    print(json.dumps(job, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
