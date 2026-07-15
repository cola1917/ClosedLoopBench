from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agents.sensor_contract import build_camera_profile, build_sensor_source_contract


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build the CARLA current-tick ego observation contract without importing CARLA or ROS2."
    )
    parser.add_argument("--camera-profile", choices=("tcp_front", "multi_view"), default="tcp_front")
    parser.add_argument("--role-name", default="ego_vehicle")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    result = {
        "schema_version": "ego_observation_contract.v1",
        "source_contract": build_sensor_source_contract(),
        "camera_profile": build_camera_profile(args.camera_profile, role_name=args.role_name),
        "aggregation": {
            "policy": "same_carla_tick",
            "required_channels": ["camera", "ego_state", "route"],
            "on_missing_stale_mismatch_or_invalid": "safe_stop",
            "runtime_binding": "not_validated_without_carla_ros2_environment",
        },
    }
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
        print(json.dumps({"status": "built", "output": str(output)}))
    else:
        print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
