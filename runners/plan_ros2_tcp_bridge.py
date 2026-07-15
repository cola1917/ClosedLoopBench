from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agents.ros2_tcp_bridge import build_ros2_tcp_bridge_plan


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Build a ROS2/TCP bridge plan without importing rclpy or TCP.")
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--role-name", default="ego_vehicle")
    parser.add_argument("--camera-profile", default="tcp_front", choices=("tcp_front", "multi_view"))
    parser.add_argument("--timeout-sec", type=float, default=0.5)
    parser.add_argument("--max-skew-sec", type=float, default=0.001)
    parser.add_argument("--qos", type=int, default=10)
    parser.add_argument("--output", default=None)
    args = parser.parse_args(argv)

    plan = build_ros2_tcp_bridge_plan(
        scenario_id=args.scenario_id,
        role_name=args.role_name,
        camera_profile=args.camera_profile,
        timeout_sec=args.timeout_sec,
        max_skew_sec=args.max_skew_sec,
        qos=args.qos,
    )

    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"status": "planned", "ros2_tcp_bridge_plan": str(output)}, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(plan, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
