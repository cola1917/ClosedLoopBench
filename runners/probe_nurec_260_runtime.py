from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_260_client import build_nurec_260_client
from adapters.nurec_multimodal import NuRecMultimodalError


def query_runtime(
    config_path: Path,
    probe_frame_path: Path | None = None,
    *,
    require_renderable_lidar: bool = False,
    client_factory=build_nurec_260_client,
) -> dict[str, Any]:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("config must contain a JSON object")
    if require_renderable_lidar and probe_frame_path is None:
        raise NuRecMultimodalError(
            "--require-renderable-lidar requires --probe-frame"
        )
    client = client_factory(config)
    try:
        inventory = client.query_runtime_inventory()
        if inventory.get("status") != "capability_only":
            raise NuRecMultimodalError(
                "NRE capability inventory returned an unexpected status"
            )
        if probe_frame_path is None:
            return inventory

        frame_bytes = probe_frame_path.read_bytes()
        frame = json.loads(frame_bytes)
        if not isinstance(frame, dict):
            raise ValueError("probe frame must contain a JSON object")
        evidence = client.dispatch_frame(frame)
        if evidence.get("status") != "passed":
            issues = evidence.get("issues") or ["unknown render failure"]
            raise NuRecMultimodalError(
                "NRE renderability probe failed: " + ", ".join(map(str, issues))
            )
        lidar_records = [
            row for row in evidence.get("records", []) if row.get("modality") == "lidar"
        ]
        rgb_records = [
            row for row in evidence.get("records", []) if row.get("modality") == "rgb"
        ]
        if not lidar_records or not rgb_records:
            raise NuRecMultimodalError(
                "NRE renderability probe requires both RGB and LiDAR records"
            )
        point_counts = [
            int((row.get("response_metadata") or {}).get("point_count") or 0)
            for row in lidar_records
        ]
        if any(count < 1 for count in point_counts):
            raise NuRecMultimodalError(
                "NRE renderability probe returned an empty LiDAR response"
            )
        inventory["lidar"]["render_verified"] = True
        inventory["lidar"]["verified_device_types"] = sorted(
            {
                str(request["sensor"]["parameters"].get("device_type") or "PANDAR128").upper()
                for request in frame["modalities"]["lidar"]["requests"]
            }
        )
        inventory["lidar"]["probe_point_counts"] = point_counts
        inventory["render_probe"] = {
            "probe_frame_sha256": hashlib.sha256(frame_bytes).hexdigest(),
            "frame_id": evidence.get("frame_id"),
            "dynamic_object_sha256": evidence.get("dynamic_object_sha256"),
            "rgb_record_count": len(rgb_records),
            "lidar_record_count": len(lidar_records),
            "evidence": evidence,
            "status": "passed",
        }
        inventory["status"] = "passed"
        return inventory
    finally:
        client.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Query live NRE 26.04 capabilities and optionally prove non-empty "
            "RGB/LiDAR rendering with a synchronized frame."
        )
    )
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--probe-frame", type=Path)
    parser.add_argument("--require-renderable-lidar", action="store_true")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    try:
        result = query_runtime(
            args.config,
            args.probe_frame,
            require_renderable_lidar=args.require_renderable_lidar,
        )
    except (OSError, ValueError, NuRecMultimodalError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"status": result["status"], "output": str(args.output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
