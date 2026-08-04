"""Render one hash-bound six-camera grid beside a same-tick BEV snapshot.

The camera overlays are deliberately derived from an existing calibrated
visibility audit.  They are a diagnostic presentation of the audit's
``calibrated_3d_box_projection`` results, not detector output and not a new
visibility acceptance result.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import tempfile
from pathlib import Path
from typing import Any, Mapping

from PIL import Image, ImageDraw, ImageFont

from runners.render_closed_loop_bev_snapshot import RGB_COLORS, render_snapshot


CAMERA_ORDER = (
    "camera_front_left",
    "camera_front",
    "camera_front_right",
    "camera_back_left",
    "camera_back",
    "camera_back_right",
)

PANEL_BACKGROUND = (9, 11, 15)
PANEL_BORDER = (74, 80, 88)
PANEL_TEXT = (236, 240, 246)
_RESAMPLE_LANCZOS = getattr(getattr(Image, "Resampling", Image), "LANCZOS")


def _read_json_object(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"{label} is not valid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain a JSON object: {path}")
    return value


def _read_frames(run_dir: Path) -> list[dict[str, Any]]:
    path = run_dir / "frame_trace.jsonl"
    if not path.is_file():
        raise FileNotFoundError(f"frame_trace.jsonl not found in {run_dir}")
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows:
        raise ValueError("frame_trace.jsonl is empty")
    if not all(isinstance(row, dict) for row in rows):
        raise ValueError("frame_trace.jsonl must contain JSON objects")
    return rows


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _resolve_payload(payload_root: Path, relative_path: Any) -> Path:
    if not isinstance(relative_path, str) or not relative_path:
        raise ValueError("visibility audit camera record lacks relative_path")
    root = payload_root.resolve()
    candidate = (root / relative_path).resolve()
    if candidate != root and root not in candidate.parents:
        raise ValueError(f"visibility audit payload escapes payload root: {relative_path}")
    if not candidate.is_file():
        raise FileNotFoundError(f"visibility audit payload is unavailable: {candidate}")
    return candidate


def _select_audit_frame(
    audit: Mapping[str, Any], frame: Mapping[str, Any]
) -> Mapping[str, Any]:
    world_tick_frame = frame.get("world_tick_frame")
    if not isinstance(world_tick_frame, int):
        raise ValueError("frame_trace row lacks integer world_tick_frame")
    matches = [
        row
        for row in audit.get("frames", [])
        if isinstance(row, Mapping) and row.get("world_tick_frame") == world_tick_frame
    ]
    if len(matches) != 1:
        raise ValueError(
            f"expected exactly one visibility-audit frame for world_tick_frame {world_tick_frame}, "
            f"found {len(matches)}"
        )
    audit_time = matches[0].get("simulation_time_sec")
    frame_time = frame.get("simulation_time_sec")
    if not isinstance(audit_time, (int, float)) or not isinstance(frame_time, (int, float)):
        raise ValueError("frame trace and visibility audit need numeric simulation_time_sec")
    if abs(float(audit_time) - float(frame_time)) > 1e-4:
        raise ValueError(
            "frame trace and visibility audit disagree on simulation_time_sec for "
            f"world_tick_frame {world_tick_frame}"
        )
    return matches[0]


def _camera_records(audit_frame: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    records = audit_frame.get("cameras")
    if not isinstance(records, list):
        raise ValueError("visibility audit frame lacks cameras")
    by_name: dict[str, Mapping[str, Any]] = {}
    for record in records:
        if not isinstance(record, Mapping):
            continue
        sensor_id = record.get("sensor_id")
        if not isinstance(sensor_id, str):
            continue
        if sensor_id in by_name:
            raise ValueError(f"visibility audit has duplicate camera record: {sensor_id}")
        by_name[sensor_id] = record
    missing = [sensor_id for sensor_id in CAMERA_ORDER if sensor_id not in by_name]
    unexpected = sorted(set(by_name) - set(CAMERA_ORDER))
    if missing or unexpected:
        raise ValueError(
            "visibility audit must bind exactly the formal six-camera set; "
            f"missing={missing} unexpected={unexpected}"
        )
    return by_name


def _actor_types(frame: Mapping[str, Any]) -> dict[str, str]:
    states = frame.get("actor_states")
    if not isinstance(states, Mapping):
        return {}
    result: dict[str, str] = {}
    for object_id, state in states.items():
        if isinstance(state, Mapping):
            result[str(object_id)] = str(state.get("actor_type") or "other").lower()
    return result


def _overlay_records(
    audit: Mapping[str, Any],
    *,
    world_tick_frame: int,
    max_distance_m: float | None,
    actor_types: Mapping[str, str],
    include_unbound_objects: bool,
) -> tuple[
    dict[str, list[Mapping[str, Any]]],
    dict[str, int],
    dict[str, int],
    dict[str, int],
]:
    result = {sensor_id: [] for sensor_id in CAMERA_ORDER}
    candidate_counts = {sensor_id: 0 for sensor_id in CAMERA_ORDER}
    excluded_by_distance_counts = {sensor_id: 0 for sensor_id in CAMERA_ORDER}
    excluded_unbound_counts = {sensor_id: 0 for sensor_id in CAMERA_ORDER}
    observations = audit.get("observations")
    if not isinstance(observations, list):
        raise ValueError("visibility audit lacks observations")
    for observation in observations:
        if not isinstance(observation, Mapping):
            continue
        if observation.get("frame_id") != world_tick_frame:
            continue
        if observation.get("observation_kind") != "calibrated_3d_box_projection":
            continue
        if observation.get("safety_relevant") is not True:
            continue
        camera = observation.get("camera")
        if camera not in result:
            raise ValueError(f"visibility audit has unknown camera in observation: {camera}")
        projection = observation.get("projection")
        box = projection.get("bbox_xyxy_px") if isinstance(projection, Mapping) else None
        if not isinstance(box, list) or len(box) != 4 or not all(
            isinstance(value, (int, float)) for value in box
        ):
            raise ValueError(
                "calibrated_3d_box_projection needs a numeric bbox_xyxy_px: "
                f"camera={camera} object={observation.get('object_id')}"
            )
        if float(box[2]) <= float(box[0]) or float(box[3]) <= float(box[1]):
            raise ValueError(
                "calibrated_3d_box_projection has a non-positive bbox: "
                f"camera={camera} object={observation.get('object_id')}"
            )
        candidate_counts[camera] += 1
        if max_distance_m is not None:
            distance_m = projection.get("distance_to_ego_m")
            if not isinstance(distance_m, (int, float)):
                raise ValueError(
                    "calibrated_3d_box_projection needs numeric distance_to_ego_m when "
                    f"a display range is requested: camera={camera} "
                    f"object={observation.get('object_id')}"
                )
            if float(distance_m) > max_distance_m:
                excluded_by_distance_counts[camera] += 1
                continue
        if not include_unbound_objects and str(observation.get("object_id") or "") not in actor_types:
            excluded_unbound_counts[camera] += 1
            continue
        result[camera].append(observation)
    return result, candidate_counts, excluded_by_distance_counts, excluded_unbound_counts


def _color_for_actor(actor_type: str) -> tuple[int, int, int]:
    return RGB_COLORS.get(actor_type, RGB_COLORS["other"])


def _render_camera_panel(
    *,
    source_path: Path,
    camera_name: str,
    observations: list[Mapping[str, Any]],
    actor_types: Mapping[str, str],
    width: int,
    max_distance_m: float | None,
) -> Image.Image:
    if width < 160:
        raise ValueError("camera cell width must be at least 160 pixels")
    with Image.open(source_path) as opened:
        source = opened.convert("RGB")
    source_width, source_height = source.size
    if source_width < 1 or source_height < 1:
        raise ValueError(f"camera payload has invalid dimensions: {source_path}")
    height = max(1, round(source_height * width / source_width))
    image = source.resize((width, height), _RESAMPLE_LANCZOS)
    draw = ImageDraw.Draw(image)
    scale_x = width / source_width
    scale_y = height / source_height
    for observation in observations:
        projection = observation["projection"]
        x0, y0, x1, y1 = (float(value) for value in projection["bbox_xyxy_px"])
        box = (
            max(0, min(width - 1, round(x0 * scale_x))),
            max(0, min(height - 1, round(y0 * scale_y))),
            max(0, min(width - 1, round(x1 * scale_x))),
            max(0, min(height - 1, round(y1 * scale_y))),
        )
        object_id = str(observation.get("object_id") or "")
        color = _color_for_actor(actor_types.get(object_id, "other"))
        draw.rectangle(box, outline=color, width=2)

    title_height = 20
    draw.rectangle((0, 0, width - 1, title_height), fill=PANEL_BACKGROUND)
    draw.line((0, title_height, width - 1, title_height), fill=PANEL_BORDER, width=1)
    range_label = f"  <= {max_distance_m:g}m" if max_distance_m is not None else ""
    draw.text(
        (7, 4), f"{camera_name}{range_label}", fill=PANEL_TEXT, font=ImageFont.load_default()
    )
    return image


def render_six_camera_bev_snapshot(
    *,
    run_dir: Path,
    visibility_audit_path: Path,
    payload_root: Path,
    output_path: Path,
    opendrive_path: Path | None = None,
    nuscenes_map_path: Path | None = None,
    scenario_ir_path: Path | None = None,
    frame_index: int = 0,
    camera_cell_width: int = 480,
    bev_width: int = 960,
    max_distance_m: float | None = 20.0,
    include_unbound_objects: bool = False,
) -> dict[str, Any]:
    """Build one same-tick six-camera grid and visual-only BEV diagnostic."""

    output_path = output_path.resolve()
    metadata_path = output_path.with_suffix(".json")
    if output_path.exists() or metadata_path.exists():
        existing = output_path if output_path.exists() else metadata_path
        raise ValueError(f"refusing to overwrite diagnostic output: {existing}")
    if bev_width < 320:
        raise ValueError("BEV width must be at least 320 pixels")
    if max_distance_m is not None and max_distance_m <= 0:
        raise ValueError("max_distance_m must be positive when supplied")

    run_dir = run_dir.resolve()
    frames = _read_frames(run_dir)
    if not 0 <= frame_index < len(frames):
        raise ValueError(f"frame_index must be in [0, {len(frames) - 1}]")
    frame = frames[frame_index]
    world_tick_frame = frame.get("world_tick_frame")
    if not isinstance(world_tick_frame, int):
        raise ValueError("frame_trace row lacks integer world_tick_frame")
    audit = _read_json_object(visibility_audit_path.resolve(), label="visibility audit")
    audit_frame = _select_audit_frame(audit, frame)
    cameras = _camera_records(audit_frame)
    actor_types = _actor_types(frame)
    (
        overlays,
        overlay_candidate_counts,
        excluded_by_distance_counts,
        excluded_unbound_counts,
    ) = _overlay_records(
        audit,
        world_tick_frame=world_tick_frame,
        max_distance_m=max_distance_m,
        actor_types=actor_types,
        include_unbound_objects=include_unbound_objects,
    )

    panels: list[Image.Image] = []
    payload_records: dict[str, dict[str, Any]] = {}
    for camera_name in CAMERA_ORDER:
        record = cameras[camera_name]
        source_path = _resolve_payload(payload_root, record.get("relative_path"))
        expected_hash = record.get("payload_sha256")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise ValueError(f"visibility audit camera record lacks SHA-256: {camera_name}")
        actual_hash = _sha256(source_path)
        if actual_hash != expected_hash:
            raise ValueError(
                f"camera payload SHA-256 does not match visibility audit: {camera_name}"
            )
        panels.append(
            _render_camera_panel(
                source_path=source_path,
                camera_name=camera_name,
                observations=overlays[camera_name],
                actor_types=actor_types,
                width=camera_cell_width,
                max_distance_m=max_distance_m,
            )
        )
        payload_records[camera_name] = {
            "relative_path": str(record["relative_path"]),
            "sha256": actual_hash,
            "calibrated_bbox_candidate_count": overlay_candidate_counts[camera_name],
            "excluded_by_distance_count": excluded_by_distance_counts[camera_name],
            "excluded_unbound_count": excluded_unbound_counts[camera_name],
            "calibrated_bbox_count": len(overlays[camera_name]),
        }

    cell_height = panels[0].height
    if any(panel.size != (camera_cell_width, cell_height) for panel in panels):
        raise ValueError("formal camera payloads do not share one aspect ratio")
    grid_width, grid_height = camera_cell_width * 3, cell_height * 2
    grid = Image.new("RGB", (grid_width, grid_height), PANEL_BACKGROUND)
    for index, panel in enumerate(panels):
        row, column = divmod(index, 3)
        grid.paste(panel, (column * camera_cell_width, row * cell_height))

    with tempfile.TemporaryDirectory(prefix="closedloopbench-bev-") as temporary:
        bev_path = Path(temporary) / "bev.png"
        bev_result = render_snapshot(
            run_dir=run_dir,
            output_path=bev_path,
            opendrive_path=opendrive_path.resolve() if opendrive_path else None,
            nuscenes_map_path=nuscenes_map_path.resolve() if nuscenes_map_path else None,
            scenario_ir_path=scenario_ir_path.resolve() if scenario_ir_path else None,
            frame_index=frame_index,
            width=bev_width,
            height=max(grid_height, 240),
            max_actor_distance_m=max_distance_m,
        )
        with Image.open(bev_path) as opened:
            bev = opened.convert("RGB")
    bev = bev.resize((bev_width, grid_height), _RESAMPLE_LANCZOS)

    composite = Image.new("RGB", (grid_width + bev_width, grid_height), PANEL_BACKGROUND)
    composite.paste(grid, (0, 0))
    composite.paste(bev, (grid_width, 0))
    divider = ImageDraw.Draw(composite)
    divider.line((grid_width, 0, grid_width, grid_height - 1), fill=PANEL_BORDER, width=2)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    composite.save(output_path)
    metadata = {
        "schema_version": "closedloopbench.six_camera_bev_snapshot.v1",
        "status": "rendered",
        "output": str(output_path),
        "frame_index": frame_index,
        "world_tick_frame": world_tick_frame,
        "simulation_time_sec": float(frame["simulation_time_sec"]),
        "camera_order": list(CAMERA_ORDER),
        "camera_payloads": payload_records,
        "max_distance_m": max_distance_m,
        "include_unbound_objects": include_unbound_objects,
        "calibrated_bbox_candidate_count": sum(overlay_candidate_counts.values()),
        "excluded_by_distance_count": sum(excluded_by_distance_counts.values()),
        "excluded_unbound_count": sum(excluded_unbound_counts.values()),
        "calibrated_bbox_count": sum(len(records) for records in overlays.values()),
        "overlay_source": "visibility_audit.calibrated_3d_box_projection",
        "actor_color_source": "frame_trace.actor_states.actor_type",
        "map_source": bev_result["map_source"],
        "map_feature_counts": bev_result["map_feature_counts"],
        "interpretation": (
            "visual diagnostic only; calibrated 3D-box envelopes are not detector output "
            "and do not independently establish visibility, collision, or lane truth"
        ),
    }
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return metadata


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Render a hash-bound six-camera grid beside a same-tick BEV diagnostic."
    )
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--visibility-audit", required=True, type=Path)
    parser.add_argument("--payload-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--opendrive", type=Path)
    parser.add_argument("--nuscenes-map", type=Path)
    parser.add_argument("--scenario-ir", type=Path)
    parser.add_argument("--frame-index", type=int, default=0)
    parser.add_argument("--camera-cell-width", type=int, default=480)
    parser.add_argument("--bev-width", type=int, default=960)
    parser.add_argument(
        "--max-distance-m",
        type=float,
        default=20.0,
        help="show only audit projections and runtime actors within this ego-centred range",
    )
    parser.add_argument(
        "--include-unbound-objects",
        action="store_true",
        help="also draw audit objects absent from this tick's CARLA actor_states",
    )
    args = parser.parse_args(argv)
    try:
        result = render_six_camera_bev_snapshot(
            run_dir=args.run_dir,
            visibility_audit_path=args.visibility_audit,
            payload_root=args.payload_root,
            output_path=args.output,
            opendrive_path=args.opendrive,
            nuscenes_map_path=args.nuscenes_map,
            scenario_ir_path=args.scenario_ir,
            frame_index=args.frame_index,
            camera_cell_width=args.camera_cell_width,
            bev_width=args.bev_width,
            max_distance_m=args.max_distance_m,
            include_unbound_objects=args.include_unbound_objects,
        )
    except (OSError, ValueError) as exc:
        print(
            json.dumps({"status": "failed", "detail": f"{type(exc).__name__}: {exc}"}),
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
