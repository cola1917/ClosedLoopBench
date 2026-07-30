"""Render one color-coded BEV state snapshot without requiring OpenCV.

The snapshot is a state explainer, not a detector visualization.  Every
dynamic box comes from the CARLA ``frame_trace.jsonl`` runtime state, so an
object absent from that trace is deliberately not invented in the image.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping

from PIL import Image, ImageDraw, ImageFont

from runners.render_closed_loop_bev_video import _offset_polyline, _sample_xodr


RGB_COLORS = {
    "ego": (255, 196, 0),
    "vehicle": (104, 220, 116),
    "pedestrian": (60, 202, 255),
    "two_wheeler": (205, 110, 255),
    "other": (184, 184, 184),
}

# These colors describe the source map only.  They never stand in for CARLA
# lane truth, collision geometry, or an M8 audit result.
MAP_COLORS = {
    "drivable_area": (49, 55, 62),
    "road_block": (55, 61, 68),
    "road_segment": (62, 68, 75),
    "intersection": (70, 76, 83),
    "lane_outline": (112, 120, 130),
}

MapPolygons = dict[str, list[list[tuple[float, float]]]]


def _bbox_corners(
    x: float, y: float, yaw_deg: float, half_length: float, half_width: float
) -> list[tuple[float, float]]:
    yaw = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return [
        (
            x + local_x * cos_y - local_y * sin_y,
            y + local_x * sin_y + local_y * cos_y,
        )
        for local_x, local_y in (
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
        )
    ]


def _color_for(actor_type: Any, *, ego: bool = False) -> tuple[int, int, int]:
    if ego:
        return RGB_COLORS["ego"]
    return RGB_COLORS.get(str(actor_type or "").lower(), RGB_COLORS["other"])


def _dim_color(color: tuple[int, int, int]) -> tuple[int, int, int]:
    return tuple(max(0, channel // 4) for channel in color)


def _read_frames(run_dir: Path) -> list[dict[str, Any]]:
    path = run_dir / "frame_trace.jsonl"
    if not path.is_file():
        raise FileNotFoundError(f"frame_trace.jsonl not found in {run_dir}")
    frames = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not frames:
        raise ValueError("frame_trace.jsonl is empty")
    return frames


def _load_nuscenes_map_geometry(
    map_path: Path, scenario_ir_path: Path
) -> tuple[MapPolygons, dict[str, int]]:
    """Load selected nuScenes map polygons into the Scenario IR local frame.

    This intentionally consumes the source vector-map polygons directly instead
    of pretending the limited lane-strip XODR is a complete road network.  The
    resulting geometry is used only as a BEV background.
    """

    map_data = json.loads(map_path.read_text(encoding="utf-8"))
    scenario_ir = json.loads(scenario_ir_path.read_text(encoding="utf-8"))
    if not isinstance(map_data, Mapping) or not isinstance(scenario_ir, Mapping):
        raise ValueError("nuScenes map and Scenario IR must each contain a JSON object")

    coordinate_frame = scenario_ir.get("coordinate_frame")
    if not isinstance(coordinate_frame, Mapping):
        raise ValueError("Scenario IR lacks coordinate_frame for map conversion")
    origin = coordinate_frame.get("origin_global_translation")
    yaw_deg = coordinate_frame.get("origin_global_yaw_deg")
    if not isinstance(origin, list) or len(origin) < 2 or yaw_deg is None:
        raise ValueError("Scenario IR lacks the global-to-local map transform")
    origin_x, origin_y = float(origin[0]), float(origin[1])
    yaw = math.radians(float(yaw_deg))
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

    nodes: dict[str, tuple[float, float]] = {}
    for node in map_data.get("node", []):
        if not isinstance(node, Mapping) or "token" not in node:
            continue
        try:
            nodes[str(node["token"])] = (float(node["x"]), float(node["y"]))
        except (KeyError, TypeError, ValueError):
            continue

    polygon_records = {
        str(record["token"]): record
        for record in map_data.get("polygon", [])
        if isinstance(record, Mapping) and "token" in record
    }

    def local_polygon(token: Any) -> list[tuple[float, float]] | None:
        polygon = polygon_records.get(str(token))
        if not isinstance(polygon, Mapping):
            return None
        exterior = polygon.get("exterior_node_tokens")
        if not isinstance(exterior, list):
            return None
        points: list[tuple[float, float]] = []
        for node_token in exterior:
            global_point = nodes.get(str(node_token))
            if global_point is None:
                return None
            dx, dy = global_point[0] - origin_x, global_point[1] - origin_y
            points.append(
                (
                    cos_yaw * dx + sin_yaw * dy,
                    -sin_yaw * dx + cos_yaw * dy,
                )
            )
        return points if len(points) >= 3 else None

    def tokens_for(record: Mapping[str, Any]) -> list[Any]:
        tokens: list[Any] = []
        polygon_token = record.get("polygon_token")
        if polygon_token is not None:
            tokens.append(polygon_token)
        polygon_tokens = record.get("polygon_tokens")
        if isinstance(polygon_tokens, list):
            tokens.extend(polygon_tokens)
        return tokens

    geometry: MapPolygons = {
        "drivable_area": [],
        "road_block": [],
        "road_segment": [],
        "intersection": [],
        "lane": [],
    }

    for record in map_data.get("drivable_area", []):
        if not isinstance(record, Mapping):
            continue
        for token in tokens_for(record):
            polygon = local_polygon(token)
            if polygon is not None:
                geometry["drivable_area"].append(polygon)

    for record in map_data.get("road_block", []):
        if not isinstance(record, Mapping):
            continue
        for token in tokens_for(record):
            polygon = local_polygon(token)
            if polygon is not None:
                geometry["road_block"].append(polygon)

    for record in map_data.get("road_segment", []):
        if not isinstance(record, Mapping):
            continue
        layer = "intersection" if bool(record.get("is_intersection")) else "road_segment"
        for token in tokens_for(record):
            polygon = local_polygon(token)
            if polygon is not None:
                geometry[layer].append(polygon)

    for record in map_data.get("lane", []):
        if not isinstance(record, Mapping):
            continue
        if str(record.get("lane_type", "CAR")).upper() != "CAR":
            continue
        for token in tokens_for(record):
            polygon = local_polygon(token)
            if polygon is not None:
                geometry["lane"].append(polygon)

    feature_counts = {layer: len(polygons) for layer, polygons in geometry.items()}
    if not any(feature_counts.values()):
        raise ValueError("nuScenes map has no usable drivable, road, or lane polygons")
    return geometry, feature_counts


def _viewport(
    frames: list[Mapping[str, Any]],
    roads: list[Mapping[str, Any]],
    *,
    max_actor_distance_m: float | None = None,
) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for row in frames:
        ego = row["ego_pose"]
        xs.append(float(ego["x"]))
        ys.append(float(ego["y"]))
        for state in (row.get("actor_states") or {}).values():
            pose = state.get("pose") or {}
            if "x" in pose and "y" in pose:
                if (
                    max_actor_distance_m is not None
                    and _distance_to_ego(pose, ego) > max_actor_distance_m
                ):
                    continue
                xs.append(float(pose["x"]))
                ys.append(float(pose["y"]))
    # Focus the explanatory view on the physical runtime state.  A full XODR
    # route can be hundreds of metres long and would reduce every nearby
    # pedestrian to an invisible pixel.  Roads are still drawn below, clipped
    # by the image boundary, but do not decide the camera framing.
    if len(xs) <= len(frames):
        for road in roads:
            for x, y in road["points"]:
                xs.append(float(x))
                ys.append(float(y))
    if not xs:
        raise ValueError("BEV snapshot has no world points")
    margin_m = 8.0
    return min(xs) - margin_m, max(xs) + margin_m, min(ys) - margin_m, max(ys) + margin_m


def _polygon_overlaps_viewport(
    polygon: list[tuple[float, float]],
    *,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
) -> bool:
    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    return not (max(xs) < min_x or min(xs) > max_x or max(ys) < min_y or min(ys) > max_y)


def _draw_nuscenes_map_geometry(
    draw: ImageDraw.ImageDraw,
    *,
    screen: Any,
    geometry: MapPolygons,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
) -> None:
    for layer in ("drivable_area", "road_block", "road_segment", "intersection"):
        for polygon in geometry[layer]:
            if _polygon_overlaps_viewport(
                polygon, min_x=min_x, max_x=max_x, min_y=min_y, max_y=max_y
            ):
                draw.polygon([screen(point) for point in polygon], fill=MAP_COLORS[layer])
    for polygon in geometry["lane"]:
        if _polygon_overlaps_viewport(
            polygon, min_x=min_x, max_x=max_x, min_y=min_y, max_y=max_y
        ):
            points = [screen(point) for point in polygon]
            draw.line([*points, points[0]], fill=MAP_COLORS["lane_outline"], width=1)


def _draw_box(
    draw: ImageDraw.ImageDraw,
    *,
    screen: Any,
    pose: Mapping[str, Any],
    extent: Mapping[str, Any],
    color: tuple[int, int, int],
) -> None:
    half_length = max(float(extent.get("x", 0.9)), 0.25)
    half_width = max(float(extent.get("y", 0.9)), 0.25)
    world_corners = _bbox_corners(
        float(pose["x"]),
        float(pose["y"]),
        float(pose.get("yaw", 0.0)),
        half_length,
        half_width,
    )
    corners = [screen(point) for point in world_corners]
    draw.polygon(corners, fill=_dim_color(color))
    draw.line([*corners, corners[0]], fill=color, width=2, joint="curve")

    # Small real-world boxes (especially pedestrians) can collapse to a few
    # pixels at scene scale. Keep their physical polygon, then add a same-color
    # marker so the actor remains visible without pretending it is larger.
    span_x = max(point[0] for point in corners) - min(point[0] for point in corners)
    span_y = max(point[1] for point in corners) - min(point[1] for point in corners)
    if max(span_x, span_y) < 6:
        center = screen((float(pose["x"]), float(pose["y"])))
        draw.rectangle(
            (center[0] - 3, center[1] - 3, center[0] + 3, center[1] + 3),
            outline=color,
            width=2,
        )


def _distance_to_ego(pose: Mapping[str, Any], ego: Mapping[str, Any]) -> float:
    return math.sqrt(
        (float(pose["x"]) - float(ego["x"])) ** 2
        + (float(pose["y"]) - float(ego["y"])) ** 2
        + (float(pose.get("z", 0.0)) - float(ego.get("z", 0.0))) ** 2
    )


def render_snapshot(
    *,
    run_dir: Path,
    output_path: Path,
    opendrive_path: Path | None = None,
    nuscenes_map_path: Path | None = None,
    scenario_ir_path: Path | None = None,
    frame_index: int = 0,
    width: int = 1280,
    height: int = 720,
    max_actor_distance_m: float | None = None,
) -> dict[str, Any]:
    """Render one frame with all dynamic CARLA actors in its trace.

    A direct nuScenes map and Scenario IR pair takes precedence over XODR and
    is rendered as visual-only background geometry.  It is not physical CARLA
    road or lane evidence.
    """

    if output_path.exists():
        raise ValueError(f"refusing to overwrite visualization: {output_path}")
    if width < 320 or height < 240:
        raise ValueError("BEV snapshot dimensions are too small")
    if max_actor_distance_m is not None and max_actor_distance_m <= 0:
        raise ValueError("max_actor_distance_m must be positive when supplied")
    if (nuscenes_map_path is None) != (scenario_ir_path is None):
        raise ValueError("--nuscenes-map and --scenario-ir must be supplied together")
    if nuscenes_map_path is None and opendrive_path is None:
        raise ValueError("supply --opendrive or both --nuscenes-map and --scenario-ir")

    frames = _read_frames(run_dir)
    if not 0 <= frame_index < len(frames):
        raise ValueError(f"frame_index must be in [0, {len(frames) - 1}]")
    map_geometry: MapPolygons | None = None
    if nuscenes_map_path is not None and scenario_ir_path is not None:
        map_geometry, map_feature_counts = _load_nuscenes_map_geometry(
            nuscenes_map_path, scenario_ir_path
        )
        roads: list[Mapping[str, Any]] = []
        map_source = "nuscenes_map_geometry_visual_only"
        map_label = "nuScenes polygons (visual only)"
    else:
        assert opendrive_path is not None
        roads = _sample_xodr(opendrive_path)
        map_feature_counts = {"xodr_road_geometry_segments": len(roads)}
        map_source = "xodr_lane_strips_visual_only"
        map_label = "XODR lane strips (visual only)"
    min_x, max_x, min_y, max_y = _viewport(
        frames, roads, max_actor_distance_m=max_actor_distance_m
    )
    padding = 30
    scale = min(
        (width - 2 * padding) / max(max_x - min_x, 1.0),
        (height - 2 * padding) / max(max_y - min_y, 1.0),
    )

    def screen(point: tuple[float, float]) -> tuple[int, int]:
        return (
            int(padding + (point[0] - min_x) * scale),
            int(height - padding - (point[1] - min_y) * scale),
        )

    image = Image.new("RGB", (width, height), (24, 27, 32))
    draw = ImageDraw.Draw(image)
    if map_geometry is not None:
        _draw_nuscenes_map_geometry(
            draw,
            screen=screen,
            geometry=map_geometry,
            min_x=min_x,
            max_x=max_x,
            min_y=min_y,
            max_y=max_y,
        )
    else:
        for road in roads:
            points = road["points"]
            if len(points) < 2:
                continue
            left, right = _offset_polyline(points, float(road["width_m"]) / 2.0)
            surface = [screen(point) for point in (*left, *reversed(right))]
            draw.polygon(surface, fill=(52, 57, 64))
            draw.line([screen(point) for point in points], fill=(76, 82, 90), width=1)

    row = frames[frame_index]
    ego = row["ego_pose"]
    actor_states = row.get("actor_states") or {}
    type_counts: dict[str, int] = {}
    displayed_type_counts: dict[str, int] = {}
    for state in actor_states.values():
        if not isinstance(state, Mapping):
            continue
        pose = state.get("pose")
        if not isinstance(pose, Mapping) or not {"x", "y"}.issubset(pose):
            continue
        actor_type = str(state.get("actor_type") or "other").lower()
        type_counts[actor_type] = type_counts.get(actor_type, 0) + 1
        if (
            max_actor_distance_m is not None
            and _distance_to_ego(pose, ego) > max_actor_distance_m
        ):
            continue
        displayed_type_counts[actor_type] = displayed_type_counts.get(actor_type, 0) + 1
        _draw_box(
            draw,
            screen=screen,
            pose=pose,
            extent=state.get("extent_m") or {},
            color=_color_for(actor_type),
        )

    ego_color = _color_for("vehicle", ego=True)
    _draw_box(
        draw,
        screen=screen,
        pose=ego,
        extent={"x": 2.4, "y": 1.0},
        color=ego_color,
    )
    ego_yaw = math.radians(float(ego.get("yaw", 0.0)))
    ego_center = screen((float(ego["x"]), float(ego["y"])))
    ego_tip = screen(
        (
            float(ego["x"]) + 3.2 * math.cos(ego_yaw),
            float(ego["y"]) + 3.2 * math.sin(ego_yaw),
        )
    )
    draw.line((ego_center, ego_tip), fill=ego_color, width=2)

    font = ImageFont.load_default()
    time_sec = float(row.get("simulation_time_sec") or frame_index * 0.05)
    actor_summary = (
        f"actors {sum(displayed_type_counts.values())}/{sum(type_counts.values())}"
        if max_actor_distance_m is not None
        else f"runtime actors {sum(type_counts.values())}"
    )
    legend = "  ".join(
        (
            "yellow ego",
            "green vehicle",
            "cyan pedestrian",
            "purple two-wheeler",
            "gray other",
        )
    )
    draw.rectangle((14, 14, width - 14, 80), fill=(9, 11, 15), outline=(74, 80, 88), width=1)
    draw.text(
        (26, 25),
        f"BEV tick {frame_index:03d}/{len(frames)}  t {time_sec:.2f}s  {actor_summary}",
        fill=(236, 240, 246),
        font=font,
    )
    draw.text((26, 43), f"map {map_label}", fill=(188, 196, 206), font=font)
    draw.text((26, 61), legend, fill=(188, 196, 206), font=font)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)
    return {
        "status": "rendered",
        "output": str(output_path.resolve()),
        "frame_index": frame_index,
        "frame_count": len(frames),
        "map_source": map_source,
        "map_feature_counts": map_feature_counts,
        "max_actor_distance_m": max_actor_distance_m,
        "runtime_actor_count": sum(type_counts.values()),
        "runtime_actor_type_counts": type_counts,
        "displayed_actor_count": sum(displayed_type_counts.values()),
        "displayed_actor_type_counts": displayed_type_counts,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Render one color-coded CARLA runtime BEV snapshot without OpenCV."
    )
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument(
        "--opendrive",
        type=Path,
        help="limited lane-strip XODR fallback when a nuScenes map is not supplied",
    )
    parser.add_argument(
        "--nuscenes-map",
        type=Path,
        help="raw nuScenes vector-map JSON for visual-only BEV road geometry",
    )
    parser.add_argument(
        "--scenario-ir",
        type=Path,
        help="Scenario IR supplying the nuScenes-global to scene-local transform",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--frame-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument(
        "--max-actor-distance-m",
        type=float,
        help="draw only non-ego runtime actors at or below this ego-centred range",
    )
    args = parser.parse_args(argv)
    try:
        result = render_snapshot(
            run_dir=args.run_dir.expanduser().resolve(),
            output_path=args.output.expanduser().resolve(),
            opendrive_path=(args.opendrive.expanduser().resolve() if args.opendrive else None),
            nuscenes_map_path=(
                args.nuscenes_map.expanduser().resolve() if args.nuscenes_map else None
            ),
            scenario_ir_path=(
                args.scenario_ir.expanduser().resolve() if args.scenario_ir else None
            ),
            frame_index=args.frame_index,
            width=args.width,
            height=args.height,
            max_actor_distance_m=args.max_actor_distance_m,
        )
    except (OSError, ValueError) as exc:
        print(json.dumps({"status": "failed", "detail": f"{type(exc).__name__}: {exc}"}), file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
