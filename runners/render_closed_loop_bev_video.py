"""Render a top-down BEV video of a native closed-loop run.

Consumes the machine-readable evidence a native run already writes
(``frame_trace.jsonl`` for per-tick ego/actor poses and, when present,
``metrics_trace.jsonl`` for per-tick route_progress / collision / ttc) plus the
loaded OpenDRIVE road, and produces an MP4 that shows the ego bounding box
driving the route with the scripted actors, a recent ego trail, heading arrows,
and a compact HUD.

This is deliberately self-contained: it needs only cv2 + numpy and never
imports the NuRec adapters, so the M1 (native, no-NuRec) visualization has no
dependency on the neural-render path. The scene-0061 OpenDRIVE is a local lane
map, not a city mesh, so the view is an honest state explainer -- lane surfaces
and boxes -- not a photorealistic camera feed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


# ----------------------------------------------------------------------------
# OpenDRIVE road sampling (small, self-contained; mirrors the dual-window one).
# ----------------------------------------------------------------------------

def _driving_lane_width(road: ET.Element) -> float:
    widths = []
    for lane in road.findall("./lanes/laneSection/*/lane[@type='driving']"):
        width = lane.find("width")
        if width is not None and "a" in width.attrib:
            widths.append(float(width.attrib["a"]))
    return max(2.0, min(widths[0] if widths else 3.5, 8.0))


def _sample_xodr(path: Path) -> list[dict[str, Any]]:
    root = ET.parse(path).getroot()
    if root.tag != "OpenDRIVE":
        raise ValueError(f"not an OpenDRIVE document: {path}")
    roads: list[dict[str, Any]] = []
    for road in root.findall("./road"):
        width_m = _driving_lane_width(road)
        for geometry in road.findall("./planView/geometry"):
            x = float(geometry.attrib.get("x", 0.0))
            y = float(geometry.attrib.get("y", 0.0))
            heading = float(geometry.attrib.get("hdg", 0.0))
            length = max(float(geometry.attrib.get("length", 0.0)), 0.0)
            count = max(2, int(math.ceil(length / 2.0)) + 1)
            arc = geometry.find("arc")
            curvature = float(arc.attrib["curvature"]) if arc is not None else 0.0
            points = []
            for index in range(count):
                distance = length * index / (count - 1)
                if abs(curvature) < 1e-12:
                    px = x + distance * math.cos(heading)
                    py = y + distance * math.sin(heading)
                else:
                    px = x + (math.sin(heading + curvature * distance) - math.sin(heading)) / curvature
                    py = y - (math.cos(heading + curvature * distance) - math.cos(heading)) / curvature
                points.append((px, py))
            roads.append({"points": points, "width_m": width_m})
    return roads


def _offset_polyline(points: list[tuple[float, float]], offset_m: float):
    left, right = [], []
    for index, point in enumerate(points):
        prev = points[max(0, index - 1)]
        nxt = points[min(len(points) - 1, index + 1)]
        dx, dy = nxt[0] - prev[0], nxt[1] - prev[1]
        length = math.hypot(dx, dy)
        nx, ny = (-dy / length, dx / length) if length > 1e-9 else (0.0, 1.0)
        left.append((point[0] + nx * offset_m, point[1] + ny * offset_m))
        right.append((point[0] - nx * offset_m, point[1] - ny * offset_m))
    return left, right


# ----------------------------------------------------------------------------
# Rendering
# ----------------------------------------------------------------------------

def _bbox_corners(x: float, y: float, yaw_deg: float, half_len: float, half_wid: float):
    yaw = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    corners = []
    for lx, ly in ((-half_len, -half_wid), (half_len, -half_wid), (half_len, half_wid), (-half_len, half_wid)):
        corners.append((x + lx * cos_y - ly * sin_y, y + lx * sin_y + ly * cos_y))
    return corners


def _actor_color(kind: str, is_ego: bool) -> tuple[int, int, int]:
    if is_ego:
        return (0, 200, 255)  # amber (BGR) -- the ego
    if kind == "pedestrian":
        return (255, 190, 72)  # light blue
    if kind == "vehicle":
        return (105, 210, 120)  # green
    return (188, 188, 188)


def render_video(
    *,
    run_dir: Path,
    opendrive_path: Path,
    output_path: Path,
    fps: int,
    width: int,
    height: int,
    trail_len: int,
) -> dict[str, Any]:
    try:
        import cv2
        import numpy as np
    except ImportError as exc:  # pragma: no cover - runtime env guard
        raise RuntimeError("OpenCV (cv2) and NumPy are required for BEV rendering") from exc

    frame_trace_path = run_dir / "frame_trace.jsonl"
    if not frame_trace_path.is_file():
        raise FileNotFoundError(f"frame_trace.jsonl not found in {run_dir}")
    frames = [json.loads(line) for line in frame_trace_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not frames:
        raise ValueError("frame_trace.jsonl is empty")

    metrics_path = run_dir / "metrics_trace.jsonl"
    metrics = []
    if metrics_path.is_file():
        metrics = [json.loads(line) for line in metrics_path.read_text(encoding="utf-8").splitlines() if line.strip()]

    roads = _sample_xodr(opendrive_path)

    # Fixed world viewport covering the whole route + all actors + road.
    xs, ys = [], []
    for row in frames:
        xs.append(float(row["ego_pose"]["x"]))
        ys.append(float(row["ego_pose"]["y"]))
        for st in (row.get("actor_states") or {}).values():
            xs.append(float(st["pose"]["x"]))
            ys.append(float(st["pose"]["y"]))
    for road in roads:
        for px, py in road["points"]:
            xs.append(px)
            ys.append(py)
    margin_world = 8.0
    min_x, max_x = min(xs) - margin_world, max(xs) + margin_world
    min_y, max_y = min(ys) - margin_world, max(ys) + margin_world
    pad = 30
    scale = min((width - 2 * pad) / max(max_x - min_x, 1.0), (height - 2 * pad) / max(max_y - min_y, 1.0))

    def screen(pt):
        sx = int(pad + (pt[0] - min_x) * scale)
        sy = int(height - pad - (pt[1] - min_y) * scale)
        return sx, sy

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_path), fourcc, float(fps), (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"cv2.VideoWriter could not open {output_path} (missing codec?)")

    sample_dir = run_dir / "bev_frames"
    sample_dir.mkdir(exist_ok=True)
    sample_indices = {0, len(frames) // 4, len(frames) // 2, (3 * len(frames)) // 4, len(frames) - 1}

    ego_trail: list[tuple[float, float]] = []
    default_ego_half = (2.4, 1.0)

    for idx, row in enumerate(frames):
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        canvas[:] = (24, 27, 32)

        # roads
        for road in roads:
            pts = road["points"]
            if len(pts) < 2:
                continue
            left, right = _offset_polyline(pts, road["width_m"] / 2.0)
            surface = np.asarray([screen(p) for p in (*left, *reversed(right))], dtype=np.int32)
            cv2.fillPoly(canvas, [surface], (52, 57, 64), cv2.LINE_AA)
            cv2.polylines(canvas, [np.asarray([screen(p) for p in pts], dtype=np.int32)], False, (76, 82, 90), 1, cv2.LINE_AA)

        # ego trail
        ego = row["ego_pose"]
        ego_trail.append((float(ego["x"]), float(ego["y"])))
        trail = ego_trail[-trail_len:]
        if len(trail) > 1:
            cv2.polylines(canvas, [np.asarray([screen(p) for p in trail], dtype=np.int32)], False, (0, 150, 200), 2, cv2.LINE_AA)

        # actors
        for st in (row.get("actor_states") or {}).values():
            pose = st["pose"]
            ext = st.get("extent_m") or {}
            hl = max(float(ext.get("x", 0.9)), 0.25)
            hw = max(float(ext.get("y", 0.9)), 0.25)
            kind = str(st.get("actor_type") or "vehicle")
            color = _actor_color(kind, False)
            corners = _bbox_corners(float(pose["x"]), float(pose["y"]), float(pose["yaw"]), hl, hw)
            pc = np.asarray([screen(c) for c in corners], dtype=np.int32)
            cv2.fillPoly(canvas, [pc], tuple(v // 4 for v in color))
            cv2.polylines(canvas, [pc], True, color, 2, cv2.LINE_AA)

        # ego box + heading
        color = _actor_color("vehicle", True)
        corners = _bbox_corners(float(ego["x"]), float(ego["y"]), float(ego["yaw"]), *default_ego_half)
        pc = np.asarray([screen(c) for c in corners], dtype=np.int32)
        cv2.fillPoly(canvas, [pc], tuple(v // 4 for v in color))
        cv2.polylines(canvas, [pc], True, color, 3, cv2.LINE_AA)
        yaw = math.radians(float(ego["yaw"]))
        tip = screen((float(ego["x"]) + 3.2 * math.cos(yaw), float(ego["y"]) + 3.2 * math.sin(yaw)))
        cv2.arrowedLine(canvas, screen((float(ego["x"]), float(ego["y"]))), tip, color, 2, cv2.LINE_AA, tipLength=0.3)

        # HUD
        m = metrics[idx] if idx < len(metrics) else {}
        progress = m.get("route_progress")
        collision = m.get("collision")
        speed = float(row.get("ego_speed_mps") or 0.0)
        t_sec = float(row.get("simulation_time_sec") or (idx * 0.05))
        cv2.rectangle(canvas, (14, 14), (430, 118), (9, 11, 15), -1)
        cv2.rectangle(canvas, (14, 14), (430, 118), (74, 80, 88), 1, cv2.LINE_AA)
        cv2.putText(canvas, "scene-0061 native closed loop (M1)", (26, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (246, 246, 246), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"tick {idx:03d}/{len(frames)}   t {t_sec:6.2f}s   speed {speed:4.1f} m/s",
                    (26, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (198, 204, 213), 1, cv2.LINE_AA)
        prog_txt = f"route_progress {progress*100:5.1f}%" if isinstance(progress, (int, float)) else "route_progress   n/a"
        cv2.putText(canvas, prog_txt, (26, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (120, 230, 140), 1, cv2.LINE_AA)
        col_txt = "COLLISION" if collision else "no collision"
        col_col = (60, 60, 255) if collision else (150, 170, 150)
        cv2.putText(canvas, col_txt, (250, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.46, col_col, 1, cv2.LINE_AA)
        # progress bar
        bar_x0, bar_y = 26, 108
        cv2.rectangle(canvas, (bar_x0, bar_y - 6), (bar_x0 + 380, bar_y), (60, 64, 70), -1)
        if isinstance(progress, (int, float)):
            cv2.rectangle(canvas, (bar_x0, bar_y - 6), (bar_x0 + int(380 * max(0.0, min(1.0, progress))), bar_y), (120, 230, 140), -1)

        writer.write(canvas)
        if idx in sample_indices:
            cv2.imwrite(str(sample_dir / f"bev_{idx:04d}.png"), canvas)

    writer.release()
    summary = {
        "status": "rendered",
        "frames": len(frames),
        "fps": fps,
        "video": str(output_path.resolve()),
        "sample_pngs": sorted(str(p) for p in sample_dir.glob("bev_*.png")),
        "viewport": {"min_x": min_x, "max_x": max_x, "min_y": min_y, "max_y": max_y},
        "final_route_progress": (metrics[-1].get("route_progress") if metrics else None),
    }
    (run_dir / "bev_render_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Render a top-down BEV MP4 from a native closed-loop run directory.")
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--opendrive", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--trail-len", type=int, default=80)
    args = parser.parse_args(argv)
    try:
        summary = render_video(
            run_dir=args.run_dir.expanduser().resolve(),
            opendrive_path=args.opendrive.expanduser().resolve(),
            output_path=args.output.expanduser().resolve(),
            fps=args.fps,
            width=args.width,
            height=args.height,
            trail_len=args.trail_len,
        )
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "failed", "detail": f"{type(exc).__name__}: {exc}"}, ensure_ascii=False))
        return 2
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
