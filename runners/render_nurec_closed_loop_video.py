"""Render a combined NuRec-camera + BEV video of a multi-tick closed-loop run.

Top: the 6 NuRec RGB cameras (nuScenes layout, FL/F/FR over BL/B/BR) rendered
from the ego's moving viewpoint each tick. Bottom: the top-down BEV state
explainer (OpenDRIVE lanes + ego/actor boxes + HUD). This is the M2 showcase:
the car driving through the neurally reconstructed scene with the algorithm's
actual camera inputs alongside the world state.

Per tick, the 6 camera JPEGs are read from
``<run>/algorithm_sensor_payloads/frame_{world_frame:08d}/camera_*.jpg`` (written
by the NuRec handler). Ticks whose NuRec frame is missing (best-effort runs)
render BEV-only with a "no NuRec frame" note, so the video never lies about
coverage.
"""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

CAMERA_LAYOUT = (
    ("camera_front_left", "camera_front", "camera_front_right"),
    ("camera_back_left", "camera_back", "camera_back_right"),
)


# --- OpenDRIVE sampling (self-contained) ------------------------------------

def _driving_lane_width(road: ET.Element) -> float:
    widths = []
    for lane in road.findall("./lanes/laneSection/*/lane[@type='driving']"):
        w = lane.find("width")
        if w is not None and "a" in w.attrib:
            widths.append(float(w.attrib["a"]))
    return max(2.0, min(widths[0] if widths else 3.5, 8.0))


def _sample_xodr(path: Path) -> list[dict[str, Any]]:
    root = ET.parse(path).getroot()
    if root.tag != "OpenDRIVE":
        raise ValueError(f"not an OpenDRIVE document: {path}")
    roads = []
    for road in root.findall("./road"):
        width_m = _driving_lane_width(road)
        for geom in road.findall("./planView/geometry"):
            x = float(geom.attrib.get("x", 0.0))
            y = float(geom.attrib.get("y", 0.0))
            hdg = float(geom.attrib.get("hdg", 0.0))
            length = max(float(geom.attrib.get("length", 0.0)), 0.0)
            count = max(2, int(math.ceil(length / 2.0)) + 1)
            arc = geom.find("arc")
            curv = float(arc.attrib["curvature"]) if arc is not None else 0.0
            pts = []
            for i in range(count):
                d = length * i / (count - 1)
                if abs(curv) < 1e-12:
                    pts.append((x + d * math.cos(hdg), y + d * math.sin(hdg)))
                else:
                    pts.append((
                        x + (math.sin(hdg + curv * d) - math.sin(hdg)) / curv,
                        y - (math.cos(hdg + curv * d) - math.cos(hdg)) / curv,
                    ))
            roads.append({"points": pts, "width_m": width_m})
    return roads


def _offset_polyline(points, offset_m):
    left, right = [], []
    for i, pt in enumerate(points):
        prev = points[max(0, i - 1)]
        nxt = points[min(len(points) - 1, i + 1)]
        dx, dy = nxt[0] - prev[0], nxt[1] - prev[1]
        length = math.hypot(dx, dy)
        nx, ny = (-dy / length, dx / length) if length > 1e-9 else (0.0, 1.0)
        left.append((pt[0] + nx * offset_m, pt[1] + ny * offset_m))
        right.append((pt[0] - nx * offset_m, pt[1] - ny * offset_m))
    return left, right


def _bbox_corners(x, y, yaw_deg, hl, hw):
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + lx * c - ly * s, y + lx * s + ly * c)
            for lx, ly in ((-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw))]


def _actor_color(kind, is_ego):
    if is_ego:
        return (0, 200, 255)
    if kind == "pedestrian":
        return (255, 190, 72)
    if kind == "vehicle":
        return (105, 210, 120)
    return (188, 188, 188)


def _draw_bev(cv2, np, canvas, region, frames, idx, metrics, roads, bounds, trail):
    x0, y0, w, h = region
    min_x, max_x, min_y, max_y = bounds
    pad = 24
    scale = min((w - 2 * pad) / max(max_x - min_x, 1.0), (h - 2 * pad) / max(max_y - min_y, 1.0))

    def scr(pt):
        return (x0 + int(pad + (pt[0] - min_x) * scale),
                y0 + int(h - pad - (pt[1] - min_y) * scale))

    cv2.rectangle(canvas, (x0, y0), (x0 + w, y0 + h), (24, 27, 32), -1)
    for road in roads:
        pts = road["points"]
        if len(pts) < 2:
            continue
        left, right = _offset_polyline(pts, road["width_m"] / 2.0)
        surface = np.asarray([scr(p) for p in (*left, *reversed(right))], dtype=np.int32)
        cv2.fillPoly(canvas, [surface], (52, 57, 64), cv2.LINE_AA)
        cv2.polylines(canvas, [np.asarray([scr(p) for p in pts], dtype=np.int32)], False, (76, 82, 90), 1, cv2.LINE_AA)

    row = frames[idx]
    ego = row["ego_pose"]
    if len(trail) > 1:
        cv2.polylines(canvas, [np.asarray([scr(p) for p in trail], dtype=np.int32)], False, (0, 150, 200), 2, cv2.LINE_AA)
    for st in (row.get("actor_states") or {}).values():
        pose = st["pose"]
        ext = st.get("extent_m") or {}
        hl = max(float(ext.get("x", 0.9)), 0.25)
        hw = max(float(ext.get("y", 0.9)), 0.25)
        color = _actor_color(str(st.get("actor_type") or "vehicle"), False)
        pc = np.asarray([scr(c) for c in _bbox_corners(float(pose["x"]), float(pose["y"]), float(pose["yaw"]), hl, hw)], dtype=np.int32)
        cv2.fillPoly(canvas, [pc], tuple(v // 4 for v in color))
        cv2.polylines(canvas, [pc], True, color, 2, cv2.LINE_AA)
    color = _actor_color("vehicle", True)
    pc = np.asarray([scr(c) for c in _bbox_corners(float(ego["x"]), float(ego["y"]), float(ego["yaw"]), 2.4, 1.0)], dtype=np.int32)
    cv2.fillPoly(canvas, [pc], tuple(v // 4 for v in color))
    cv2.polylines(canvas, [pc], True, color, 3, cv2.LINE_AA)
    yaw = math.radians(float(ego["yaw"]))
    cv2.arrowedLine(canvas, scr((float(ego["x"]), float(ego["y"]))),
                    scr((float(ego["x"]) + 3.2 * math.cos(yaw), float(ego["y"]) + 3.2 * math.sin(yaw))),
                    color, 2, cv2.LINE_AA, tipLength=0.3)

    m = metrics[idx] if idx < len(metrics) else {}
    progress = m.get("route_progress")
    speed = float(row.get("ego_speed_mps") or 0.0)
    t_sec = float(row.get("simulation_time_sec") or (idx * 0.05))
    prog = f"{progress*100:.1f}%" if isinstance(progress, (int, float)) else "n/a"
    cv2.putText(canvas, f"BEV  tick {idx:03d}/{len(frames)}   t {t_sec:5.2f}s   speed {speed:4.1f} m/s   route {prog}",
                (x0 + 12, y0 + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (210, 216, 224), 1, cv2.LINE_AA)


def render_video(*, run_dir: Path, opendrive_path: Path, output_path: Path,
                 fps: int, cam_w: int, cam_h: int, bev_h: int, trail_len: int) -> dict[str, Any]:
    try:
        import cv2
        import numpy as np
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("OpenCV (cv2) and NumPy are required") from exc

    frames = [json.loads(l) for l in (run_dir / "frame_trace.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()]
    if not frames:
        raise ValueError("frame_trace.jsonl is empty")
    mpath = run_dir / "metrics_trace.jsonl"
    metrics = [json.loads(l) for l in mpath.read_text(encoding="utf-8").splitlines() if l.strip()] if mpath.is_file() else []
    roads = _sample_xodr(opendrive_path)
    payload_root = run_dir / "algorithm_sensor_payloads"

    # world viewport
    xs, ys = [], []
    for row in frames:
        xs.append(float(row["ego_pose"]["x"])); ys.append(float(row["ego_pose"]["y"]))
        for st in (row.get("actor_states") or {}).values():
            xs.append(float(st["pose"]["x"])); ys.append(float(st["pose"]["y"]))
    for road in roads:
        for px, py in road["points"]:
            xs.append(px); ys.append(py)
    bounds = (min(xs) - 8, max(xs) + 8, min(ys) - 8, max(ys) + 8)

    grid_w = cam_w * 3
    canvas_w = grid_w
    canvas_h = cam_h * 2 + bev_h
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (canvas_w, canvas_h))
    if not writer.isOpened():
        raise RuntimeError(f"cv2.VideoWriter could not open {output_path}")

    sample_dir = run_dir / "nurec_bev_frames"
    sample_dir.mkdir(exist_ok=True)
    sample_idx = {0, len(frames) // 2, len(frames) - 1}
    trail: list[tuple[float, float]] = []
    nurec_present = 0

    for idx, row in enumerate(frames):
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        canvas[:] = (18, 20, 24)
        world_frame = row.get("snapshot_frame") or row.get("world_tick_frame")
        frame_dir = payload_root / f"frame_{int(world_frame):08d}" if world_frame is not None else None
        have_nurec = bool(frame_dir and frame_dir.is_dir())
        if have_nurec:
            nurec_present += 1
        for r, cam_row in enumerate(CAMERA_LAYOUT):
            for c, cam in enumerate(cam_row):
                cell = np.zeros((cam_h, cam_w, 3), dtype=np.uint8)
                cell[:] = (30, 33, 38)
                img = None
                if frame_dir is not None:
                    jpg = frame_dir / f"{cam}.jpg"
                    if jpg.is_file():
                        raw = cv2.imread(str(jpg), cv2.IMREAD_COLOR)
                        if raw is not None:
                            img = cv2.resize(raw, (cam_w, cam_h), interpolation=cv2.INTER_AREA)
                if img is not None:
                    cell = img
                else:
                    cv2.putText(cell, "no NuRec frame", (14, cam_h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (90, 90, 110), 1, cv2.LINE_AA)
                cv2.putText(cell, cam, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(cell, cam, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
                y0, x0 = r * cam_h, c * cam_w
                canvas[y0:y0 + cam_h, x0:x0 + cam_w] = cell

        _draw_bev(cv2, np, canvas, (0, cam_h * 2, canvas_w, bev_h), frames, idx, metrics, roads, bounds, trail)
        ego = row["ego_pose"]
        trail.append((float(ego["x"]), float(ego["y"])))
        trail[:] = trail[-trail_len:]
        writer.write(canvas)
        if idx in sample_idx:
            cv2.imwrite(str(sample_dir / f"nb_{idx:04d}.png"), canvas)

    writer.release()
    summary = {
        "status": "rendered",
        "frames": len(frames),
        "nurec_frames_present": nurec_present,
        "fps": fps,
        "video": str(output_path.resolve()),
        "canvas": [canvas_w, canvas_h],
        "sample_pngs": sorted(str(p) for p in sample_dir.glob("nb_*.png")),
    }
    (run_dir / "nurec_bev_render_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Render combined NuRec-camera + BEV MP4 from a multi-tick run dir.")
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--opendrive", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--cam-w", type=int, default=427)
    parser.add_argument("--cam-h", type=int, default=240)
    parser.add_argument("--bev-h", type=int, default=520)
    parser.add_argument("--trail-len", type=int, default=80)
    args = parser.parse_args(argv)
    try:
        summary = render_video(
            run_dir=args.run_dir.expanduser().resolve(),
            opendrive_path=args.opendrive.expanduser().resolve(),
            output_path=args.output.expanduser().resolve(),
            fps=args.fps, cam_w=args.cam_w, cam_h=args.cam_h, bev_h=args.bev_h, trail_len=args.trail_len,
        )
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "failed", "detail": f"{type(exc).__name__}: {exc}"}, ensure_ascii=False))
        return 2
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
