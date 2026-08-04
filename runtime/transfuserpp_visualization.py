from __future__ import annotations

import json
import hashlib
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from agents.plugin_contract import strict_json_loads
from agents.transfuserpp_contract import validate_intermediate_record


_RESAMPLING = getattr(Image, "Resampling", Image)


BEV_PALETTE_RGB = np.asarray(
    [
        [0, 0, 0],
        [200, 200, 200],
        [255, 255, 255],
        [255, 255, 0],
        [157, 234, 50],
        [160, 160, 0],
        [0, 255, 0],
        [255, 255, 0],
        [255, 0, 0],
        [30, 170, 250],
        [0, 255, 0],
    ],
    dtype=np.uint8,
)


def render_intermediate_debug_frame(
    record: Mapping[str, Any], output: str | Path, *, evidence_root: str | Path | None = None
) -> Path:
    row = validate_intermediate_record(record)
    output = Path(output)
    if output.exists():
        raise ValueError(f"refusing to overwrite visualization: {output}")
    root = Path(evidence_root).resolve() if evidence_root is not None else None
    rgb_path = _resolve_ref(row["inputs"]["camera_front"], root)
    dense_path = _resolve_ref(row["dense_outputs"], root)
    if not rgb_path.is_file() or not dense_path.is_file():
        raise ValueError("visualization inputs are unavailable")
    if _sha256(rgb_path) != row["inputs"]["camera_front"]["sha256"]:
        raise ValueError("camera_front source SHA-256 mismatch")
    if _sha256(dense_path) != row["dense_outputs"]["sha256"]:
        raise ValueError("dense output SHA-256 mismatch")

    rgb = Image.open(rgb_path).convert("RGB")
    rgb.thumbnail((720, 405), _RESAMPLING.LANCZOS)
    with np.load(dense_path) as dense:
        if "bev_semantic_labels" not in dense:
            raise ValueError("dense output has no bev_semantic_labels")
        labels = dense["bev_semantic_labels"]
    if labels.ndim != 2 or int(labels.max(initial=0)) >= len(BEV_PALETTE_RGB):
        raise ValueError("BEV semantic labels are invalid")
    # Up is ego-forward and right is ego-right in the display. The upstream
    # grid indexes ego-forward on rows and ego-right on columns.
    bev = np.flip(labels, axis=0)
    bev_image = Image.fromarray(BEV_PALETTE_RGB[bev], mode="RGB").resize(
        (512, 512), _RESAMPLING.NEAREST
    )
    draw_bev = ImageDraw.Draw(bev_image)
    grid = (row.get("dynamic_bev_proxy") or {}).get("grid") or {}
    for box in row["outputs"].get("bounding_boxes_ego") or []:
        if isinstance(box, list) and len(box) >= 9:
            center = _bev_pixel(box[0], box[1], grid, bev_image.size)
            color = (255, 165, 0) if int(round(float(box[7]))) == 0 else (0, 255, 0)
            draw_bev.ellipse(
                (center[0] - 4, center[1] - 4, center[0] + 4, center[1] + 4),
                outline=color,
                width=2,
            )
    for proxy in row.get("actor_proxies") or []:
        center = proxy.get("center_ego_m")
        if isinstance(center, list) and len(center) >= 2:
            pixel = _bev_pixel(center[0], center[1], grid, bev_image.size)
            draw_bev.rectangle(
                (pixel[0] - 5, pixel[1] - 5, pixel[0] + 5, pixel[1] + 5),
                outline=(0, 255, 255),
                width=2,
            )
    waypoints = row["outputs"].get("route_checkpoints_ego_m") or []
    waypoint_pixels = [
        _bev_pixel(point[0], point[1], grid, bev_image.size) for point in waypoints
    ]
    if len(waypoint_pixels) >= 2:
        draw_bev.line(waypoint_pixels, fill=(255, 0, 255), width=3)

    canvas = Image.new("RGB", (1400, 560), (20, 20, 20))
    canvas.paste(rgb, (20, 50))
    canvas.paste(bev_image, (760, 24))
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default()
    draw.text(
        (20, 20),
        "NuRec raw-source camera_front (resized preview)",
        fill="white",
        font=font,
    )
    draw.text((760, 8), "TF++ BEV / boxes / checkpoints", fill="white", font=font)
    output_values = row["outputs"]
    control = output_values["control"]
    lines = [
        f"frame_id: {row['frame_id']}",
        f"timestamp: {row['timestamp']:.3f}s",
        f"case: {(row.get('experiment') or {}).get('case_id')}",
        f"target_speed: {output_values['target_speed_mps']:.2f} m/s",
        f"control T/S/B: {control['throttle']:.2f} / {control['steer']:.2f} / {control['brake']:.2f}",
        f"inference: {row['latency_ms']['inference']:.1f} ms",
        f"sync_error: {row['synchronization'].get('error_ms')} ms",
        "occupancy: dynamic-BEV proxy only",
        "full 3D OCC GT: unavailable",
    ]
    for index, line in enumerate(lines):
        draw.text((20, 470 + index * 11), line, fill=(235, 235, 235), font=font)
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)
    return output


def load_and_render(
    record_path: str | Path,
    output: str | Path,
    *,
    evidence_root: str | Path | None = None,
) -> Path:
    record = strict_json_loads(Path(record_path).read_text(encoding="utf-8"))
    return render_intermediate_debug_frame(record, output, evidence_root=evidence_root)


def _resolve_ref(reference: Mapping[str, Any], root: Path | None) -> Path:
    declared = Path(str(reference.get("path") or ""))
    if declared.is_file():
        return declared
    host_value = str(reference.get("host_path") or "")
    if host_value and Path(host_value).is_file():
        return Path(host_value)
    relative = str(reference.get("relative_path") or "").replace("\\", "/")
    relative_path = Path(relative)
    if root is not None and relative and not relative_path.is_absolute() and ".." not in relative_path.parts:
        candidate = (root / relative_path).resolve()
        try:
            candidate.relative_to(root)
        except ValueError:
            return declared
        if candidate.is_file():
            return candidate
    return declared


def _bev_pixel(
    x_forward: Any,
    y_right: Any,
    grid: Mapping[str, Any],
    size: tuple[int, int],
) -> tuple[int, int]:
    min_x = float(grid.get("min_x_m", -32.0))
    max_x = float(grid.get("max_x_m", 32.0))
    min_y = float(grid.get("min_y_m", -32.0))
    max_y = float(grid.get("max_y_m", 32.0))
    px = int((float(y_right) - min_y) / (max_y - min_y) * size[0])
    py = int((max_x - float(x_forward)) / (max_x - min_x) * size[1])
    return max(0, min(size[0] - 1, px)), max(0, min(size[1] - 1, py))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
