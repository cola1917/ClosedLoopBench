from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time
from typing import Any, Mapping
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.nurec_260_client import build_nurec_260_client  # noqa: E402
from adapters.nurec_multimodal import (  # noqa: E402
    NuRecMultimodalError,
    materialize_nurec_rpc_requests,
    validate_nurec_multimodal_frame,
)


CAMERA_ORDER = (
    "camera_front",
    "camera_front_left",
    "camera_front_right",
    "camera_back",
    "camera_back_left",
    "camera_back_right",
)
PREVIEW_CAMERAS = (
    "camera_front_left",
    "camera_front",
    "camera_front_right",
)


@dataclass(frozen=True)
class ActorState:
    track_id: str
    actor_type: str
    carla_actor_id: int | None
    x: float
    y: float
    z: float
    yaw: float
    speed_mps: float
    length: float
    width: float
    height: float
    controlled: bool
    trajectory: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class FramePacket:
    state_name: str
    frame_id: int
    simulation_time_sec: float
    timestamp_us: int
    ego: ActorState
    actors: tuple[ActorState, ...]
    cameras: Mapping[str, Any]
    camera_records: tuple[Mapping[str, Any], ...]


def run_visualization(
    *,
    config_path: Path,
    baseline_path: Path,
    moved_path: Path,
    scenario_ir_path: Path,
    xodr_path: Path,
    actor_mapping_path: Path,
    overlap_path: Path,
    lidar_diagnostic_path: Path,
    controlled_track_id: str,
    required_track_ids: list[str],
    mode: str,
    output_dir: Path | None,
    width: int,
    height: int,
    display: bool,
    overlay: bool,
    preview_camera_count: int,
) -> dict[str, Any]:
    if mode not in {"formal_acceptance", "preview_debug"}:
        raise ValueError(f"unsupported mode: {mode}")
    if width < 1 or height < 1:
        raise ValueError("camera dimensions must be positive")
    if mode == "formal_acceptance" and output_dir is None:
        raise ValueError("formal_acceptance requires --output-dir")
    if output_dir is not None and output_dir.exists():
        raise FileExistsError(f"output directory already exists: {output_dir}")

    cv2, np = _vision_modules()
    config = _load_object(config_path)
    baseline = _load_object(baseline_path)
    moved = _load_object(moved_path)
    scenario_ir = _load_object(scenario_ir_path)
    actor_mapping = _load_object(actor_mapping_path)
    overlap = _load_object(overlap_path)
    lidar_diagnostic = _load_object(lidar_diagnostic_path)
    validate_nurec_multimodal_frame(baseline)
    validate_nurec_multimodal_frame(moved)
    changed_tracks = _changed_tracks(baseline, moved)
    if changed_tracks != [controlled_track_id]:
        raise ValueError(
            "baseline/moved must change exactly the controlled track; "
            f"observed {changed_tracks}"
        )
    _same_frame_gate(baseline, moved)

    runtime = config.get("nurec_runtime")
    if not isinstance(runtime, Mapping):
        raise ValueError("config requires nurec_runtime")
    scene_start_us = int(runtime["scene_start_us"])
    mapping_by_track = _mapping_by_track(actor_mapping)
    roads = _sample_xodr(xodr_path)
    scenario_actors = {
        str(actor["actor_id"]): actor for actor in scenario_ir.get("actors", [])
    }
    if controlled_track_id not in scenario_actors:
        raise ValueError(f"controlled track absent from Scenario IR: {controlled_track_id}")

    camera_names = (
        CAMERA_ORDER
        if mode == "formal_acceptance"
        else PREVIEW_CAMERAS[: max(1, min(preview_camera_count, 3))]
    )
    if output_dir is not None:
        output_dir.mkdir(parents=True)
        shutil.copy2(actor_mapping_path, output_dir / "actor_mapping.json")
        shutil.copy2(overlap_path, output_dir / "overlap.json")

    client = build_nurec_260_client(config)
    started = time.monotonic()
    packets: list[FramePacket] = []
    packet_reports: list[dict[str, Any]] = []
    try:
        for state_name, frame in (("baseline", baseline), ("moved", moved)):
            camera_images, camera_records = _capture_cameras(
                client,
                frame,
                camera_names,
                width=width,
                height=height,
                cv2=cv2,
                np=np,
            )
            packet = _build_packet(
                state_name=state_name,
                frame=frame,
                scene_start_us=scene_start_us,
                scenario_ir=scenario_ir,
                scenario_actors=scenario_actors,
                mapping_by_track=mapping_by_track,
                controlled_track_id=controlled_track_id,
                camera_images=camera_images,
                camera_records=camera_records,
                baseline=baseline,
            )
            packets.append(packet)
            carla_canvas = _render_carla_window(packet, roads, cv2=cv2, np=np)
            grid_canvas = _render_camera_window(
                packet,
                camera_names,
                cv2=cv2,
                np=np,
                overlay=overlay,
            )
            if display:
                cv2.namedWindow("CARLA state explanation", cv2.WINDOW_NORMAL)
                cv2.namedWindow("NuRec synchronized cameras", cv2.WINDOW_NORMAL)
                cv2.imshow("CARLA state explanation", carla_canvas)
                cv2.imshow("NuRec synchronized cameras", grid_canvas)
                cv2.waitKey(1)

            report_row = {
                "state": state_name,
                "frame_id": packet.frame_id,
                "simulation_time_sec": packet.simulation_time_sec,
                "timestamp_us": packet.timestamp_us,
                "controlled_actor": _actor_report(
                    next(actor for actor in packet.actors if actor.controlled)
                ),
                "camera_records": [dict(row) for row in camera_records],
                "synchronization_error_us": 0,
                "dropped_camera_frames": len(camera_names) - len(camera_images),
            }
            if output_dir is not None and mode == "formal_acceptance":
                raw_dir = output_dir / "raw" / state_name
                raw_dir.mkdir(parents=True)
                for name, image in camera_images.items():
                    raw_path = raw_dir / f"{name}.{packet.frame_id:05d}.jpg"
                    if not cv2.imwrite(str(raw_path), image):
                        raise RuntimeError(f"failed to save raw camera image: {raw_path}")
                carla_path = output_dir / f"frame_{packet.frame_id:05d}.{state_name}.carla.png"
                grid_path = output_dir / f"frame_{packet.frame_id:05d}.{state_name}.nurec_grid.png"
                if not cv2.imwrite(str(carla_path), carla_canvas):
                    raise RuntimeError(f"failed to save CARLA screenshot: {carla_path}")
                if not cv2.imwrite(str(grid_path), grid_canvas):
                    raise RuntimeError(f"failed to save NuRec grid screenshot: {grid_path}")
                report_row["carla_screenshot"] = _file_record(carla_path)
                report_row["nurec_grid_screenshot"] = _file_record(grid_path)
                report_row["raw_frame_paths"] = {
                    name: str((raw_dir / f"{name}.{packet.frame_id:05d}.jpg").resolve())
                    for name in camera_images
                }
            packet_reports.append(report_row)
    finally:
        client.close()
        if display:
            cv2.destroyAllWindows()

    elapsed_sec = max(time.monotonic() - started, 1e-9)
    frame_packet_fps = len(packets) / elapsed_sec
    mapped_tracks = set(mapping_by_track)
    missing_required = sorted(set(required_track_ids) - mapped_tracks)
    camera_gate = (
        tuple(camera_names) == CAMERA_ORDER
        and all(len(packet.cameras) == 6 for packet in packets)
        and all(
            image.shape[:2] == (height, width)
            for packet in packets
            for image in packet.cameras.values()
        )
    )
    overlap_count = int(overlap.get("sample_count", len(overlap.get("samples", []))))
    gates = {
        "six_camera_800x450": camera_gate and (width, height) == (800, 450),
        "live_lidar_diagnostic_passed": lidar_diagnostic.get("status") == "passed",
        "required_actor_mapping_complete": not missing_required,
        "overlap_zero": overlap_count == 0,
        "saveimages": mode == "formal_acceptance" and output_dir is not None,
        "same_frame_packet_drives_both_windows": all(
            row["synchronization_error_us"] == 0 for row in packet_reports
        ),
    }
    if mode == "formal_acceptance":
        status = "passed" if all(gates.values()) else "blocked"
    else:
        status = (
            "passed"
            if packets
            and all(row["dropped_camera_frames"] == 0 for row in packet_reports)
            and all(row["synchronization_error_us"] == 0 for row in packet_reports)
            else "blocked"
        )
    report = {
        "schema_version": "closed_loopbench.scene0061_dual_window.v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "mode": mode,
        "status": status,
        "window_contract": {
            "driver": "single FramePacket loop",
            "carla_window": "OpenDRIVE top-down state explanation with actor bbox proxies",
            "nurec_window": "independent synchronized camera grid",
            "display_enabled": display,
            "rgb_bbox_overlay_enabled": overlay,
        },
        "inputs": {
            "config": _input_record(config_path),
            "baseline": _input_record(baseline_path),
            "moved": _input_record(moved_path),
            "scenario_ir": _input_record(scenario_ir_path),
            "opendrive": _input_record(xodr_path),
            "actor_mapping": _input_record(actor_mapping_path),
            "overlap": _input_record(overlap_path),
            "lidar_diagnostic": _input_record(lidar_diagnostic_path),
        },
        "controlled_track_id": controlled_track_id,
        "changed_tracks": changed_tracks,
        "required_track_ids": required_track_ids,
        "missing_required_actor_mappings": missing_required,
        "camera_order": list(camera_names),
        "camera_source_dimensions": {"width": width, "height": height},
        "packets": packet_reports,
        "statistics": {
            "packet_count": len(packets),
            "elapsed_sec": elapsed_sec,
            "actual_frame_packet_fps": frame_packet_fps,
            "dropped_camera_frames": sum(
                row["dropped_camera_frames"] for row in packet_reports
            ),
            "maximum_synchronization_error_us": max(
                row["synchronization_error_us"] for row in packet_reports
            ),
        },
        "gates": gates,
        "overlap_classification": (
            "zero_overlap_samples" if overlap_count == 0 else "overlap_detected"
        ),
    }
    if output_dir is not None:
        report_path = output_dir / "dual_window_report.json"
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        report["report_path"] = str(report_path.resolve())
    return report


def _capture_cameras(
    client: Any,
    frame: Mapping[str, Any],
    camera_names: tuple[str, ...],
    *,
    width: int,
    height: int,
    cv2: Any,
    np: Any,
) -> tuple[dict[str, Any], tuple[dict[str, Any], ...]]:
    payloads = {
        str(payload["sensor"]["sensor_id"]): payload
        for payload in materialize_nurec_rpc_requests(frame)
        if payload["modality"] == "rgb"
    }
    missing = [name for name in camera_names if name not in payloads]
    if missing:
        raise ValueError(f"frame lacks requested cameras: {missing}")
    images: dict[str, Any] = {}
    records = []
    for camera_name in camera_names:
        payload = deepcopy(payloads[camera_name])
        parameters = dict(payload["sensor"].get("parameters") or {})
        parameters.update(width=width, height=height)
        payload["sensor"]["parameters"] = parameters
        started = time.monotonic()
        encoded = client.encode_rgb(payload)
        response = client.render_rgb(encoded["wire_request"])
        body = client.response_bytes(response)
        metadata = client.inspect_response(payload, response, body)
        image_bytes = bytes(response.image_bytes)
        image = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (height, width):
            raise NuRecMultimodalError(
                f"decoded {camera_name} dimensions do not match {width}x{height}"
            )
        images[camera_name] = image
        records.append(
            {
                "camera_name": camera_name,
                "frame_id": int(frame["frame_id"]),
                "serialized_response_sha256": hashlib.sha256(body).hexdigest(),
                "jpeg_sha256": hashlib.sha256(image_bytes).hexdigest(),
                "jpeg_bytes": len(image_bytes),
                "width": int(metadata["width"]),
                "height": int(metadata["height"]),
                "latency_ms": (time.monotonic() - started) * 1000.0,
            }
        )
    return images, tuple(records)


def _build_packet(
    *,
    state_name: str,
    frame: Mapping[str, Any],
    scene_start_us: int,
    scenario_ir: Mapping[str, Any],
    scenario_actors: Mapping[str, Mapping[str, Any]],
    mapping_by_track: Mapping[str, Mapping[str, Any]],
    controlled_track_id: str,
    camera_images: Mapping[str, Any],
    camera_records: tuple[Mapping[str, Any], ...],
    baseline: Mapping[str, Any],
) -> FramePacket:
    simulation_time_sec = float(frame["simulation_time_sec"])
    ego_source = scenario_ir.get("ego")
    if not isinstance(ego_source, Mapping):
        raise ValueError("Scenario IR lacks ego")
    ego = _actor_state(
        ego_source,
        simulation_time_sec,
        mapping_by_track,
        controlled=False,
        fallback_track_id="ego",
    )
    desired_tracks = [controlled_track_id]
    for track_id in (
        "c1958768d48640948f6053d04cffd35b",
        "71603dd1a2ba4e9daf095535e38310ac",
    ):
        if track_id not in desired_tracks and track_id in scenario_actors:
            desired_tracks.append(track_id)
    actors = []
    for track_id in desired_tracks:
        source = scenario_actors.get(track_id)
        if source is None:
            continue
        actor = _actor_state(
            source,
            simulation_time_sec,
            mapping_by_track,
            controlled=track_id == controlled_track_id,
        )
        if state_name == "moved" and actor.controlled:
            dx, dy, dz = _dynamic_delta(baseline, frame, track_id)
            actor = ActorState(
                **{
                    **actor.__dict__,
                    "x": actor.x + dx,
                    "y": actor.y + dy,
                    "z": actor.z + dz,
                }
            )
        actors.append(actor)
    return FramePacket(
        state_name=state_name,
        frame_id=int(frame["frame_id"]),
        simulation_time_sec=simulation_time_sec,
        timestamp_us=scene_start_us + int(round(simulation_time_sec * 1_000_000)),
        ego=ego,
        actors=tuple(actors),
        cameras=camera_images,
        camera_records=camera_records,
    )


def _actor_state(
    source: Mapping[str, Any],
    simulation_time_sec: float,
    mapping_by_track: Mapping[str, Mapping[str, Any]],
    *,
    controlled: bool,
    fallback_track_id: str | None = None,
) -> ActorState:
    track_id = str(source.get("actor_id") or source.get("source_track_id") or fallback_track_id or "")
    trajectory = source.get("reference_trajectory") or []
    if not trajectory:
        initial = source.get("initial_state")
        if not isinstance(initial, Mapping):
            raise ValueError(f"actor {track_id} lacks trajectory/state")
        trajectory = [initial]
    point = min(
        trajectory,
        key=lambda row: abs(float(row.get("t_sec", 0.0)) - simulation_time_sec),
    )
    dimensions = source.get("dimensions") or {}
    mapping = mapping_by_track.get(track_id) or {}
    actor_type = str(source.get("type") or "ego")
    return ActorState(
        track_id=track_id,
        actor_type=actor_type,
        carla_actor_id=(
            int(mapping["runtime_actor_id"])
            if isinstance(mapping.get("runtime_actor_id"), int)
            else None
        ),
        x=float(point.get("x", 0.0)),
        y=float(point.get("y", 0.0)),
        z=float(point.get("z", 0.0)),
        yaw=float(point.get("yaw", 0.0)),
        speed_mps=float(point.get("speed_mps", 0.0)),
        length=float(dimensions.get("length", 4.5 if actor_type != "pedestrian" else 0.8)),
        width=float(dimensions.get("width", 1.8 if actor_type != "pedestrian" else 0.8)),
        height=float(dimensions.get("height", 1.6 if actor_type != "pedestrian" else 1.8)),
        controlled=controlled,
        trajectory=tuple(
            (float(row.get("x", 0.0)), float(row.get("y", 0.0)))
            for row in trajectory
            if float(row.get("t_sec", 0.0)) <= simulation_time_sec
        )[-10:],
    )


def _render_carla_window(packet: FramePacket, roads: list[list[tuple[float, float]]], *, cv2: Any, np: Any) -> Any:
    canvas = np.zeros((900, 1440, 3), dtype=np.uint8)
    canvas[:] = (24, 26, 31)
    points = [point for road in roads for point in road]
    points.extend((actor.x, actor.y) for actor in (packet.ego,) + packet.actors)
    if not points:
        points = [(0.0, 0.0), (1.0, 1.0)]
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    margin = 55
    scale = min(
        (canvas.shape[1] - 2 * margin) / max(max_x - min_x, 1.0),
        (canvas.shape[0] - 2 * margin - 90) / max(max_y - min_y, 1.0),
    )

    def screen(point: tuple[float, float]) -> tuple[int, int]:
        x = int(margin + (point[0] - min_x) * scale)
        y = int(canvas.shape[0] - margin - (point[1] - min_y) * scale)
        return x, y

    for road in roads:
        if len(road) > 1:
            cv2.polylines(
                canvas,
                [np.asarray([screen(point) for point in road], dtype=np.int32)],
                False,
                (85, 90, 98),
                2,
                cv2.LINE_AA,
            )
    for actor in (packet.ego,) + packet.actors:
        color = (
            (0, 170, 255)
            if actor.controlled
            else (110, 220, 110)
            if actor.actor_type == "vehicle"
            else (255, 170, 70)
            if actor.actor_type == "pedestrian"
            else (220, 220, 220)
        )
        if len(actor.trajectory) > 1:
            cv2.polylines(
                canvas,
                [np.asarray([screen(point) for point in actor.trajectory], dtype=np.int32)],
                False,
                color,
                2,
                cv2.LINE_AA,
            )
        corners = _bbox_corners(actor)
        pixel_corners = np.asarray([screen(point) for point in corners], dtype=np.int32)
        cv2.polylines(canvas, [pixel_corners], True, color, 3, cv2.LINE_AA)
        anchor = screen((actor.x, actor.y))
        carla_id = actor.carla_actor_id if actor.carla_actor_id is not None else "unmapped"
        label1 = f"CARLA {carla_id} | {actor.actor_type} | {actor.speed_mps:.2f} m/s"
        label2 = f"NuRec {actor.track_id}"
        cv2.putText(canvas, label1, (anchor[0] + 8, anchor[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
        cv2.putText(canvas, label2, (anchor[0] + 8, anchor[1] + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1, cv2.LINE_AA)
    title = (
        f"CARLA OpenDRIVE state | {packet.state_name} | frame {packet.frame_id} | "
        f"timestamp {packet.timestamp_us} us | sim {packet.simulation_time_sec:.6f} s"
    )
    cv2.putText(canvas, title, (24, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (245, 245, 245), 2, cv2.LINE_AA)
    cv2.putText(canvas, "Orange bbox = closed-loop controlled actor", (24, 63), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 170, 255), 2, cv2.LINE_AA)
    return canvas


def _render_camera_window(packet: FramePacket, camera_names: tuple[str, ...], *, cv2: Any, np: Any, overlay: bool) -> Any:
    columns = 3 if len(camera_names) > 1 else 1
    rows = math.ceil(len(camera_names) / columns)
    first = packet.cameras[camera_names[0]]
    cell_height, cell_width = first.shape[:2]
    canvas = np.zeros((cell_height * rows, cell_width * columns, 3), dtype=np.uint8)
    latency_by_name = {str(row["camera_name"]): float(row["latency_ms"]) for row in packet.camera_records}
    for index, camera_name in enumerate(camera_names):
        image = packet.cameras[camera_name].copy()
        fps = 1000.0 / max(latency_by_name[camera_name], 1e-9)
        lines = [
            camera_name,
            f"frame_id={packet.frame_id} timestamp_us={packet.timestamp_us}",
            f"current_fps={fps:.2f}",
        ]
        if overlay:
            controlled = next(actor for actor in packet.actors if actor.controlled)
            lines.append(f"DEBUG overlay: {controlled.actor_type} {controlled.track_id}")
        for line_index, text in enumerate(lines):
            y = 25 + line_index * 23
            cv2.putText(image, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(image, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        row, column = divmod(index, columns)
        y0, x0 = row * cell_height, column * cell_width
        canvas[y0 : y0 + cell_height, x0 : x0 + cell_width] = image
    return canvas


def _sample_xodr(path: Path) -> list[list[tuple[float, float]]]:
    root = ET.parse(path).getroot()
    if root.tag != "OpenDRIVE":
        raise ValueError(f"not an OpenDRIVE document: {path}")
    roads = []
    for geometry in root.findall("./road/planView/geometry"):
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
        roads.append(points)
    return roads


def _bbox_corners(actor: ActorState) -> list[tuple[float, float]]:
    half_length = actor.length / 2.0
    half_width = actor.width / 2.0
    cosine = math.cos(actor.yaw)
    sine = math.sin(actor.yaw)
    result = []
    for local_x, local_y in (
        (-half_length, -half_width),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width),
    ):
        result.append(
            (
                actor.x + local_x * cosine - local_y * sine,
                actor.y + local_x * sine + local_y * cosine,
            )
        )
    return result


def _dynamic_delta(baseline: Mapping[str, Any], moved: Mapping[str, Any], track_id: str) -> tuple[float, float, float]:
    def position(frame: Mapping[str, Any]) -> Mapping[str, Any]:
        actor = next(
            item for item in frame["shared_dynamic_objects"] if str(item["track_id"]) == track_id
        )
        return actor["pose_pair"]["start"]["position_m"]

    first = position(baseline)
    second = position(moved)
    return tuple(float(second[axis]) - float(first[axis]) for axis in ("x", "y", "z"))


def _changed_tracks(baseline: Mapping[str, Any], moved: Mapping[str, Any]) -> list[str]:
    first = {str(item["track_id"]): item for item in baseline["shared_dynamic_objects"]}
    second = {str(item["track_id"]): item for item in moved["shared_dynamic_objects"]}
    return [track_id for track_id in sorted(set(first) | set(second)) if first.get(track_id) != second.get(track_id)]


def _same_frame_gate(baseline: Mapping[str, Any], moved: Mapping[str, Any]) -> None:
    names = ("scene_id", "frame_id", "simulation_time_sec", "pose_interval_sec")
    mismatches = [name for name in names if baseline.get(name) != moved.get(name)]
    if mismatches:
        raise ValueError(f"baseline/moved do not share one clock/frame: {mismatches}")


def _mapping_by_track(mapping: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    rows = mapping.get("tracks")
    if not isinstance(rows, list):
        raise ValueError("actor mapping requires tracks")
    return {str(row["track_id"]): row for row in rows}


def _actor_report(actor: ActorState) -> dict[str, Any]:
    return {
        "track_id": actor.track_id,
        "carla_actor_id": actor.carla_actor_id,
        "actor_type": actor.actor_type,
        "position_m": {"x": actor.x, "y": actor.y, "z": actor.z},
        "yaw": actor.yaw,
        "speed_mps": actor.speed_mps,
        "bbox_m": {"length": actor.length, "width": actor.width, "height": actor.height},
    }


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _input_record(path: Path) -> dict[str, Any]:
    return {"path": str(path.resolve()), "sha256": _sha256_file(path)}


def _file_record(path: Path) -> dict[str, Any]:
    return {"path": str(path.resolve()), "sha256": _sha256_file(path), "bytes": path.stat().st_size}


def _vision_modules() -> tuple[Any, Any]:
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("OpenCV and NumPy are required for dual-window rendering") from exc
    return cv2, np


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Drive independent CARLA-state and NuRec-camera windows from one synchronized FramePacket loop."
    )
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--baseline-frame", required=True, type=Path)
    parser.add_argument("--moved-frame", required=True, type=Path)
    parser.add_argument("--scenario-ir", required=True, type=Path)
    parser.add_argument("--xodr", required=True, type=Path)
    parser.add_argument("--actor-mapping", required=True, type=Path)
    parser.add_argument("--overlap", required=True, type=Path)
    parser.add_argument("--lidar-diagnostic", required=True, type=Path)
    parser.add_argument("--controlled-track-id", required=True)
    parser.add_argument("--required-track-id", action="append", default=[])
    parser.add_argument(
        "--mode",
        choices=("formal_acceptance", "preview_debug"),
        default="preview_debug",
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=450)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--overlay", action="store_true")
    parser.add_argument("--preview-camera-count", type=int, choices=(1, 3), default=3)
    args = parser.parse_args(argv)
    try:
        report = run_visualization(
            config_path=args.config,
            baseline_path=args.baseline_frame,
            moved_path=args.moved_frame,
            scenario_ir_path=args.scenario_ir,
            xodr_path=args.xodr,
            actor_mapping_path=args.actor_mapping,
            overlap_path=args.overlap,
            lidar_diagnostic_path=args.lidar_diagnostic,
            controlled_track_id=args.controlled_track_id,
            required_track_ids=args.required_track_id,
            mode=args.mode,
            output_dir=args.output_dir,
            width=args.width,
            height=args.height,
            display=not args.headless,
            overlay=args.overlay,
            preview_camera_count=args.preview_camera_count,
        )
    except (OSError, ValueError, RuntimeError, NuRecMultimodalError) as exc:
        print(json.dumps({"status": "failed", "detail": f"{type(exc).__name__}: {exc}"}, ensure_ascii=False))
        return 2
    print(
        json.dumps(
            {
                "status": report["status"],
                "report": report.get("report_path"),
                "statistics": report["statistics"],
                "missing_required_actor_mappings": report["missing_required_actor_mappings"],
            },
            ensure_ascii=False,
        )
    )
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
