"""Materialize a frame-bound TF++ trace from the reconstructed scene outputs.

The reconstruction package stores 800x450 RGB and NRE-axis LiDAR payloads.
TF++'s formal sensor contract consumes 1600x900 JPEG and CARLA-axis XYZI, so
this builder records the deterministic RGB handling and LiDAR axis transform
in the trace instead of hiding either conversion in the runtime.

RGB quality note (M8 follow-up): upsampling the native 800x450 NuRec camera
canvas to the formal 1600x900 canvas with BILINEAR degraded car-patch edge
energy by ~15x (Laplacian variance 198 -> 14 at the same canvas resolution),
which the TF++ CenterNet head could not recover: reconstructed-input inference
matched 0/39 GT boxes while CARLA-native 1600x900 RGB matched 25/39.  The r5
builder therefore materializes the camera at its native 800x450 resolution and
binds an 800x450 camera-adaptation contract, so the adapter's resize becomes a
single 800x450 -> 1024x512 upsampling pass instead of a double round trip.
The NVIDIA harmonizer RGB canvas is 2400x900; it is center-cropped to 16:9 and
downsampled to 800x450 with LANCZOS before the same contract binding.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agents.plugin_contract import strict_json_loads
from agents.transfuserpp_contract import camera_adaptation_contract, validate_observation
from adapters.open_loop_bbox_binding import frame_binding, load_actor_manifest
from runners.capture_open_loop_transfuserpp_stage_a import _route_for_frame


TRACE_SCHEMA = "transfuserpp_triplicate_observation_trace.v1"
RECONSTRUCTED_SOURCE = "reconstructed_rgb_lidar"
HARMONIZED_SOURCE = "harmonized_rgb_reconstructed_lidar"
SOURCE_RGB_SIZE = (800, 450)
FORMAL_RGB_SIZE = (800, 450)
HARMONIZER_RGB_SIZE = (2400, 900)
# NRE 26.04 renders LiDAR in x_forward_y_right_z_up; the CARLA sensor frame is
# also x_forward_y_right_z_up, so the response is a +90 deg rotation about z
# (x'=-y, y'=x, z'=z).  Same-frame NN registration against CARLA native clouds
# additionally shows a systematic vertical offset: the NRE cloud sits ~1.0 m
# higher than the CARLA cloud (median z offset -1.02 m, std 0.11 m over the 39
# scored frames; 3D <1 m overlap rises from 25-47% to 62-75% after the -1.0 m
# correction).  The triplicate trace materialization therefore carries the
# rotation plus a -1.0 m z translation; the runtime RPC axis-normalization
# contract (origin-preserving) stays a pure rotation and applies no height
# correction.
LIDAR_RESPONSE_TO_SENSOR = [
    0.0,
    -1.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    -1.0,
    0.0,
    0.0,
    0.0,
    1.0,
]


class TriplicateTraceError(ValueError):
    """Raised when a route cannot be bound without guessing."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = [
        strict_json_loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not isinstance(rows, list) or any(not isinstance(row, dict) for row in rows):
        raise TriplicateTraceError(f"frame metadata must contain JSON objects: {path}")
    return rows


def _load_json(path: Path) -> dict[str, Any]:
    value = strict_json_loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise TriplicateTraceError(f"JSON object required: {path}")
    return value


def _normalise_track(value: Any) -> list[dict[str, float]]:
    if not isinstance(value, list) or len(value) < 2:
        raise TriplicateTraceError("Scenario IR ego trajectory must contain at least two frames")
    result = []
    previous_t = -math.inf
    for index, raw in enumerate(value):
        if not isinstance(raw, Mapping):
            raise TriplicateTraceError(f"ego reference frame {index} is not an object")
        state = {
            "t_sec": float(raw["t_sec"]),
            "x": float(raw["x"]),
            "y": float(raw["y"]),
            "z": float(raw.get("z", 0.0)),
            "yaw": float(raw.get("yaw", 0.0)),
            "speed_mps": float(raw.get("speed_mps", 0.0)),
        }
        if any(not math.isfinite(item) for item in state.values()):
            raise TriplicateTraceError(f"ego reference frame {index} contains a non-finite value")
        if state["t_sec"] < previous_t:
            raise TriplicateTraceError("ego reference timestamps are not monotonic")
        previous_t = state["t_sec"]
        result.append(state)
    return result


def _nearest_frame(rows: list[Mapping[str, Any]], target_sec: float, start_us: int) -> tuple[int, int, float]:
    candidates: list[tuple[float, int, int]] = []
    for index, row in enumerate(rows):
        timestamp_us = row.get("timestamp_us")
        if isinstance(timestamp_us, bool) or not isinstance(timestamp_us, int):
            raise TriplicateTraceError(f"reconstruction frame {index} has no integer timestamp_us")
        delta_sec = (timestamp_us - start_us) / 1_000_000.0
        candidates.append((abs(delta_sec - target_sec), index, timestamp_us))
    if not candidates:
        raise TriplicateTraceError("reconstruction frame metadata is empty")
    _, index, timestamp_us = min(candidates)
    delta_us = int(round((timestamp_us - start_us) - target_sec * 1_000_000.0))
    return index, timestamp_us, delta_us


def _container_relative(path: Path, evidence_root: Path, container_root: str) -> tuple[str, str]:
    try:
        relative = path.resolve().relative_to(evidence_root.resolve()).as_posix()
    except ValueError as exc:
        raise TriplicateTraceError(f"materialized payload is outside evidence root: {path}") from exc
    return f"{container_root.rstrip('/')}/{relative}", relative


def _materialize_rgb(source: Path, target: Path) -> dict[str, Any]:
    try:
        with Image.open(source) as image:
            rgb = image.convert("RGB")
            if rgb.size == SOURCE_RGB_SIZE:
                resized = rgb
                method = "native"
            elif rgb.size == HARMONIZER_RGB_SIZE:
                left = (HARMONIZER_RGB_SIZE[0] - FORMAL_RGB_SIZE[0]) // 2
                top = (HARMONIZER_RGB_SIZE[1] - FORMAL_RGB_SIZE[1]) // 2
                resized = rgb.crop(
                    (left, top, left + FORMAL_RGB_SIZE[0], top + FORMAL_RGB_SIZE[1])
                )
                method = "center_crop_2400x900"
            else:
                raise TriplicateTraceError(
                    f"RGB source must be {SOURCE_RGB_SIZE[0]}x{SOURCE_RGB_SIZE[1]} or "
                    f"{HARMONIZER_RGB_SIZE[0]}x{HARMONIZER_RGB_SIZE[1]}: {source}"
                )
            target.parent.mkdir(parents=True, exist_ok=True)
            resized.save(target, format="JPEG", quality=95, subsampling=0)
    except OSError as exc:
        raise TriplicateTraceError(f"cannot materialize RGB source {source}: {exc}") from exc
    return {
        "source_path": str(source.resolve()),
        "source_sha256": sha256_file(source),
        "source_size": list(rgb.size),
        "target_size": list(FORMAL_RGB_SIZE),
        "method": method,
        "target_sha256": sha256_file(target),
    }


def _materialize_lidar(source: Path, target: Path) -> dict[str, Any]:
    values = np.fromfile(source, dtype="<f4")
    if values.size < 4 or values.size % 4:
        raise TriplicateTraceError(f"LiDAR source is not a non-empty XYZI stream: {source}")
    points = values.reshape((-1, 4)).astype(np.float64, copy=False)
    matrix = np.asarray(LIDAR_RESPONSE_TO_SENSOR, dtype=np.float64).reshape((4, 4))
    homogeneous = np.concatenate((points[:, :3], np.ones((points.shape[0], 1))), axis=1)
    transformed = (matrix @ homogeneous.T).T[:, :3]
    output = np.column_stack((transformed, points[:, 3])).astype("<f4", copy=False)
    target.parent.mkdir(parents=True, exist_ok=True)
    output.tofile(target)
    return {
        "source_path": str(source.resolve()),
        "source_sha256": sha256_file(source),
        "source_encoding": "float32_xyzi_little_endian",
        "source_coordinate_frame": "nre_26_04_lidar_sensor",
        "source_axis_convention": "nre_26_04_render_axes",
        "target_coordinate_frame": "sensor_local",
        "target_axis_convention": "carla_sensor",
        "response_to_sensor": list(LIDAR_RESPONSE_TO_SENSOR),
        "target_point_count": int(output.shape[0]),
        "target_sha256": sha256_file(target),
    }


def _payload_ref(
    path: Path,
    *,
    evidence_root: Path,
    container_root: str,
    encoding: str,
    coordinate_frame: str,
    axis_convention: str | None = None,
    sensor_to_ego: list[float] | None = None,
    materialization: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    container_path, relative_path = _container_relative(path, evidence_root, container_root)
    result: dict[str, Any] = {
        "path": container_path,
        "host_path": str(path.resolve()),
        "relative_path": relative_path,
        "sha256": sha256_file(path),
        "byte_count": path.stat().st_size,
        "encoding": encoding,
        "coordinate_frame": coordinate_frame,
    }
    if axis_convention is not None:
        result["axis_convention"] = axis_convention
    if sensor_to_ego is not None:
        result["sensor_to_ego"] = list(sensor_to_ego)
    if materialization is not None:
        result["materialization"] = deepcopy(dict(materialization))
    return result


def _run_context(runtime_config: Mapping[str, Any], run_id: str) -> dict[str, Any]:
    experiment = runtime_config.get("experiment")
    if not isinstance(experiment, Mapping):
        raise TriplicateTraceError("runtime config experiment identity is missing")
    required = (
        "scene_id",
        "case_id",
        "seed",
        "artifact_sha256",
        "scene_package_sha256",
        "scenario_ir_sha256",
        "immutable_matrix_sha256",
        "source_run_config_sha256",
        "variant_config_sha256",
        "run_config_sha256",
    )
    identity = {name: experiment.get(name) for name in required[3:]}
    if any(not isinstance(value, str) or len(value) != 64 for value in identity.values()):
        raise TriplicateTraceError("runtime config experiment identity is incomplete")
    return {
        "run_id": run_id,
        "scene_id": experiment.get("scene_id"),
        "case_id": experiment.get("case_id"),
        "seed": experiment.get("seed"),
        "identity": identity,
    }


def _actor_frame_provenance(
    manifest: Mapping[str, Any],
    frame: Mapping[str, Any],
    manifest_path: Path,
) -> dict[str, Any]:
    return {
        "schema_version": "open_loop_bbox_dynamic_provenance.v1",
        "actor_manifest_path": str(manifest_path.resolve()),
        "actor_manifest_sha256": str(manifest["manifest_sha256"]),
        "actor_manifest_file_sha256": str(
            manifest.get("manifest_file_sha256") or sha256_file(manifest_path)
        ),
        "frame_id": int(frame["frame_id"]),
        "active_actor_ids": list(frame["active_actor_ids"]),
        "active_actor_set_sha256": str(frame["active_actor_set_sha256"]),
        "pose_digest": str(frame["pose_digest"]),
        "manifest_dynamic_object_sha256": str(frame["dynamic_object_sha256"]),
        "pose_source": "scenario_ir_actor_reference_trajectory",
        "track_binding_source": "formal_usdz_sequence_tracks.json",
        "coordinate_frame": "scene_local_ego_start_x_forward_y_left_z_up",
        "usdz_track_bindings": deepcopy(frame["usdz_track_bindings"]),
    }


def _source_spec(
    variant: str,
    *,
    reconstruction_root: Path,
    harmonizer_root: Path | None,
    recon_rows: list[Mapping[str, Any]],
    ir_time_sec: float,
    start_us: int,
) -> tuple[Path, Path, dict[str, Any]]:
    recon_index, recon_timestamp_us, recon_delta_us = _nearest_frame(
        recon_rows, ir_time_sec, start_us
    )
    recon_row = recon_rows[recon_index]
    recon_rgb = recon_row.get("rgb") or {}
    recon_lidar = recon_row.get("lidar") or {}
    original_rgb = reconstruction_root / str(recon_rgb.get("original_path") or "")
    original_lidar = reconstruction_root / str(recon_lidar.get("original_path") or "")
    if not original_rgb.is_file() or not original_lidar.is_file():
        raise TriplicateTraceError(
            f"reconstruction frame {recon_index} is missing original RGB/LiDAR payloads"
        )
    if variant == "reconstructed":
        return original_rgb, original_lidar, {
            "source_kind": "neural_scene_bridge_multimodal_20fps",
            "source_frame_index": recon_index,
            "source_timestamp_us": recon_timestamp_us,
            "source_time_sec": (recon_timestamp_us - start_us) / 1_000_000.0,
            "delta_us": recon_delta_us,
            "rgb_mode": "reconstructed",
            "lidar_mode": "reconstructed",
        }
    if variant != "harmonized":
        raise TriplicateTraceError(f"unsupported variant: {variant}")
    if harmonizer_root is None:
        raise TriplicateTraceError("harmonizer_root is required for the harmonized variant")
    harmonizer_frames = sorted(harmonizer_root.glob("*.jpg"))
    if not harmonizer_frames:
        raise TriplicateTraceError(f"Harmonizer frame directory is empty: {harmonizer_root}")
    harmonizer_fps = 30.0
    harmonizer_index = min(
        len(harmonizer_frames) - 1,
        max(0, int(round(ir_time_sec * harmonizer_fps))),
    )
    name_width = len(harmonizer_frames[0].stem)
    harmonized_rgb = harmonizer_root / f"{harmonizer_index:0{name_width}d}.jpg"
    return original_lidar, original_lidar, {
        "source_kind": "nvidia_harmonizer_rgb_only",
        "source_frame_index": recon_index,
        "source_timestamp_us": recon_timestamp_us,
        "source_time_sec": (recon_timestamp_us - start_us) / 1_000_000.0,
        "delta_us": recon_delta_us,
        "rgb_mode": "harmonized",
        "rgb_source_path": str(harmonized_rgb.resolve()),
        "rgb_source_frame_index": harmonizer_index,
        "rgb_source_time_sec": harmonizer_index / harmonizer_fps,
        "rgb_source_delta_us": int(round((harmonizer_index / harmonizer_fps - ir_time_sec) * 1_000_000.0)),
            "lidar_mode": "reconstructed_original",
        "lidar_source_path": str(original_lidar.resolve()),
    }


def build_trace(
    *,
    variant: str,
    scenario_ir_path: Path,
    opendrive_path: Path,
    runtime_config_path: Path,
    actor_manifest_path: Path | None,
    template_trace_path: Path,
    reconstruction_root: Path,
    harmonizer_root: Path | None,
    output_dir: Path,
    evidence_root: Path,
    run_id: str,
    container_root: str = "/sim-data",
    expected_frame_count: int = 39,
) -> dict[str, Any]:
    if output_dir.exists():
        raise TriplicateTraceError(f"refusing to overwrite output directory: {output_dir}")
    scenario_ir = _load_json(scenario_ir_path)
    runtime_config = _load_json(runtime_config_path)
    if actor_manifest_path is None:
        raise TriplicateTraceError(
            "actor_manifest_path is required for actor-aware reconstructed traces"
        )
    actor_manifest = load_actor_manifest(
        actor_manifest_path,
        expected_scenario_ir_sha256=sha256_file(scenario_ir_path),
        expected_scene_id=str(scenario_ir["scenario_id"]),
    )
    template = _load_json(template_trace_path)
    template_frames = template.get("frames")
    if not isinstance(template_frames, list) or not template_frames:
        raise TriplicateTraceError("template trace has no frames")
    track = _normalise_track((scenario_ir.get("ego") or {}).get("reference_trajectory"))
    if expected_frame_count > 0 and len(track) != expected_frame_count:
        raise TriplicateTraceError(
            f"expected {expected_frame_count} IR frames, got {len(track)}"
        )
    if len(actor_manifest["frames"]) < len(track):
        raise TriplicateTraceError(
            "actor manifest has fewer frames than the Scenario IR ego replay"
        )
    recon_metadata_path = reconstruction_root / "frames.jsonl"
    recon_rows = _load_jsonl(recon_metadata_path)
    if not recon_rows or any(row.get("status") != "passed" for row in recon_rows):
        raise TriplicateTraceError("reconstruction metadata is not fully passed")
    start_us = int(recon_rows[0]["timestamp_us"])
    run_context = _run_context(runtime_config, run_id)
    template_frame = template_frames[0]
    calibration = deepcopy(template_frame.get("calibration"))
    if not isinstance(calibration, dict):
        raise TriplicateTraceError("template frame calibration is missing")
    calibration["camera_adaptation"] = camera_adaptation_contract(
        source_width=FORMAL_RGB_SIZE[0], source_height=FORMAL_RGB_SIZE[1]
    )
    lidar_sensor_to_ego = calibration.get("lidar_sensor_to_ego")
    if not isinstance(lidar_sensor_to_ego, list) or len(lidar_sensor_to_ego) != 16:
        raise TriplicateTraceError("template LiDAR extrinsic is missing")
    output_dir.mkdir(parents=True)
    frames: list[dict[str, Any]] = []
    source_bindings: list[dict[str, Any]] = []
    source_name = RECONSTRUCTED_SOURCE if variant == "reconstructed" else HARMONIZED_SOURCE
    for frame_id, state in enumerate(track):
        actor_frame = frame_binding(actor_manifest, frame_id)
        actor_provenance = _actor_frame_provenance(
            actor_manifest,
            actor_frame,
            actor_manifest_path,
        )
        rgb_source, lidar_source, binding = _source_spec(
            variant,
            reconstruction_root=reconstruction_root,
            harmonizer_root=harmonizer_root,
            recon_rows=recon_rows,
            ir_time_sec=state["t_sec"],
            start_us=start_us,
        )
        frame_dir = output_dir / "payloads" / f"frame_{frame_id:08d}"
        rgb_target = frame_dir / "camera_front.jpg"
        lidar_target = frame_dir / "lidar_top.bin"
        if variant == "harmonized":
            rgb_source = Path(str(binding["rgb_source_path"]))
        rgb_materialization = _materialize_rgb(rgb_source, rgb_target)
        lidar_materialization = _materialize_lidar(lidar_source, lidar_target)
        camera_ref = _payload_ref(
            rgb_target,
            evidence_root=evidence_root,
            container_root=container_root,
            encoding="jpeg",
            coordinate_frame="camera_optical",
            materialization=rgb_materialization,
        )
        lidar_ref = _payload_ref(
            lidar_target,
            evidence_root=evidence_root,
            container_root=container_root,
            encoding="float32_xyzi_little_endian",
            coordinate_frame="sensor_local",
            axis_convention="carla_sensor",
            sensor_to_ego=lidar_sensor_to_ego,
            materialization=lidar_materialization,
        )
        binding = {
            **binding,
            "ir_frame_id": frame_id,
            "ir_timestamp_sec": state["t_sec"],
            "rgb_source_sha256": sha256_file(rgb_source),
            "lidar_source_sha256": sha256_file(lidar_source),
            "rgb_materialized_sha256": camera_ref["sha256"],
            "lidar_materialized_sha256": lidar_ref["sha256"],
        }
        observation = {
            "schema_version": "transfuserpp_observation.v1",
            "observation_id": f"{run_id}-frame-{frame_id:08d}",
            "source": source_name,
            "frame_id": frame_id,
            "timestamp": state["t_sec"],
            "rgb": {"camera_front": camera_ref},
            "lidar": lidar_ref,
            "sensor_validity": {"camera_front": True, "lidar": True},
            "calibration": deepcopy(calibration),
            "ego_state": {
                "pose": {
                    "x": state["x"],
                    "y": state["y"],
                    "z": state["z"],
                    "yaw": state["yaw"],
                },
                "speed_mps": state["speed_mps"],
                "speed_source": "scenario_ir_reference_trajectory",
                "compass_source": "scenario_ir_reference_trajectory",
            },
            "route": _route_for_frame(track, frame_id),
            "synchronization": {
                "frame_id": frame_id,
                "clock": "scenario_ir_reference_trajectory",
                "error_ms": 0.0,
                "dynamic_object_sha256": actor_frame["dynamic_object_sha256"],
                "sensor_age_ticks": 0,
            },
            "run_context": deepcopy(run_context),
            "provenance": {
                "gt_pose_replay": True,
                "pose_source": "scenario_ir_reference_trajectory",
                "input_source": source_name,
                "input_variant": variant,
                "control_applied": False,
                "physics_enabled": False,
                "dynamic_object_animation": True,
                "dynamic_object_count": len(actor_frame["dynamic_objects"]),
                "actor_manifest": deepcopy(actor_provenance),
                "source_frame_binding": deepcopy(binding),
            },
        }
        validate_observation(observation)
        frames.append(observation)
        source_bindings.append(binding)

    trace = {
        "schema_version": TRACE_SCHEMA,
        "source": source_name,
        "variant": variant,
        "scenario_ir": {"path": str(scenario_ir_path.resolve()), "sha256": sha256_file(scenario_ir_path)},
        "opendrive": {"path": str(opendrive_path.resolve()), "sha256": sha256_file(opendrive_path)},
        "runtime_config": {"path": str(runtime_config_path.resolve()), "sha256": sha256_file(runtime_config_path)},
        "input_binding": {
            "source": source_name,
            "variant": variant,
            "frame_count": len(frames),
            "reconstruction_metadata_path": str(recon_metadata_path.resolve()),
            "reconstruction_metadata_sha256": sha256_file(recon_metadata_path),
            "reconstruction_root": str(reconstruction_root.resolve()),
            "harmonizer_root": str(harmonizer_root.resolve()) if harmonizer_root else None,
            "camera_source_size": list(SOURCE_RGB_SIZE),
            "camera_formal_size": list(FORMAL_RGB_SIZE),
            "camera_materialization": "native_800x450",
            "lidar_axis_normalization": {
                "schema_version": "nre_lidar_axis_normalization.v1",
                "response_to_sensor": list(LIDAR_RESPONSE_TO_SENSOR),
                "source_axis_convention": "nre_26_04_render_axes",
                "target_axis_convention": "carla_sensor",
            },
            "harmonizer_rgb_only": variant == "harmonized",
            "lidar_source": "reconstructed_original" if variant == "harmonized" else source_name,
            "actor_manifest_sha256": actor_manifest["manifest_sha256"],
            "actor_manifest_file_sha256": actor_manifest["manifest_file_sha256"],
            "dynamic_object_binding": "formal_usdz_sequence_tracks",
            "source_frame_bindings": source_bindings,
        },
        "capture": {
            "ego_pose_owner": "scenario_ir_reference_trajectory",
            "control_affects_next_ego_pose": False,
            "dynamic_actor_creation": False,
            "dynamic_object_animation": True,
            "dynamic_object_count_max": max(
                len(item["actor_manifest"]["active_actor_ids"])
                for item in [
                    frame["provenance"] for frame in frames
                ]
            ),
            "actor_manifest": {
                "path": str(actor_manifest_path.resolve()),
                "sha256": actor_manifest["manifest_sha256"],
                "file_sha256": actor_manifest["manifest_file_sha256"],
                "summary": deepcopy(actor_manifest["summary"]),
            },
            "camera_count_used_by_tfpp": 1,
            "lidar_count_used_by_tfpp": 1,
        },
        "frames": frames,
        "actor_manifest": {
            "path": str(actor_manifest_path.resolve()),
            "sha256": actor_manifest["manifest_sha256"],
            "file_sha256": actor_manifest["manifest_file_sha256"],
            "summary": deepcopy(actor_manifest["summary"]),
        },
    }
    trace_path = output_dir / "triplicate_observations.json"
    trace_path.write_text(
        json.dumps(trace, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return {
        "status": "captured",
        "variant": variant,
        "source": source_name,
        "trace": str(trace_path),
        "trace_sha256": sha256_file(trace_path),
        "frame_count": len(frames),
        "payload_count": len(frames) * 2,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=("reconstructed", "harmonized"), required=True)
    parser.add_argument("--scenario-ir", type=Path, required=True)
    parser.add_argument("--opendrive", type=Path, required=True)
    parser.add_argument("--runtime-config", type=Path, required=True)
    parser.add_argument("--actor-manifest", type=Path, required=True)
    parser.add_argument("--template-trace", type=Path, required=True)
    parser.add_argument("--reconstruction-root", type=Path, required=True)
    parser.add_argument("--harmonizer-root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--container-root", default="/sim-data")
    parser.add_argument("--expected-frame-count", type=int, default=39)
    args = parser.parse_args(argv)
    try:
        result = build_trace(
            variant=args.variant,
            scenario_ir_path=args.scenario_ir,
            opendrive_path=args.opendrive,
            runtime_config_path=args.runtime_config,
            actor_manifest_path=args.actor_manifest,
            template_trace_path=args.template_trace,
            reconstruction_root=args.reconstruction_root,
            harmonizer_root=args.harmonizer_root,
            output_dir=args.output_dir,
            evidence_root=args.evidence_root,
            run_id=args.run_id,
            container_root=args.container_root,
            expected_frame_count=args.expected_frame_count,
        )
    except (OSError, ValueError, TypeError, json.JSONDecodeError, TriplicateTraceError) as exc:
        print(json.dumps({"status": "failed", "error": f"{type(exc).__name__}: {exc}"}))
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
