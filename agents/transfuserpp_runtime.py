from __future__ import annotations

import hashlib
import importlib
import json
import math
import re
import sys
import time
from collections import deque
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

from agents.transfuserpp_contract import (
    ALGORITHM_ID,
    ALGORITHM_VERSION,
    TransFuserPPContractError,
    assert_runtime_prepared,
    camera_center_crop_window,
    REQUIRED_DENSE_KEYS,
    route_command_index,
    validate_intermediate_record,
    validate_observation,
)


class TransFuserPPRuntimeError(RuntimeError):
    """Raised when the pinned upstream TF++ runtime cannot run safely."""


class TransFuserPPModelRuntime:
    """Direct, single-checkpoint adapter for CARLA Garage TF++ v5.

    Heavy upstream dependencies are imported only after the immutable repo,
    checkpoint, and model-config identities pass preflight.  This class is
    intentionally not a replacement model and does not vendor upstream code.
    """

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.runtime_config = deepcopy(dict(config))
        self.manifest = assert_runtime_prepared(self.runtime_config)
        self.output_root = Path(self.runtime_config["intermediate_output_dir"]).expanduser()
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.shared_data_root = Path(
            str(self.runtime_config.get("shared_data_path") or "/sim-data")
        ).expanduser()
        self.output_dir = self.output_root
        self._active_run_id: str | None = None
        self.max_sync_error_ms = float(
            self.runtime_config.get("max_synchronization_error_ms", 1.0)
        )
        self._closed = False
        self._load_runtime()

    def _load_runtime(self) -> None:
        repo = Path(self.runtime_config["repo_path"]).resolve()
        team_code = repo / "team_code"
        if str(team_code) not in sys.path:
            sys.path.insert(0, str(team_code))
        carla_agents = Path(self.runtime_config["carla_agents_path"]).resolve()
        agents_package = importlib.import_module("agents")
        package_path = getattr(agents_package, "__path__", None)
        if package_path is None:
            raise TransFuserPPRuntimeError("ClosedLoopBench agents package has no __path__")
        if str(carla_agents) not in package_path:
            package_path.append(str(carla_agents))
        try:
            self.np = importlib.import_module("numpy")
            self.cv2 = importlib.import_module("cv2")
            self.torch = importlib.import_module("torch")
            jsonpickle = importlib.import_module("jsonpickle")
            config_module = importlib.import_module("config")
            data_module = importlib.import_module("data")
            model_module = importlib.import_module("model")
            self.utils = importlib.import_module("transfuser_utils")
            navigation_module = importlib.import_module(
                "agents.navigation.global_route_planner"
            )
        except Exception as exc:
            raise TransFuserPPRuntimeError(f"cannot import CARLA Garage runtime: {exc}") from exc
        for module in (config_module, data_module, model_module, self.utils):
            module_path = Path(str(getattr(module, "__file__", ""))).resolve()
            if team_code not in module_path.parents:
                raise TransFuserPPRuntimeError(
                    f"upstream module resolved outside pinned team_code: {module_path}"
                )
        navigation_path = Path(
            str(getattr(navigation_module, "__file__", ""))
        ).resolve()
        if carla_agents not in navigation_path.parents:
            raise TransFuserPPRuntimeError(
                f"CARLA navigation resolved outside pinned agents package: {navigation_path}"
            )

        device_name = str(self.runtime_config.get("device", "cuda:0"))
        if not device_name.startswith("cuda"):
            raise TransFuserPPRuntimeError("formal TransFuser++ binding requires a CUDA device")
        if not self.torch.cuda.is_available():
            raise TransFuserPPRuntimeError("CUDA is unavailable")
        self.device = self.torch.device(device_name)

        try:
            encoded = Path(self.runtime_config["model_config_path"]).read_text(
                encoding="utf-8"
            )
            loaded = jsonpickle.decode(encoded)
            self.model_config = config_module.GlobalConfig()
            values = loaded.__dict__ if hasattr(loaded, "__dict__") else dict(loaded)
            self.model_config.__dict__.update(values)
        except Exception as exc:
            raise TransFuserPPRuntimeError(f"cannot load upstream model config: {exc}") from exc
        if self.model_config.backbone not in {"transFuser", "bev_encoder"}:
            raise TransFuserPPRuntimeError(
                f"unsupported TF++ backbone for multimodal binding: {self.model_config.backbone}"
            )
        if int(getattr(self.model_config, "lidar_seq_len", 1)) != 1:
            raise TransFuserPPRuntimeError(
                "scene0061 adapter currently requires lidar_seq_len==1; temporal sweep alignment is not implicit"
            )

        try:
            net = model_module.LidarCenterNet(self.model_config)
            if bool(getattr(self.model_config, "sync_batch_norm", False)):
                net = self.torch.nn.SyncBatchNorm.convert_sync_batchnorm(net)
            state = self.torch.load(
                self.runtime_config["checkpoint_path"], map_location=self.device
            )
            state_key = self.runtime_config.get("checkpoint_state_key")
            if state_key:
                if not isinstance(state, Mapping) or state_key not in state:
                    raise TransFuserPPRuntimeError(
                        f"checkpoint_state_key is absent: {state_key}"
                    )
                state = state[state_key]
            net.load_state_dict(state, strict=True)
            net.to(self.device)
            net.eval()
            if bool(self.runtime_config.get("compile_model", False)):
                net = self.torch.compile(net, mode=str(getattr(self.model_config, "compile_mode", "default")))
            self.net = net
            self.data = data_module.CARLA_Data(
                root=[], config=self.model_config, shared_dict=None
            )
        except Exception as exc:
            if isinstance(exc, TransFuserPPRuntimeError):
                raise
            raise TransFuserPPRuntimeError(f"cannot load TF++ checkpoint: {exc}") from exc

        self.identity = {
            "repo_sha256": self.runtime_config["repo_sha256"],
            "checkpoint_sha256": self.runtime_config["checkpoint_sha256"],
            "model_config_sha256": self.runtime_config["model_config_sha256"],
            "repo_revision": self.runtime_config["repo_revision"],
            "runtime_config_sha256": self.manifest["identity"]["runtime_config_sha256"],
            "carla_agents_sha256": self.manifest["identity"]["carla_agents_sha256"],
            "adapter_source_sha256": self.manifest["identity"]["adapter_source_sha256"],
            "container_image_digest": self.manifest["identity"]["container_image_digest"],
        }
        self.experiment = deepcopy(dict(self.runtime_config["experiment"]))
        self.real_checkpoint_loaded = True
        self._successful_inference_count = 0
        self.reset()

    def reset(self) -> None:
        self._last_frame_id: int | None = None
        for name in (
            "turn_controller",
            "speed_controller",
            "turn_controller_direct",
            "speed_controller_direct",
            "lateral_pid_controller",
            "longitudinal_pid_controller",
        ):
            controller = getattr(self.net, name, None)
            reset = getattr(controller, "reset", None)
            if callable(reset):
                reset()
                continue
            for window_name in ("window", "_window", "_saved_window"):
                window = getattr(controller, window_name, None)
                if window is None:
                    continue
                if hasattr(window, "clear"):
                    window.clear()
                elif getattr(window, "maxlen", None):
                    setattr(
                        controller,
                        window_name,
                        deque(maxlen=int(window.maxlen)),
                    )

    def health_check(self) -> dict[str, Any]:
        return {
            "status": "ready" if self.real_checkpoint_loaded and not self._closed else "closed",
            "algorithm_id": ALGORITHM_ID,
            "real_checkpoint_loaded": bool(self.real_checkpoint_loaded and not self._closed),
            "device": str(self.device),
            "torch_version": str(self.torch.__version__),
            "torch_cuda_version": str(self.torch.version.cuda),
            "cuda_device_name": str(self.torch.cuda.get_device_name(self.device)),
            "successful_inference_count": self._successful_inference_count,
            "tensor_warmup_completed": self._successful_inference_count > 0,
            "cuda_peak_memory_allocated_bytes": int(
                self.torch.cuda.max_memory_allocated(self.device)
            ),
            "identity": deepcopy(self.identity),
            "experiment": {
                "scene_id": self.experiment.get("scene_id"),
                "case_id": self.experiment.get("case_id"),
                "seed": self.experiment.get("seed"),
            },
        }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if hasattr(self, "net"):
            del self.net
        if hasattr(self, "torch") and self.torch.cuda.is_available():
            self.torch.cuda.empty_cache()

    def predict(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        if self._closed:
            raise TransFuserPPRuntimeError("runtime is closed")
        obs = validate_observation(
            observation,
            max_synchronization_error_ms=self.max_sync_error_ms,
        )
        frame_id = int(obs["frame_id"])
        self._activate_run(obs)
        if self._last_frame_id is not None and frame_id <= self._last_frame_id:
            raise TransFuserPPRuntimeError(
                f"stale_frame: {frame_id} is not newer than {self._last_frame_id}"
            )
        started = time.perf_counter()
        tensors, input_summary = self._preprocess(obs)
        preprocess_ms = (time.perf_counter() - started) * 1000.0
        inference_started = time.perf_counter()
        with self.torch.inference_mode():
            outputs = self.net.forward(**tensors)
        if not isinstance(outputs, (tuple, list)) or len(outputs) != 10:
            raise TransFuserPPRuntimeError(
                "unexpected TF++ forward signature; expected 10 outputs from leaderboard_2"
            )
        inference_ms = (time.perf_counter() - inference_started) * 1000.0
        self._successful_inference_count += 1
        post_started = time.perf_counter()
        result = self._postprocess(obs, outputs, input_summary, preprocess_ms, inference_ms)
        result["latency_ms"]["postprocess"] = (time.perf_counter() - post_started) * 1000.0
        result["latency_ms"]["total"] = (time.perf_counter() - started) * 1000.0
        record = self._write_record(result)
        self._last_frame_id = frame_id
        return record

    def _activate_run(self, observation: Mapping[str, Any]) -> None:
        context = observation.get("run_context")
        if not isinstance(context, Mapping):
            raise TransFuserPPRuntimeError("observation run_context is required")
        run_id = str(context.get("run_id") or "")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", run_id):
            raise TransFuserPPRuntimeError("observation run_id is unsafe or missing")
        for name in ("scene_id", "case_id", "seed"):
            if context.get(name) != self.experiment.get(name):
                raise TransFuserPPRuntimeError(f"run_context {name} mismatch")
        context_identity = context.get("identity")
        if not isinstance(context_identity, Mapping):
            raise TransFuserPPRuntimeError("run_context formal identity is required")
        for name in (
            "artifact_sha256",
            "scene_package_sha256",
            "scenario_ir_sha256",
            "immutable_matrix_sha256",
            "source_run_config_sha256",
            "variant_config_sha256",
            "run_config_sha256",
        ):
            if context_identity.get(name) != self.experiment.get(name):
                raise TransFuserPPRuntimeError(f"run_context identity {name} mismatch")
        if run_id == self._active_run_id:
            return
        self.reset()
        self._active_run_id = run_id
        self.output_dir = self.output_root / run_id
        self.output_dir.mkdir(parents=True, exist_ok=False)

    def _preprocess(self, obs: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        np = self.np
        cv2 = self.cv2
        torch = self.torch
        rgb_ref = obs["rgb"]["camera_front"]
        lidar_ref = obs["lidar"]
        self._verify_payload(rgb_ref, "camera_front")
        self._verify_payload(lidar_ref, "lidar_top")

        image = cv2.imread(str(rgb_ref["path"]), cv2.IMREAD_COLOR)
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise TransFuserPPRuntimeError("camera_front is not a decodable color image")
        if bool(self.runtime_config.get("jpeg_roundtrip", False)):
            ok, encoded = cv2.imencode(".jpg", image)
            if not ok:
                raise TransFuserPPRuntimeError("camera_front JPEG roundtrip encode failed")
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        source_height, source_width = image.shape[:2]
        if (source_width, source_height) != (800, 450):
            raise TransFuserPPRuntimeError(
                "formal camera_front payload must decode to exactly 800x450"
            )
        expected_width = int(self.model_config.camera_width)
        expected_height = int(self.model_config.camera_height)
        adaptation = str(
            self.runtime_config.get(
                "camera_adaptation", "center_crop_to_model_aspect_then_resize"
            )
        )
        if adaptation != "center_crop_to_model_aspect_then_resize":
            raise TransFuserPPRuntimeError(
                f"unsupported explicit camera adaptation: {adaptation}"
            )
        image, crop_window = self._adapt_camera_image(
            image, expected_width=expected_width, expected_height=expected_height
        )
        image = self.utils.crop_array(self.model_config, image)
        image = np.transpose(image, (2, 0, 1))
        rgb = torch.from_numpy(image).to(self.device, dtype=torch.float32).unsqueeze(0)

        lidar = np.fromfile(str(lidar_ref["path"]), dtype="<f4")
        if lidar.size < 4 or lidar.size % 4:
            raise TransFuserPPRuntimeError("lidar_top is not a non-empty float32 XYZI stream")
        lidar = lidar.reshape((-1, 4))
        xyz = lidar[:, :3].astype(np.float64, copy=False)
        if lidar_ref["coordinate_frame"] == "sensor_local":
            transform = np.asarray(lidar_ref["sensor_to_ego"], dtype=np.float64).reshape((4, 4))
            homogeneous = np.concatenate((xyz, np.ones((xyz.shape[0], 1))), axis=1)
            xyz = (transform @ homogeneous.T).T[:, :3]
        lidar_histogram = self.data.lidar_to_histogram_features(
            xyz, use_ground_plane=bool(self.model_config.use_ground_plane)
        )
        lidar_bev = torch.from_numpy(lidar_histogram).to(
            self.device, dtype=torch.float32
        ).unsqueeze(0)

        speed = float(obs["ego_state"]["speed_mps"])
        velocity = torch.tensor([[speed]], device=self.device, dtype=torch.float32)
        target = obs["route"]["target_point_ego_m"]
        target_point = torch.tensor([target], device=self.device, dtype=torch.float32)
        command_index = route_command_index(obs["route"]["route_command"])
        command = self.utils.command_to_one_hot(command_index)
        command_tensor = torch.from_numpy(command[None]).to(
            self.device, dtype=torch.float32
        )
        tensors = {
            "rgb": rgb,
            "lidar_bev": lidar_bev,
            "target_point": target_point,
            "target_point_next": None,
            "ego_vel": velocity,
            "command": command_tensor,
        }
        if bool(getattr(self.model_config, "two_tp_input", False)):
            next_target = obs["route"].get("target_point_next_ego_m")
            if not isinstance(next_target, list) or len(next_target) != 2:
                raise TransFuserPPRuntimeError(
                    "checkpoint requires route.target_point_next_ego_m"
                )
            tensors["target_point_next"] = torch.tensor(
                [next_target], device=self.device, dtype=torch.float32
            )
        return tensors, {
            "camera_front": deepcopy(dict(rgb_ref)),
            "lidar_top": deepcopy(dict(lidar_ref)),
            "calibration": deepcopy(dict(obs["calibration"])),
            "rgb_tensor_shape": list(rgb.shape),
            "camera_adaptation": {
                "mode": adaptation,
                "source_width": source_width,
                "source_height": source_height,
                "model_sensor_width": expected_width,
                "model_sensor_height": expected_height,
                "center_crop_xyxy": crop_window,
                "model_crop_applied_by_upstream": bool(self.model_config.crop_image),
                "official_leaderboard_sensor_equivalent": False,
            },
            "lidar_bev_shape": list(lidar_bev.shape),
            "lidar_point_count": int(xyz.shape[0]),
            "route_command": obs["route"]["route_command"],
            "target_point_ego_m": list(target),
            "route_progress_index": obs["route"].get("progress_index"),
            "route_target_distance_along_m": obs["route"].get(
                "target_distance_along_route_m"
            ),
            "route_lookahead_m": obs["route"].get("lookahead_m"),
            "model_ego_coordinate_frame": "carla_x_forward_y_right_z_up",
            "speed_mps": speed,
            "ego_pose": deepcopy(dict(obs["ego_state"]["pose"])),
            "ego_pose_coordinate_frame": "closedloopbench_scene_x_forward_y_left_z_up",
            "nurec_frame_id": int(obs["frame_id"]),
            "dynamic_object_sha256": (obs.get("synchronization") or {}).get(
                "dynamic_object_sha256"
            ),
        }

    def _postprocess(
        self,
        obs: Mapping[str, Any],
        outputs: Any,
        input_summary: Mapping[str, Any],
        preprocess_ms: float,
        inference_ms: float,
    ) -> dict[str, Any]:
        torch = self.torch
        (
            pred_wp,
            pred_target_speed,
            pred_checkpoint,
            pred_semantic,
            pred_bev_semantic,
            pred_depth,
            pred_bb_features,
            attention_weights,
            pred_wp_1,
            selected_path,
        ) = outputs
        if pred_target_speed is None or pred_checkpoint is None:
            raise TransFuserPPRuntimeError(
                "checkpoint does not provide direct target-speed/checkpoint outputs"
            )
        probabilities = torch.softmax(pred_target_speed[0], dim=0).detach().cpu().numpy()
        target_speeds = self.np.asarray(self.model_config.target_speeds, dtype=float)
        if probabilities.shape[0] != target_speeds.shape[0]:
            raise TransFuserPPRuntimeError("target-speed class count differs from model config")
        brake_uncertainty_threshold = float(
            self.model_config.brake_uncertainty_threshold
        )
        target_speed_selected_index: int | None = None
        if bool(self.runtime_config.get("uncertainty_weight", True)):
            if probabilities[0] > brake_uncertainty_threshold:
                target_speed = float(target_speeds[0])
                target_speed_selection_mode = "brake_uncertainty_override"
                target_speed_selected_index = 0
            else:
                target_speed = float(self.np.sum(probabilities * target_speeds))
                target_speed_selection_mode = "weighted_expectation"
        else:
            target_speed_selected_index = int(self.np.argmax(probabilities))
            target_speed = float(target_speeds[target_speed_selected_index])
            target_speed_selection_mode = "argmax"
        checkpoints = pred_checkpoint[0].detach().cpu().numpy()
        waypoints_tensor = pred_wp if pred_wp is not None else pred_checkpoint
        selected_path_probability = None
        if (
            pred_wp is not None
            and bool(getattr(self.model_config, "use_wp_gru", False))
            and bool(getattr(self.model_config, "multi_wp_output", False))
        ):
            if pred_wp_1 is None or selected_path is None:
                raise TransFuserPPRuntimeError(
                    "multi-waypoint checkpoint omitted alternate path or selector"
                )
            selected_path_probability = float(torch.sigmoid(selected_path)[0].item())
            if selected_path_probability > 0.5:
                waypoints_tensor = pred_wp_1
        waypoints = waypoints_tensor[0].detach().cpu().numpy()

        if bool(getattr(self.model_config, "inference_direct_controller", False)):
            speed_tensor = torch.tensor(
                [float(obs["ego_state"]["speed_mps"])],
                device=self.device,
                dtype=torch.float32,
            )
            steer, throttle, brake = self.net.control_pid_direct(
                checkpoints, target_speed, speed_tensor
            )
        elif pred_wp is not None and bool(getattr(self.model_config, "use_wp_gru", False)):
            speed_tensor = torch.tensor(
                [float(obs["ego_state"]["speed_mps"])],
                device=self.device,
                dtype=torch.float32,
            )
            steer, throttle, brake = self.net.control_pid(
                waypoints_tensor, speed_tensor,
                tuned_aim_distance=bool(self.runtime_config.get("tuned_aim_distance", False)),
            )
        else:
            raise TransFuserPPRuntimeError("checkpoint has no supported control representation")

        if any(
            value is None
            for value in (pred_semantic, pred_bev_semantic, pred_depth, pred_bb_features)
        ):
            raise TransFuserPPRuntimeError(
                "checkpoint lacks one or more required intermediate perception heads"
            )
        boxes = []
        if not bool(getattr(self.model_config, "detect_boxes", False)):
            raise TransFuserPPRuntimeError(
                "model config detect_boxes must be enabled for intermediate evaluation"
            )
        converted_boxes = self.net.convert_features_to_bb_metric(pred_bb_features)
        nms_boxes = self.utils.non_maximum_suppression(
            [converted_boxes], float(self.model_config.iou_treshold_nms)
        )
        boxes = [
            [float(value) for value in row]
            for row in nms_boxes
        ]
        bev_labels = (
            torch.argmax(pred_bev_semantic, dim=1)[0].detach().cpu().numpy().astype("uint8")
            if pred_bev_semantic is not None
            else None
        )
        perspective_labels = (
            torch.argmax(pred_semantic, dim=1)[0].detach().cpu().numpy().astype("uint8")
            if pred_semantic is not None
            else None
        )
        depth = pred_depth[0].detach().cpu().numpy() if pred_depth is not None else None
        proxy_samples = self._sample_actor_proxies(obs.get("actor_proxies") or [], bev_labels)
        dense_path = self._write_dense_outputs(
            int(obs["frame_id"]),
            bev_semantic_labels=bev_labels,
            perspective_semantic_labels=perspective_labels,
            depth=depth,
            target_speed_probabilities=probabilities,
        )
        control = {
            "throttle": float(throttle),
            "steer": float(steer),
            "brake": float(brake),
            "hand_brake": False,
            "reverse": False,
        }
        return {
            "schema_version": "transfuserpp_intermediate_frame.v1",
            "algorithm_id": ALGORITHM_ID,
            "algorithm_version": ALGORITHM_VERSION,
            "frame_id": int(obs["frame_id"]),
            "timestamp": float(obs.get("timestamp", obs.get("t_sec"))),
            "identity": deepcopy(self.identity),
            "experiment": {
                **deepcopy(self.experiment),
                "run_id": str((obs.get("run_context") or {}).get("run_id")),
            },
            "provenance": {
                "execution_mode": "remote_model_inference",
                "real_checkpoint_loaded": True,
                "upstream_repository": self.manifest["upstream"]["repository"],
                "upstream_branch": self.manifest["upstream"]["branch"],
                "official_leaderboard_sensor_equivalent": False,
            },
            "inputs": dict(input_summary),
            "synchronization": deepcopy(dict(obs["synchronization"])),
            "actor_proxies": deepcopy(list(obs.get("actor_proxies") or [])),
            "outputs": {
                "waypoints_ego_m": self._points(waypoints),
                "route_checkpoints_ego_m": self._points(checkpoints),
                "target_speed_probabilities": [float(value) for value in probabilities],
                "target_speed_bins_mps": [float(value) for value in target_speeds],
                "target_speed_mps": target_speed,
                "target_speed_selection_mode": target_speed_selection_mode,
                "target_speed_selected_index": target_speed_selected_index,
                "target_speed_brake_uncertainty_threshold": brake_uncertainty_threshold,
                "bounding_boxes_ego": boxes,
                "attention_summary": self._tensor_summary(attention_weights),
                "selected_path": self._scalar(selected_path),
                "selected_path_probability": selected_path_probability,
                "alternate_waypoints_available": pred_wp_1 is not None,
                "control": control,
            },
            "dynamic_bev_proxy": {
                "class_mapping": {"vehicle": 9, "pedestrian": 10},
                "actor_samples": proxy_samples,
                "grid": {
                    "min_x_m": float(self.model_config.min_x),
                    "max_x_m": float(self.model_config.max_x),
                    "min_y_m": float(self.model_config.min_y),
                    "max_y_m": float(self.model_config.max_y),
                    "height": int(bev_labels.shape[0]) if bev_labels is not None else None,
                    "width": int(bev_labels.shape[1]) if bev_labels is not None else None,
                    "row_axis": "ego_x_forward",
                    "column_axis": "ego_y_right",
                },
                "full_scene_ground_truth": False,
            },
            "dense_outputs": {
                "path": str(dense_path),
                "relative_path": self._shared_relative_path(dense_path),
                "sha256": self._file_sha256(dense_path),
                "encoding": "numpy_npz",
                "bev_labels_key": "bev_semantic_labels",
                "perspective_labels_key": "perspective_semantic_labels",
                "depth_key": "depth",
                "required_keys": list(REQUIRED_DENSE_KEYS),
            },
            "latency_ms": {
                "preprocess": float(preprocess_ms),
                "inference": float(inference_ms),
                "postprocess": 0.0,
                "total": 0.0,
            },
            "semantics": {
                "occupancy_evaluation": "dynamic_bev_proxy_only",
                "full_3d_occupancy_ground_truth_available": False,
                "full_3d_occupancy_reason": (
                    "scene0061 CARLA OpenDRIVE/proxy world is not full NuRec scene occupancy ground truth"
                ),
                "render_quality_gate_required_for_perception_ranking": True,
                "upstream_sensor_agent_safety_controllers_bypassed": True,
                "direct_model_forward": True,
            },
        }

    def _adapt_camera_image(
        self, image: Any, *, expected_width: int, expected_height: int
    ) -> tuple[Any, list[int]]:
        height, width = image.shape[:2]
        left, top, right, bottom = camera_center_crop_window(
            width, height, expected_width, expected_height
        )
        cropped = image[top:bottom, left:right]
        resized = self.cv2.resize(
            cropped,
            (expected_width, expected_height),
            interpolation=self.cv2.INTER_LINEAR,
        )
        return resized, [left, top, right, bottom]

    def _tensor_summary(self, value: Any) -> Any:
        if value is None:
            return None
        if isinstance(value, (list, tuple)):
            return [self._tensor_summary(item) for item in value]
        if isinstance(value, Mapping):
            return {str(key): self._tensor_summary(item) for key, item in value.items()}
        try:
            tensor = value.detach().float().cpu()
            return {
                "shape": list(tensor.shape),
                "min": float(tensor.min()) if tensor.numel() else None,
                "max": float(tensor.max()) if tensor.numel() else None,
                "mean": float(tensor.mean()) if tensor.numel() else None,
            }
        except Exception:
            if isinstance(value, (str, int, float, bool)) or value is None:
                return deepcopy(value)
            return {"type": type(value).__name__, "serializable": False}

    def _sample_actor_proxies(self, proxies: list[Any], labels: Any) -> list[dict[str, Any]]:
        result = []
        for raw in proxies:
            if not isinstance(raw, Mapping):
                continue
            actor_type = str(raw.get("actor_type") or "")
            expected = 9 if actor_type == "vehicle" else 10 if actor_type == "pedestrian" else None
            center = raw.get("center_ego_m")
            predicted = None
            in_bounds = False
            if labels is not None and expected is not None and isinstance(center, list) and len(center) >= 2:
                row = int(
                    (float(center[0]) - float(self.model_config.min_x))
                    / (float(self.model_config.max_x) - float(self.model_config.min_x))
                    * labels.shape[0]
                )
                col = int(
                    (float(center[1]) - float(self.model_config.min_y))
                    / (float(self.model_config.max_y) - float(self.model_config.min_y))
                    * labels.shape[1]
                )
                in_bounds = 0 <= row < labels.shape[0] and 0 <= col < labels.shape[1]
                if in_bounds:
                    predicted = int(labels[row, col])
            result.append(
                {
                    "actor_id": raw.get("actor_id"),
                    "track_id": raw.get("track_id"),
                    "actor_type": actor_type,
                    "center_ego_m": deepcopy(center),
                    "expected_bev_class": expected,
                    "predicted_center_bev_class": predicted,
                    "center_in_bev_bounds": in_bounds,
                    "center_class_match": predicted == expected if predicted is not None else None,
                    "metric_scope": "center_sample_proxy_not_full_occupancy",
                }
            )
        return result

    def _write_dense_outputs(self, frame_id: int, **arrays: Any) -> Path:
        target = self.output_dir / f"frame_{frame_id:08d}.dense.npz"
        if target.exists():
            raise TransFuserPPRuntimeError(f"refusing to overwrite dense output: {target}")
        serializable = {
            key: value
            for key, value in arrays.items()
            if value is not None
        }
        self.np.savez_compressed(target, **serializable)
        return target

    def _shared_relative_path(self, path: Path) -> str:
        try:
            relative = path.resolve().relative_to(self.shared_data_root.resolve())
        except ValueError as exc:
            raise TransFuserPPRuntimeError(
                f"intermediate output is outside shared_data_path: {path}"
            ) from exc
        return relative.as_posix()

    def _write_record(self, record: dict[str, Any]) -> dict[str, Any]:
        validate_intermediate_record(record)
        target = self.output_dir / f"frame_{record['frame_id']:08d}.intermediate.json"
        if target.exists():
            raise TransFuserPPRuntimeError(f"refusing to overwrite intermediate output: {target}")
        target.write_text(
            json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        result = deepcopy(record)
        result["record_ref"] = {
            "path": str(target),
            "relative_path": self._shared_relative_path(target),
            "sha256": self._file_sha256(target),
        }
        return result

    def _verify_payload(self, reference: Mapping[str, Any], name: str) -> None:
        path = Path(str(reference["path"]))
        if not path.is_file():
            raise TransFuserPPRuntimeError(f"{name} payload is absent: {path}")
        if bool(self.runtime_config.get("verify_payload_hashes", True)):
            actual = self._file_sha256(path)
            if actual != reference["sha256"]:
                raise TransFuserPPRuntimeError(f"{name} payload sha256 mismatch")

    @staticmethod
    def _file_sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    @staticmethod
    def _points(value: Any) -> list[list[float]]:
        result = [[float(point[0]), float(point[1])] for point in value]
        if any(not all(math.isfinite(item) for item in point) for point in result):
            raise TransFuserPPRuntimeError("model emitted non-finite waypoints")
        return result

    @staticmethod
    def _scalar(value: Any) -> float | None:
        if value is None:
            return None
        try:
            result = float(value.detach().cpu().reshape(-1)[0])
        except Exception:
            result = float(value)
        if not math.isfinite(result):
            raise TransFuserPPRuntimeError("model emitted a non-finite scalar")
        return result
