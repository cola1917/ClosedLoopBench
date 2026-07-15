NUSCENES_CAMERA_NAMES = (
    "CAM_FRONT",
    "CAM_FRONT_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
)

CARLA_CAMERA_ROLE_NAMES = (
    "rgb_front",
    "rgb_front_left",
    "rgb_front_right",
    "rgb_back",
    "rgb_back_left",
    "rgb_back_right",
)

_CAMERA_PROFILES = {
    "tcp_front": ("rgb_front",),
    "multi_view": CARLA_CAMERA_ROLE_NAMES,
}


def build_sensor_source_contract():
    """Describe dataset-vs-runtime sensor ownership for closed-loop evaluation."""
    return {
        "dataset_source": "nuscenes",
        "dataset_cameras": list(NUSCENES_CAMERA_NAMES),
        "nuscenes_usage": ["mining", "reconstruction_source", "scenario_seed"],
        "closed_loop_observation_source": "carla_current_tick",
        "closed_loop_requirement": (
            "Ego policies must consume simulator observations from the current CARLA "
            "tick; recorded nuScenes images are not valid ego runtime inputs."
        ),
    }


def build_camera_profile(name="tcp_front", role_name="ego_vehicle"):
    """Build a CARLA camera profile for external ego adapters."""
    if name not in _CAMERA_PROFILES:
        raise ValueError(
            "Unsupported camera profile {!r}; expected one of {}".format(
                name, ", ".join(sorted(_CAMERA_PROFILES))
            )
        )

    role = _normalize_role_name(role_name)
    required_cameras = list(_CAMERA_PROFILES[name])
    from agents.ego_observation import build_camera_sensor_specs

    return {
        "name": name,
        "source": "carla_current_tick",
        "role_name": role,
        "required_cameras": required_cameras,
        "topics": {
            camera: "/carla/{}/{}/image".format(role, camera)
            for camera in required_cameras
        },
        "timestamp_policy": "same_tick",
        "calibration": {
            "intrinsics": "from_carla_sensor",
            "extrinsics": "from_carla_sensor_transform",
        },
        "sensor_specs": build_camera_sensor_specs(required_cameras),
        "preprocessing": {
            "resize": None,
            "crop": None,
            "normalization": "adapter_owned",
        },
        "fallback": "fail_fast",
    }


def _normalize_role_name(role_name):
    return str(role_name or "ego_vehicle").strip("/") or "ego_vehicle"
