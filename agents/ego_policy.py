SUPPORTED_ROS2_STACKS = ("transfuser", "interfuser", "tcp", "uniad", "external")

_ROS2_INPUT_TOPICS = (
    "/closed_loop/ego/state",
    "/closed_loop/world/state",
    "/closed_loop/route",
    "/closed_loop/sensors",
)

_ROS2_OUTPUT_TOPIC = "/closed_loop/ego/control_cmd"


def build_ego_policy_config(
    kind="basic_agent",
    stack=None,
    role_name="ego_vehicle",
    runtime_path=None,
    checkpoint_path=None,
):
    """Build the ego policy adapter config used by CARLA run orchestration."""
    if kind == "basic_agent":
        return {
            "type": "basic_agent",
            "runtime": "builtin",
            "required_runtime": True,
            "settings": {
                "agent": "basic_agent",
                "control_mode": "vehicle_control",
                "target_speed_mps": 8.0,
                "lookahead_m": 12.0,
            },
        }

    if kind == "ros2_external_agent":
        stack_name = stack or "external"
        if stack_name not in SUPPORTED_ROS2_STACKS:
            raise ValueError(
                "Unsupported ROS2 ego stack {!r}; expected one of {}".format(
                    stack_name, ", ".join(SUPPORTED_ROS2_STACKS)
                )
            )

        config = {
            "type": "ros2_external_agent",
            "runtime": "ros2_bridge",
            "stack": stack_name,
            "required_runtime": True,
            "availability": "supported",
            "input_topics": list(_ROS2_INPUT_TOPICS),
            "output_topic": _ROS2_OUTPUT_TOPIC,
            "carla_ros_bridge_topics": _carla_ros_bridge_topics(role_name),
            "timeout": {
                "startup_seconds": 30.0,
                "tick_seconds": 0.5,
                "shutdown_seconds": 5.0,
            },
            "safety_fallback": {
                "policy": "basic_agent",
                "trigger": "timeout_or_invalid_command",
            },
            "notes": "ROS2 bridge adapter boundary for external ego policy stacks.",
        }

        if stack_name == "tcp":
            config["required_runtime"] = False
            config["availability"] = "optional_adapter"
            config["plugin"] = "external_ros2_tcp"
            config["adapter"] = _tcp_adapter(runtime_path, checkpoint_path)
            config["ros2_bridge_topics"] = _normalized_ros2_bridge_topics(role_name)
            config["runtime_path"] = _tcp_runtime_path(runtime_path)
            config["notes"] = (
                "TCP is an optional ROS2/external ego adapter. ClosedLoopBench owns "
                "the benchmark contract; the TCP repo or container owns model runtime."
            )

        if stack_name == "uniad":
            config["required_runtime"] = False
            config["availability"] = "optional_showcase"
            config["notes"] = (
                "UniAD is an optional showcase plugin behind the ROS2 bridge; "
                "it is not required for the core runtime."
            )

        return config

    raise ValueError("Unsupported ego policy kind {!r}".format(kind))


def _carla_ros_bridge_topics(role_name):
    role = str(role_name or "ego_vehicle").strip("/") or "ego_vehicle"
    prefix = "/carla/{}".format(role)
    return {
        "vehicle_status": "{}/vehicle_status".format(prefix),
        "odometry": "{}/odometry".format(prefix),
        "camera_front": "{}/rgb_front/image".format(prefix),
        "control_cmd": "{}/vehicle_control_cmd".format(prefix),
    }


def _tcp_adapter(runtime_path, checkpoint_path):
    return {
        "name": "tcp",
        "runtime_path": runtime_path,
        "checkpoint_path": checkpoint_path,
        "expected_inputs": ["camera", "speed", "route"],
        "expected_outputs": ["vehicle_control", "waypoints"],
    }


def _normalized_ros2_bridge_topics(role_name):
    carla_topics = _carla_ros_bridge_topics(role_name)
    return {
        "ego_state": {
            "internal": "/closed_loop/ego/state",
            "carla_ros_bridge": carla_topics["odometry"],
        },
        "world_state": {
            "internal": "/closed_loop/world/state",
            "carla_ros_bridge": "/carla/world_info",
        },
        "route": {
            "internal": "/closed_loop/route",
            "carla_ros_bridge": "/carla/{}/waypoints".format(
                str(role_name or "ego_vehicle").strip("/") or "ego_vehicle"
            ),
        },
        "sensors": {
            "internal": "/closed_loop/sensors",
            "carla_ros_bridge": "/carla/{}/sensors".format(
                str(role_name or "ego_vehicle").strip("/") or "ego_vehicle"
            ),
        },
        "control_cmd": {
            "internal": "/closed_loop/ego/control_cmd",
            "carla_ros_bridge": carla_topics["control_cmd"],
        },
    }


def _tcp_runtime_path(runtime_path):
    return {
        "adapter_kind": "python_module",
        "repo_path": runtime_path,
        "entrypoint": "tcp_ros2_adapter",
        "launch_package": None,
    }
