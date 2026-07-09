from agents.sensor_contract import build_camera_profile


def build_tcp_io_contract(camera_profile="tcp_front", role_name="ego_vehicle"):
    """Build the model-free IO contract for a TCP-style external ego adapter."""
    return {
        "adapter": "tcp",
        "boundary": "external_ego_policy",
        "inputs": {
            "sensor_profile": build_camera_profile(camera_profile, role_name=role_name),
            "ego_state": {
                "source": "carla_current_tick",
                "fields": ["speed_mps", "pose", "velocity", "acceleration"],
            },
            "route": {
                "source": "closed_loop_route_planner",
                "fields": ["route_waypoints", "route_command", "target_point"],
            },
        },
        "outputs": {
            "control": {
                "type": "vehicle_control",
                "fields": ["throttle", "steer", "brake", "hand_brake", "reverse"],
                "topic": "/closed_loop/ego/control_cmd",
            },
            "optional_debug": {
                "fields": ["predicted_waypoints", "confidence"],
                "required": False,
            },
        },
    }


def build_tcp_adapter_config(
    camera_profile="tcp_front",
    role_name="ego_vehicle",
    runtime_path=None,
    checkpoint_path=None,
):
    """Generate a TCP adapter config without importing or installing TCP."""
    return {
        "plugin": "external_ros2_tcp",
        "availability": "optional_adapter",
        "required_runtime": False,
        "vendors_model_code": False,
        "io": build_tcp_io_contract(camera_profile=camera_profile, role_name=role_name),
        "runtime": {
            "execution_mode": "external_process_or_ros2_node",
            "repo_path": runtime_path,
            "checkpoint_path": checkpoint_path,
            "entrypoint": "tcp_ros2_adapter",
        },
        "safety": {
            "invalid_command_policy": "reject_and_fallback",
            "fallback_policy": "basic_agent",
        },
    }
