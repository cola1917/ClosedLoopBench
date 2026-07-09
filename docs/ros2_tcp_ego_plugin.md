# ROS2 TCP Ego Plugin Contract

## Purpose

ClosedLoopBench treats TCP as an external ROS2 ego policy profile, not as a
vendored autonomy stack. The benchmark owns scenario orchestration, normalized
topics, timeout behavior, safety fallback, and report metadata. A TCP adapter
owns model loading, preprocessing, and publishing control commands.

## Policy Identity

The TCP ego policy uses the existing ROS2 external-agent boundary:

```json
{
  "type": "ros2_external_agent",
  "runtime": "ros2_bridge",
  "stack": "tcp",
  "plugin": "external_ros2_tcp",
  "availability": "optional_adapter"
}
```

`availability=optional_adapter` means the policy config can be generated and
validated without installing a TCP repository. Runtime execution is enabled only
when a user supplies a valid adapter command or runtime path.

## Required Fields

- `type`: must be `ros2_external_agent`.
- `runtime`: must be `ros2_bridge`.
- `stack`: must be `tcp`.
- `plugin`: must be `external_ros2_tcp`.
- `input_topics`: normalized ClosedLoopBench observations.
- `output_topic`: normalized vehicle-control command topic.
- `ros2_bridge_topics`: mapping from normalized topics to CARLA ROS bridge
  topics.
- `timeout`: startup, tick, and shutdown timeout values.
- `safety_fallback`: fallback policy used if TCP is unavailable or returns an
  invalid command.
- `runtime_path`: optional path or command metadata for a local TCP adapter.

## Topic Contract

ClosedLoopBench internal topics stay stable across autonomy stacks:

- `/closed_loop/ego/state`
- `/closed_loop/world/state`
- `/closed_loop/route`
- `/closed_loop/sensors`
- `/closed_loop/ego/control_cmd`

The CARLA ROS bridge mapping is explicit so later runtime code can connect the
same policy config to real topics:

- ego state -> `/carla/ego_vehicle/odometry`
- world state -> `/carla/world_info`
- route -> `/carla/ego_vehicle/waypoints`
- sensors -> `/carla/ego_vehicle/sensors`
- control command -> `/carla/ego_vehicle/vehicle_control_cmd`

## Optional TCP Runtime Path

The config may include:

```json
{
  "runtime_path": {
    "adapter_kind": "python_module",
    "repo_path": null,
    "entrypoint": "tcp_ros2_adapter",
    "launch_package": null
  }
}
```

`repo_path=null` is valid for design-time and unit-test validation. A later
environment-specific integration can set `repo_path` to an installed TCP
checkout, or replace `entrypoint` with a launch file, Docker command, or process
adapter.

## Non Goals

- Do not vendor the TCP open-source repository into ClosedLoopBench.
- Do not require TCP model weights for contract tests.
- Do not make TCP the only ego policy. BasicAgent remains the smoke-test
  fallback, and other ROS2 stacks stay behind the same plugin boundary.
