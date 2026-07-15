# External Ego Observation Runtime Audit

## Finding

The control return path is more complete than the observation path. The CARLA
runner can lazily create a real `rclpy` subscriber for
`CarlaEgoVehicleControl`, validate commands, and safe-stop on timeout. The
opposite direction, from the current CARLA tick to the external algorithm, is
not yet bound to a real CARLA/ROS 2 environment.

The following parts remain plan-only:

- spawning the declared `sensor.camera.rgb` actors in the CARLA runner
- deriving calibration from the actual spawned sensor attributes and transform
- decoding CARLA image callbacks into a stable image payload
- publishing tick-stamped ego pose, velocity, acceleration, and speed
- publishing tick-stamped route waypoints, command, and target point
- binding `sensor_msgs/Image`, `CameraInfo`, state, route, and
  `CarlaEgoVehicleControl` message classes and QoS to `Ros2TcpBridge`
- running the real algorithm container plugin and measuring observation-to-control latency

`Ros2TcpBridge` still registers `dict` with an injectable fake node. This is a
test seam, not a working `rclpy` node, and its runtime metadata explicitly says
`real_ros2_message_binding_implemented=false`. No local result should be
reported as a real TCP/CARLA execution.

## Implemented Without CARLA Or ROS 2

`agents.ego_observation` provides the executable model-free contract:

- explicit CARLA RGB blueprint attributes and nominal ego mount transforms
- pinhole intrinsics plus a required 4x4 sensor-to-ego transform
- required ego-state and route fields
- per-channel CARLA `frame_id` and timestamp envelopes
- strict same-frame aggregation across every camera, ego state, and route
- maximum timestamp skew and observation-age checks
- fail-closed results for missing, stale, future, mixed-tick, malformed, or
  uncalibrated observations

The fake `Ros2TcpBridge` now uses this aggregator. It only invokes
`TcpRuntimeAdapter.predict_control()` after a complete current-tick observation
is ready. Every blocked state publishes the existing full-brake control.

Generate an inspectable contract with:

```powershell
python runners/build_ego_observation_contract.py `
  --camera-profile tcp_front `
  --role-name ego_vehicle `
  --output outputs/ego_observation_contract.json
```

The output says that its runtime binding is unvalidated. It does not import
CARLA, `rclpy`, PyTorch, or an external model.

## Real Environment Work

The first CARLA/ROS 2 integration should preserve one simulator-clock owner and
perform this sequence for every world snapshot:

1. ClosedLoopBench ticks CARLA once and records `snapshot.frame` and simulation time.
2. Camera callbacks for that exact frame are collected with actual calibration.
3. Ego state and route are sampled and tagged with the same frame id.
4. The aggregator accepts the bundle or commands a safe stop.
5. The ROS 2 binding publishes typed messages to the external plugin.
6. A command tagged for that observation is accepted before the deadline or
   `Ros2ControlDriver` applies full braking.

Acceptance requires logged frame ids on both sides, at least one accepted
control, demonstrated stale-command braking, and a CARLA-backed report. Until
then, only the contract and fail-closed behavior are locally verified.
