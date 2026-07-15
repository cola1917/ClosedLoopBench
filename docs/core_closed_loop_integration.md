# Core Closed-Loop Integration

This is the environment handoff for the core ClosedLoopBench target. NuRec,
Cosmos, and novel-view rendering are intentionally outside this path.

## Runtime Ownership

```text
ClosedLoopBench                 owns CARLA synchronous ticks and KPI collection
CARLA TrafficManager           owns traffic_manager_reactive actor control
ClosedLoopBench scripted loop  owns scripted actor control
CARLA ROS Bridge               exposes current-tick sensors and vehicle state
External ROS2 algorithm        publishes ego vehicle control
```

Only one process may own CARLA synchronous ticks. When ClosedLoopBench runs the
tick loop, configure CARLA ROS Bridge in passive mode so it observes the same
world instead of advancing it independently.

## 1. BasicAgent Baseline

Start CARLA, then run the built-in baseline before attempting ROS2:

```bash
python runners/run_carla_basic_agent.py \
  --run-config outputs/scene-1077-integrated/carla_run_config.json \
  --host 127.0.0.1 --port 2000 --max-ticks 600 \
  --execute --snap-to-map --follow-ego --debug-draw
```

`traffic_manager_reactive` actors bind to TrafficManager by default. Pass
`--no-actor-autopilot` only when debugging actor spawn without interactive
traffic control.

The run may report `interactive_closed_loop` only when at least one actor has
runtime control evidence. Merely declaring an interactive actor is not enough.

## 2. ROS2 Ego Control

Required runtime packages:

- CARLA Python API matching the server
- ROS2 and `rclpy`
- `carla_msgs`
- CARLA ROS Bridge configured for the same ego role and in passive mode
- an external algorithm publishing `CarlaEgoVehicleControl`

Run the same scenario with ROS2 control:

```bash
python runners/run_carla_basic_agent.py \
  --run-config outputs/scene-1077-integrated/carla_run_config.json \
  --host 127.0.0.1 --port 2000 --max-ticks 600 \
  --ego-driver ros2_control \
  --control-topic /carla/ego_vehicle/vehicle_control_cmd \
  --control-timeout-sec 0.5 \
  --execute --snap-to-map
```

The ROS2 driver applies a full-brake safe stop before the first valid command
and whenever the latest command is stale. A run that receives no valid ROS2
control command fails instead of being reported as ego closed-loop.

CARLA ROS Bridge or an algorithm-specific adapter owns sensor and route topic
publication. ClosedLoopBench subscribes to the normalized control result; it
does not vendor model preprocessing or checkpoints.

## 3. KPI Evidence

The runtime records:

- collision events from an attached CARLA collision sensor
- route progress from projection of the physical ego pose onto the configured route
- ego speed, finite-difference acceleration, jerk, and hard braking
- actor distance, TTC estimate, decision, and control evidence
- ROS2 control and safe-stop counts when the ROS2 driver is selected

If the collision sensor cannot be created, collision KPI is `unknown`; it must
not silently pass as zero collisions.

## 4. Environment Acceptance Gate

The core runtime is accepted only after all of these pass on the target host:

1. BasicAgent completes a fixed route three consecutive times.
2. A TrafficManager actor changes its physical state and reports
   `traffic_manager` control evidence.
3. A scripted actor receives `scripted_vehicle_control` and visibly changes
   speed or braking behavior.
4. A forced collision is detected by the collision sensor and fails the KPI.
5. A stationary ego does not gain route progress merely because ticks advance.
6. An external ROS2 algorithm produces at least one accepted control command.
7. Missing or stale ROS2 control produces a safe stop and a diagnostic count.
8. Every success and failure restores world settings and destroys spawned actors
   and sensors.

## Remaining Environment Work

The repository is prepared for, but cannot prove without the target runtime:

- exact CARLA/ScenarioRunner/ROS2 version compatibility
- CARLA ROS Bridge passive-mode topic names and QoS
- collision callback timing under synchronous stepping
- TrafficManager behavior on the selected maps
- algorithm sensor calibration and preprocessing
- deterministic repeatability and KPI thresholds

These are integration validations, not missing offline architecture.

## 5. Compare Algorithms And ODDs

Each runtime report records `algorithm_id`, `algorithm_version`, `odd_id`, and
`seed` from the run plan. After running the same scenario with multiple
algorithms or CARLA weather presets, aggregate the reports with:

```bash
python runners/compare_reports.py \
  outputs/tcp-clear/closed_loop_report.json \
  outputs/tcp-rain/closed_loop_report.json \
  outputs/basic-agent-clear/closed_loop_report.json \
  --output outputs/comparison.json
```

The comparison groups runs by algorithm and reports scenario/ODD coverage,
result counts, and mean KPI values. Statistical confidence and repeated-seed
policy remain environment validation work.
