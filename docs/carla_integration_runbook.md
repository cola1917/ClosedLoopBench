# CARLA Integration Runbook

This runbook defines the handoff from local contract validation to a CARLA-backed
closed-loop smoke run. It intentionally does not define ROS2 callbacks, CARLA
sensor listeners, or model inference code.

## Scope

ClosedLoopBench owns:

- exchange artifacts: `scenario.latest.xosc`, optional `road.xodr`, and `carla_run_config.json`
- runtime plans: CARLA probe, ScenarioRunner command plan, BasicAgent plan
- metric rows and `closed_loop_report.mvp.v0`
- optional handoff metadata for ROS2/TCP ego stacks

CARLA, ScenarioRunner, ROS2 bridge, and TCP/other ego repositories own their own
process lifecycle and runtime dependencies.

## 1. Start CARLA

Start the CARLA server that matches the intended Python API and ScenarioRunner
environment.

Expected default endpoint:

```powershell
127.0.0.1:2000
```

The first integration target is a normal CARLA map smoke run. A reconstructed or
custom map should be introduced only after the basic probe and ScenarioRunner
path are stable.

For the complete BasicAgent, TrafficManager, scripted actor, and ROS2 control
acceptance gate, see `docs/core_closed_loop_integration.md`.

## 2. Probe CARLA

Run the probe before any scenario execution:

```powershell
python runners/probe_carla.py --host 127.0.0.1 --port 2000
```

Expected result:

- `status="available"` when the CARLA Python API and server are reachable
- `status="unavailable"` with a concrete reason when the API is missing, the
  server is not reachable, or the requested map does not match

The probe must not spawn vehicles or mutate world settings.

## 3. Plan ScenarioRunner OpenSCENARIO Execution

Build the ScenarioRunner command from the generated `.xosc`:

```powershell
python runners/plan_scenario_runner.py `
  --openscenario outputs/scene-1077-integrated/scenario.latest.xosc `
  --scenario-runner-root <SCENARIO_RUNNER_ROOT> `
  --host 127.0.0.1 `
  --port 2000
```

The command plan should use OpenSCENARIO as the primary scenario exchange path.
Python ScenarioRunner scenarios remain a fallback for CARLA-specific runtime
extensions that cannot be expressed in OpenSCENARIO.

## 4. Plan BasicAgent Ego Closed Loop

Build the BasicAgent runtime plan:

```powershell
python runners/run_carla_basic_agent.py `
  --run-config outputs/scene-1077-integrated/carla_run_config.json
```

This plan is the first ego closed-loop target. In the real runtime, BasicAgent
observes the CARLA world each tick, computes a fresh control command, and applies
that command to the ego vehicle.

The local code now includes a BasicAgent runtime skeleton behind `--execute`.
Without CARLA or CARLA's BasicAgent package it returns a structured `failed`
result. With CARLA available, the expected sequence is:

```text
connect CARLA client
load requested map or current world
save current world settings
enable synchronous stepping
spawn ego vehicle
set BasicAgent destination
for each tick:
  world.tick()
  agent.run_step()
  vehicle.apply_control()
  add metric row
restore original world settings
destroy ego vehicle
write/return closed_loop_report
```

This runtime path has fake-CARLA unit coverage, but the first real CARLA run is
still an integration/debug task.

Actor closure level is reported separately:

- `replay`: ego closed-loop with fixed actor trajectories
- `scripted`: actors can react through scripted triggers
- `traffic_manager_reactive`: interactive closed-loop through CARLA TrafficManager
  or an equivalent reactive controller

## 5. Metric Collector Handoff

Runtime code should convert each CARLA tick into the minimal tick row:

```json
{
  "t_sec": 0.1,
  "ego": {
    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "speed_mps": 5.0,
    "control": {"throttle": 0.2, "brake": 0.0, "steer": 0.0}
  },
  "actor_distances_m": {"actor-1": 12.0},
  "ttc": 3.4,
  "min_ttc": 3.4,
  "collision": false,
  "route_progress": 0.25,
  "hard_brake": false,
  "jerk": 1.2
}
```

`metrics.collector.TickMetricCollector` stores these rows and hands them to
`metrics.report.build_closed_loop_report`. The collector does not subscribe to
CARLA sensors and does not own ROS2 runtime behavior.

Supported report statuses:

- `not_run`: dry-run report or no runtime attempt
- `planned`: command/runner plan produced
- `ego_closed_loop`: ego controls were generated from simulator state
- `interactive_closed_loop`: ego and at least one actor reacted to runtime state
- `failed`: runtime attempted but did not complete
- `completed`: legacy success status kept for compatibility

## 6. Later ROS2/TCP Handoff

After BasicAgent works, TCP or another end-to-end ego stack should enter through
the external ego boundary:

```text
CARLA sensors / state / route
  -> CARLA ROS2 bridge
  -> external TCP adapter or container
  -> vehicle control command
  -> CARLA ego vehicle
```

ClosedLoopBench should validate the topic/config contract and report metrics.
The TCP repository or container should own model dependencies, checkpoints,
preprocessing, and inference.


