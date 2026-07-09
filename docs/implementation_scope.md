# ClosedLoopBench Implementation Scope

ClosedLoopBench is the primary simulation-test engineering project. It owns executable scenario evaluation, not data mining and not neural reconstruction.

## Primary Goal

Turn TriggerEngine `Scenario IR` into runnable and testable simulation evaluation artifacts:

```text
Scenario IR
  -> OpenSCENARIO / OpenDRIVE portable scenario artifacts
  -> CARLA run config
  -> ScenarioRunner fallback / CARLA runtime extension
  -> actor policy configuration
  -> ego policy plugin configuration
  -> metrics and closed-loop report
```

## MVP Scope

MVP must work without a full CARLA workstation.

Required:

- consume `Scenario IR`
- generate `carla_run_config.json`
- generate `scenario.xosc`
- generate minimal `road.xodr`
- validate `.xosc/.xodr` with esmini when available
- generate `closed_loop_report.json` in dry-run mode
- report actor policy modes
- support optional `Reconstruction Package` artifact reference
- provide named metric primitives for `collision`, `min_ttc`, `min_distance`, `route_progress`, `hard_brake`, and `max_jerk`

MVP does not require:

- launching CARLA
- launching ScenarioRunner
- UniAD runtime
- ROS2 runtime
- NuRec/Cosmos runtime

## Runtime Scope

When CARLA/ScenarioRunner is available, ClosedLoopBench should add:

- CARLA server connection probe
- ScenarioRunner scenario class generation or wrapper only for fallback/runtime extension behavior beyond portable OpenSCENARIO
- ego vehicle spawn and control loop
- actor spawn and controller loop
- per-tick metric collection
- report generation from real simulation traces
- final named metrics for `TET`, `TIT`, `PET`, and `DRAC` in addition to MVP safety and comfort summaries

## Ego Policy Plugins

ClosedLoopBench should not be tied to one ego stack. Ego policy is a plugin boundary.

Supported stages:

1. `baseline_lane_following`
   - deterministic MVP baseline
   - suitable for smoke and metrics pipeline

2. `ros2_stack`
   - external AD stack through ROS2 topics/services
   - used when local CARLA ROS2 bridge is ready
   - initial classic E2E targets should prioritize TransFuser, InterFuser, and TCP

3. `uniad`
   - end-to-end model plugin
   - later plugin path behind the ROS2 bridge
   - optional showcase path, not required runtime and not core architecture

4. `external_agent`
   - generic adapter for other planners/controllers

The ROS2 bridge is the adapter boundary for E2E policies: ClosedLoopBench owns normalized scenario, route, ego state, world state, timeout, and fallback configuration, while each external stack owns its runtime process and model dependencies.

## Actor Model Scope

Actor behavior is reference-conditioned and style-parameterized.

The actor layer intentionally separates scenario intent from controller
complexity. Reference trajectories, trigger metadata, and actor roles come from
Scenario IR; style templates tune the policy parameters used to replay, script,
or react to that reference. This is enough for counterfactual closed-loop runs:
the same recorded scenario can be evaluated with a defensive, normal,
aggressive, delayed, or noncompliant actor without writing a bespoke rule system
for every case.

### Actor Closed-Loop Levels

ClosedLoopBench uses three explicit actor closed-loop levels:

- `replay`: the actor follows the recorded Scenario IR trajectory. This is a half-closed-loop baseline because ego can be controlled by an agent, but actor motion does not change when ego behavior changes.
- `scripted`: the actor executes a reference-conditioned maneuver and can branch on ego state, trigger timing, TTC, or accepted gap. This is an interactive closed-loop level and requires CARLA runtime execution.
- `traffic_manager_reactive`: the actor is delegated to CARLA TrafficManager or an equivalent reactive controller configured from style templates. This is the lowest-code interactive closed-loop level and requires CARLA runtime execution.

The selected level must be visible in both actor policy config and closed-loop reports. MVP can validate these fields without CARLA; runtime work later fills per-tick behavior and metrics.

Stages:

1. `replay_actor`
   - follows Scenario IR reference trajectory
   - half-closed-loop baseline

2. `ghost_actor`
   - does not control a physical CARLA vehicle
   - used for TTC/collision-envelope metrics and fast dry-runs

3. `traffic_manager_actor`
   - delegates behavior to CARLA TrafficManager
   - lowest-code interactive option once CARLA is available

4. `scripted_trigger_actor`
   - executes maneuver templates extracted from Scenario IR
   - used for cut-in, hard-brake, crossing, unprotected-left-turn showcases

## Driving Style Templates

Style profiles are parameter sets, not separate rule systems.

Required templates:

- `defensive`
- `normal`
- `assertive`
- `aggressive`
- `delayed`
- `noncompliant`

These should influence gap acceptance, TTC thresholds, reaction time, acceleration, braking, and abort behavior.

## Exchange Formats

OpenSCENARIO is the primary portable scenario format. OpenDRIVE is the paired static road/map exchange artifact. Python ScenarioRunner artifacts are fallback/runtime extensions for CARLA-specific behavior that cannot be represented portably in OpenSCENARIO.

- `.xosc` expresses entities, init, triggers, and stop conditions.
- `.xodr` is a minimal road placeholder in MVP.
- esmini validates exchange artifacts before CARLA is available.

OpenDRIVE is not the source of truth for nuScenes map reconstruction in MVP.

## Out Of Scope

ClosedLoopBench does not:

- mine raw logs
- compute TriggerEngine risk tags
- run NuRec or Cosmos
- own sim-to-real visual metrics
- require UniAD for the first closed-loop demo

## Interview Summary

ClosedLoopBench demonstrates simulation-test engineering by separating:

```text
scenario contract
  -> exchange validation
  -> dry-run metric reporting
  -> runtime handoff
  -> full CARLA closed-loop execution
```

The design lets the project be useful before the heavy environment is available and executable once CARLA/ScenarioRunner/ROS2 are connected.
