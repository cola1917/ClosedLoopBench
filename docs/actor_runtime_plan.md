# Actor Runtime Plan

This document defines the MVP actor runtime plan boundary for ClosedLoopBench.
It is intentionally small: the actor layer produces serializable plans that a
later CARLA runtime can consume. It does not call CARLA APIs, spawn actors, tick
the world, or implement a custom rule engine.

## Inputs

Actor runtime planning consumes:

- `carla_run_config.mvp.v0` actors
- actor policy config from `actors.policy_config`
- actor style profiles from `actors.style_profiles`

The plan keeps Scenario IR reference trajectories as the source of scenario
intent. Runtime execution may later bind these plans to CARLA actor ids, but
that binding is not part of this batch.

## Runtime Modes

### `replay`

Replay actors follow the recorded reference trajectory from Scenario IR.

- closed-loop level: `replay`
- ego-responsive: no
- CARLA runtime required: no for plan generation
- evaluation meaning: half-closed-loop when ego is controlled by an agent

This is the default for context/background actors and is useful as a stable
baseline.

### `scripted`

Scripted actors are reference-conditioned maneuver plans. The plan exposes
trigger-condition parameters such as reaction time, TTC threshold, and abort
behavior from the selected style profile.

- closed-loop level: `scripted`
- ego-responsive: yes, at runtime
- CARLA runtime required: yes for execution
- evaluation meaning: interactive closed-loop candidate

The MVP does not implement a full rule engine. It only emits a small controller
contract that a future CARLA runtime can bind to trigger checks.

### `traffic_manager`

TrafficManager actors delegate reactive behavior to CARLA TrafficManager or an
equivalent runtime controller. The plan converts style profile fields into
low-code runtime parameters such as desired headway, minimum gap, reaction time,
and lane-change gap acceptance.

- closed-loop level: `traffic_manager_reactive`
- ego-responsive: yes, at runtime
- CARLA runtime required: yes for execution
- evaluation meaning: lowest-code interactive closed-loop candidate

The plan does not call TrafficManager. It only records deferred runtime binding
and parameters.

## Output Contract

`build_actor_runtime_plan_set(run_config)` emits:

- `schema_version`
- `scenario_id`
- per-actor plans
- runtime mode summary
- interactive candidate count
- runtime boundary flags

The `runtime_boundary` field must continue to state:

- `owns_carla_control_loop: false`
- `owns_traffic_manager_api_calls: false`
- `plan_only: true`

This keeps the actor planning layer explainable and prevents it from growing
into a simulator runtime.

## Out of Scope

This batch does not implement:

- real CARLA actor spawning
- `world.tick()` loops
- `vehicle.apply_control`
- TrafficManager API calls
- complex behavior trees or custom actor rule engines
- ROS2 actor control

Those belong to later CARLA runtime integration work.
