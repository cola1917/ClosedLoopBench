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

## Reactive Actor Runtime

Project 3 now has a lightweight ego-reactive actor runtime that can be tested
without CARLA:

- entry point: `actors.reactive_actor.plan_reactive_actor_control`
- inputs: actor state, optional ego state, actor style, optional reference speed
- ego signals: distance and relative closing speed, or x/y position fallback
- outputs: desired speed, brake flag, yield flag, abort flag, lane-change flag,
  TTC, distance, and an explainable reason

The runtime reuses `actors.style_profiles.ActorStyleProfile`:

- `defensive`: larger minimum gap and TTC threshold, disables lane changes in
  tight interactions, aborts on low TTC
- `normal`: moderate minimum gap and TTC threshold, aborts on low TTC
- `aggressive`: smaller accepted gap and TTC threshold, can keep lane changes
  enabled, does not abort on low TTC

When no ego state is available, the runtime falls back to reference speed if
provided, otherwise a safe stopped default. This keeps replay/reference behavior
available while making ego-conditioned decisions explicit when sensor state is
present.

CARLA TrafficManager execution remains a binding layer rather than the actor
control loop. `actors.traffic_manager_executor.build_traffic_manager_actor_settings`
maps `traffic_manager_reactive` and `scripted` actor plans to TrafficManager
configuration knobs:

- `min_gap_m` -> `distance_to_leading_vehicle`
- style speed policy -> `vehicle_percentage_speed_difference`
- style gap policy -> `auto_lane_change`

Only `runtime_mode=traffic_manager` is spawned by the executor. `scripted` plans
remain plan-only fallbacks in this executor, but they now carry the same
TrafficManager-compatible configuration so a runner can bind them later.

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
- complex behavior trees or custom actor rule engines
- ROS2 actor control

Those belong to later CARLA runtime integration work.
