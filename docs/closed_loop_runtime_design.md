# Closed Loop Runtime Design Gate

ClosedLoopBench treats CARLA execution as a staged runtime boundary. The core contract must stay runnable without CARLA, while CARLA, ScenarioRunner, ROS2 bridge, and model-specific ego stacks plug in only after exchange artifacts and dry-run metrics are already validated.

## Runtime Stages

1. `exchange_validation`
   - Input: Scenario IR
   - Output: `.xosc`, `.xodr`, `carla_run_config.json`
   - Required runtime: none, optional esmini
   - Goal: prove the scenario contract is portable before CARLA is launched

2. `carla_baseline_closed_loop`
   - Input: `carla_run_config.json`, `.xosc`, `.xodr`
   - Ego: CARLA BasicAgent or BehaviorAgent
   - Actors: replay, ghost, TrafficManager, or scripted trigger
   - Goal: ego closed-loop control with minimal dependencies

3. `ros2_external_ego`
   - Input: same run config plus ROS2 topic contract
   - Ego: external stack publishing vehicle control commands
   - Required runtime: CARLA ROS2 bridge and stack process
   - Goal: benchmark real AD stack behavior using the same scenario contract

4. `optional_tcp_adapter`
   - Input: ROS2/external ego contract plus TCP-specific runtime path and checkpoint metadata
   - Ego: TCP model adapter
   - Required runtime: user-provided TCP repo/checkpoints/container
   - Goal: demonstrate a classic end-to-end baseline without making TCP a core dependency

## Closed Loop Definition

- Ego closed-loop: ego observes simulator state each tick and publishes new controls.
- Half closed-loop: ego is closed-loop but actors replay fixed trajectories.
- Interactive closed-loop: ego and at least one actor react to the evolving simulator state.
- Full stack closed-loop: ego controls are produced by an external ROS2 stack, while actors are replay/scripted/reactive according to the run config.

## TCP Boundary

TCP exists as an open-source CARLA end-to-end baseline, but it should not be a required dependency for this project. ClosedLoopBench should expose TCP through the same `ros2_external_agent` or `external_agent` boundary used by TransFuser, InterFuser, UniAD, and custom stacks.

The core repo should only own:

- policy config schema
- ROS2 topic mapping
- timeout and fallback behavior
- runtime path/checkpoint metadata validation
- benchmark report fields

The TCP repo/container should own:

- model dependencies
- checkpoint loading
- camera preprocessing
- inference loop
- conversion from model output to vehicle command

## Test Gate

Tests must pass without CARLA, ScenarioRunner, ROS2, or TCP installed. Unit tests should validate:

- configs and command plans are deterministic
- import guards fail with actionable messages
- TCP is optional and never required for the core dry-run path
- metrics reports distinguish dry-run, ego closed-loop, and interactive closed-loop statuses
