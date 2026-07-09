# TrafficManager Actor Execution Skeleton

This batch adds the narrow runtime skeleton that binds `traffic_manager` actor
plans to CARLA TrafficManager-style calls. It remains deliberately small: it
spawns planned TrafficManager actors, enables autopilot, and applies style
parameters through standard TrafficManager hooks. It does not own the CARLA tick
loop and does not implement a custom actor controller.

## Boundary

Input is the existing `actor_runtime_plan.mvp.v0` artifact from
`actors.runtime_plan`.

Runtime handling is mode-specific:

- `traffic_manager`: spawn a vehicle actor, call `set_autopilot(True, tm_port)`,
  and bind min-gap, speed-difference, and lane-change settings to TrafficManager.
- `replay`: return `plan_only_fallback`; replay execution stays outside this
  skeleton.
- `scripted`: return `plan_only_fallback`; scripted trigger execution stays
  outside this skeleton.

The executor accepts injected `world`, `client`, `traffic_manager`, and optional
`transform_factory` objects so unit tests can run without CARLA and real runtime
debug can swap in CARLA objects later.

## Closed-loop Meaning

TrafficManager actors are the first low-code actor path toward interactive
closed-loop evaluation. Ego behavior can change the CARLA world state, and
TrafficManager can react through its built-in car-following and lane-change
logic. This is stronger than replay, but still intentionally simpler than a
custom rule engine.

## Real Environment Debug Steps

1. Start CARLA and confirm `runners/probe_carla.py` reports available.
2. Build or load `actor_runtime_plan.mvp.v0` from `carla_run_config`.
3. Inject real CARLA `world` and `client.get_trafficmanager(tm_port)` into the
   executor.
4. Replace the default dict transform with a CARLA `Transform(Location, Rotation)`
   factory.
5. Run a short synchronous tick loop owned by the CARLA runner, not by this
   actor module.
6. Inspect the executor binding report for `traffic_manager_bound` actors and
   `plan_only_fallback` actors.

## Non-goals

- No CARLA world tick loop.
- No ScenarioRunner behavior tree integration.
- No custom actor controller or complex rule engine.
- No ROS2 actor control.
- No replay trajectory controller.
