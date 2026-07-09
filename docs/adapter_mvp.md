# Adapter MVP Plan

ClosedLoopBench starts with CARLA ScenarioRunner configuration, not OpenSCENARIO/OpenDRIVE export.

## MVP Adapter

`adapters.ir_to_carla.build_carla_run_config(scenario_ir)` returns a deterministic CARLA run config:

- CARLA version and default map
- event and warmup windows
- ego initial state and reference trajectory
- actor initial states and replay/reactive policy hints
- metric list
- optional reconstruction package hook

## Format Priority

1. ScenarioRunner JSON/Python config: MVP and primary execution path.
2. OpenSCENARIO `.xosc`: optional exchange/export format after the Python path works.
3. Scenic: optional scenario-family generator.
4. OpenDRIVE `.xodr`: later map enhancement. It is not required for MVP because CARLA existing towns can host semantic-equivalent scenarios.

## Final Adapter Direction

The final adapter should compile scenario-family variants, reactive actor configs, ROS2 ego stack launch parameters, and optional NuRec/Cosmos sensor realism hooks.
