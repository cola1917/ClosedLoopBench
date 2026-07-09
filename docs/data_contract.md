# ClosedLoopBench Data Contract Design

## Boundary

ClosedLoopBench consumes Project 1 `Scenario IR` as the source of truth for scene logic and closed-loop setup.

ClosedLoopBench may optionally consume NeuralSceneBridge `Reconstruction Package` for NuRec/Cosmos visual realism, but ClosedLoopBench must still run without NeuralSceneBridge.

## MVP Contract

### Required Input From Scenario IR

- `scenario_id`
- `scenario_type`
- `windows.event`
- `windows.warmup`
- `coordinate_frame`
- `map_context`
- `ego.initial_state`
- `ego.reference_trajectory`
- `ego.route`
- `actors[].initial_state`
- `actors[].reference_trajectory`
- `actors[].role`
- `events.trigger`
- `evaluation.metrics`

### Optional Input From NeuralSceneBridge

- `reconstruction_package_id`
- `nurec.scene_path`
- `alignment.sim_from_log_transform`
- `cosmos.transfer_config_path`
- `quality.view_validity`

### MVP Output

- `carla_scenario_config.json`
- ScenarioRunner Python scenario or OpenSCENARIO export
- `closed_loop_report.json`
- per-tick metrics trace

## Final Closed-Loop Contract

The final version should additionally support:

- reference-conditioned reactive actor policies
- scenario-family parameter sweeps
- ego policy variants and reaction-delay sweeps
- ROS2 ego stack integration
- NuRec/Cosmos sensor pipeline selection
- closed-loop replay artifacts and comparable reports

## Closure Definition

MVP closure means ego control changes CARLA world state while actors may replay or follow scripts.

Final interactive closure means trigger actors react to ego state through rule-based or learned policies.

