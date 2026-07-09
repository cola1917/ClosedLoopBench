# ClosedLoopBench Architecture and Contract

## One-Line Positioning

ClosedLoopBench is the closed-loop evaluation layer that consumes TriggerEngine Scenario IR and compiles it into portable OpenSCENARIO/OpenDRIVE, simulator-ready CARLA/ScenarioRunner fallback artifacts, and report artifacts.

## System Boundary

ClosedLoopBench starts after scenario mining and reconstruction. It treats Scenario IR as the source of truth for scenario intent, timing, route, actor state, trigger logic, and evaluation metrics.

### Inputs

Required:

- TriggerEngine `Scenario IR`

Optional:

- Reconstruction Package from NeuralSceneBridge
- NuRec scene reference and alignment metadata
- Cosmos sensor transfer configuration

### Outputs

ClosedLoopBench produces:

- CARLA run config
- OpenSCENARIO `.xosc` portable scenario artifact
- OpenDRIVE `.xodr` static map/road exchange artifact
- ScenarioRunner-oriented config and launch artifacts when Python runtime extensions are needed
- closed-loop report, including execution status, metric summary, actor policy modes, and artifact references

### Out of Scope

ClosedLoopBench does not own:

- scenario mining
- Scenario IR authoring
- NuRec reconstruction
- Cosmos reconstruction or transfer training
- dataset curation
- learned agent training

Those systems may feed ClosedLoopBench, but ClosedLoopBench must remain executable without them.

## Architecture

```text
Scenario IR
    |
    v
IR contract validation
    |
    v
Scenario compiler
    |-- OpenSCENARIO export
    |-- OpenDRIVE static export
    |-- CARLA run config
    |-- ScenarioRunner fallback/runtime-extension artifacts
    |
    v
Closed-loop runner
    |-- dry-run runner in MVP
    |-- CARLA runtime runner in final system
    |
    v
Closed-loop report
```

The scenario compiler is the stable contract boundary. Runtime integrations can evolve from dry-run to CARLA execution without changing the upstream Scenario IR contract or downstream report shape.

## Contract Shape

### Scenario IR Consumption

ClosedLoopBench consumes the following Scenario IR concepts:

- scenario identity and family
- event window and warmup window
- coordinate frame and map context
- ego initial state, route, and reference trajectory
- actor initial states, roles, and reference trajectories
- trigger/event metadata
- evaluation metric definitions

The compiler may enrich those concepts into CARLA-specific parameters, but the original Scenario IR remains the behavioral source of truth.

### CARLA Run Config

The CARLA run config is the execution-normalized form. It should include:

- run identity and schema version
- CARLA version, map, timestep, weather, and seed when available
- ego policy plugin selection and adapter parameters
- actor policy mode per actor
- warmup and event windows
- metric list and reporting configuration
- optional reconstruction package hook
- artifact references for ScenarioRunner, OpenSCENARIO, OpenDRIVE, and report outputs

### Portable Scenario Artifacts

OpenSCENARIO is the primary portable scenario format for ClosedLoopBench. OpenDRIVE carries the static road/map exchange surface paired with the `.xosc` scenario. These artifacts are compiled from Scenario IR and are not separate sources of truth.

- OpenSCENARIO expresses scenario structure, entities, initialization, triggers, and stop conditions for external tools and simulator handoff.
- OpenDRIVE expresses static road geometry/map exchange when available.
- ScenarioRunner Python artifacts are fallback/runtime extensions for CARLA-specific behavior that cannot be expressed portably in OpenSCENARIO.
- Lossy fields must be reported in the closed-loop report or export diagnostics.

### Closed-Loop Report

The report is the comparable evaluation artifact. MVP reports can be dry-run reports, but the final report shape should also support real CARLA execution.

Required report concepts:

- run id and scenario id
- execution status
- ego policy plugin and actor policy modes
- generated artifact paths
- metric summary
- per-tick metrics trace slot
- warnings for unsupported or lossy export behavior

Named metric coverage:

- MVP metrics: `collision`, `min_ttc`, `min_distance`, `route_progress`, `hard_brake`, `max_jerk`
- Final metrics: `collision`, `min_ttc`, `min_distance`, `route_progress`, `hard_brake`, `max_jerk`, `TET`, `TIT`, `PET`, `DRAC`

The named metrics library is pure and simulator-agnostic. Runtime adapters should emit per-tick primitive values such as TTC, distance, longitudinal acceleration, jerk, collision flags, and route progress; the report layer aggregates comparable summary fields from those traces.

## Ego Policy Plugin Boundary

The ego vehicle is always controlled through a plugin interface. ClosedLoopBench must not be tied to UniAD or to any single autonomy stack.

Supported plugin classes:

- `baseline`: built-in deterministic policies for smoke tests and reproducibility.
- `ros2_stack`: external ROS2 autonomy stack launched through an adapter. The ROS2 bridge is the integration boundary: ClosedLoopBench publishes normalized ego/world/route inputs and consumes control-command output.
- `uniad`: optional showcase adapter implemented as one external-agent plugin, not a required runtime dependency.
- `external_agent`: generic process, RPC, container, or Python module adapter.

For classic end-to-end driving evaluation, the preferred first stacks are TransFuser, InterFuser, and TCP. UniAD remains a later plugin path behind the same ROS2 boundary.

Expected ego plugin responsibilities:

- receive ego state, world state, route, and optional sensor observations
- return control commands or trajectory commands at each tick
- expose initialization, reset, tick, and shutdown lifecycle hooks
- declare required sensors, maps, runtime dependencies, and control mode
- emit adapter diagnostics for the report

ClosedLoopBench responsibilities:

- load and configure the selected plugin
- provide normalized world state and scenario context
- apply returned control commands to CARLA
- record plugin identity, version, parameters, and failures

The plugin contract allows a simple baseline controller, a ROS2 stack, UniAD, or a future learned policy to run behind the same evaluation surface.

## Actor Model

Actors are modeled separately from the ego plugin. The actor controller layer chooses a policy mode per actor and can mix modes in one scenario.

Actor behavior is reference-conditioned and style-parameterized. The reference
condition keeps each actor anchored to Scenario IR timing, route, and maneuver
intent, while the selected style profile changes time headway, accepted gaps,
reaction time, TTC yielding, and abort behavior. This lets ClosedLoopBench run
counterfactual closed-loop evaluations without encoding complex bespoke rules
for every mined scenario.

### Actor Layers

- `replay_actor`: follows the Scenario IR reference trajectory as recorded.
- `ghost_actor`: replays reference motion without collision authority or with reduced physical interaction, useful for perception/path comparison.
- `traffic_manager_actor`: delegates behavior to CARLA TrafficManager with configured route, speed, and behavior parameters.
- `reference_conditioned_scripted_actor`: follows a scripted controller conditioned on the reference trajectory, ego state, triggers, and style template.

### Driving Style Templates

Reference-conditioned and TrafficManager-backed actors should support style templates:

- `defensive`: larger gaps, lower acceleration, earlier braking.
- `normal`: nominal legal behavior and reference tracking.
- `assertive`: smaller gaps and stronger acceleration while still compliant.
- `aggressive`: high acceleration, late braking, tight gaps.
- `delayed`: delayed reaction to triggers or ego motion.
- `noncompliant`: rule-violating or adversarial behavior when explicitly requested by the Scenario IR.

The selected actor mode and style must be visible in the run config and closed-loop report.

## MVP Scope

MVP proves the contract without requiring a live CARLA runtime.

MVP deliverables:

- dry-run closed-loop report
- OpenSCENARIO portable scenario export
- OpenDRIVE static export
- ScenarioRunner fallback/runtime-extension config
- actor policy mode reporting

MVP non-goals:

- real CARLA process management
- closed-loop ego control in the simulator
- reactive actor controllers
- ROS2 or UniAD runtime execution
- NuRec/Cosmos sensor rendering path

## Final Scope

Final ClosedLoopBench executes closed-loop scenarios in CARLA and keeps the same contract surfaces.

Final deliverables:

- CARLA runtime orchestration
- ego control loop through plugin adapters
- actor controller with replay, ghost, TrafficManager, and reference-conditioned scripted modes
- per-tick metrics and event traces
- ROS2 adapter
- UniAD or external-agent adapter
- optional NuRec/Cosmos sensor path from a Reconstruction Package
- comparable reports across baseline, stack, and external-agent runs

## Integration Principles

- Keep Scenario IR as the source of truth.
- Keep reconstruction optional.
- Keep ego autonomy behind plugins.
- Treat OpenSCENARIO as the primary portable scenario artifact, with Python ScenarioRunner only as fallback/runtime extension.
- Make lossy export or unsupported runtime behavior visible in reports.
- Preserve the same report contract from dry-run MVP to final CARLA execution.
