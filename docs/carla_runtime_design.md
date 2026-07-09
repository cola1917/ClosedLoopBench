# CARLA Runtime and ScenarioRunner Design

This document defines the next TDD boundary for ClosedLoopBench. It is intentionally runtime-facing but implementation-light: tests should be able to run on a laptop without CARLA, ScenarioRunner, ROS2, or the CARLA Python package installed.

## Scope

The CARLA runtime layer starts after these artifacts already exist:

- `carla_run_config.json`
- `scenario.xosc`
- optional `road.xodr`
- optional reconstruction package reference

The runtime layer must not reinterpret Scenario IR directly. Scenario IR is compiled once into exchange/runtime artifacts, then CARLA consumes those artifacts.

## Stage 1: CARLA Probe

Purpose: answer whether this workstation can run a CARLA-backed evaluation before launching a full scenario.

Planned module:

- `runtime.carla_probe`

Planned public functions:

- `build_probe_config(host, port, timeout_sec, map_name=None)`
- `probe_carla(config, carla_module=None)`

Required behavior:

- Import `carla` only inside the probe function or accept an injected `carla_module`.
- Return a structured result instead of raising when CARLA is missing.
- Verify client creation, timeout setup, world access, optional map name check, and simulator version when available.
- Never spawn vehicles or mutate the world.

Expected result shape:

```json
{
  "status": "available",
  "host": "127.0.0.1",
  "port": 2000,
  "map": "Town04",
  "carla_version": "0.9.16",
  "warnings": []
}
```

When CARLA is unavailable, `status` should be `unavailable` and `reason` should explain `missing_python_api`, `connection_failed`, or `map_mismatch`.

## Stage 2: ScenarioRunner OpenSCENARIO Run

Purpose: run the portable `.xosc` artifact through CARLA ScenarioRunner without custom Python scenario code.

Planned module:

- `runtime.scenario_runner`

Planned public function:

- `build_scenario_runner_command(config)`

Command boundary:

```text
python <SCENARIO_RUNNER_ROOT>/scenario_runner.py
  --openscenario <scenario.xosc>
  --host <host>
  --port <port>
  --output
```

Required behavior:

- Build commands as argument lists, not shell strings.
- Keep ScenarioRunner root, Python executable, host, port, timeout, output directory, and `.xosc` path explicit in config.
- Validate the `.xosc` path before execution in the real runner.
- Execution itself should live behind a separate function so command construction remains unit-testable.

OpenSCENARIO remains the primary path. Python ScenarioRunner scenarios are fallback/runtime extensions only when reactive actor behavior, ROS2 synchronization, or CARLA-specific hooks cannot be expressed in OpenSCENARIO.

## Stage 3: BasicAgent Runner

Purpose: create the first true ego closed-loop mode in CARLA.

Planned module:

- `runners.run_carla_basic_agent`

Planned public functions:

- `build_basic_agent_plan(run_config, host, port, max_ticks, synchronous=True)`
- `run_basic_agent(plan, carla_module=None, agent_module=None)`

Runtime boundary:

- Load the map from `carla_run_config["carla"]["map"]`.
- Spawn ego from `run_config["ego"]["initial_state"]`.
- Derive destination from the last ego reference trajectory point, or from an explicit route endpoint when added later.
- Tick CARLA synchronously.
- Let `BasicAgent` compute control every tick.
- Apply vehicle control to ego.
- Collect primitive metrics per tick.
- Emit the same `closed_loop_report.mvp.v0` shape as dry-run mode, with `status="ego_closed_loop"` or `status="failed"`.

Current implementation status:

- `run_basic_agent` contains a real control-loop skeleton.
- The CARLA Python API and BasicAgent class are imported lazily or injected by tests.
- Unit tests use fake CARLA and fake BasicAgent objects to verify the call sequence:
  connect/load world, sync settings, spawn ego, set destination, `world.tick`,
  `agent.run_step`, `vehicle.apply_control`, metric collection, settings restore,
  and ego destroy.
- Real CARLA 0.9.16 execution is intentionally marked as environment integration
  work. The skeleton is ready for debug once CARLA, ScenarioRunner, and the Python
  API are available on the target machine.

This is closed-loop for the ego because control is recomputed from simulator state each tick. If actors are replayed, the run is only half-closed-loop. Interactive closed-loop starts when actors use TrafficManager or reference-conditioned scripted controllers that can react to ego.

## ROS2 and TCP Boundary

TCP has a public implementation and should be treated as a ROS2/external ego stack candidate, not as the first built-in runner. In ClosedLoopBench it belongs behind the existing `ros2_external_agent` boundary:

```text
ClosedLoopBench scenario/runtime
  -> CARLA / ROS2 bridge
  -> external TCP stack
  -> vehicle control command
  -> CARLA ego vehicle
```

The first CARLA runtime milestone should use BasicAgent because it proves spawning, tick control, and metric reporting with minimal dependencies. TCP, TransFuser, InterFuser, and UniAD can then reuse the same runtime contract.

## Test Strategy

Tests should not require a real simulator.

Required test types:

- Import-guard tests: missing `carla` or missing ScenarioRunner should not fail import of ClosedLoopBench modules.
- Command-construction tests: ScenarioRunner command must be a list with `--openscenario`, host, port, and output arguments.
- Plan-construction tests: BasicAgent plan must contain map, ego spawn state, destination, tick policy, and report path.
- Mock probe tests: injected fake CARLA module should prove client/world/version access without launching CARLA.

Real CARLA smoke tests should be opt-in and skipped unless explicit environment variables are set.
