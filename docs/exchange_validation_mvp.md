# Exchange Validation MVP

ClosedLoopBench should produce exchange artifacts that can be validated without CARLA.

## Scope

This batch adds:

- minimal OpenDRIVE export for static road-network placeholders
- esmini discovery as an optional smoke-test dependency
- artifact validation helpers for XML parseability and required root nodes

## Interview Story

The project uses three validation levels:

1. Static contract tests: no external tools, always run in CI.
2. Optional esmini smoke tests: run only when `ESMINI_BIN` or `esmini` is available.
3. Full CARLA/ScenarioRunner runtime: run only in the simulation workstation environment.

This keeps development fast while preserving a direct handoff path to real runtime validation.
