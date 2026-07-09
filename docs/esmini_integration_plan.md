# esmini Integration Plan

esmini is treated as an optional external simulator/checker, not as a Python package dependency.

## Installation Strategy

ClosedLoopBench should discover esmini in this order:

1. `ESMINI_BIN` environment variable pointing to the executable.
2. `tools/esmini/bin/esmini.exe` inside this project, if a local binary bundle is provided.
3. `esmini` available on `PATH`.

The Python test suite must not require esmini. Tests that execute esmini should skip when the binary is not available.

## Phase 1: Static Format Contract

Goal: produce valid, stable exchange artifacts without any simulator installed.

Deliverables:

- `Scenario IR -> OpenSCENARIO .xosc` MVP adapter
- XML parse tests
- required-node contract tests
- golden scenario snapshots for one nuScenes mini seed
- optional `Scenario IR -> minimal OpenDRIVE .xodr` placeholder road

Validation:

- Python standard-library XML parsing
- deterministic golden output
- no CARLA, NuRec, Cosmos, or esmini required

## Phase 2: Optional esmini Smoke Test

Goal: when esmini is available, validate that exported `.xosc` can be loaded by a lightweight player.

Deliverables:

- `tools/find_esmini.py` or equivalent resolver
- `tests/test_esmini_smoke.py` with `skipUnless(esmini_available)`
- generated `outputs/<scenario>/scenario.xosc`
- generated `outputs/<scenario>/road.xodr` only if needed by the `.xosc`

Validation:

- run esmini in load/smoke mode
- assert process exits successfully
- capture stdout/stderr under `outputs/<scenario>/esmini_smoke.log`

## Phase 3: Runtime Adapter Handoff

Goal: use the same Scenario IR to support full runtime once CARLA/ScenarioRunner is available.

Deliverables:

- `Scenario IR -> ScenarioRunner Python config` as primary path
- `Scenario IR -> OpenSCENARIO .xosc` as exchange path
- optional `Scenario IR -> Scenic` for scenario-family generation
- optional `Scenario IR -> OpenDRIVE .xodr` for local map enhancement

Validation:

- ClosedLoopBench can run without OpenSCENARIO by using ScenarioRunner config
- OpenSCENARIO export remains a portable compatibility artifact
- OpenDRIVE stays scoped to simplified/map-enhancement use until a real map-conversion pipeline exists

## Practical Note

Do not block MVP on OpenDRIVE. CARLA existing towns can host semantic-equivalent closed-loop scenes. OpenDRIVE becomes important when the project needs map-accurate local roads rather than task-family evaluation.
