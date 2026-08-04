# Runner Surface

The repository currently contains 75 top-level compatibility scripts under
`runners/`. They are not 75 independent product workflows. Most are builders,
diagnostics, renderers, probes, or historical integration helpers.

Use the canonical boundary for new work:

```bash
python3 -m runners inventory
python3 -m runners offline-acceptance --output outputs/offline_acceptance.json
python3 -m runners build-scene --help
python3 -m runners run-basic-agent --help
```

The canonical commands are the exchange build, CARLA config build, exchange
validation, offline acceptance, BasicAgent runtime, triplicate acceptance,
host orchestration, CARLA probe, evaluation result, and report comparison paths.
The original files remain import-compatible while callers migrate.

New development rules:

- Do not add another top-level runner for a new option or experiment.
- Put reusable behavior in `actors/`, `adapters/`, `agents/`, `metrics/`, or
  `runtime/`.
- Add a canonical command only when it represents a stable user workflow.
- Keep one-off probes, renderers, and historical S1/M8 experiments out of the
  canonical list and out of the next product milestone.

The inventory and classification are tested by
`tests/test_runner_registry.py`.
