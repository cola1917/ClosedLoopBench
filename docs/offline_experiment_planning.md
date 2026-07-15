# Offline Experiment Planning

The experiment planner freezes every comparison dimension before CARLA is
available. It creates one `evaluation_run_request.v1` for each Cartesian product
of scene, algorithm, ODD and seed. Algorithm IDs, algorithm versions, scene
versions and seeds are part of the comparison identity.

## Plan

Update the Scene Package digest and algorithm version in
`examples/core_experiment_matrix.v0.example.json`, then run:

```powershell
python runners/plan_experiment_matrix.py `
  --matrix examples/core_experiment_matrix.v0.example.json `
  --created-at 2026-07-14T00:00:00Z `
  --output outputs/core_experiment_plan.json
```

The sample produces 12 deterministic runs:

```text
1 scene x 2 algorithms x 2 ODDs x 3 seeds = 12 runs
```

Each request carries the immutable Scene Package digest, exact algorithm
version, actor control mode, CARLA version, synchronous delta, timeout and output
prefix. Changing any dimension requires a new plan.

## Host Handoff

An individual request can be handed directly to the native-host runner:

```powershell
python runners/run_host_closed_loop.py `
  --exchange-root E:/sim-data `
  --evaluation-request evaluation_request.json
```

The adapter rejects a non-canonical Scene Package path, digest mismatch,
unsupported driver or protocol error before probing CARLA.

## Result Handoff

Convert a finalized report into `evaluation.run.result`:

```powershell
python runners/build_evaluation_result.py `
  --request evaluation_request.json `
  --report E:/sim-data/runs/<run-id>/closed_loop_report.json `
  --exchange-root E:/sim-data `
  --started-at 2026-07-14T00:00:00Z `
  --finished-at 2026-07-14T00:01:00Z `
  --producer-version <commit> `
  --output evaluation_result.json
```

The result adapter refuses mismatched run, scene, algorithm, version, ODD or
seed identities. Unknown KPI values remain null.

## Coverage Gate

```powershell
python runners/plan_experiment_matrix.py `
  --plan outputs/core_experiment_plan.json `
  --reports E:/sim-data/runs/*/closed_loop_report.json `
  --output outputs/core_experiment_coverage.json
```

Comparison is allowed only when every expected run appears exactly once, all
reports are successful, requested interactive Actor modes contain
`interactive_closed_loop` evidence, and all core KPI summaries are known.
