# Closed-loop metrics and evaluation

`build_closed_loop_report` emits both metric summaries and pass/fail evaluation:

- `summary` contains aggregate metrics such as `collision_count`, `route_progress`, and `min_ttc`.
- `evaluation.overall_result` is one of `pass`, `fail`, or `unknown`.
- `evaluation.criteria` contains one result per criterion with `name`, `metric`, `op`, `expected`, `actual`, and `result`.

Default criteria:

| Metric | Operator | Expected | Missing data behavior |
| --- | --- | --- | --- |
| `collision_count` | `==` | `0` | Count defaults to `0`, so no collision samples pass. |
| `route_progress` | `>=` | `0.95` | Unknown until a route-progress sample exists. |
| `min_ttc` | `>=` | `1.0` | Unknown until TTC is sampled. |

Overall result rules:

- `fail` if any criterion fails.
- `pass` if every criterion passes.
- `unknown` if there are no failures but at least one criterion is unknown.

Run configs can override criteria under `evaluation.criteria`:

```json
{
  "evaluation": {
    "criteria": [
      {"metric": "collision_count", "op": "==", "value": 0},
      {"metric": "route_progress", "op": ">=", "value": 0.95},
      {"metric": "min_ttc", "op": ">=", "value": 1.0}
    ]
  }
}
```

For compatibility with metrics-oriented configs, criteria can also be supplied as
`metrics.criteria` or `metrics.thresholds` when `metrics` is an object.
