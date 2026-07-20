# Ego algorithm plugin contract

This contract prepares algorithms for remote CARLA/NuRec validation without
claiming that an offline replay is a closed loop. Public reports keep runtime
outcome (`execution_status`) separate from evidence meaning
(`evidence_classification`). Offline plugin outputs always use
`evidence_classification: offline_conformance`.

## Lifecycle and capability

A plugin loaded as `module:factory` implements:

```python
initialize(config)
reset(scene_context)
predict_control(observation)
health_check()
close()
```

Its `capability` object declares route/ego-state use, required RGB camera
names, LiDAR, perception/GPU requirements, checkpoint identity, supported
control rate, and timeout. `agents.plugin_contract.AlgorithmPluginExecutor`
enforces lifecycle order and validates every observation and control.

The observation boundary carries a monotonically increasing integer
`frame_id`, timestamp, RGB references, optional LiDAR reference, calibration,
ego state, route, sensor validity, and synchronization identity. Controls
carry bounded throttle/steer/brake, booleans for hand brake/reverse, the source
frame, measured inference latency, status, and reason.

Contract failures produce brake-safe control, never a fabricated command:

- stale source frames and frame mismatch;
- missing capability-required sensors;
- unhealthy or throwing backends;
- measured timeout;
- NaN/Inf, missing, or out-of-range controls;
- predict before reset or after close.

The in-process timeout guard measures a synchronous call and converts an
over-budget result to safe stop after that call returns. Remote production
binding must additionally enforce a process/RPC deadline; offline conformance
does not claim hard real-time preemption.

The identity ledger records configuration, repository, and checkpoint hashes.
The geometry-only Pure Pursuit baseline explicitly records checkpoint identity
as `not_applicable` and `is_perception_algorithm: false`.

## Reference Pure Pursuit

`agents.reference_pure_pursuit:create_plugin` is compatible with the existing
`agents.algorithm_backend.load_backend`. It supports `short` and `long`
lookahead profiles, target-speed proportional throttle/brake control,
monotonic route progress, steering saturation, deterministic reset, and
per-frame diagnostics. It proves the replaceable plugin/control boundary only;
offline replay does not prove CARLA route completion.

From the repository root:

```powershell
python -m runners.replay_algorithm_plugin `
  --plugin agents.reference_pure_pursuit:create_plugin `
  --config examples/algorithm_plugins/reference_pure_pursuit.short.json `
  --observations examples/algorithm_plugins/synthetic_route_observations.jsonl `
  --control-trace output/control_trace.jsonl `
  --report output/offline_replay_report.json `
  --verify-determinism
```

No wall clock is used by default. `--wall-clock` replays timestamp gaps.
`--simulate-timeout-frame` and `--simulate-exception-frame` exercise offline
safe-stop paths without misrepresenting an actual model failure.

## Conformance

```powershell
python -m runners.run_algorithm_plugin_conformance `
  --plugin agents.reference_pure_pursuit:create_plugin `
  --config examples/algorithm_plugins/reference_pure_pursuit.long.json `
  --output algorithm_plugin_conformance.json
```

The machine report covers normal control, missing camera/LiDAR, stale and
mismatched frames, timeout, exception, invalid range, NaN/Inf, health failure,
reset/close, absent checkpoint, capability/input mismatch, and safe stop. Only
a fully passing plugin is marked `remote_validation_queue_eligible`.

## TCP and TransFuser boundary

`agents.model_plugin_wrappers` defines stable factories, configuration schema,
preprocessing boundary, readiness, hashes, and control normalization for TCP
and TransFuser. The repository owns tensor conversion and route-command
encoding; this project does not vendor model code or download checkpoints.

Templates under `examples/algorithm_plugins` intentionally contain placeholder
remote paths and hashes. Their manifests remain fail-closed with:

```json
{
  "real_checkpoint_loaded": false,
  "remote_gpu_validation_required": true,
  "evidence_classification": "remote_validation_required"
}
```

An explicit `allow_test_backend` plus recorded controls exists only for unit
and recorded-output tests. It cannot be interpreted as real TCP/TransFuser
inference or perception-eligible evidence.

Generate the fail-closed binding manifests before remote installation:

```powershell
python -m runners.build_external_model_plugin_manifest `
  --algorithm tcp `
  --config examples/algorithm_plugins/tcp.remote-binding.template.json `
  --output tcp.runtime_manifest.json
```

## TransFuser++ v5 formal boundary

`agents.transfuserpp_plugin:TransFuserPPPlugin` is the pinned CARLA Garage
Leaderboard 2.0 adapter, not the older generic placeholder wrapper. It consumes
one NuRec `camera_front`, live `lidar_top`, ego speed/pose, and route
target/command while preserving the six-camera formal gate. Each control is
bound to the same frame, dynamic-object digest, case/seed/run identity,
repository revision, checkpoint/config hashes, and materialized input hashes.
The matching CARLA PythonAPI navigation package is also hash-bound, and every
run compares the full artifact/package/scenario/matrix/source/variant identity
between host and sidecar.

The adapter exposes perspective/BEV semantics, depth, boxes, waypoints,
checkpoints, target-speed distribution, control, and latency. Dynamic BEV actor
proxies are explicitly not full 3D occupancy. See
`docs/transfuserpp_scene0061_integration.md` for remote binding and acceptance.
