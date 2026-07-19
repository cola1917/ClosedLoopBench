# scene-0061 local evaluation and plugin handoff

This handoff is the local-development boundary for the scene-0061
counterfactual evaluation system. It does not start CARLA or NuRec and does
not claim a real closed-loop pass. Persistent generated evidence belongs under
`E:/code/scene0061-evidence/local_development`; test scratch data belongs in a
temporary directory.

## Frozen identities and experiment scope

The checked-in matrix freezes:

- scene ID `cc8c0bf57f984915a77078b10eb33198`;
- NuRec run `9aChcizbAsm4oDQKJMdBHM` and artifact SHA-256
  `69e2c36e31e113f9ad66968a0a0a4243c7989dae5582a91a43d150051eaf98b4`;
- runtime-validated Scene Package SHA-256
  `0d6b724b0dea9ff3f97717f893f19baf69904057511ad374cfd510c5cc9b9119`;
- actor-ready Scenario IR SHA-256
  `ae340b43c2ecbcf416cb89895e63ea59241b240ff83bc9dc4e6f1632a3f1ded7`;
- OpenDRIVE SHA-256
  `d3913c4d0019d4c9165ae90e2a5025703ed5e1b423d688168951428341892537`;
- vehicle track `c1958768d48640948f6053d04cffd35b` and pedestrian track
  `71603dd1a2ba4e9daf095535e38310ac`.

S0-S7 cover original replay, lead slowdown, hard brake, bounded longitudinal
shift, pedestrian early crossing/yield/noncompliance, and vehicle removal.
Pedestrian edits stay on the source reference corridor and only adjust speed,
pause, yield, or abort. Vehicle removal is quality stress and can never enter
the perception ranking by default.

Build and validate a new immutable matrix output:

```powershell
python -m runners.build_scene0061_counterfactual_matrix `
  --output <new-matrix.json>
python -m runners.build_scene0061_counterfactual_matrix `
  --validate <new-matrix.json> `
  --validation-output <new-validation.json>
```

## Data flow and evidence classes

```text
matrix + scene/artifact identity
  -> algorithm plugin + synchronized observation
  -> guarded control trace + runtime metric sources
  -> closed-loop report + render-quality report
  -> strict counterfactual suite aggregation
  -> control-only / perception-eligible / quality-stress result
```

The plugin guard validates lifecycle, capabilities, sensor availability,
frame identity, health, timeout, finite/ranged controls, and safe-stop. The
Pure Pursuit implementation is a real deterministic geometry controller but
is not a perception algorithm. TCP and TransFuser wrappers freeze the loading,
preprocessing, route-command, output normalization, readiness, identity, and
fallback boundaries; they do not ship checkpoints or fake model inference.

Reports use these meanings without promotion between them:

- `offline_conformance`: local interface, determinism, and safety checks only;
- `control_only`: valid control/state comparison without proven perception
  image eligibility;
- `perception_eligible`: a real run whose required image, mask, identity,
  synchronization, and RGB/LiDAR gates all passed;
- `quality_stress`: retained rendering limitation, never mixed into formal
  perception ranking;
- `remote_validation_required`: installation, GPU, CARLA, NuRec, or live
  checkpoint evidence remains outstanding.

## Local commands

Pure Pursuit conformance and deterministic replay:

```powershell
python -m runners.run_algorithm_plugin_conformance `
  --plugin agents.reference_pure_pursuit:create_plugin `
  --config examples/algorithm_plugins/reference_pure_pursuit.long.json `
  --output <new-conformance.json>

python -m runners.replay_algorithm_plugin `
  --plugin agents.reference_pure_pursuit:create_plugin `
  --config examples/algorithm_plugins/reference_pure_pursuit.short.json `
  --observations examples/algorithm_plugins/synthetic_route_observations.jsonl `
  --control-trace <new-control-trace.jsonl> `
  --report <new-replay-report.json> `
  --verify-determinism
```

Fail-closed suite readiness with no runtime reports:

```powershell
python -m runners.evaluate_counterfactual_suite `
  --matrix configs/scene0061_counterfactual_matrix.v1.json `
  --output <new-suite-readiness.json>
```

Render quality and video collection planning are documented in
`docs/scene0061_render_quality_and_video.md`. Plugin and external-model binding
details are documented in `docs/algorithm_plugin_contract.md`. Every new CLI
refuses to overwrite an existing output; use a run-specific output name.

## Fail-closed KPI and quality rules

The report records progress/time/speed/stopped/following, distance/TTC/PET/DRAC,
collision and hard-brake events, jerk percentiles, control latency and
timeout/fallback rates, actor outcomes, dropped frames, and synchronization
error. Hard braking and collision are counted by continuous events. Jerk
excludes invalid intervals and endpoint transients. Missing collision, TTC, or
required actor-outcome sources remain unavailable and fail their gates.

Aggregation rejects missing or duplicate seeds, offline/fake runs, identity
mismatch, incomplete triplicates, absent KPI sources, and incompatible
artifact/scene/algorithm/checkpoint hashes. Baseline deltas and three-seed
mean/std/failure rate are computed only within a compatible identity group.

The image gate reports per-camera and aggregate dark/invalid pixels, actor ROI
holes, boundary discontinuity, flicker, edited-region change, unchanged
background stability, sharpness, PSNR/SSIM, and RGB/LiDAR change consistency.
Mask-dependent metrics are unavailable unless mask provenance is reliable.
Harmonizer is an optional A/B input and cannot upgrade failed source evidence.

## Minimum remote validation sequence

1. Use the `autodrive` environment from the ClosedLoopBench repository root.
   Bind the immutable matrix identities to the installed artifact, actor-ready
   Scenario IR, runtime-validated package, OpenDRIVE, actor mapping, and plugin
   repo/config/checkpoint hashes. Do not modify the formal 40k artifact.
2. Install and health-check the actual selected algorithm backend. Pure Pursuit
   needs no checkpoint; TCP/TransFuser remain ineligible until a real repository
   and checkpoint are loaded and hashed.
3. Start the existing CARLA script and the formal NuRec service, then verify six
   RGB cameras and a non-empty actor-changing LiDAR response. Capability-only
   success is insufficient.
4. Run each required matrix case for seeds 41, 42, and 43. Capture synchronized
   runtime reports, six raw camera streams, LiDAR, actor mapping, quality masks
   where available, dual-window screenshots/video, frame/timestamp/FPS/drop/sync
   statistics, and all logs/hashes into new output directories.
5. For each formal attempt, run:

```bash
python -m runners.run_carla_acceptance_triplicate \
  --run-config /path/to/run_config.json \
  --output-root /new/path/to/acceptance \
  --opendrive /path/to/road.xodr \
  --sensor-handler-factory adapters.nurec_260_client:build_nurec_260_handler \
  --require-multimodal

python -m runners.validate_multimodal_closed_loop \
  --runtime-result /path/to/runtime_result.json \
  --output /new/path/to/multimodal_validation.json
```

6. Evaluate render quality, then aggregate the exact triplicate reports. Any
   live LiDAR, actor mapping, synchronization, identity, KPI-source, quality,
   or checkpoint failure stays fail-closed and is reported with source paths.

The final video manifest is a capture contract, not generated remote footage.
It requires original and edited comparisons, CARLA bbox state, six-camera
grid, LiDAR inset, KPI and synchronization overlays, black-hole limitations,
algorithm identity, and baseline-versus-edit evidence.
