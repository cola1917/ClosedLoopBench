# scene-0061 TransFuser++ remote handoff

## Local completion boundary

The repository now contains the real CARLA Garage `leaderboard_2` TransFuser
v5/TransFuser++ adapter boundary, ROS2 sidecar, immutable runtime identity,
S0/S2/S4 corridor-constrained variants, host/container payload remapping,
intermediate recording/evaluation/visualization, and automatic formal
acceptance gates. Local tests do not claim that a checkpoint was loaded or that
CARLA/NuRec closed loop passed.

The remaining work is remote-only: install/bind the exact upstream repository,
checkpoint/config and hash-pinned `agents/navigation` source, prove live LiDAR
coordinates, build the GPU image, then run and debug real inference.

## Immutable inputs

- NuRec artifact SHA-256:
  `69e2c36e31e113f9ad66968a0a0a4243c7989dae5582a91a43d150051eaf98b4`
- scene ID: `cc8c0bf57f984915a77078b10eb33198`
- vehicle track: `c1958768d48640948f6053d04cffd35b`
- pedestrian track: `71603dd1a2ba4e9daf095535e38310ac`
- canonical OpenDRIVE: `outputs/scene0061_exchange_v2/road.xodr`
  (229 roads, 17 junctions, no diagnostic Ego corridor; SHA-256
  `eb117dd99f84cdd8072e13aaacc502702dd815658ed4b53e81a00ace931b109e`)
- matrix: `configs/scene0061_counterfactual_matrix.v1.json`
- focused remote plan: `configs/scene0061_transfuserpp_remote_plan.v1.json`
- runtime template:
  `configs/scene0061_transfuserpp_runtime.remote.template.json`

## Required remote bindings

Run from the ClosedLoopBench repository root with the Miniconda `autodrive`
environment for every host-side Python command.

1. Bind one exact `carla_garage` `leaderboard_2` Git checkout and its compatible
   single checkpoint/config. Fill revision and file hashes; do not call a
   mutable branch name a runtime identity. Preflight also requires the checkout
   origin to normalize to the official autonomousvision repository, resolves
   `refs/remotes/origin/leaderboard_2`, proves the pinned revision is in that
   ref's history, and requires a clean worktree including untracked files.
2. Mount only the CARLA `agents/navigation` source directory. The sidecar
   image already contains `carla==0.9.15`; the host CARLA/ROS runtime remains
   `0.9.16`. Fill `carla_agents_sha256`; the sidecar will explicitly resolve
   and verify `agents.navigation.global_route_planner` from this mount.
3. Attach a real LiDAR coordinate-validation JSON path and SHA-256 to the formal
   base run config. Bundle creation verifies the declared convention, and the
   acceptance runner re-reads and re-hashes the file before CARLA starts.
4. Keep `carla.fixed_delta_seconds=0.05`. The prepared host and sidecar topics
   must exactly match; a topic mismatch fails preflight.
5. Build the sidecar to a local tag, read its immutable Docker image ID
   (`docker image inspect --format '{{.Id}}' ...`), and use that exact
   `sha256:...` value both as `TFPP_IMAGE_DIGEST` and in the bound runtime
   JSON. The formal compose file selects that image ID directly; a mutable tag
   is never the executed image identity.

To calculate candidate hashes on the host, make a disposable host-view copy of
the runtime template and replace every `/opt/...` and `/sim-data/...` path with
the corresponding host path. This host-view manifest is diagnostic only; its
path-dependent runtime-config identity is not the formal container identity:

```bash
conda run -n autodrive python -m runners.build_transfuserpp_runtime_manifest \
  --config /path/to/host-view.transfuserpp.runtime.json \
  --output /new/evidence/host_view.runtime_manifest.json \
  --print-identities
```

## Bundle matrix

Prepare a fresh, non-existing output directory for every case/seed pair:

- `S0_original_replay`: seeds 41, 42, 43; no event timestamp.
- `S2_lead_hard_brake`: seeds 41, 42, 43; use the verified vehicle source
  event timestamp.
- `S4_pedestrian_early_crossing`: seeds 41, 42, 43; use the verified pedestrian
  source crossing event timestamp. The frozen edit begins two seconds before
  that source event, preserves every earlier sample, and advances crossing by
  one second along the source corridor.

```bash
conda run -n autodrive python -m runners.prepare_scene0061_transfuserpp_remote_run \
  --base-run-config /path/to/formal.base.json \
  --runtime-template /path/to/bound.transfuserpp.runtime.json \
  --matrix configs/scene0061_counterfactual_matrix.v1.json \
  --case-id S2_lead_hard_brake --seed 41 \
  --event-timestamp-sec EVENT_SEC \
  --output-dir /new/evidence/S2_lead_hard_brake/seed_41
```

Each bundle contains `carla_run_config.json`,
`runtime/transfuserpp.runtime.json`, and `remote_run_bundle.json`. Set that
bundle directory as `SIM_DATA_HOST_PATH`; do not reuse it for another case or
seed.

The generated CARLA config is deliberately `algorithm_gpu_validation=pending`.
After the image and prepared manifest steps below, run the real CUDA probe
inside that image using one valid recorded observation, then bind the report
on the host into a new config. Both commands use strict JSON parsing and refuse
to overwrite. The binder
checks report hash, runtime identity, full experiment identity, warmup count,
measured samples, peak VRAM and P95/P99 thresholds. The CUDA evidence reference
is the sole field excluded from the formal config hash, avoiding an impossible
report-hash/config-hash self-reference. A pending or manually mismatched report
always fails acceptance.

There are nine immutable bundles (3 cases x 3 seeds). Run the triplicate
acceptance command below once per bundle, producing three consecutive physical
attempts per bundle and 27 attempts in total. Do not count a bundle as three
seeds or a single attempt as a triplicate.

## Sidecar preflight and formal execution

Fill a new env file from `docker/transfuserpp.env.example`, including the
read-only repository/checkpoint/agents-source paths and the exact prepared ROS
topics. Build the image separately, bind its immutable image ID, then preflight
before CARLA execution:

```bash
docker build -f docker/transfuserpp/Dockerfile \
  -t closed-loop-bench/transfuserpp-v5:leaderboard2 .
export TFPP_IMAGE_DIGEST="$(docker image inspect \
  --format '{{.Id}}' closed-loop-bench/transfuserpp-v5:leaderboard2)"
export TFPP_COMPOSE_PROJECT="scene0061-tfpp-S2-seed41"
# Put the same value in the bound runtime JSON and env file before continuing.
docker compose -p "$TFPP_COMPOSE_PROJECT" --env-file /path/to/transfuserpp.env \
  -f docker/compose.transfuserpp.yml run --rm --entrypoint python3 transfuserpp \
  -m runners.build_transfuserpp_runtime_manifest \
  --config /sim-data/runtime/transfuserpp.runtime.json \
  --output /sim-data/runtime/transfuserpp.runtime_manifest.json \
  --require-prepared
docker compose -p "$TFPP_COMPOSE_PROJECT" --env-file /path/to/transfuserpp.env \
  -f docker/compose.transfuserpp.yml run --rm transfuserpp preflight
docker compose -p "$TFPP_COMPOSE_PROJECT" --env-file /path/to/transfuserpp.env \
  -f docker/compose.transfuserpp.yml run --rm --entrypoint python3 transfuserpp \
  -m runners.run_transfuserpp_cuda_preflight \
  --config /sim-data/runtime/transfuserpp.runtime.json \
  --observation /sim-data/runtime/formal_warmup_observation.json \
  --output /sim-data/runtime/transfuserpp.cuda-preflight.json
conda run -n autodrive python -m runners.bind_transfuserpp_cuda_preflight \
  --run-config BUNDLE/carla_run_config.json \
  --cuda-evidence BUNDLE/runtime/transfuserpp.cuda-preflight.json \
  --output BUNDLE/carla_run_config.cuda-bound.json
docker compose -p "$TFPP_COMPOSE_PROJECT" --env-file /path/to/transfuserpp.env \
  -f docker/compose.transfuserpp.yml up -d transfuserpp
```

The first container command re-hashes the actual read-only container mounts and
writes the formal prepared manifest. The second must load the real checkpoint,
import the pinned upstream model and CARLA navigation source, and expose
CUDA/PyTorch/device identity. The third performs measured inference in the
same immutable image; the fourth only binds its verified report using the host
`autodrive` Python. Start CARLA
only with `/home/cwadmin/workspace/env_build/start_carla.sh`, then the formal
NRE and sidecar.

Use a unique compose project name for every case/seed bundle so no historical
container is replaced. Preserve `docker compose logs --no-color transfuserpp`
with the bundle evidence; do not prune images, volumes, or prior containers.

Use the existing real NuRec handler and the prepared bundle root:

```bash
conda run -n autodrive python -m runners.run_carla_acceptance_triplicate \
  --run-config BUNDLE/carla_run_config.cuda-bound.json \
  --output-root BUNDLE \
  --ego-driver ros2_observation_control \
  --sensor-handler-factory adapters.nurec_260_client:build_nurec_260_handler \
  --require-multimodal
```

For TransFuser++, triplicate acceptance automatically fails if any of these is
present: backend exception trace, non-initialization fallback, mismatched
control, rejected sensor frame, missing model control, invalid dense output,
frame/identity gap, or LiDAR coordinate evidence mismatch. The first declared
initialization safe stop is reported separately and is not hidden.

A passed triplicate is deliberately classified `control_only`; it proves real
perception-in-loop execution and multimodal transport but does not by itself
grant perception-ranking eligibility. That promotion requires a separately
SHA-bound strict six-camera render-quality report.

Then run the existing strict multimodal validator on every result and aggregate
only identical artifact/scene/checkpoint/config identities.

## Intermediate evaluation

Single-run evaluation and visualization run on the host. Container paths are
resolved through the bundle root and every file hash is checked:

```bash
conda run -n autodrive python -m runners.evaluate_transfuserpp_intermediates \
  --trace BUNDLE/transfuserpp_intermediates/CASE/seed_SEED/RUN_ID \
  --evidence-root BUNDLE \
  --render-quality-report /path/to/render_quality_report.json \
  --output /new/evidence/intermediate_evaluation.json

conda run -n autodrive python -m runners.render_transfuserpp_intermediate \
  --record /path/to/frame.intermediate.json \
  --evidence-root BUNDLE \
  --output /new/evidence/frame.panel.png
```

Counterfactual comparison accepts separate baseline and edited bundle roots.
It fails when pre-event evidence is not equivalent and skips post-event BEV
pairs after ego poses diverge instead of treating viewpoint change as an edit:

```bash
conda run -n autodrive python -m runners.evaluate_transfuserpp_intermediates \
  --trace S0_BUNDLE/transfuserpp_intermediates/S0_original_replay/seed_41/RUN_ID \
  --evidence-root S0_BUNDLE \
  --edited-trace S2_BUNDLE/transfuserpp_intermediates/S2_lead_hard_brake/seed_41/RUN_ID \
  --edited-evidence-root S2_BUNDLE \
  --event-timestamp EVENT_SEC --expected-case-id S2_lead_hard_brake \
  --output /new/evidence/S0_vs_S2.intermediate.json
```

`perception_eligible` additionally requires a SHA-bound valid
`render_quality_report.v1` with all six formal cameras eligible. The report's
RGB/LiDAR consistency must be backed by a SHA/size-bound
`rgb_lidar_actor_change_source_report.v1` that binds the same experiment,
target track, paired frame range and actual baseline/edited payload hashes;
hand-written booleans or path strings fail closed. Dynamic BEV actor proxies
are evaluated; full 3D occupancy remains explicitly unavailable.

## Fail-closed handback

If any gate fails, preserve the bundle, sidecar log, backend failure trace,
runtime manifest, intermediate records, render-quality report and validator
output. Report `remote_validation_required` or failed; never replace missing
model/LiDAR/CARLA evidence with offline or fake outputs.
