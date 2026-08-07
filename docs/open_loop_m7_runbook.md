# Open-loop M7 formal runbook

This runbook freezes scene-0061 `S0_original_replay` for seeds `41`, `43`, and
`47`. It produces `open_loop_multimodal_m7_triplicate_report.v1`; it does not
authorize M8 or M9 claims.

## Pinned environment

- image: `closed-loop-bench/transfuserpp-v5:m6-stage-d`
- image digest: `sha256:d5f814b8aab88bbef08e70a4f915771658221190d2501e2894815977d3db6394`
- CARLA Garage revision: `72f39a63423a5edef6904b1487e0360a64bcf445`
- checkpoint SHA-256: `d6fbdc28f7398354beadc7cf6765d866457c957f7b470c88ba206e73311a3b44`
- model config SHA-256: `895e3e9704ceda443169ca32aaef2712b1becf2d42473d7273071ec6ceda113e`
- NuRec API: `SensorsimService/26.04`, target `127.0.0.1:46443`
- host evidence root mounted as `/sim-data`:
  `outputs/scene0061-transfuserpp`

Run host NuRec commands with the `autodrive` interpreter. The ROS workspace
currently puts the system protobuf package before the Conda package, so the
NuRec protobuf site-packages directory must be prepended:

```bash
export CLB_ROOT=/home/cwadmin/workspace/ClosedLoopBench
export CLB_PY=/home/cwadmin/sim-env/miniconda3/envs/autodrive/bin/python
export CLB_NUREC_PYTHONPATH=/home/cwadmin/sim-env/miniconda3/envs/autodrive/lib/python3.10/site-packages:/home/cwadmin/sim-env/data/CARLA_0.9.16/PythonAPI/examples/nvidia/nurec:$CLB_ROOT
export PYTHONPATH=$CLB_NUREC_PYTHONPATH
```

The NuRec service and its static scene must already be healthy. Formal Stage B
creates no CARLA dynamic actors and never applies TF++ control to the next ego
pose.

## Per-seed execution

1. Build an independent binding with the pinned repo, checkpoint, config,
   sensor bundle, USDZ, image digest, case `S0_original_replay`, and seed.
   Use `runners/build_open_loop_transfuserpp_stage_b_binding.py`; write the
   binding to `runtime/transfuserpp.m7.seed-${SEED}.runtime.json` and the copied
   input bundle to `m7-inputs-seed-${SEED}`. The builder must report the same
   `input_set_sha256` for all three seeds.
2. Capture all 39 NuRec frames into a new directory. Never reuse a failed or
   partial directory:

```bash
$CLB_PY runners/capture_open_loop_transfuserpp_stage_b.py \
  --scenario-ir $CLB_ROOT/outputs/scene0061-transfuserpp/inputs/scene_ir.json \
  --opendrive $CLB_ROOT/outputs/scene0061-transfuserpp/inputs/road.xodr \
  --runtime-config $CLB_ROOT/outputs/scene0061-transfuserpp/runtime/transfuserpp.m7.seed-${SEED}.runtime.json \
  --output-dir $CLB_ROOT/outputs/scene0061-transfuserpp/m7-seed-${SEED}-capture \
  --max-frames 39 --nurec-concurrency 7 --nurec-max-attempts 2 \
  --container-root /sim-data
```

   Inspect the trace before inference. Every payload path must be
   `/sim-data/m7-seed-${SEED}/payloads/...`; the directory must contain
   `39 * 8 = 312` sensor payload files.
3. Run real TF++ in the immutable image. Use module execution so the image
   package root is on `sys.path`, and use the retained HF cache offline:

```bash
docker run --rm --gpus all --network host --entrypoint python3 \
  -e HF_HOME=/sim-data/hf_home -e HF_HUB_OFFLINE=1 -e TRANSFORMERS_OFFLINE=1 \
  -v $CLB_ROOT/outputs/scene0061-transfuserpp:/sim-data:rw \
  -v /home/cwadmin/workspace/external/carla_garage:/opt/algorithm/carla_garage:ro \
  -v /home/cwadmin/workspace/external/carla_garage_checkpoints:/opt/algorithm/checkpoints:ro \
  -v /home/cwadmin/sim-env/data/CARLA_0.9.16/PythonAPI/carla/agents:/opt/carla-pythonapi/agents:ro \
  closed-loop-bench/transfuserpp-v5:m6-stage-d \
  -m runners.run_open_loop_transfuserpp_stage_b \
  --scenario-ir /sim-data/inputs/scene_ir.json \
  --opendrive /sim-data/inputs/road.xodr \
  --runtime-config /sim-data/runtime/transfuserpp.m7.seed-${SEED}.runtime.json \
  --observations /sim-data/m7-seed-${SEED}-capture/nurec_stage_b_observations.json \
  --run-id scene0061-open-loop-m7-seed-${SEED} \
  --report /sim-data/runtime/m7_seed_${SEED}_report.json
```

   Require `execution_status=completed`, `real_tfpp_checkpoint_loaded=true`,
   `39/39` matched frames, `fallback_count=0`, and 39 intermediate plus 39
   dense files before continuing.
4. Evaluate intermediates on the host. Pass the scene output directory as the
   evidence root so both `/sim-data/m7-seed-${SEED}/payloads` and
   `/sim-data/transfuserpp_intermediates/...` resolve to host files:

```bash
python -m runners.evaluate_transfuserpp_intermediates \
  --trace $CLB_ROOT/outputs/scene0061-transfuserpp/transfuserpp_intermediates/S0_original_replay/seed_${SEED}/<run-id> \
  --evidence-root $CLB_ROOT/outputs/scene0061-transfuserpp \
  --output $CLB_ROOT/outputs/scene0061-transfuserpp/runtime/m7_seed_${SEED}_intermediate_evaluation.json
```

   Require `status=evaluated`, `frame_count=39`, and an empty
   `fail_closed_reasons` list. The evaluator can validate BEV semantic labels,
   perspective semantic labels, depth, target-speed probabilities, waypoints,
   route checkpoints, boxes, control, and latency. It cannot claim full-scene
   3D occupancy without matching dense ground truth.

## Freeze

After all three real seed reports and intermediate evaluations pass, run:

```bash
python -m runners.aggregate_open_loop_transfuserpp_m7 \
  --report $CLB_ROOT/outputs/scene0061-transfuserpp/runtime/m6_stage_b_report.full39.json \
  --report $CLB_ROOT/outputs/scene0061-transfuserpp/runtime/m7_seed_43_report.json \
  --report $CLB_ROOT/outputs/scene0061-transfuserpp/runtime/m7_seed_47_report.json \
  --intermediate-evaluation $CLB_ROOT/outputs/scene0061-transfuserpp/runtime/m7_seed_41_intermediate_evaluation.json \
  --intermediate-evaluation $CLB_ROOT/outputs/scene0061-transfuserpp/runtime/m7_seed_43_intermediate_evaluation.json \
  --intermediate-evaluation $CLB_ROOT/outputs/scene0061-transfuserpp/runtime/m7_seed_47_intermediate_evaluation.json \
  --evidence-root $CLB_ROOT/outputs/scene0061-transfuserpp \
  --output $CLB_ROOT/outputs/scene0061-transfuserpp/runtime/open_loop_m7_triplicate_report.v1.json
```

The aggregator fails closed unless the exact seed set, fixed algorithm/input
identity, frame gates, intermediate gates, and evidence hashes all match. It
uses population variance and records the per-seed values. Keep only the
successful directories and the final report; move failed attempts to Trash and
record the reason in `docs/open_loop_m7_debug_log.md`.
