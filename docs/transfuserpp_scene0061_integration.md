# scene-0061 TransFuser++ v5 integration

## Status and claim boundary

Local development is complete up to remote installation, immutable
repository/checkpoint binding, CUDA execution, NRE LiDAR coordinate validation,
and real CARLA/NuRec runs. Nothing here claims that a real checkpoint or learned
perception-in-loop scene-0061 run has already passed.

The pinned upstream family is CARLA Garage `leaderboard_2`, TransFuser v5
(TransFuser++ for Leaderboard 2.0):

- repository: `https://github.com/autonomousvision/carla_garage`
- branch: `leaderboard_2`
- model boundary: `team_code/model.py:LidarCenterNet`
- official agent reference: `team_code/sensor_agent.py`

The repo and checkpoint remain external read-only mounts. Formal binding
requires a 40-hex Git revision plus content hashes for the tracked repo,
checkpoint, model config, runtime config, formal artifact, Scene Package,
scenario IR, immutable matrix, source run config, and case variant. The matching
CARLA `agents/navigation` source is also an external read-only mount with a
deterministic Python-source snapshot hash; this prevents the local
ClosedLoopBench `agents` package from silently shadowing the dependency used by
upstream `nav_planner.py`.
The upstream Git origin must normalize to the official autonomousvision URL,
and `git status --porcelain --untracked-files=all` must be empty; untracked code
cannot enter the import graph outside the repository snapshot identity.
The dedicated compose service selects a literal immutable Docker image ID and
passes that same ID into the runtime identity; it cannot execute a mutable tag
while merely declaring an unrelated digest.

## Sensor and navigation boundary

The formal renderer still produces six 800x450 NuRec cameras and `lidar_top`.
The model consumes only `camera_front` and `lidar_top`; the other five cameras
remain synchronization and render-quality evidence.

The official v5 agent uses one 1024x512 front camera. The adapter center-crops
the 800x450 source to 800x400, resizes to 1024x512, then invokes the upstream
model crop. Every frame records this adaptation and
`official_leaderboard_sensor_equivalent=false`. JPEG roundtrip behavior is also
frozen in runtime identity.

Navigation sources are:

- speed: current CARLA actor velocity;
- ego pose/heading: current CARLA transform in the canonical scene frame;
- model target point and actor proxies: explicitly converted from canonical
  x-forward/y-left to CARLA ego x-forward/y-right;
- route command: the frozen route waypoint command, falling back to
  `LANE_FOLLOW` only when absent; common `RoadOption` aliases are normalized;
- route progress: monotonic and sampling-density-independent, using 7.5 m
  along-route lookahead rather than a fixed number of waypoint indices;
- GPS: bypassed; no fake GNSS value is created;
- raw IMU: not consumed by the direct model forward;
- calibration: immutable front-camera and LiDAR `sensor_to_ego` matrices,
  explicit 800x450 source dimensions, and the frozen crop/resize contract.
  These values travel in every observation and intermediate record; payload
  dimensions or LiDAR extrinsic drift fails before model inference.

NRE LiDAR is not admitted on an assumption. The base run config must declare
`lidar_response_coordinate_frame=sensor_local`,
`lidar_axis_convention=carla_sensor`,
`lidar_sensor_to_ego_coordinate_frame=carla_x_forward_y_right_z_up`, and a real
coordinate-validation evidence path/hash. Otherwise bundle preparation fails.

The official SensorAgent merges adjacent half scans and applies UKF/GNSS route
filtering, stop-sign memory, initial delay, stuck/creep recovery, and a LiDAR
safety box. This integration calls the pinned model and learned controller
directly, so those helpers are bypassed and recorded. It is a real model
integration, but not an official Leaderboard score or sensor-agent-equivalent
input distribution.

## Synchronized data flow

```text
CARLA tick and actor state
  -> one canonical frame context
  -> NRE 6 RGB + lidar_top render
  -> immutable payload files and hashes
  -> attempt-relative remap into /sim-data
  -> frame-matched ROS2 observation
  -> TransFuser++ forward and learned controller
  -> frame-matched control or error-tagged safe stop
  -> CARLA next tick + KPI/intermediate evidence
```

The first tick is a declared initialization safe stop. No world step occurs
between materializing frame N and requesting its control. `run_id` travels in
the same observation. When a triplicate attempt changes run ID, controller
history resets and a new output directory is created, so repeated CARLA frame
IDs never overwrite prior evidence.

Materialization records host paths for audit and paths relative to the
triplicate output root. Compose mounts that root at `/sim-data`; only validated
relative paths are rewritten. A host absolute path is never assumed to exist in
the sidecar.

Backend exceptions write exact type, message, traceback, frame/run/case/seed,
and formal identity to `backend_failures/<run_id>.jsonl`. The safe stop uses an
`error:` frame identity, so the host counts fallback/mismatch rather than
successful matched control. Formal acceptance additionally requires zero
non-initialization fallbacks, zero mismatches/rejections, and one valid
intermediate record for every accepted model control.

## Intermediate outputs and evaluation

Every successful frame records perspective and BEV semantics, depth,
NMS-filtered boxes, learned waypoints, route checkpoints, target-speed bins and
probabilities, selected path, attention summaries, vehicle control, latency,
input hashes, synchronization, dynamic-object identity, and the complete
model/scene/artifact/matrix/case/seed/variant/run identity.

Dense NPZ keys are fixed as `bev_semantic_labels`,
`perspective_semantic_labels`, `depth`, and `target_speed_probabilities`.
Missing heads/files, hash mismatch, shape/dtype/probability mismatch, frame
gaps, or identity drift fail closed. Dense and input references carry both
container and attempt-relative identities, so host evaluation resolves them
with `--evidence-root` and rechecks hashes.

The evaluator reports dynamic actor center/box proxies in predicted BEV,
world-normalized waypoint/checkpoint change, target-speed/yield/braking
response, and latency. Baseline/edit causality requires equivalent pre-event
sensor/dynamic state, pose, BEV, target-speed and control evidence. Direct BEV
pixel comparison uses only post-event pairs whose ego poses remain aligned and
only the edited actor's proxy ROI; divergent closed-loop views are disclosed
and skipped rather than misclassified as scene edits. It does not claim full
3D occupancy because scene-0061 has no
matching dense voxel/free-space ground truth. A bare classification string
cannot grant `perception_eligible`; a matching `render_quality_report.v1` is
required, bound by file SHA-256, and it must contain all six formal camera
results before it can grant `perception_eligible`. Its RGB/LiDAR change claim
must itself point to a SHA/size-bound
`rgb_lidar_actor_change_source_report.v1`; the evaluator reads back and checks
the experiment, target track, paired frame range, payload hashes and derived
change flags both when producing the quality report and when ranking it.

The visualizer verifies the source RGB hash, copies it into a resized preview,
and renders BEV/boxes/checkpoints separately. It never changes the source RGB.

## Local CLI

The unbound template must remain blocked locally:

```powershell
python -m runners.build_transfuserpp_runtime_manifest `
  --config configs/scene0061_transfuserpp_runtime.remote.template.json `
  --output <new-output>/runtime_manifest.json
```

After remote LiDAR convention evidence is attached, generate a case/seed bundle:

```powershell
python -m runners.prepare_scene0061_transfuserpp_remote_run `
  --base-run-config <formal-base-run-config.json> `
  --runtime-template configs/scene0061_transfuserpp_runtime.remote.template.json `
  --matrix configs/scene0061_counterfactual_matrix.v1.json `
  --case-id S2_lead_hard_brake `
  --seed 41 `
  --event-timestamp-sec <verified-source-event-sec> `
  --output-dir <new-output>/S2_lead_hard_brake/seed_41
```

Evaluate real outputs:

```powershell
python -m runners.evaluate_transfuserpp_intermediates `
  --trace <run>/intermediates `
  --evidence-root <bundle-or-triplicate-root> `
  --render-quality-report <run>/render_quality_report.json `
  --output <run>/intermediate_trace_evaluation.json

python -m runners.evaluate_transfuserpp_intermediates `
  --trace <S0-run>/intermediates `
  --edited-trace <S2-run>/intermediates `
  --event-timestamp <event-sec> `
  --expected-case-id S2_lead_hard_brake `
  --evidence-root <S0-bundle-root> `
  --edited-evidence-root <S2-bundle-root> `
  --output <run>/counterfactual_comparison.json

python -m runners.render_transfuserpp_intermediate `
  --record <run>/frame_00000042.intermediate.json `
  --evidence-root <bundle-or-triplicate-root> `
  --output <run>/frame_00000042.panel.png
```

## Minimal remote work

1. Fetch the official remote-tracking ref
   `refs/remotes/origin/leaderboard_2`, check out an exact commit from its
   history, and obtain the real pretrained checkpoint/config. The runtime
   manifest resolves that ref and uses `merge-base --is-ancestor` to reject a
   same-origin commit that is not in the configured ref history.
2. Bind repo/checkpoint/config hashes plus a hash-pinned `agents/navigation`
   source mount and `carla_agents_sha256`. The TF++ image supplies
   `carla==0.9.15`; the host CARLA/ROS runtime remains `0.9.16`. Require a `prepared`
   runtime manifest generated inside the container against the real read-only
   mounts. Container preflight must successfully import both the pinned
   upstream model and `agents.navigation.global_route_planner`.
3. Build the GPU sidecar, select it by the exact `docker image inspect .Id`
   value, then run the independent CUDA gate and bind its immutable result to
   a new run config (neither command overwrites an existing file):

   ```bash
   docker compose --env-file /path/to/transfuserpp.env \
     -f docker/compose.transfuserpp.yml run --rm --entrypoint python3 transfuserpp \
     -m runners.run_transfuserpp_cuda_preflight \
     --config /sim-data/runtime/transfuserpp.runtime.json \
     --observation /sim-data/runtime/formal_warmup_observation.json \
     --output /sim-data/runtime/transfuserpp.cuda-preflight.json
   conda run -n autodrive python -m runners.bind_transfuserpp_cuda_preflight \
     --run-config run.unbound.json \
     --cuda-evidence /new/evidence/transfuserpp.cuda-preflight.json \
     --output run.cuda-bound.json
   ```

   The probe runs inside the pinned GPU container because the official model,
   checkpoint paths and Python dependencies are container-bound. Only the
   evidence binder runs in the host `autodrive` environment. The gate performs
   actual warm-up and measured forward passes with CUDA
   synchronization, resets peak-memory counters after warm-up, and records
   peak allocated VRAM plus P50/P95/P99 latency. Acceptance re-hashes the
   report and requires exact runtime/experiment/gate identity. The mutable
   evidence reference is explicitly outside the formal run-config hash scope,
   preventing a report-hash/run-hash self-reference; all other inputs remain
   immutable.
4. Prove live NRE LiDAR coordinates/extrinsic and attach the evidence hash.
5. Generate S0/S2/S4 bundles for seeds 41/42/43 (nine bundles total). Run
   three consecutive attempts per bundle (27 physical attempts). Mount each
   bundle's triplicate
   output root as `SIM_DATA_HOST_PATH` and place the selected runtime JSON at
   `/sim-data/runtime/transfuserpp.runtime.json`.
6. Start CARLA with the existing `start_carla.sh`, then NRE and the sidecar;
   execute the real NuRec handler with `ros2_observation_control`.
7. Run `run_carla_acceptance_triplicate --ego-driver ros2_observation_control
   --sensor-handler-factory adapters.nurec_260_client:build_nurec_260_handler
   --require-multimodal` with the bundle
   directory as its output/evidence root. The runner re-hashes the LiDAR
   coordinate evidence and automatically enforces zero backend failures,
   zero non-initialization fallback/mismatch, and frame-complete intermediate
   traces before writing a passed, explicitly `control_only` triplicate. Then run
   `validate_multimodal_closed_loop.py`.
8. Require valid intermediate/quality reports,
   RGB-LiDAR edit consistency, mapping, closed-loop KPI, and cleanup. Finally
   retain strict S0 triplicate acceptance evidence.
9. Capture the CARLA state window, six-camera grid, LiDAR inset, TF++ panels,
   KPI overlay, comparisons, and hashes listed by the video manifest.

Any failed gate remains `remote_validation_required` or rejected. Offline and
fake traces are never admitted as real CARLA/NuRec results.
