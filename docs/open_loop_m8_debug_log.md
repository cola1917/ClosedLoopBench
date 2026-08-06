# Open-loop M8 debug and cleanup log

Scope: scene-0061 three-route TransFuser++ open-loop comparison completed on
2026-08-06. This log records failed attempts and the fixes that produced the
retained final evidence. It does not turn the run into closed-loop M8.2/M8.3
evidence.

## Input contract locked

The final comparison uses one Scenario IR and one TF++ runtime identity for all
routes:

1. `raw_original`: CARLA Stage-A native RGB + native LiDAR.
2. `reconstructed`: NuRec reconstructed RGB + NuRec reconstructed LiDAR from
   the original-replay branch.
3. `harmonized`: Harmonizer RGB + the same NuRec reconstructed LiDAR payloads
   as route 2.

All three use the original Scenario IR for ego/actor ground truth. The NuRec
filename suffix `_original` is a branch name within the NuRec output; it does
not mean that the file came from CARLA. This distinction is required for the
final comparison to be meaningful.

## Debug history

| Failure | Evidence | Fix |
|---|---|---|
| First comparison used a custom report schema that the shared open-loop validator did not accept. | The comparison could not validate the route reports before input checks ran. | Reused the shared `open_loop_multimodal_report.v1` report contract and kept the triplicate comparison as a separate `open_loop_transfuserpp_triplicate_comparison.v1` artifact. |
| Raw payload references omitted the run-specific capture directory. | Container paths resolved as `/sim-data/payloads/...`, while the mounted capture was `/sim-data/<run>/payloads/...`. | Fixed the Stage-A payload remapping and added a regression test for the capture-directory prefix. |
| Reusing an old run ID collided with existing intermediate output directories. | A repeated attempt could mix or overwrite frame outputs from an earlier run. | Every formal route uses a unique run ID; the runner refuses ambiguous or reused output identities. |
| Raw inference hit the plugin timeout on the first scored frame. | `m8_raw_r1` produced 0/39 intermediates; `m8_raw_r2` and `m8_raw_final` still had 2 and 1 fallbacks respectively. | Added three same-process CUDA/model warm-up passes before formal frame 0. Warm-up is excluded from scoring and does not write formal intermediates. |
| The final route check initially verified labels but did not make the reconstructed LiDAR provenance obvious. | NuRec files named `*_original.xyzi.bin` were easy to confuse with raw CARLA LiDAR. | The comparison now binds the materialization source, checks the NuRec LiDAR coordinate contract, and verifies reconstructed/Harmonizer per-frame LiDAR SHA equality. |
| The first M8 score used the legacy center-point/actor-proxy evaluator. | The old evaluations had no same-frame actor manifest and were not valid bbox scores. | Formal M8 now requires `open_loop_bbox_actor_manifest.v1`, GT dimensions/yaw, oriented BEV IoU, unique same-frame class matching, AP25/AP50, and size/yaw errors. |
| Reconstructed and Harmonizer inference records pointed at r1 traces. | Their payload hashes matched r2, but their record references and dynamic-object digest were stale. | Added the derived-binding runner. It copies dense NPZs into new final directories, verifies payload/source-frame hashes, and rewrites the record/report provenance to r2. |
| Host intermediate directories were root-owned after container inference. | Host Python could read the records but could not create final sibling directories. | Ran the read-only derived binding and scoring commands in the existing TF++ image as root; no image rebuild or model rerun was needed. |
| Reconstructed/Harmonizer bbox detection was dead after the matrix fix was still pending: r2 routed 4 and 0 predictions respectively, both with matched=0. | All-zero TP/recall/mAP on the formal bbox scores. | **Root cause: wrong NuRec LiDAR `response_to_sensor` axis matrix.** The old r18 matrix mapped the render vertical axis onto the ego forward axis. NN-registration against CARLA native point clouds measured 1.67% <1 m overlap for r18 vs 24.5%+ for the corrected candidate. | — |
| The corrected matrix had to satisfy the NuRec rotation contract. | `LiDARAxisNormalizationError: rotation determinant must be +1` rejected the naive axis-swap candidate `[x,-y,z]` (det=-1). | Chose the pure +90° z-rotation `[0,-1,0,0; 1,0,0,0; 0,0,1,0; 0,0,0,1]` (det=+1). NuRec declares `x_forward_y_right_z_up` and the trace calibration confirms `carla_x_forward_y_right_z_up`, so the render frame is the CARLA frame rotated +90° about z. | — |
| r3 traces rebuilt with the corrected matrix but scores remained far below raw. | reconstructed r3: 19 predictions, 1 match, recall 0.0010, mAP50 0.0009. harmonized r3: 17 predictions, 0 matches, recall 0.0. Raw r6 keeps 66 predictions, 39 matches, recall 0.0407, mAP50 0.2736. | The matrix fix is real (predictions rose from 4 to 19 and the single matched box has center error 0.71 m, IoU 0.52, yaw error 22°), but the NuRec rendered point cloud itself differs from CARLA native LiDAR: per-frame <1 m NN overlap of the rendered cloud against the CARLA cloud is only 8-37% (frame 0: 8%, frame 5: 37%, frame 19: 25%), with no rigid rotation or mirror scan reaching a better fit. The residual gap is reconstruction geometry, not a recoverable transform; downstream TF++ detection follows the rendered geometry and drifts several meters. | — |
| r4 traces added a -1.0 m z compensation for the measured sensor-height offset but detection did not improve. | 39-frame paired scan: NuRec cloud sits 1.02 m (std 0.11) higher than CARLA; after z-shift, <1 m 3D overlap rises 25-47% to 62-75%. reconstructed r4: 35 predictions, 0 matches. harmonized r4: 23 predictions, 1 match, mAP50 0.0016. | The z offset was a real sensor-height difference and is correctly compensated in the r4 matrix `[0,-1,0,0; 1,0,0,0; 0,0,1,-1; 0,0,0,1]`, but matching stayed at noise level. Vehicle height band (ego z 0-2.2 m) is ~2x denser in NuRec than CARLA (guardrails/low structures), and TF++ boxes drift 5-20 m with yaw error ~77°: detection follows reconstructed static structures, not vehicles. | — |
| NuRec LiDAR renders the lead vehicle and most vehicles as empty while RGB shows them. | All five raw-matched tracks (c1958768 x25, bc38961c x10, 42641eb6 x2, 085fb7c4 x1, a60047ad x1) are in the controllable set and are all present in the V04 `dynamic_objects` request with correct poses. In the NuRec point cloud the lead vehicle c1958768 (the target; visible in RGB - edited-vs-original pixel diff at x 358-392, y 229-261) has 0 points at the server-reported ROI; 42641eb6 has 0; bc38961c 0-2; 85246a44 1-4; a60047ad 3-6; only 085fb7c4 renders fully (45 points). | Root cause is in the NRE server LiDAR renderer (ncore 26.04): dynamic vehicle meshes are mostly not sampled by the LiDAR pass while the same objects render in RGB. This is an upstream rendering-pipeline quality limitation, not a coordinate transform: the r3 matrix and r4 z compensation are correct and are retained, but no ClosedLoopBench-side fix can recover vehicles the renderer never emits. Scores with the NuRec input are the final evidence for the reconstructed/harmonized routes; raw r6 remains the reference route. | — |

## Retained final evidence

| Artifact | Result |
|---|---|
| `runtime/m8_raw_r6_bbox_final_open_loop_report.json` | completed; 39/39 intermediates; 0 fallback; current raw r6 trace |
| `runtime/m8_reconstructed_r4_bbox_final_open_loop_report.json` | completed; 39/39 intermediates; current r4 trace binding (matrix fix + z compensation) |
| `runtime/m8_harmonized_r4_bbox_final_open_loop_report.json` | completed; 39/39 intermediates; current r4 trace binding (matrix fix + z compensation) |
| `runtime/m8_raw_r6_bbox_final_intermediate_evaluation.json` | formal actor-aware bbox; 39 frames; no fail-closed reasons |
| `runtime/m8_reconstructed_r4_bbox_final_intermediate_evaluation.json` | formal actor-aware bbox; 39 frames; no fail-closed reasons |
| `runtime/m8_harmonized_r4_bbox_final_intermediate_evaluation.json` | formal actor-aware bbox; 39 frames; no fail-closed reasons |
| `runtime/m8_triplicate_bbox_comparison.v5.json` | `status=ready`; formal bbox gate and common actor manifest passed; per-class (vehicle/pedestrian) tp/fp/fn/recall/AP in `intermediate_metrics.bbox_per_class` |
| `runtime/m8_triplicate_bbox_comparison.v4.json` | moved to `/tmp/closedloopbench-m8-cleanup-20260807/runtime/` (historical r4 aggregate evidence) |
| `runtime/m8_triplicate_bbox_comparison.v3.json` | moved to `/tmp/closedloopbench-m8-cleanup-20260807/runtime/` (historical r3 evidence) |
| `runtime/m8_triplicate_bbox_comparison.v2.json` | moved to `/tmp/closedloopbench-m8-cleanup-20260807/runtime/` (historical r2 evidence) |
| `runtime/m8_reconstructed_r3_bbox_final_open_loop_report.json` / `m8_harmonized_r3_bbox_final_open_loop_report.json` / r3 evaluations | moved to `/tmp/closedloopbench-m8-cleanup-20260807/runtime/` (historical matrix-fix evidence) |
| `runtime/m8_reconstructed_r2_bbox_binding.json` / `m8_harmonized_r2_bbox_binding.json` / r2 evaluations | moved to `/tmp/closedloopbench-m8-cleanup-20260807/runtime/` (historical pre-fix evidence) |

The raw final warm-up report records three excluded warm-up inferences. Formal
latency and intermediate metrics include only the 39 scored frames.

## Cleanup

Moved out of the M8 working set after the final comparison passed:

- failed raw capture `m8-triplicate-raw-r1`
- failed raw report files `m8_raw_debug_r1_open_loop_report.json`,
  `m8_raw_r1_open_loop_report.json`, `m8_raw_r2_open_loop_report.json`, and
  `m8_raw_final_open_loop_report.json`
- superseded raw intermediate directory `scene0061-open-loop-m8-raw-final`

The removed working-set artifacts are recoverable in
`/tmp/closedloopbench-m8-cleanup-20260805/`; the root-owned intermediate was
moved there through the local Docker runtime because the host user could not
rename it directly.

After auditing the final report references, the old r1 reconstructed/Harmonizer
trace set, the raw r2-r5 attempts, their superseded intermediate directories,
and the old proxy reports were moved on 2026-08-06. The move was performed with
root in the existing TF++ image because the container-created intermediate
directories are root-owned on the host. The cleanup is recoverable at
`/tmp/closedloopbench-m8-cleanup-20260806/`, organized as `route-captures/`,
`intermediates/`, and `runtime/`.

The retained M8 set is now only the raw r6 route, the two r4 routes, the three
bbox evaluations, the v5 comparison, and the trace/input directories the
reports reference. On 2026-08-07 the r2/r3 route traces, the r2/r3 reports and
evaluations, the r2 bindings, the v2/v3/v4 comparisons, the r3 intermediates,
and the superseded preflight/warm-up records were moved to
`/tmp/closedloopbench-m8-cleanup-20260807/` (organized as `routes/`,
`intermediates/`, and `runtime/`; the root-owned intermediates were moved
through the local Docker runtime). The seven formal evidence files (three
reports, three evaluations, v5 comparison) are tracked in git under
`outputs/scene0061-transfuserpp/runtime/` so the M8 result survives in the
repository. Earlier M6/M7
evidence remains because their formal reports and documentation still reference
it. No retained final evidence was moved.

## Remaining metric boundary

Available: waypoint ADE/FDE, route-checkpoint error, target-speed error and
bin accuracy, plus formal dynamic-actor oriented BEV bbox metrics. The bbox
GT is the original Scenario IR actor trajectory bound to the USDZ sequence
tracks: 68 actors (10 vehicles and 58 pedestrians), with GT dimensions and
yaw. Matching is same-frame, same-class, confidence-sorted, unique matching at
oriented IoU 0.50; AP25/AP50 use the same geometry at their declared IoU
thresholds. Depth remains `unavailable` without a same-frame projected depth
target. Control is `prediction_only`; no human-driver control labels are
present. Full-scene 3D occupancy is unavailable. No result in this log is a
closed-loop score.
