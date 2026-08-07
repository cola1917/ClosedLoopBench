# Open-loop multimodal evaluation (scene-0061)

Branch: `feat/open-loop-multimodal`
Status: M1-M4 implemented and pushed; M5 Stage A, M6 Stage B, M7 formal
acceptance, and the M8 three-route comparison with formal actor-aware bbox
scoring completed on 2026-08-06.

Current M5 evidence:

- report: `outputs/scene0061-transfuserpp/runtime/m5_stage_a_report.v3.json`
- CUDA gate: `outputs/scene0061-transfuserpp/runtime/transfuserpp.cuda-preflight.json`
- native trace: `outputs/scene0061-transfuserpp/native-stage-a-3frames-r7/native_stage_a_observations.json`

Current M6 Stage B evidence (the real 39-frame full-run is the sole retained M6 result; superseded smoke attempts are recorded in the debug log):

- report: `outputs/scene0061-transfuserpp/runtime/m6_stage_b_report.full39.json`
- runtime binding: `outputs/scene0061-transfuserpp/runtime/transfuserpp.m6.stage-b.runtime.v5.json`
- NuRec trace: `outputs/scene0061-transfuserpp/m6-stage-b-full-r11/nurec_stage_b_observations.json`
- TF++ image: `closed-loop-bench/transfuserpp-v5:m6-stage-d`, digest
  `sha256:d5f814b8aab88bbef08e70a4f915771658221190d2501e2894815977d3db6394`
- result: `execution_status=completed`, 39/39 intermediates, `fallback_count=0`,
  39/39 matched frames, 0 dropped or mismatched frames
- NuRec `SensorsimService/26.04` (`26.4.146`): 6 RGB + `lidar_top` on every
  frame; dynamic actor creation `false`, dynamic object count `0`; trace SHA
  `d6c61100f4a1b940e0d9f7006dfa9935d0428a1d0bfc504dcfad80316fda8b53`
- the first three CUDA warm-up passes are excluded from scored frames; formal
  frame 0 starts after warm-up and remains below the 0.5 s plugin timeout;
  formal latency mean/p95/max is `158.54/183.77/235.76 ms`
- this is M6 open-loop evidence only; IR actors are used as offline collision
  proxies and no CARLA dynamic actors or TF++ control are applied
- debug history and cleanup record: `docs/open_loop_m6_debug_log.md`

Current M7 formal acceptance evidence:

- frozen triplicate report:
  `outputs/scene0061-transfuserpp/runtime/open_loop_m7_triplicate_report.v1.json`
- seed reports: `runtime/m6_stage_b_report.full39.json` (seed 41 retained M6
  full-run), `runtime/m7_seed_43_report.json`, and
  `runtime/m7_seed_47_report.json`
- intermediate evaluations:
  `runtime/m7_seed_{41,43,47}_intermediate_evaluation.json`; all three are
  `status=evaluated`, `frame_count=39`, with no fail-closed reasons
- formal result: `S0_original_replay` x seeds `{41,43,47}`; 39/39 frames and
  39/39 intermediates for every seed, zero fallback/drop/mismatch, six RGB plus
  `lidar_top` on every NuRec frame
- fixed algorithm identity: checkpoint
  `d6fbdc28f7398354beadc7cf6765d866457c957f7b470c88ba206e73311a3b44`, model
  config `895e3e9704ceda443169ca32aaef2712b1becf2d42473d7273071ec6ceda113e`,
  repo revision `72f39a63423a5edef6904b1487e0360a64bcf445`, image digest
  `sha256:d5f814b8aab88bbef08e70a4f915771658221190d2501e2894815977d3db6394`
- aggregate SHA-256:
  `71e49fc7a8532dbfa2b92033c16e2f75516e7bd38d323ac8d4820283bf1d03e5`
- the aggregate keeps BEV/depth/target-speed dense outputs in the intermediate
  evaluation boundary; full-scene 3D occupancy remains unavailable
- reproducible procedure: `docs/open_loop_m7_runbook.md`; debug and cleanup
  history: `docs/open_loop_m7_debug_log.md`

GUI observability smoke (not formal evidence):

- report: `outputs/scene0061-transfuserpp/runtime/gui-smoke-20260805/gui_smoke.json`
- screenshot: `outputs/scene0061-transfuserpp/runtime/gui-smoke-20260805/carla_gui_smoke.png`
- CARLA `0.9.16`, 15.012 s, 702 synchronous ticks; `formal_evidence: false`
- uses the pinned OpenDRIVE replay world and GT ego trajectory; this is a
  visual sanity check and does not claim a full Town asset or M5 score

Current M8 three-route comparison and formal bbox evidence:

- comparison report: `outputs/scene0061-transfuserpp/runtime/m8_triplicate_bbox_comparison.v2.json`
- status: `ready`; 39/39 frames, 39/39 intermediates, zero fallback, zero
  frame mismatch, and formal actor-aware bbox gate passed on all three routes
- formal route reports:
  `runtime/m8_raw_r6_bbox_final_open_loop_report.json`,
  `runtime/m8_reconstructed_r2_bbox_final_open_loop_report.json`, and
  `runtime/m8_harmonized_r2_bbox_final_open_loop_report.json`
- formal intermediate evaluations:
  `runtime/m8_raw_r6_bbox_final_intermediate_evaluation.json`,
  `runtime/m8_reconstructed_r2_bbox_final_intermediate_evaluation.json`, and
  `runtime/m8_harmonized_r2_bbox_final_intermediate_evaluation.json`
- actor manifest: `inputs/open_loop_bbox_actor_manifest.v1.json`; 68 dynamic
  actors, 39 frames, shared by all routes
- comparison SHA-256: `f794520b828877af0292074824d0dbd3048e165d6b4692b643ae0e7f8cb8a666`

The route binding is deliberately asymmetric:

| Route | RGB input | LiDAR input | Ground truth |
|---|---|---|---|
| `raw_original` | CARLA Stage-A native RGB | CARLA Stage-A native LiDAR | Original Scenario IR |
| `reconstructed` | NuRec reconstructed RGB, original-replay branch | NuRec reconstructed LiDAR, original-replay branch | Original Scenario IR |
| `harmonized` | NVIDIA Harmonizer RGB | The exact same NuRec reconstructed LiDAR as the reconstructed route | Original Scenario IR |

In the NuRec package, a file such as
`multimodal_20fps/lidar/000000_original.xyzi.bin` is **not** raw CARLA LiDAR.
`original` names the original-replay branch inside the reconstructed NuRec
capture. The raw route is the only route using CARLA Stage-A sensor payloads.
The comparison gate checks the materialized source provenance and requires all
39 reconstructed/Harmonizer LiDAR SHA-256 values to match. Harmonizer is RGB
only; it does not regenerate or alter LiDAR.

M8 intermediate metrics currently available for all three routes are waypoint
ADE/FDE, route-checkpoint error, target-speed error/bin accuracy, and formal
dynamic actor oriented BEV bbox metrics. The bbox scorer uses GT dimensions
and yaw, same-frame same-class unique matching, oriented BEV IoU, TP/FP/FN,
Precision/Recall, AP25/AP50, mAP, center/size/yaw error. Depth is explicitly
`unavailable` because no bound same-frame camera-LiDAR depth target is present;
control remains `prediction_only` because Scenario IR has no human-driver
control labels.

## 1. Goal

Deliver a **repeatable open-loop multimodal simulation evaluation** for
TransFuser++ (TF++) on scene-0061:

- ego follows **GT / Scenario IR reference poses** (teleport each tick);
- multimodal observations are published on the existing ROS2 boundary;
- TF++ predicts waypoints / control / intermediate heads;
- predictions are scored against IR / nuScenes GT;
- the algorithm **does not** determine the next ego pose.

This is an independent product track. It is **not** Goal M8 / M9 closure and
must never be relabeled as such.

## 2. Claim boundary (fail-closed)

| Allowed claim | Forbidden claim |
|---|---|
| Open-loop TF++ eval on pinned IR GT poses | Closed-loop Goal / M9 pass |
| Multimodal obs along logged trajectory | Causal NuRec LiDAR–world M8.2 pass |
| ADE/FDE, control error, detection AP vs IR GT | Interactive counterfactual M10/M11 |
| Engineering sync / latency / determinism | Official CARLA Leaderboard score |
| CARLA as sync clock + GT replay host | “Map-faithful lane driving” from current XODR fit |

Required report field:

```text
evidence_classification: open_loop_multimodal
```

Do not reuse `perception_eligible` or any M-series pass string unless the
existing closed-loop gates independently allow it. Open-loop reports keep
`execution_status` separate from evidence meaning (same split as the plugin
contract).

## 3. Architecture

```text
Pinned Scenario IR (ego + actors @ t)
        │
        ▼
 CARLA sync mode
   - ego set_transform(GT)
   - actors replayed from IR
   - fixed delta / frame_id
        │
        ├─ Stage A (smoke): CARLA native RGB + LiDAR
        └─ Stage B (formal multimodal): NuRec RGB(6)+lidar_top @ GT pose
        │
        ▼
 Local ROS2 observation topics
   (existing ros2_observation_control contract)
        │
        ▼
 TF++ backend (camera_front + lidar_top)
        │
        ▼
 Log control + intermediates
   ❌ do not let control change next ego pose
        │
        ▼
 Open-loop metrics report
```

### CARLA role

CARLA is the **synchronized replay host**, not a closed-loop world:

1. Advance one sync tick.
2. Teleport ego to IR pose at `t`.
3. Place dynamic actors at IR poses at `t`.
4. Materialize sensors (native or NuRec) at that pose.
5. Hand observation to ROS / TF++.
6. Record outputs; next tick still teleports to the next GT pose.

`apply_control` may be called only for plumbing smoke tests. Formal open-loop
acceptance requires **GT teleport ownership** of ego pose every scored frame.

M6 Stage B uses a static NuRec scene instead of a CARLA actor world: no dynamic
CARLA actors are created, and Scenario IR actor tracks are retained only for
offline collision-proxy scoring.

### ROS role

ROS stays local. Reuse:

- `agents/ros2_observation_control_driver.py`
- TF++ compose / `transfuserpp_ros2_backend`
- frame-matched observation identity (`frame_id`, `run_id`, hashes)

### TF++ role

Pinned Leaderboard 2 / TransFuser++ v5 path remains as in
`docs/transfuserpp_scene0061_integration.md`:

- primary inputs: `camera_front`, `lidar_top`;
- other cameras are sync / quality evidence only;
- intermediates (waypoints, BEV, boxes, control, latency, hashes) are the
  evaluation source of truth.

## 4. Pinned inputs (scene-0061)

### Open-loop GT pin (use this)

| Artifact | Path | SHA-256 |
|---|---|---|
| Scenario IR | `outputs/scene0061_exchange_v2/scene_ir.json` | `754a48f2a8eff3878229d3c6f80d0912bdd00016c86c77d5d295fc8f51e418d0` |
| OpenDRIVE (preferred) | `outputs/scene-0061/road.xodr` | `46e759ff00aff53b489b175822c33c7e03dc1f78d93c287b538d9b70801273a4` |

IR facts (verified 2026-08-04):

- `schema_version`: `scenario_ir.v1`
- `scenario_id`: `cc8c0bf57f984915a77078b10eb33198`
- ego `reference_trajectory`: **39** samples, `t_sec` `0.0` → `19.149566`
- actors: **227**
- first pose: `(x,y,yaw,speed) = (0, 0, 0, 8.988…)`
- last pose: `(68.418…, 35.025…, 98.271…, 2.060…)`

### Preferred XODR quality (static route audit, 2026-08-04)

Compared with the older exchange map on the same IR ego poses
(`tools/validate_xodr_route.ps1`, scope `all`):

| Map | Roads | `inside_lane` | Centerline p95 | Heading p95 |
|---|---:|---:|---:|---:|
| `outputs/scene-0061/road.xodr` (**use**) | 68 | **0.974** | **1.63 m** | 15.6° |
| `outputs/scene0061_exchange_v2/road.xodr` (legacy exchange) | 229 | 0.513 | 3.42 m | 10.0° |

`scene-0061/road.xodr` is the tracked native map (commits through
`86b4231` / lane-alignment fixes). Keep it as the open-loop map pin.
CARLA waypoint / junction runtime remains a remote gate
(`tools/audit_carla_xodr_runtime.py`).

Note: the strict `opendrive_contract.validate_topology_artifact` currently
rejects this native file on lane-link id encoding; that contract failure is
**not** a reason to fall back to the poorly aligned exchange map for open-loop
GT-vs-centerline work. Treat topology-contract repair as a separate follow-up.

### Do not use for open-loop GT

| Artifact | Why |
|---|---|
| `outputs/alignment-validation/.../scene_ir.json` | Legacy mined IR (deleted from tree): **12** actors; ego pose drift vs exchange IR mean **1.19 m**, max **3.66 m** |
| `outputs/scene0061_exchange_v2/road.xodr` as primary map | Still valid IR sibling, but map fit is poor (~51% inside_lane) |
| Probe / repaired / `.tmp_*` XODRs | Obsolete experiments; removed from `outputs/` |
| Corridor-only historical maps | Diagnostic only |

### Formal matrix gap (disclose, do not silently substitute)

`configs/scene0061_counterfactual_matrix.v1.json` still pins:

- `actor_ready_scenario_ir` SHA `ae340b43c2ecbcf416cb89895e63ea59241b240ff83bc9dc4e6f1632a3f1ded7`
- `scene_package_sha256` `0d6b724b0dea9ff3f97717f893f19baf69904057511ad374cfd510c5cc9b9119`

Those artifacts are **not present** in the local tree. Open-loop work pins
`exchange_v2` IR (`754a48f2…`) explicitly and records that this is **not** the
matrix actor-ready identity. Closed-loop matrix claims must not be made from
this substitute without re-binding hashes.

## 5. IR drift audit (2026-08-04)

### What is *not* drifting

- Regenerating IR from local `nuscenes-mini` scene-0061 matches
  `exchange_v2` ego poses (max horizontal error **0**).
- Actor count and semantic object graph match `exchange_v2` (227 actors).
- Multiple exchange/topology copies are byte-identical to `754a48f2…`.

### What people are calling “IR drift”

1. **Legacy vs current IR**  
   `alignment-validation` IR vs `exchange_v2`: ego kinematics differ (max
   ~3.7 m / ~5°), actor set 12 vs 227. This is an **old artifact**, not live
   regeneration drift.

2. **IR poses vs older exchange XODR (poor fit)**  
   Legacy `scene0061_exchange_v2/road.xodr` against IR:
   - `inside_lane_fraction` ≈ **0.513**
   - centerline p95 ≈ **3.42 m**  
   The newer native `outputs/scene-0061/road.xodr` recovers ≈ **0.974**
   inside_lane / centerline p95 ≈ **1.63 m** on the same IR. Map fit issues on
   the exchange artifact are **not** proof that nuScenes GT poses are wrong.

3. **Identity gap**  
   Matrix actor-ready IR hash is missing on disk; using exchange IR without
   disclosure would be an identity drift for closed-loop matrix work.

### Open-loop implication

| Need | Trustworthy? |
|---|---|
| Ego GT poses for open-loop teleport / ADE | **Yes**, if pinned to `exchange_v2` |
| Actor GT boxes along IR for detection AP | **Yes** from `exchange_v2` actors |
| Map-relative lane / centerline KPIs | **Conditional** — only with pinned `scene-0061/road.xodr`; do not use exchange map fit |
| Substituting exchange IR for actor-ready matrix pin | **No** without re-hash / re-bind |

## 6. Metrics

### Primary (must ship)

| Metric | Definition |
|---|---|
| ADE / FDE | Predicted waypoints vs future IR ego trajectory |
| Lateral / heading error | Vs IR reference at matched horizons |
| Open-loop collision proxy | Predicted ego path intersects future IR actor boxes (GT ego motion) |
| Control L2 (optional baseline) | Steer/throttle/brake vs logged / expert proxy when available |
| Inference latency + drop/mismatch rate | Frame sync and engineering health |

### Multimodal / perception (TF++ intermediates)

| Metric | Definition |
|---|---|
| Detection AP / recall | Vehicle/pedestrian vs IR boxes, distance buckets |
| BEV / semantic consistency | Only if dense GT exists; else `not_applicable` |
| Target-point / checkpoint error | Vs frozen route checkpoints |

### Contract gates (must pass or fail closed)

- `frame_id` / timestamp alignment; zero scored mismatches
- sensor contract: resolution, crop/resize, LiDAR axis/frame, payload hashes
- triplicate seeds: mean ± variance reported
- `evidence_classification: open_loop_multimodal` on every formal report

### Explicitly non-primary

Closed-loop comfort / route progress / interactive TTC may be attached as
**GT-trajectory proxies** only. They do not decide open-loop pass/fail.

## 7. Delivery stages (overview)

| Stage | Deliverable | Exit | Status |
|---|---|---|---|
| **D0** | Boundary doc + IR/XODR pin | Merged / pushed on open-loop branch | **Done** |
| **D1** | Runner `open_loop_gt_replay` | scene0061 dry-run, control cannot own next pose | **Done (offline)** |
| **D2** | Local ROS + TF++ on CARLA sensors (Stage A) | Intermediates + ADE report | **Done; real M5 Stage A evidence captured** |
| **D3** | NuRec multimodal at GT poses (Stage B) | Same metrics; still not M8/M9 | **Done; real 39-frame M6 full-run** |
| **D4** | Formal acceptance | `S0_original_replay` × 3 seeds + frozen report schema | **Done; 2026-08-05 triplicate passed** |

Implementation extends existing runners/adapters. Prefer additive evidence
labels over renaming closed-loop gates.

## 7.1 Detailed development plan

**Working branch:** `feat/open-loop-multimodal`  
**Goal product:** open-loop multimodal TF++ eval on scene-0061  
**Hard rule:** every formal report sets `evidence_classification: open_loop_multimodal`; never claim M8/M9.

### Milestone map

```text
D0 boundary ──► M1 runner skeleton ──► M2 GT replay smoke
                      │
                      ▼
              M3 metrics v0 (ADE/FDE)
                      │
                      ▼
         M4 ROS local + Pure Pursuit open-loop
                      │
                      ▼
         M5 TF++ Stage A (CARLA sensors)     ← first "usable" demo
                      │
                      ▼
         M6 NuRec Stage B multimodal
                      │
                      ▼
         M7 triplicate acceptance + freeze
                      │
                      ▼
         M8 raw/reconstructed/Harmonizer comparison ← phase complete
```

### Small milestones

| ID | Milestone | Work items | Done when | Est. |
|---|---|---|---|---|
| **M1** | Open-loop runner skeleton | Add `open_loop_gt_replay` mode (or flag) on existing CARLA runner; pin IR/XODR paths + SHA checks; `control_affects_next_ego_pose=false` in run config/report | Unit/integration test: after predict, next ego pose equals IR sample N+1 | **Done** |
| **M2** | GT teleport + actor replay smoke | Ego `set_transform` each tick from IR; actors follow IR trajectories; no NuRec/TF++ yet; stub or null control sink | One scene0061 short run log: 39 IR ticks (or subset) with pose error≈0 vs IR | **Done offline; GUI-only CARLA visual smoke verified; actor replay not claimed** |
| **M3** | Metrics v0 | ADE/FDE, lateral/heading vs IR; latency/drop counters; report schema draft with §9 fields | Offline JSON report from synthetic or stub predictions validates schema | **Done** |
| **M4** | Local ROS boundary | Reuse `ros2_observation_control`; publish GT-pose observations; Pure Pursuit (or stub plugin) consumes ROS; still no pose authority from control | Matched frame_id obs→control trace; Pure Pursuit open-loop report | **Done offline** |
| **M5** | TF++ Stage A | Wire TF++ compose/backend to Stage A CARLA RGB+LiDAR at GT poses; dump intermediates; score ADE + sync gates | One successful TF++ open-loop run on scene0061 with intermediates + ADE | **Done; 2026-08-05 real smoke** |
| **M6** | NuRec Stage B | Swap sensor source to NuRec @ GT pose; keep same TF++/metrics path; disclose M8.2 not claimed | Same report shape as M5 with NuRec modality hashes | **Done; 2026-08-05 real 39-frame full-run** |
| **M7** | Formal acceptance | `S0_original_replay` × seeds `{41,43,47}` (or matrix seeds); perception AP if GT boxes available; freeze schema + artifact hashes | Triplicate mean±var report; CI or remote checklist green | 2–3 d |
| **M8** | Three-route input comparison and bbox scoring | Hold GT, TF++, runtime, and frame identity fixed while comparing CARLA native, NuRec reconstructed, and Harmonizer RGB + shared NuRec LiDAR; score real actor boxes | Three completed reports, three formal bbox evaluations, and a ready comparison artifact | **Done; 2026-08-06 39-frame comparison** |

**Calendar hint (single engineer, host healthy):**  
usable demo ≈ **M5 (~1.5–2.5 weeks)**; phase complete ≈ **M7 (~4–6 weeks)** including NuRec.

### Exit checklist per milestone

**M1**
- [ ] Mode/flag documented in runner help
- [ ] Fail-closed if IR/XODR SHA mismatch
- [ ] Test proves control output does not change next ego pose

**M2**
- [ ] Tick log binds `frame_id` ↔ IR `t_sec` / pose
- [ ] Actor count / ids audited against IR selection
- [ ] No claim of perception or TF++

**M3**
- [ ] ADE/FDE computed at declared horizons
- [ ] Report includes §9 identity block
- [ ] `claims_m8` / `claims_m9` always false

**M4**
- [ ] ROS topics match existing observation/control contract
- [ ] Zero scored frame mismatches on smoke run
- [ ] Pure Pursuit (or stub) intermediates optional

**M5**
- [ ] TF++ consumes `camera_front` + `lidar_top` only as model inputs
- [ ] Intermediate hashes recorded
- [ ] ADE report generated from TF++ waypoints

**M6**
- [x] Sensor provenance = NuRec (path + hash)
- [x] Explicit note: not M8.2 LiDAR–world closure
- [x] Metrics path identical to M5 (no forked scorer)
- [x] No CARLA dynamic actors or TF++ control; IR actors are offline collision proxies
- [x] Formal in-process CUDA warm-up is excluded from scored frames
- [x] Full 39-frame Scenario IR trajectory runs with zero fallback and zero frame mismatch

**M7**
- [x] Three seeds, same case, comparable pinned config
- [x] Frozen `open_loop_multimodal_m7_triplicate_report.v1` with mean/variance
- [x] Runbook and debug/cleanup record for the remote xt167-style host
- [x] Host/container `/sim-data` path binding and intermediate evaluation
- [x] No M8/M9 claim; static NuRec scene and control-only boundary retained

**M8**
- [x] Raw route binds native CARLA RGB and native CARLA LiDAR from the same frame
- [x] Reconstructed route binds NuRec reconstructed RGB and reconstructed LiDAR
- [x] Harmonizer route binds Harmonizer RGB and the same reconstructed LiDAR SHA per frame
- [x] All three routes use the original Scenario IR as GT
- [x] One shared USDZ/Scenario IR actor manifest binds 68 dynamic actors over 39 frames
- [x] Oriented BEV bbox matching uses GT dimensions/yaw, same-frame class matching, and unique assignments
- [x] TP/FP/FN, Precision/Recall, AP25/AP50, mAP, IoU, size error, and yaw error are scored for all routes
- [x] Reconstructed/Harmonizer derived outputs are bound to the current r2 trace without rerunning TF++
- [x] 39/39 frame and intermediate gates pass with zero fallback and zero mismatch
- [x] Depth/control/occupancy unavailable boundaries are explicit

### M7 runbook

The exact formal commands and required environment details are in
`docs/open_loop_m7_runbook.md`. The runbook requires a real report and a real
intermediate evaluation for each seed; it does not permit copying seed 41
artifacts to fill a missing seed.

### Out of scope until after M7

- Interactive counterfactuals (M10/M11)
- Claiming Goal closed-loop / M9
- Replacing IR GT with exchange XODR lane KPIs as primary score
- Merging dormant M8 experiment branches into this track

## 8. Relationship to blocked closed-loop work

| Track | State | Dependency |
|---|---|---|
| Closed-loop Goal (M8→M9) | Blocked on M8.2 LiDAR–world / M8.3 visibility | Unchanged |
| Open-loop multimodal (this doc) | Unblocked for IR-GT scoring | Uses CARLA/ROS/TF++ plumbing; avoids control-owned state |

Success here does **not** unblock M9. Failure of M8 does **not** block D1–D2.

## 9. Acceptance snippet (report must include)

```json
{
  "evidence_classification": "open_loop_multimodal",
  "scene_id": "scene-0061",
  "scenario_id": "cc8c0bf57f984915a77078b10eb33198",
  "scenario_ir_path": "outputs/scene0061_exchange_v2/scene_ir.json",
  "scenario_ir_sha256": "754a48f2a8eff3878229d3c6f80d0912bdd00016c86c77d5d295fc8f51e418d0",
  "opendrive_path": "outputs/scene-0061/road.xodr",
  "opendrive_sha256": "46e759ff00aff53b489b175822c33c7e03dc1f78d93c287b538d9b70801273a4",
  "ego_pose_source": "scenario_ir_reference_trajectory",
  "control_affects_next_ego_pose": false,
  "claims_m8": false,
  "claims_m9": false,
  "matrix_actor_ready_ir_bound": false
}
```
