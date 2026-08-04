# Open-loop multimodal evaluation (scene-0061)

Branch: `feat/open-loop-multimodal-eval`  
Status: planning boundary + IR pin audit. Implementation follows this doc.

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

## 7. Delivery stages

| Stage | Deliverable | Exit |
|---|---|---|
| **D0** | This boundary doc + IR pin | Done when merged to the feature branch |
| **D1** | Runner mode `open_loop_gt_replay` (teleport, no pose authority from control) | One scene0061 case dry-run with Pure Pursuit or stub |
| **D2** | Local ROS + TF++ on Stage A sensors | Intermediates + ADE report |
| **D3** | Stage B NuRec multimodal at GT poses | Same metrics; still not M8/M9 |
| **D4** | scene0061 `S0_original_replay` × 3 seeds acceptance | Frozen report schema + hashes |

Implementation should extend existing runners/adapters rather than fork a new
repo. Prefer additive evidence labels over renaming closed-loop gates.

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
