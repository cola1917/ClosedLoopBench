# ClosedLoopBench Agent Handoff Prompt (2026-07-30)

Copy the block below to the next agent as the authoritative handoff. The
prompt intentionally contains no password or private key material.

```text
You are taking over the ClosedLoopBench project. Respond to the user in
Chinese, but keep commit names, paths, commands, and evidence identifiers
exactly as written.

## How to use this handoff

This is the sole current operational handoff. It is intentionally self-
contained enough to begin the next local task without reading a long chat
history. Read it in this order:

1. This handoff for current facts, constraints, and first actions.
2. `docs/development_sync_policy.md` for Git and remote-test governance.
3. `docs/nurec_multimodal_actor_closed_loop.md` only when changing the
   CARLA/NuRec transaction or actor bindings.
4. `docs/architecture.md` only when changing product boundaries or the
   external-algorithm plugin contract.

The following are historical records, not execution baselines:

- `docs/scene0061_handoff_20260725.md`
- `docs/scene0061_live_tick_runbook.md`
- `docs/scene0061_nurec_handoff_20260716.md`

They predate the M6-M8 redesign, contain stale operational assumptions, and
some may render with legacy encoding artifacts. Do not copy their endpoint,
service-state, milestone-status, or command assumptions into new work. Keep
them only as historical evidence.

## Scope and current baseline

The global objective is to make ClosedLoopBench evaluate an external driving
algorithm in one causally consistent CARLA/NuRec scene. Every safety-relevant
object visible to the algorithm through NuRec RGB/LiDAR must correspond to a
same-time CARLA object with the same pose, lane state, and collision boundary.
The lead vehicle and one pedestrian are the only actors that may eventually
become controlled counterfactuals; all other context remains replay-only.

## Causal system model and non-negotiable invariants

The intended per-tick causal chain is:

```text
nuScenes / Scenario IR
  -> full scene-object registry
  -> CARLA physical actors, collision boundaries, lanes, and route
  -> same-tick CARLA pose binding -> NuRec RGB and LiDAR transaction
  -> external driving algorithm / ROS2 adapter -> CARLA ego control
  -> per-tick safety and evaluation evidence
```

The following are hard rules, not optional quality improvements:

- A safety-relevant object visible to the algorithm must have a same-time,
  same-pose CARLA representation with a collision policy. A CARLA mesh does
  not automatically create NuRec RGB or LiDAR geometry, and rendered content
  does not automatically create CARLA collision authority.
- RGB and LiDAR must come from the same scene time, ego pose interval, camera
  calibration, and dynamic-object payload. Successful gRPC, a video, or a
  non-zero point count alone does not prove this.
- All background vehicles, pedestrians, parked objects, barriers, and cones
  stay replay-only or static collision proxies. The named lead vehicle and
  pedestrian remain replayed until M10; registration never grants control.
- CARLA collision events, lane membership/lane-invasion events, pose binding,
  calibrated camera geometry, and LiDAR-world support are separate evidence
  streams. Passing one cannot compensate for a failure in another.
- The BEV map overlay is diagnostic visualization only. Direct nuScenes map
  polygons are not CARLA physical road truth, and a visually continuous road
  image cannot replace waypoint/lane/collision evidence.

## Goal phases and terminology

The global Goal is split into ordered goal phases so that one expensive or
ambiguous subsystem cannot be used to hide another:

| Goal phase | M-series gate | Meaning |
|---|---|---|
| A. operational replay | M1-M5 | CARLA, NuRec, ROS2/algorithm plumbing and portable short-run evidence |
| B. causal scene closure | M6-M8 | complete physical registry, same-tick pose binding, then collision/lane/RGB/LiDAR truth |
| C. algorithm replay evaluation | M9 | repeatable audited TF++ replay baseline |
| D. counterfactual interaction | M10 | only the pinned lead vehicle and pedestrian become controlled |
| E. attributable comparison | M11 | seeded replay/counterfactual experiment matrix and reports |

The historical TF++ case identifiers `S0_original_replay`,
`S2_lead_hard_brake`, and `S4_pedestrian_early_crossing` are distinct from the
new capability-stage labels `S0` through `S6`. Use their full case IDs in
configs and reports. Do not start a formal TF++ case merely because an adapter,
container, or short visual replay exists; M9 is blocked until M8 passes.

## Decoupled capability stages (current execution plan)

The forward plan additionally uses `S0` through `S6` as capability stages.
They prevent the old failure mode where a NuRec reconstruction problem blocked
all replay, evaluation, and control work. M-series names remain historical
integration evidence; S-stages are the planning and delivery model.

| Stage | Independent product | Current state |
|---|---|---|
| S0 | scene, tick, control, and immutable-evidence contracts | `offline_ready` |
| S1 | deterministic CARLA replay, complete physical registry, raw truth trace | `partial` |
| S2 | trace-only collision/lane/TTC/progress/comfort evaluator | `offline_ready` |
| S3 | ego plugin lifecycle, safe-stop, native control, and ROS2 boundary | offline contracts ready; live evidence incomplete |
| S4 | NuRec RGB/LiDAR transport and CARLA-world consistency | `partial`; LiDAR-world consistency failed |
| S5 | bounded lead-vehicle and pedestrian interaction | schemas/plans exist; live acceptance not started |
| S6 | integrated algorithms, counterfactual matrix, and Goal report | not started |

S1-S3 may progress with native CARLA traces or local fixtures while S4/M8.2 is
deferred. They must not be relabeled as NuRec multimodal closure. S6 is the
only integration stage that combines passed component capabilities; its
multimodal TF++ variant remains blocked by S4/M8.

Current source baseline (2026-07-30):

- ClosedLoopBench checkout: `E:\code\ClosedLoopBench`
- ClosedLoopBench branch: `master`
- ClosedLoopBench implementation baseline: `d1e3eadb5a79d5476d6f64a1607f1f0e5446070a`
  (`feat: add scene0061 topology and runtime audit gates`). Handoff-only
  documentation commits may be descendants of this implementation baseline.
- NeuralSceneBridge checkout: `E:\code\NeuralSceneBridge`
- NeuralSceneBridge branch: `main`
- NeuralSceneBridge local HEAD: `cae8fa66a93d0d24cd084851b293f24f87e0ac48`
  (`feat: add NuRec registry track closure recipes`). It is three local
  commits ahead of `origin/main`; it has not been pushed by this handoff.
- These two SHAs are the implementation baseline for the next agent. Always
  run `git rev-parse HEAD`, `git status --short`, and compare local, GitHub,
  and remote SHAs before attributing a result to code. A newer descendant that
  only updates handoff documentation is acceptable; verify ancestry explicitly.
- ClosedLoopBench currently has only local generated `tmp_test/` and
  `.tmp_test/` directories outside the commit. Do not stage them. Local
  `.claude/settings.local.json` is also not source and must not be committed.
- M8 experiment branches are deliberately not merged into `master`.

## Runtime boundary and local validation limit

- Local work happens in `E:\code\ClosedLoopBench`; this Windows checkout is
  suitable for source review, audit analysis, rendering diagnostics, and
  focused unit tests.
- Formal CARLA/NuRec/ROS2/TF++ runs require the restored Linux test host and
  its `autodrive` Conda environment. The environment build definition lives in
  `E:\code\env_build`; it installs CARLA 0.9.16, ROS 2 Humble integration, and
  the NuRec runtime layers separately. Do not try to substitute local system
  Python for that runtime.
- The focused visibility test passed locally. Some multimodal tests cannot be
  collected with the bare system Python because `scene_exchange_contracts` is
  not installed there; treat that as an environment-precondition failure, not
  proof that the M8 runtime is correct or incorrect.
- The remote runtime host was reported offline on 2026-07-29. That is a
  historical fact, not a current availability claim. A remote development
  agent must verify its own checkout, branch, exact SHA, and runtime service
  state before use. No local test, generated PNG, or historical evidence may
  be relabeled as a new remote result.

Remote test host (configured out of band; values must not be committed):

- Supply the SSH endpoint and repository locations only through
  `${CLB_REMOTE_SSH_USER}`, `${CLB_REMOTE_SSH_HOST}`, `${CLB_REMOTE_SSH_PORT}`,
  `${CLB_REMOTE_REPO}`, and `${CLB_REMOTE_NSB_REPO}`. Do not write their
  concrete values into source, evidence, prompts, or commits.
- Use the existing SSH authentication on the machine. Never request,
  echo, store, or commit a password or private key.
- The remote runtime checkout is not automatically a development checkout.
  The user may authorize a separate remote development agent; it must first
  verify its absolute checkout path, branch, remotes, and exact source SHA.
- Do not probe, restart, stop, or otherwise operate the runtime host until its
  owner reports it restored. When it returns, recheck NuRec service state
  before use; do not stop or restart it unless explicitly authorized.

## Mandatory development and sync policy

The only normal source flow is:

1. Develop and run focused tests locally.
2. Commit on local `master` or a short-lived feature branch merged locally.
3. Push normally to GitHub: `git push origin master`.
4. On the configured remote checkout, run `git pull --ff-only origin master`.
5. Run remote validation from that exact commit and keep evidence outside the
   tracked source tree.

Rules:

- Never edit source directly on the remote host.
- Never use bundle files for normal source transfer.
- Never use `git reset --hard`, force-push, or overwrite a divergent branch.
- Before any history reconciliation, create dated backup refs and record
  worktree status.
- Do not stage or commit remote runtime directories.
- Preserve remote `.runtime/`, `.sync-backups/`, `incoming/`, outputs,
  caches, containers, checkpoints, and evidence. They are not source-sync
  material and must not be deleted during Git governance.
- Before attributing a remote result to code, require exact SHA equality. Use:
  `python tools/scene0061_sync.py status --repo E:/code/ClosedLoopBench
   --remote-host "${CLB_REMOTE_SSH_USER}@${CLB_REMOTE_SSH_HOST}"
   --remote-repo "${CLB_REMOTE_REPO}"
   --ssh-port "${CLB_REMOTE_SSH_PORT}" --require-equal`
- Do not claim a milestone passed from an RPC success, a video, or
  `collision_count == 0` alone.

Repository policy is also recorded in:
`E:\code\ClosedLoopBench\docs\development_sync_policy.md`.

## M-series design and status

M-series milestones are ordered evidence gates for the global Goal. The Goal
does not replace the milestones.

M1-M5: Phase 1 operational vertical slice. CARLA execution, real ROS2
frame-matched ego-control plumbing, NuRec RGB/LiDAR transaction plumbing,
six-camera diagnostics, TF++ adapter/control-message contracts, short runs,
and portable evidence exist. This establishes integration surfaces, not a
physically consistent scene or an audited TF++ performance result.

M6: full scene-object registry and physical CARLA representation. PASSED.
Every safety-relevant dynamic/static object and the road boundary receive a
registry record and CARLA representation.

M7: same-tick CARLA physical pose to NuRec render-pose binding. Historically
PASSED for the recorded pose-reference contract. The newer
`fix/m8-source-lidar-frame-index` branch contains a pedestrian centre-reference
correction; promotion status is pending review. If that correction changes the
runtime pose origin, rerun M7 and all M8 streams before making any new pass
claim. Per-tick actor pose identity, translation/yaw thresholds, and RGB/LiDAR
A/A/B pose probes are available.

M8.1: runtime collision, lane, and physical-box truth on every tick.
Implemented; the three-tick probe is valid.

M8.2: occlusion-aware expected LiDAR support and LiDAR-to-world audit.
FAILED on the three-tick probe. It blocks formal M8/M9 multimodal promotion,
but does not block independently scoped S1-S3 work.

M8.3: calibrated six-camera geometric visibility. The historical visibility
pass is INVALID: it mixed canonical runtime poses (`x-forward / y-left`) with
unreflected nuScenes camera extrinsics after applying a CARLA y reflection,
which mirrored projected boxes left-to-right. A canonical-frame correction and
corrected manifest exist, but the complete four-stream audit has not yet been
regenerated. Status: REQUIRES RE-AUDIT; do not count it as passed.

M9: three-run TF++ replay baseline in the audited multimodal scene. BLOCKED by
M8. A native-sensor S6 control baseline may be advanced independently once its
own S1-S3 inputs are evidenced, but it is not M9 and must not be renamed as
such.

M10: controlled lead vehicle and controlled pedestrian counterfactuals.
NOT STARTED. Only the predeclared lead vehicle and pedestrian may transition
from replay to scripted/reactive control.

M11: seeded Goal-level experiment matrix and attributable comparison.
NOT STARTED. This is the final M-series gate for the global Goal.

Strict M-series order for an integrated Global Goal claim:

M6 -> M7 -> M8.1 -> M8.2 -> M8.3 -> M9 -> M10 -> M11 -> Global Goal

Do not reinterpret Phase 1 success as M8/M9 success. In particular, a
reference controller proving `algorithm -> ROS2 -> CARLA ego` control is not a
TF++ replay baseline, and NuRec responses from a historical scene are not
LiDAR-world closure for the current physical registry.

The strict M-series order does not prohibit independent S1-S3 work. It only
prohibits promotion to a causally consistent multimodal M9/M10/M11 claim while
an earlier M gate remains unresolved.

The detailed historical Phase 2 definition is preserved at:
`E:\code\.tmp\clb-m8-converged\docs\scene0061_phase2_physical_consistency_milestones.md`
and is available from the local `integration/m8-sync-20260728` branch. It is
supporting material only; this handoff is the authoritative current status
because the old detailed document predates the artifact021 LiDAR failure and
the canonical-frame visibility correction.

## Pinned actor roles and source-quality facts

The only future controlled actors are immutable source tracks:

- lead vehicle: `c1958768d48640948f6053d04cffd35b`
- pedestrian: `71603dd1a2ba4e9daf095535e38310ac`

Their registry roles are `controlled_lead_vehicle` and
`controlled_pedestrian`, but both have runtime mode `replay` until M10. All
other dynamic tracks are `background_replay`; static objects are uncontrolled
collision proxies. Do not create a new controllable actor merely to make a
counterfactual easier.

The current source-quality evidence is material to M8.2:

- The source-catalog audit has 89 dynamic tracks: 12 are
  `raw_lidar_supported`, 72 are sparse, and 5 have no source LiDAR support.
- The lead vehicle has 39 frames and about 7,712 source LiDAR points overall,
  so it is a plausible small-window candidate.
- The protected pedestrian has about 660 points across 37 frames; only 30 of
  those frames have box-internal points and the per-frame median is about four
  points. It must remain present but be explicitly labeled
  `low_lidar_support / rebuild-needed` when its selected window lacks support.
- Counts vary by evidence scope: the M6 one-tick runtime recorded 89 dynamic
  replay actors and 138 static collision proxies, while the current candidate
  planning registry describes 228 records (87 dynamic, 140 static, and one
  road boundary). Artifact021 still requires a source-content path for the 140
  safety-relevant static obstacles; only two have static track IDs and 138 lack
  a source track or static-generation path. Always cite the registry hash and
  its exact record count for the run being discussed; never mix these scopes.

This is why an ego-corridor candidate can lower reconstruction cost, but may
not delete a protected actor, a route-relevant parked vehicle, a barrier, a
cone, or a road boundary to manufacture a pass.

## M8 source workspace and pending correction

The M8 implementation is intentionally isolated from `master` in:

- Worktree: `E:\code\.tmp\clb-m8-converged`
- Branch: `integration/m8-run-converged-20260729`
- Converged baseline: `5d8596118b95ca003da79b7d4c710fa6df34e624`
- Canonical-projection correction: `b98e4b992cf80d8c927f62ff72cc25deb6957394`

The canonical projection correction is already a narrow commit affecting
`adapters/scene_object_visibility.py` and
`tests/test_scene_object_registry.py`. The worktree still has staged changes
to two historical handoff/runbook documents; leave those out of any M8 code
review or follow-up commit. The focused correction test passed with:

`python -m pytest tests/test_scene_object_registry.py -q` (`21 passed`)

The corrected local evidence is:

- `E:\code\ClosedLoopBench\outputs\diagnostics\m8_artifact021_expected_visibility_canonical_frame_fixed.v1.json`
- `E:\code\ClosedLoopBench\outputs\diagnostics\m8_artifact021_tick156072_six_camera_bev_range20m_actor_bound_canonical_frame_fixed.png`

The calibration capture SHA remains source-bound at
`270a6ffe1d6b78340db3947e46fdd02407f7a0bdadc95317ec397da9e474ad3d`.
This correction does not require a CARLA or NuRec reconstruction rerun, but it
does require a fresh visibility audit and a fresh M8 four-stream summary.

The bug was a projection-frame error, not a six-camera panel/layout error:
runtime actor poses were already canonical `x-forward / y-left`, but the old
projection reflected only actor/ego poses into CARLA `y-right` while retaining
original nuScenes camera extrinsics. Do not flip JPEGs or exchange
`camera_front_left` and `camera_front_right`. Their calibrated headings are
approximately `+55.2 deg` and `-56.4 deg`; a left-side target appearing on the
right half of the left-front camera can be geometrically correct. The bug did
not alter raw RGB, TF++ inputs, or CARLA collision state; it altered only the
derived visibility overlay/audit.

### Separate M8.2 source-quality worktree

The executable M8.2 recovery tools are not in the `m8-run-converged`
worktree. They are in a separate, unmerged worktree:

- Worktree: `E:\code\.tmp\clb-m8-source-lidar`
- Branch: `fix/m8-source-lidar-frame-index`
- Current commit: `11d933c35e86022bb0136db6fd095b368aa3a7cb`
- Key implementation: `adapters/lidar_quality_windows.py`,
  `adapters/nurec_reconstruction_smoke.py`, `adapters/m8_promotion.py`,
  `runners/build_lidar_quality_window_manifest.py`, and
  `runners/audit_m8_formal_promotion.py`

This branch is a large candidate change set relative to `m8-run-converged`
(57 files, 5,739 insertions, and 138 deletions). Review and integrate it
deliberately; do not assume a command described below exists in `master` or
`m8-run-converged`. Its focused quality/smoke/promotion test suite passed:

`python -m pytest tests/test_lidar_quality_windows.py tests/test_nurec_reconstruction_smoke.py tests/test_m8_promotion.py -q` (`17 passed`)

There is also a predecessor worktree at
`E:\code\.tmp\clb-m8-control-contract` on
`fix/m8-control-contract-reference`. It carries the pedestrian
centre-reference and aggregate pose-contract changes. Treat it as an explicit
review dependency for the M7 promotion decision, not as an implicit baseline.

Before changing or committing any of these worktrees, run `git status --short`
and `git diff --cached`. The main checkout currently has user-owned untracked
BEV renderer/test files, while the M8 worktrees contain historical-document
changes that must not be swept into a code fix. Never use reset, cleanup, or a
bulk add to make the worktrees appear clean.

### Concrete M8.2 quality-window failure

The failed preflight is:

`E:\code\.remote-evidence\m8_artifact021_r2_static_source_reaudit_20260729\smoke_preflight_v4\lidar_quality_window_manifest.v1.json`

It used `exact>=1`, `padded>=1`, three consecutive frames, and a maximum
100 ms gap. The lead vehicle has a valid window in the old frames near
`0.05/0.10/0.15 s`; the protected pedestrian does not, because its source
lifecycle begins at about `1.050097 s`. The manifest is explicitly
selection-only and declares that quality is not a CARLA-physics filter.

The next source audit must use native NCore frame-end timestamps inside the
pedestrian lifecycle, searching near `1.10/1.15/1.20 s` rather than moving the
old timestamps or lowering the threshold. It must produce a continuous
three-frame protected-track window, fresh dynamic/static source-support
evidence, and the complete registry hash. A failed pedestrian window cannot be
fixed by deleting the pedestrian, converting it to background, or claiming
closure outside the selected window.

If no qualifying protected-pedestrian window exists after that native-timestamp
search, stop the candidate before training and record that Scene-0061/window
cannot satisfy the claimed LiDAR-world closure. Select a different supported
window or source scene; do not force a reconstruction to hide the absence.

The formal boundary is executable and fail-closed: only
`runners/audit_m8_formal_promotion.py` producing
`formal_reconstruction_allowed=true` after a passed quality window, source/
config smoke, artifact hash, all four streams, and one common non-empty frame
set may authorize a formal 40k reconstruction.

## Evidence index and interpretation

Use these local paths before searching for old chat attachments or remote
runtime directories:

| Evidence | Location | What it proves / does not prove |
|---|---|---|
| M6 one-tick full-role gate | `E:\code\ClosedLoopBench\outputs\phase2_m6\scene0061_full_object_roles_nurec_1tick_20260727\nurec_1tick_bound_actor_gate_v2` | registry coverage, CARLA representation, and payload-bound geometry for that gate; not M8 LiDAR-world closure |
| Artifact021 historical four-stream bundle | `E:\code\.remote-evidence\m8_artifact021_r2_static_source_reaudit_20260729` | exact failure evidence for M8.2 and the old visibility result; never a current M8 pass |
| Corrected visibility manifest | `E:\code\ClosedLoopBench\outputs\diagnostics\m8_artifact021_expected_visibility_canonical_frame_fixed.v1.json` | canonical-frame projection correction; it still needs four-stream re-audit |
| Corrected six-camera/BEV diagnostic | `E:\code\ClosedLoopBench\outputs\diagnostics\m8_artifact021_tick156072_six_camera_bev_range20m_actor_bound_canonical_frame_fixed.png` | a debugging visualization of correct left/right geometric placement, not pixel-level detection |
| Reconstruction smoke report | `E:\code\.remote-evidence\m8_artifact021_r2_static_source_reaudit_20260729\smoke_preflight_v2\nurec_reconstruction_smoke.v1.final.json` | why Artifact021 was not a qualified low-cost candidate |

The remote Artifact021 visibility manifests refer to M8 camera JPEG paths and
SHA-256 values, but the corresponding raw M8 `camera_*.jpg` files are not
present locally. Do not draw or claim pixel-level M8 RGB overlays from only the
manifest. M6 has a complete local six-camera payload set and can be used to
test visualization code, while M8 RGB proof requires the source JPEGs to be
returned with their hashes.

The M8 safety writer has four required streams:

1. `collision_audit`: CARLA ego contact events and minimum physical clearance.
2. `lane_audit`: waypoint/lane membership, invasions, lateral position, and
   route progress.
3. `visibility_audit`: source-calibrated CARLA 3D-box projection into the six
   NuRec cameras, bound to payload identity. It is geometry coverage, not an
   independent image-detector claim.
4. `lidar_world_audit`: same-tick expected scannable support and occupancy in
   the physical CARLA boxes, including relevant static objects and boundary.

All four must exist and pass on every required tick. An independent RGB
detector is explicitly deferred until after M11; installing one cannot repair
or promote M8.

The same physical collision contract is now required for ordinary CARLA
`acceptance_evidence` runs, even when the four-stream M8 audit is disabled.
Those runs capture ego/object physical bounding boxes and native-frame collision
payloads in `frame_trace.jsonl`, write `collision_audit.v1.jsonl`, and fail
closed on an OBB overlap without an attributed callback, an unregistered
callback, missing scene-object registry, or an unmatched native frame. A zero
`collision_count` or a historical video is not sufficient evidence.

## Physical-road and map boundary

The current canonical `road.xodr` is a local derivative of nuScenes lane
polygons: each selected `CAR` lane becomes an OpenDRIVE road, ambiguous
transitions use explicit junction connector roads, and the recorded Ego route
declares a mixed path
`2001(source-gap) -> 44(map lane) -> 1133(map connector) -> 7(map lane) ->
1053(map connector) -> 33(map lane)`. Only `2001` carries explicit
`synthetic_reference_trajectory` source-gap metadata; the other five
route-path roads use source map geometry. The mixed path covers all 39 scene
Ego samples, but the map centerline alignment and CARLA waypoint continuity
remain runtime gates. The separate one-road `ego_route_corridor` is
diagnostic-only and is not part of the canonical exchange default. The
selected local lane network still has boundary components, so this is
structural topology and route continuity evidence, not a claim of a complete
or physically verified CARLA road network.

The static BEV renderer can instead draw direct nuScenes `drivable_area`,
`road_segment`, intersection, and lane polygons. That background is explicitly
`nuscenes_map_geometry_visual_only`: it improves human interpretation but is
not CARLA physics, CARLA lane truth, or collision evidence. CARLA waypoint
membership, lane-invasion events, collision events, and repeated replay remain
separate physical acceptance evidence.

Before treating the generated world as physically road-faithful, complete this
remediation gate:

1. Expand/verify the selected local lane network and junction, road, and lane
   links at the required CARLA map boundary rather than accepting disconnected
   lane strips as a full map.
2. Generate a fresh CARLA world from that map and validate waypoint continuity
   and route connectivity at every relevant branch/intersection.
3. Run lane-invasion fault injection and repeated replay, retaining the CARLA
   collision and lane evidence for the exact tested source revision.

Do not retroactively upgrade historical M7/M8 evidence merely because the BEV
background becomes visually continuous; the physical map gate is independent.

## Cross-cutting observability (O1)

The combined CARLA-state/BEV plus six-camera Pygame view is an observability
tool, not a milestone or sensor-closure substitute. It must join one
`FramePacket` by common frame ID and timestamp so that its BEV ego/actor boxes,
lane context, and six NuRec images refer to the same instant.

- Existing integrated viewer: `runners/run_scene0061_dual_window.py`.
- The six-camera layout is `front_left / front / front_right` above
  `back_left / back / back_right`; do not infer calibration correctness from
  the panel order alone.
- The main checkout currently has untracked BEV snapshot renderer/test files.
  They are user-owned visual tooling, must be preserved, and are not yet part
  of an M8 commit or a formal acceptance claim.
- A 20 m draw range, category colors, 2D/3D box outlines, map polygons, and
  screenshots improve debugging only. They do not reduce the registry/audit
  scope, prove camera occlusion, prove LiDAR occupancy, prove collision
  absence, or prove TF++ inference.

Use O1 to diagnose frame, pose, box, and camera-calibration mistakes. Preserve
frame/timestamp/payload-hash metadata next to any screenshot. A local or
synthetic screenshot must never be relabeled as a remote closed-loop result.

## M8 current evidence

Historical formal audit directory:
`E:\code\.remote-evidence\m8_artifact021_r2_static_source_reaudit_20260729`

Manifest:
`E:\code\.remote-evidence\m8_artifact021_r2_static_source_reaudit_20260729\manifest.v1.json`

Historical four-stream status (not a current M8 pass):

- collision: passed on 3/3 frames
- lane: passed on 3/3 frames
- calibrated visibility: INVALID because of the canonical-frame mirror bug;
  the old result must be regenerated from the corrected projection
- LiDAR-world: failed on 3/3 frames

LiDAR-world failure counts in the manifest are 37, 33, and 32 issues by
frame; static source/CARLA conflicts are 4, 4, and 3; dynamic missing NuRec
occupancy counts are 22, 19, and 19. These counts can overlap and must not be
added as independent totals.

Artifact021 source smoke evidence:
`E:\code\.remote-evidence\m8_artifact021_r2_static_source_reaudit_20260729\smoke_preflight_v2\nurec_reconstruction_smoke.v1.final.json`

The smoke report failed because:

- formal training uses `n_samples_per_epoch=40000`, outside the low-cost smoke
  budget;
- `generate_static_rigid_cuboid_tracks.enabled=false`;
- 87 dynamic registry tracks match source IDs, but static representation is
  incomplete;
- 140 safety-relevant static obstacles are required, only 2 have static track
  IDs, and 138 have no source track or generation path.

The NCore diagnostics show 87 selected dynamic tracks, but 85 are sparse and
only 2 are supported for the relevant LiDAR audit. Aggregate track presence
over a long sequence is not enough; the exact same-tick evaluation frames
must contain usable source LiDAR occupancy.

Previous targeted attempts also exposed layer/source mismatches: vehicles
must not be put in a pedestrian-only deformable layer, and the selected source
is `EXTERNAL`, not `AUTOLABEL`. Post-hoc edits to `sequence_tracks.json` do
not create movable Gaussian or LiDAR content.

Do not interpret this as a TF++ failure. M8 is testing scene representation
and causal sensor/physics consistency before algorithm evaluation.

## Algorithm, ROS2, and TF++ status

Phase 1 established a real control boundary: an external plugin can consume a
current-tick observation, send a frame-matched control through the ROS2 adapter,
and apply it to CARLA ego. Reference/pure-pursuit controllers are evidence that
this boundary can control the vehicle; they are not perception-model results.

TF++ has retained integration assets for the six-camera/LiDAR observation
contract, control-message binding, runtime/container configuration, CUDA
preflight, and S0/S2/S4 bundle construction. It must be described as an
integrated algorithm boundary, not as a currently accepted M9 benchmark:

- M9 has no valid three-run audited replay baseline because M8 is not closed.
- The currently visible parked vehicle or other road-side object cannot be used
  to judge "why TF++ collided" until it has matching CARLA collision authority,
  source-supported NuRec content, and same-tick pose/calibration evidence.
- Historical M5-style videos that show contact or lane departure are debugging
  evidence only. A zero collision counter from those runs is not trustworthy if
  the visible obstacle lacked a CARLA collider or the road topology was
  incomplete.
- When M9 becomes eligible, the lead vehicle and pedestrian remain replayed;
  the exact same audited six-camera/LiDAR transaction drives TF++, and three
  independently seeded runs must retain algorithm identity, checkpoint/CUDA
  evidence, frame-matched controls, collision/lane results, and LiDAR-world
  results.

Do not start a model-performance investigation, tune a collision threshold, or
introduce a detector as a workaround for M8. The current question is whether
the scene itself can provide a causally consistent sensor and physics contract.

## M8.2 repair path: quality-qualified small editable window

Do not start another all-track formal 40k reconstruction. The first repair
candidate is a small editable window around the ego trajectory. This reduces
cost, but does not relax physical truth: the full scene-object registry and
CARLA collision/lane checks remain active, and protected actors or any object
in the ego risk corridor may not be silently dropped.

1. Create a no-training selection manifest:
   - Select an exact three-or-more-tick window by ego corridor, route overlap,
     and sensor range.
   - Include the protected lead vehicle and pedestrian whenever they overlap
     that window, plus all static obstacles and road boundaries that can affect
     the ego route.
   - Keep ordinary far-background tracks out of dynamic reconstruction only
     when the manifest explicitly marks them outside the editable/safety
     envelope.

2. Grade source support per selected object and tick:
   - Record box-internal LiDAR point count, nonzero-frame ratio, consecutive
     coverage, and distance bucket.
   - The lead vehicle has usable overall support; the protected pedestrian is
     sparse and must be marked `low_lidar_support / rebuild-needed`, never
     silently excluded.
   - For every route-relevant static object, identify a valid source track,
     static cuboid-generation path, or source point-cloud/map geometry.

3. Correct only a source-supported candidate configuration:
   - Keep `track_label_sources: [EXTERNAL]`.
   - Map vehicles and pedestrians to the correct dynamic layers.
   - Enable static cuboid generation only when valid source geometry exists;
     configuration flags cannot invent missing point-cloud content.

4. Low-cost content smoke:
   - Run the source/config smoke with a <=1000-step budget.
   - Probe one vehicle, one pedestrian, one static obstacle, and the road
     boundary in the exact target window.
   - Require RGB A/A/B content change, LiDAR A/A/B content change, and points
     inside corresponding CARLA physical boxes.
   - Reject empty source point clouds, `lidar_render_unchanged`, missing static
     content, or source/CARLA frame disagreement before formal training.

5. Formal reconstruction and M8 exit:
   - Only after the candidate passes, run the formal reconstruction and retain
     artifact, parsed config, launcher log, source manifests, and hashes.
   - Re-run CARLA/NuRec for at least three consecutive ticks and regenerate all
     four streams. M8 passes only when collision, lane, corrected calibrated
     visibility, and LiDAR-world pass for every required row.

No axis permutation, tolerance relaxation, detector installation, or
successful gRPC response can substitute for source LiDAR/world support.
CARLA collision meshes do not automatically become NuRec USDZ/LiDAR content.

## First actions for the next agent

1. Read this prompt and `docs/development_sync_policy.md`. If operating as
   the user-authorized remote development agent, treat the two source SHAs in
   the current source baseline as required starting points; do not reconstruct
   or silently replace them.
2. Record `git rev-parse --show-toplevel`, `git remote -v`, `git branch
   --show-current`, `git rev-parse HEAD`, `git status --short`, and
   `git diff --cached` in every active worktree before editing. Preserve the
   main-checkout BEV files and the two staged historical documents in the M8
   worktrees.
3. Confirm that the remote development checkout is not the runtime-only test
   checkout. Keep source changes in Git, and keep runtime outputs, caches,
   checkpoints, credentials, and evidence outside tracked source.
4. Run focused tests for the newly committed source in the correct environment
   before beginning a new repair. For ClosedLoopBench, include
   `tests/test_opendrive_contract.py`,
   `tests/test_nuscenes_topology_exchange.py`,
   `tests/test_nuscenes_topology_opendrive.py`,
   `tests/test_route_aligned_opendrive.py`,
   `tests/test_scene_safety_audit.py`,
   `tests/test_esmini_xodr_runtime_audit.py`, and
   `tests/test_carla_xodr_runtime_audit.py`. For NeuralSceneBridge, include
   `tests/test_nurec_dynamic_tracks.py`,
   `tests/test_nurec_scene0061_recipes.py`,
   `tests/test_validate_nurec_artifacts.py`,
   `tests/test_validate_nurec_usdz_tracks.py`, and
   `tests/test_derive_nurec_controllable_tracks_usdz.py`. Report missing
   dependencies as preconditions, not as pass or fail evidence.
5. Inspect the Artifact021 manifest, smoke report, corrected visibility
   manifest, and four-stream files locally. Review committed correction
   `b98e4b9`; do not claim M8.3 or M8 passed and do not exchange camera sides.
6. Review `fix/m8-source-lidar-frame-index` against
   `integration/m8-run-converged-20260729`, including the pedestrian
   centre-reference dependency. Run its focused quality/smoke/promotion tests
   before deciding a narrow integration sequence; do not assume its tools are
   present on `master`.
7. Build a native-NCore-timestamp ego-corridor plus LiDAR-quality selection
   manifest. It must cover both protected tracks inside their source
   lifecycles, preserve the full CARLA registry, and identify static geometry
   paths before any NuRec configuration or training is edited.
8. Advance independent S1-S3 work with isolated fixtures when useful, but keep
   M8/M9/M10/M11 promotion claims separate. Keep every M8 source change on a
   feature branch; do not merge a formal M8 claim into `master` until its
   complete exit gate passes.
9. For a remote development session, follow focused tests -> focused commit ->
   normal GitHub push -> remote checkout update -> exact-SHA verification. Do
   not use `git reset --hard`, force-push, source bundles, or bulk-add local
   runtime directories. Only the formal promotion gate may authorize a 40k
   reconstruction.
```
