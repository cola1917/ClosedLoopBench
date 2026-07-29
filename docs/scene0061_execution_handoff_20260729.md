# Scene-0061 Current Execution Handoff (2026-07-29)

This is the current handoff for the ClosedLoopBench / NeuralSceneBridge work.
It supersedes the execution instructions in older handoff documents, while
those documents and their evidence remain historical records.

## Current source state

- Worktree: `E:/code/.tmp/clb-m8-source-lidar`
- Branch: `fix/m8-source-lidar-frame-index`
- Current commit: `bd5f2e1 docs: decouple goal into capability stages`
- GitHub remote: `origin` is the repository configured in the local clone.
- The normal development rule is: edit and test locally, commit locally, push
  to GitHub, then let the remote workspace fetch/pull the exact commit for
  runtime validation.
- Do not make source edits directly on the remote host. Runtime outputs,
  caches, `.runtime/`, `incoming/`, and evidence directories are not source
  branches and must not be deleted as part of source synchronization.

The remote VM is currently unavailable. Do not repeatedly probe or restart it.
When it returns, first verify its reachability and repository state read-only;
do not assume that an old remote checkout is current.

## Planning authority

Read these files in this order:

1. `docs/global_goal_capability_milestones.md`
2. `docs/closed_loop_execution_ladder.md` (legacy dependency view only)
3. `docs/scene0061_phase2_physical_consistency_milestones.md` (historical M
   evidence and detailed M8 failure record)
4. `docs/m8_candidate_recommendation.md` (deferred recovery path)
5. this document

The current plan is capability-independent. Do not resume the old strategy of
making the entire project wait for a formal NuRec reconstruction.

## New stage model

Each stage is an independent product with its own fixture, immutable evidence,
and report. Stages exchange artifacts through contracts, not internal state.

| Stage | Product | Current state |
|---|---|---|
| S0 | Scene, tick, control, and evidence contracts | `offline_ready` |
| S1 | CARLA physical deterministic replay and raw truth | `partial`; registry/one-tick probes exist, three-run evidence is missing |
| S2 | Trace-based collision/lane/TTC/progress/comfort evaluator | `offline_ready`; live triplicate/fault bundle pending |
| S3 | Ego control plugin, safe-stop, native control, ROS2 boundary | lifecycle/safe-stop `offline_ready`; live CARLA/ROS2 pending |
| S4 | NuRec RGB/LiDAR transport and world consistency | `partial`; LiDAR-world consistency currently failed |
| S5 | Lead-vehicle and pedestrian interactive actors | schemas/offline plans exist; live acceptance not started |
| S6 | Integrated algorithms, counterfactual matrix, Global Goal report | not started |

The important independence rule is: S4 failure does not fail S1, S2, or S3.
S6 integration is the only stage allowed to require multiple stage gates.

## TF++ clarification

TF++ has a separate status from NuRec/M8. The adapter and ROS2/container
boundary can be complete even when the real checkpoint has not run. Classify
the evidence as follows:

| Evidence observed | Classification |
|---|---|
| adapter, module, lifecycle, and ROS2 contract tests | S3 implementation |
| real repository/checkpoint/config/image hashes bound | S3 runtime-ready |
| real checkpoint produces frame-matched controls in CARLA with native sensors | S3.M5 passed / S6.M1 candidate |
| same controls use paired NuRec RGB/LiDAR transactions | S4.M5 + S6.M2 |
| three repeated TF++ replay runs share the evaluator and have no unclassified fallback | S6.M3 / historical M9 |

Do not infer TF++ execution from `algorithm_id=transfuserpp_v5` alone. The
runtime report must identify the actual ego driver/backend, checkpoint hash,
accepted controls, frame identity, and fallback count. Existing local closeout
evidence explicitly remains `real_checkpoint_loaded=false`; existing remote
M8 reports also contain `basic_agent` or `topology_follower` driver records.
Those are not proof of a TF++ inference run. This means the current defensible
state is: **TF++ integration boundary prepared, real TF++ replay baseline not
yet evidenced**.

## M-series mapping and status

Historical M numbers remain stable and must not be retroactively relabelled:

| Historical milestone | New stage | Status |
|---|---|---|
| M1-M5 | S0/S3 operational slice | local evidence retained; not full physical acceptance |
| M6 | S1 registry/physical presence | one-tick scope passed; repeated replay gate pending |
| M7 | S4 pose binding | must rerun after centre-reference correction |
| M8.1 | S1/S2 collision and lane truth | probe valid; independent repeat gate pending |
| M8.2 | S4 LiDAR-world consistency | failed; formally deferred |
| M8.3 | S4 RGB geometry | geometry evidence exists; no detector claim |
| M9 | S6 TF++ replay baseline | integration boundary prepared; real three-run TF++ evidence not yet established |
| M10 | S5 interactive lead/pedestrian | not started for live acceptance |
| M11 | S6 Goal experiment matrix | not started |

## M8.2 decision

M8.2 is **deferred, not deleted**. Do not start another formal 40k rebuild now.
The retained attempt and failure evidence are diagnostic and immutable. The
current failure is object-level LiDAR/source-content/world support: collision,
lane, and calibrated RGB geometry passed in the retained probe, while
LiDAR-world support failed for expected objects.

Before resuming M8.2, complete the independent prerequisites:

1. S1 replay trace with full physical registry, including the roadside parked
   vehicle and road boundary.
2. S2 evaluator and fault-injection checks on the same trace contract.
3. S3 native control/no-op smoke, proving frame-matched control and safe-stop.
4. S4 low-cost RGB and LiDAR transport probes, including native timestamps.
5. A valid protected-track window for the lead vehicle and pedestrian, with the
   pedestrian lifecycle actually covered by the selected frames.
6. Candidate source/config smoke before any formal reconstruction.

Candidate render selection may be limited to an ego corridor and quality-
eligible objects for cost control, but it must not reduce the CARLA registry or
remove collision bodies. Sparse protected tracks are marked low-support, not
silently converted to background.

Formal M8 promotion requires one candidate artifact, one registry identity,
one common non-empty frame set, and all four streams passing:

- collision;
- lane;
- calibrated RGB visibility;
- occlusion-aware LiDAR-world support.

Only `formal_reconstruction_allowed=true` authorizes a formal 40k run.

## Local evidence already available

- `docs/global_goal_capability_milestones.md`: current stage plan and status.
- `docs/m8_candidate_recommendation.md`: M8 recovery and candidate rules.
- `docs/scene0061_phase2_physical_consistency_milestones.md`: detailed M6-M8
  evidence, including the failed LiDAR-world audit.
- `docs/algorithm_plugin_contract.md`: offline conformance and safe-stop rules.
- `docs/core_closed_loop_integration.md`: CARLA/ROS2 runtime ownership.
- `docs/closed_loop_execution_ladder.md`: legacy integrated ordering.

The current branch's focused plugin/evaluator regression set passes (`18
passed`). A full suite may require the sibling `SceneExchangeContracts`
dependency and is not evidence of live CARLA/NuRec acceptance.

## Next actions when remote returns

1. Confirm local branch is clean and push the exact local commit to GitHub.
2. On the remote, fetch and check out the exact commit; record commit and
   working-tree status before running anything.
3. Run S1 replay smoke and S2 evaluation first. Save new immutable evidence
   directories; do not overwrite old M8 outputs.
4. Run S3 no-op/native control smoke and verify the same evaluator consumes its
   trace.
5. Run bounded S4 RGB/LiDAR transport smoke. Only if S4 prerequisites pass,
   resume the deferred M8.2 candidate path.
6. After M8 promotion, proceed to S6 TF++ replay, then S5 interactive actors
   and the Goal matrix as evidence permits.

## Non-negotiable rules

- No password, private key, or public remote address belongs in source,
  prompts, logs, commits, or new handoff files.
- Never call offline replay `real_carla_nurec_closed_loop`.
- Never treat a non-empty LiDAR RPC, zero collision count, or video as proof of
  world consistency.
- Never delete old runtime/output/cache/evidence to make a stage green.
- Never change registry scope to hide an expected collision or missing sensor
  occupancy.
- If a stage fails, fix that stage's contract or evidence producer; do not
  rewrite unrelated stage statuses.
