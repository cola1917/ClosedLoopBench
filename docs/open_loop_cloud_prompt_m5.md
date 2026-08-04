# Cloud / Automation prompt: open-loop through M5

Paste the block below into a **Cloud Agent** run or **Automation** instruction.
Repo: `cola1917/ClosedLoopBench` · Branch: `feat/open-loop-multimodal`

Before starting Cloud: ensure this branch is **committed and pushed**. Cloud does not see dirty local files.

---

## Prompt (copy from here)

```text
You are continuing ClosedLoopBench open-loop multimodal evaluation work.

## Goal (stop only when M5 is done)
Implement milestones M1 → M5 on branch `feat/open-loop-multimodal` until the M5 exit criteria below are all satisfied. Do not stop after partial progress. After each milestone, run the narrowest relevant tests, commit with a clear message, and push the branch.

Canonical plan (read first, obey claim boundaries):
- `docs/open_loop_multimodal_eval.md` (especially §2 claim boundary, §4 pins, §7.1 M1–M5)

## Hard rules (fail-closed)
1. `evidence_classification` for formal outputs must be `open_loop_multimodal`.
2. NEVER claim M8, M9, Goal closed-loop, or NuRec LiDAR–world causal closure.
3. Every report / run config must include:
   - `control_affects_next_ego_pose: false`
   - `claims_m8: false`
   - `claims_m9: false`
4. Pin artifacts (verify SHA, fail if mismatch):
   - IR: `outputs/scene0061_exchange_v2/scene_ir.json`
     sha256 `754a48f2a8eff3878229d3c6f80d0912bdd00016c86c77d5d295fc8f51e418d0`
   - XODR: `outputs/scene-0061/road.xodr`
     sha256 `46e759ff00aff53b489b175822c33c7e03dc1f78d93c287b538d9b70801273a4`
5. Do NOT merge or rewrite dormant M8 experiment branches.
6. Prefer extending existing runners/adapters/ROS/TF++ plumbing; no new repo.
7. Stage A sensors for M5 = CARLA native RGB + LiDAR at GT poses (NuRec is M6, out of scope for this run).

## Milestone order (complete sequentially)

### M1 — open_loop_gt_replay skeleton
- Add runner mode/flag `open_loop_gt_replay` (or equivalent) on the existing CARLA path.
- Enforce IR/XODR path + SHA checks.
- Encode `control_affects_next_ego_pose=false`.
Done when: test proves after predict_control, next ego pose equals IR sample N+1 (control cannot own pose).

### M2 — GT teleport + actor replay smoke
- Each tick: ego set_transform from IR; actors replay IR trajectories.
- Stub/null control sink OK; no TF++/NuRec required.
Done when: short scene-0061 run log binds frame_id ↔ IR t_sec/pose with near-zero ego pose error vs IR.

### M3 — metrics v0
- ADE/FDE + lateral/heading vs IR; latency/drop counters.
- Report includes §9 identity block from the plan doc.
Done when: offline JSON report from stub/synthetic predictions validates; claims_m8/m9 false.

### M4 — local ROS boundary
- Reuse `ros2_observation_control` contract; publish GT-pose observations.
- Pure Pursuit or stub plugin consumes ROS; control still must not own next ego pose.
Done when: matched frame_id obs→control trace with zero scored mismatches on smoke.

### M5 — TF++ Stage A (THIS RUN’S FINISH LINE)
- Wire TF++ (existing compose/backend) to CARLA `camera_front` + `lidar_top` at GT poses.
- Dump intermediates + hashes; score ADE from TF++ waypoints; sync gates.
Done when ALL are true:
  [ ] TF++ model inputs are camera_front + lidar_top only
  [ ] Intermediate hashes recorded
  [ ] ADE report generated from TF++ waypoints
  [ ] Run report has evidence_classification=open_loop_multimodal and §9 fields
  [ ] Relevant unit/integration tests pass
  [ ] Changes committed and pushed on feat/open-loop-multimodal

## Out of scope for this run
- M6 NuRec Stage B
- M7 triplicate formal acceptance
- M8/M9 closed-loop work
- Deleting or merging M8 branches

## Working style
- Read the plan doc before coding.
- Small commits; one milestone intent per commit when practical.
- If blocked (missing checkpoint mount, no GPU, ROS not in cloud VM), document the blocker in the PR/branch notes, finish everything that does not require that host resource, and leave a precise handoff for the remote xt167-style host — but still complete all cloud-feasible M1–M5 code + tests.
- End with a short status: which milestones done, test commands run, artifact paths, and explicit “M5 complete: yes/no”.
```

---

## How to use

### Cloud Agent (one long push)
1. Commit + push any pending doc/code on `feat/open-loop-multimodal`.
2. Open Cloud Agent → select repo `ClosedLoopBench` → branch `feat/open-loop-multimodal`.
3. Paste the prompt above.
4. Optional: ask it to open/update a draft PR when M5 lands.

### Automation (recurring until M5)
- Trigger: daily cron or “push to `feat/open-loop-multimodal`”.
- Instructions: same prompt, plus first line:
  `If M5 exit criteria are already satisfied on HEAD, do nothing and report M5 complete.`
- Stop/disable the automation manually after M5 is green.

### Local follow-up (when Cloud lacks CARLA/ROS/GPU)
On xt167-style host, run only the live smoke pieces for M2/M4/M5 using the code Cloud already pushed; do not re-implement.
