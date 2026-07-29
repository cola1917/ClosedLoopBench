# ClosedLoopBench Agent Handoff Prompt

Copy the block below to the next agent as the authoritative handoff. The
prompt intentionally contains no password or private key material.

```text
You are taking over the ClosedLoopBench project. Respond to the user in
Chinese, but keep commit names, paths, commands, and evidence identifiers
exactly as written.

## Scope and current baseline

The global objective is to make ClosedLoopBench evaluate an external driving
algorithm in one causally consistent CARLA/NuRec scene. Every safety-relevant
object visible to the algorithm through NuRec RGB/LiDAR must correspond to a
same-time CARLA object with the same pose, lane state, and collision boundary.
The lead vehicle and one pedestrian are the only actors that may eventually
become controlled counterfactuals; all other context remains replay-only.

Current source baseline:

- Local checkout: `E:\code\ClosedLoopBench`
- Current branch: `master`
- Current verified commit after endpoint-history redaction:
  `92df4df040790daca4d38ef00baed4f4b5a2f79a`. Always run `git rev-parse HEAD`
  and compare all three `master` SHAs before testing.
- GitHub remote: `origin` (`git@github.com:cola1917/ClosedLoopBench.git`)
- Local `master`, GitHub `origin/master`, and remote `master` were verified
  equal at commit `215979e` on 2026-07-29.
- M8 experiment branches are deliberately not merged into `master`.

Remote test host (configured out of band; values must not be committed):

- SSH endpoint: `${CLB_REMOTE_SSH_USER}@${CLB_REMOTE_SSH_HOST}`, port
  `${CLB_REMOTE_SSH_PORT}`
- ClosedLoopBench checkout: `/home/cwadmin/workspace/ClosedLoopBench`
- NeuralSceneBridge checkout: `/home/cwadmin/workspace/NeuralSceneBridge`
- Use the existing SSH authentication on the machine. Never request,
  echo, store, or commit a password or private key.
- The remote checkout is a test checkout, not a development checkout.
- The last known NuRec `serve-grpc` process listens on localhost port `46455`.
  Recheck before use; do not stop or restart it unless explicitly authorized.

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
   --remote-repo /home/cwadmin/workspace/ClosedLoopBench
   --ssh-port "${CLB_REMOTE_SSH_PORT}" --require-equal`
- Do not claim a milestone passed from an RPC success, a video, or
  `collision_count == 0` alone.

Repository policy is also recorded in:
`E:\code\ClosedLoopBench\docs\development_sync_policy.md`.

## M-series design and status

M-series milestones are ordered evidence gates for the global Goal. The Goal
does not replace the milestones.

M1-M5: Phase 1 operational vertical slice. CARLA, NuRec, ROS2/TF++, short
runs, and portable evidence exist, but these records are not a physical or
safety acceptance.

M6: full scene-object registry and physical CARLA representation. PASSED.
Every safety-relevant dynamic/static object and the road boundary receive a
registry record and CARLA representation.

M7: same-tick CARLA physical pose to NuRec render-pose binding. PASSED.
Per-tick actor pose identity, translation/yaw thresholds, and RGB/LiDAR A/A/B
pose probes are available.

M8.1: runtime collision, lane, and physical-box truth on every tick.
Implemented; the three-tick probe is valid.

M8.2: occlusion-aware expected LiDAR support and LiDAR-to-world audit.
FAILED on the three-tick probe. This is the current blocker.

M8.3: calibrated six-camera geometric visibility. Geometry evidence exists
and does not require an independent detector. The latest four-stream bundle's
visibility stream passes, but M8 as a whole remains failed until M8.2 passes.

M9: three-run TF++ replay baseline in the audited scene. BLOCKED by M8.

M10: controlled lead vehicle and controlled pedestrian counterfactuals.
NOT STARTED. Only the predeclared lead vehicle and pedestrian may transition
from replay to scripted/reactive control.

M11: seeded Goal-level experiment matrix and attributable comparison.
NOT STARTED. This is the final M-series gate for the global Goal.

Strict order:

M6 -> M7 -> M8.1 -> M8.2 -> M8.3 -> M9 -> M10 -> M11 -> Global Goal

The full milestone definition is available from the preserved M8 branch with:
`git show refs/heads/integration/m8-sync-20260728:docs/scene0061_phase2_physical_consistency_milestones.md`

## M8 current evidence

Latest formal audit directory:
`E:\code\.remote-evidence\m8_artifact021_r2_static_source_reaudit_20260729`

Manifest:
`E:\code\.remote-evidence\m8_artifact021_r2_static_source_reaudit_20260729\manifest.v1.json`

Four-stream status:

- collision: passed on 3/3 frames
- lane: passed on 3/3 frames
- calibrated visibility: passed on 3/3 frames
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

## M8 first repair path: source-complete reconstruction

The preferred repair is to rebuild a source-complete NuRec artifact. Do not
start another formal 40k run until the cheap gates below pass.

1. Source audit, no training:
   - For all 87 dynamic tracks, inspect per-frame source LiDAR support in the
     exact candidate evaluation window.
   - For all 140 static obstacles, identify a real source track, static
     cuboid-generation path, or source point-cloud/map geometry.
   - Verify the road-boundary representation and timestamp/world-frame chain.
   - If the source window cannot support an object, choose a source window
     that does or explicitly document that the original scene cannot close.

2. Correct the NuRec configuration:
   - Keep `track_label_sources: [EXTERNAL]`.
   - Select the NCore-eligible 87 IDs from the source manifest.
   - Map vehicle and pedestrian classes to the correct dynamic layers.
   - Enable static cuboid generation or provide a valid source static
     point-cloud/track path for every required static object.
   - Do not assume enabling a flag invents geometry when the source has no
     points.

3. Low-cost content candidate:
   - Run the source/config smoke with a <=1000-step budget.
   - Probe one vehicle, one pedestrian, one static obstacle, and the road
     boundary in the exact target time window.
   - Require RGB A/A/B content change, LiDAR A/A/B content change, and points
     inside the corresponding CARLA physical boxes.
   - Reject the candidate on empty source point clouds,
     `lidar_render_unchanged`, missing static content, or source/CARLA frame
     disagreement.

4. Formal reconstruction:
   - Only after the candidate passes, run the full 40k reconstruction.
   - Preserve the artifact, parsed config, launcher log, source manifests,
     and hashes as immutable evidence.

5. Same-tick M8 run:
   - Re-run CARLA/NuRec for at least the required three consecutive ticks.
   - Regenerate all four audit streams from the new artifact.
   - M8 passes only when collision, lane, calibrated visibility, and
     LiDAR-world all pass for every required row.

No axis permutation, tolerance relaxation, detector installation, or
successful gRPC response can substitute for source LiDAR/world support.
CARLA collision meshes do not automatically become NuRec USDZ/LiDAR content.

## First actions for the next agent

1. Read this prompt and `docs/development_sync_policy.md`.
2. Verify local and remote `master` SHA equality before any remote test.
3. Inspect the latest artifact021 manifest, smoke report, and four-stream
   files; do not restart training or stop `serve-grpc`.
4. Audit the source dataset window and static-object generation path before
   editing configuration.
5. Keep all M8 changes on a feature branch. Do not merge into `master` until
   the complete M8 exit gate passes.
6. After every local source change: focused tests -> commit -> GitHub push ->
   remote `pull --ff-only` -> exact-SHA verification -> remote evidence.
```
