# M6 Debug Log (scene-0061)

Date: 2026-08-05

This log records the M6 Stage B debugging path. The old debug outputs are
removed after the full-run passed; the final evidence and the failure causes
remain documented here.

## Debug timeline

| Attempt or change | Observation | Resolution |
| --- | --- | --- |
| `m6-inputs` through `m6-inputs-v4` | Iterative input and scene-package bindings during environment preparation. | Pinned the final runtime-validated input set as `m6-inputs-v5`. |
| Stage B smoke `r6` | The first formal inference took `567.750834 ms`, above the `500 ms` plugin timeout. The run ended with `fallback_count=1` and only two predictions. | Added a same-process CUDA/model warm-up before formal frame 0. Warm-up iterations do not consume scored frames or write intermediates. |
| Runtime/plugin identity check | The declared plugin identity could disagree with the checkpoint evidence observed by the health path. | Made plugin identity use the actual health evidence and carried that evidence into the run report. |
| Container image overlay | The image used for a run did not contain the latest plugin/runtime overlay. | Rebuilt the overlay image with the current plugin, runtime, and runner files. The final image is `closed-loop-bench/transfuserpp-v5:m6-stage-d`. |
| Stage B smoke `r7` | Payload paths were emitted under `/sim-data/payloads`, while the run mount was `/sim-data/<run>/payloads`. | Passed an explicit `--container-root` so paths resolve under the run-specific mount. |
| Repeated smoke runs `r8` through `r10` | The same run identity could cause intermediate outputs from repeated attempts to overwrite each other. | Added `--run-id-suffix` and kept each attempt isolated. These smoke outputs are now superseded by the full run. |
| Full run `r11` | All 39 formal frames completed with real checkpoint evidence and no fallback. | Accepted as the sole M6 Stage B result. |

## Final result

- Run: `scene-0061-open-loop-m6-full39`
- Status: `execution_status=completed`
- Formal inference: `39/39` frames, `39/39` intermediates, `fallback_count=0`
- Synchronization: `39/39` matched, `0` dropped, `0` mismatched
- Sensors: 6 RGB cameras plus `lidar_top` on every frame
- NuRec API: `SensorsimService/26.04` (`26.4.146`)
- Dynamic actors: `0`; no CARLA dynamic actors are created
- Ego pose: Scenario IR reference trajectory; TF++ control does not affect the next pose
- Collision result: offline IR actor-trajectory proxy only
- Claim boundary: `claims_m8=false`, `claims_m9=false`
- Formal latency: mean/p95/max `158.54/183.77/235.76 ms`

Final artifact hashes:

| Artifact | SHA-256 |
| --- | --- |
| `outputs/scene0061-transfuserpp/runtime/m6_stage_b_report.full39.json` | `b6f03ecf25019e4ac08538a6eddf78e769dff9fa2db04ec65ba0ce494dc81894` |
| `outputs/scene0061-transfuserpp/runtime/transfuserpp.m6.stage-b.runtime.v5.json` | `748fc75080aa4d0ca28b517153c801e2566a60ba774fbbbf178eb556b5c7345b` |
| `outputs/scene0061-transfuserpp/m6-stage-b-full-r11/nurec_stage_b_observations.json` | `d6c61100f4a1b940e0d9f7006dfa9935d0428a1d0bfc504dcfad80316fda8b53` |
| Docker image `closed-loop-bench/transfuserpp-v5:m6-stage-d` | `sha256:d5f814b8aab88bbef08e70a4f915771658221190d2501e2894815977d3db6394` |

## Cleanup disposition

Retained as the final M6 set:

- `outputs/scene0061-transfuserpp/inputs/` (pinned Scenario IR/XODR inputs)
- `outputs/scene0061-transfuserpp/m6-inputs-v5/`
- `outputs/scene0061-transfuserpp/m6-stage-b-full-r11/`
- `outputs/scene0061-transfuserpp/runtime/m6_stage_b_report.full39.json`
- `outputs/scene0061-transfuserpp/runtime/transfuserpp.m6.stage-b.runtime.v5.json`

Removed as superseded M6 debug output:

- `m6-inputs`, `m6-inputs-v2`, `m6-inputs-v3`, `m6-inputs-v4`
- `m6-stage-b-3frames-r1` through `m6-stage-b-3frames-r10`
- `runtime/m6_stage_b_report.r6.json`, `runtime/m6_stage_b_report.r8.json`, `runtime/m6_stage_b_report.final.json`
- `runtime/transfuserpp.m6.stage-b.runtime.v1.json` through `v4.json`
- Docker tags `m6-stage-b` and `m6-stage-c`

M5 evidence, GUI smoke evidence, and non-M6 diagnostic/runtime artifacts are
kept because they belong to separate validation stages. No broad Docker or
filesystem prune is part of this cleanup.
