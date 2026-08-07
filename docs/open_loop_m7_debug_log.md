# M7 Debug Log (scene-0061)

Date: 2026-08-05

This log records only the M7 formal acceptance issues. Failed attempts were
not used as evidence and were moved out of the retained output tree.

## Timeline

| Attempt or change | Observation | Resolution |
| --- | --- | --- |
| Environment audit | Host checkpoint candidate and config matched the report bindings exactly; CARLA Garage was at the pinned revision and the M6 image exposed the required mounts. | Cleared the historical missing-checkpoint blocker; keep the external repo/checkpoint/config read-only. |
| First seed 43 capture | System/ROS Python selected protobuf from `/usr/lib/python3/dist-packages`, which lacks the generated NuRec `internal.builder` API. | Prepend the Conda `autodrive` protobuf site-packages and NuRec API path in `PYTHONPATH`. |
| First seed 43/47 captures | Sensor references were emitted as `/sim-data/payloads/...` even though each capture directory was mounted below `/sim-data`. | Fixed `_container_payload_ref` to bind `/sim-data/<capture-directory>/payloads/...`; added a regression test. Re-captured both seeds. |
| First container invocation | Executing the runner as a script omitted `/opt/closed-loop-bench` from Python module search. | Use `python3 -m runners.run_open_loop_transfuserpp_stage_b` inside the image. |
| First seed 43 inference | Backbone cache was present on the host but the container attempted to reach Hugging Face, which is unavailable on this host. | Mount `/sim-data/hf_home` and set `HF_HOME`, `HF_HUB_OFFLINE=1`, and `TRANSFORMERS_OFFLINE=1`. |
| Formal seed 43 and 47 inference | Real checkpoint load, warm-up, 39 predictions, 39 intermediate files, and zero fallback completed for both seeds. | Accepted as formal M7 seed evidence. |

## Final acceptance

- aggregate: `outputs/scene0061-transfuserpp/runtime/open_loop_m7_triplicate_report.v1.json`
- aggregate SHA-256: `71e49fc7a8532dbfa2b92033c16e2f75516e7bd38d323ac8d4820283bf1d03e5`
- seed set: `S0_original_replay` x `{41, 43, 47}`
- every seed: 39/39 matched frames, 39/39 intermediates, zero fallback/drop/mismatch
- sensors: six RGB cameras plus `lidar_top` on every frame; NuRec dynamic actor count `0`
- intermediate evaluation: all three reports `status=evaluated`, 39 frames, no fail-closed reasons
- image: `closed-loop-bench/transfuserpp-v5:m6-stage-d@sha256:d5f814b8aab88bbef08e70a4f915771658221190d2501e2894815977d3db6394`
- checkpoint SHA-256: `d6fbdc28f7398354beadc7cf6765d866457c957f7b470c88ba206e73311a3b44`
- model config SHA-256: `895e3e9704ceda443169ca32aaef2712b1becf2d42473d7273071ec6ceda113e`
- repo revision: `72f39a63423a5edef6904b1487e0360a64bcf445`

The seed 41 report/trace/runtime binding is the retained M6 full39 result.
Seeds 43 and 47 have independent runtime bindings, NuRec traces, reports, and
intermediate directories. No report was copied or relabeled to fill a seed.

## Cleanup

Retained in the workspace:

- the three successful seed captures and `transfuserpp_intermediates` trees;
- the three runtime bindings, seed reports, intermediate evaluations, and the
  frozen aggregate;
- the pinned M6/M5 evidence and the HF cache used by the formal image.

Moved to system Trash as failed/superseded attempts:

- `ClosedLoopBench-m7-seed-43-capture-failed-protobuf`
- `ClosedLoopBench-m7-seed-43-capture-path-v1`
- `ClosedLoopBench-m7-seed-47-capture-path-v1`

These were not referenced by the aggregate. M8/M9 remain explicitly false;
static NuRec rendering and IR collision proxies do not become closed-loop
actor evidence.
