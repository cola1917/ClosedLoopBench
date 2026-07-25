# Scene-0061 provenance-bound 1-tick G0 diagnostic

This command is the only approved replacement for the historical hard-coded
`repro.py` flows. It is a physical-evidence smoke test, not a full route or
TransFuser++ acceptance run. Do not edit a copied repro script or substitute a
different configuration after preparation.

## Preconditions

1. Compare exact repository commits with `python tools/scene0061_sync.py status`;
   do not infer synchronization from `ahead` counters.
2. Verify the remote copies of `smoke_currentbound.json` and its actor-binding
   sidecar hash to the values recorded in `docs/scene0061_handoff_20260725.md`.
3. Verify the explicit OpenDRIVE path exists. If CARLA is down, use only
   `tools/remote_restart_carla.sh` (which delegates to the existing env_build
   launcher) and preserve its server log in the new run directory.
4. Choose a previously nonexistent output directory and a unique run ID.

## Remote sequence

Run preparation first using env_build's existing `autodrive` interpreter, then inspect `runtime_environment.json`. All four
explicit inputs must remain identical for the execution command.

```bash
/home/cwadmin/sim-env/miniconda3/envs/autodrive/bin/python runners/scene0061_live_tick.py \
  --config /home/cwadmin/workspace/ClosedLoopBench/outputs/scene-0061-final-closure-v2/diagnostics/native_scan_original_v7_sidewalks8m_bbox_bottom_1tick_ee3760d_v12_r11/smoke_currentbound.json \
  --output-dir /home/cwadmin/workspace/ClosedLoopBench/outputs/scene-0061-final-closure-v2/diagnostics/scene0061_live_tick_r12 \
  --run-id scene0061-live-tick-r12 \
  --opendrive /home/cwadmin/workspace/ClosedLoopBench/outputs/scene-0061-final-closure-v2/runtime/road.nurec-route-extended-both-v7.sidewalks8m.bfe8fe6.xodr \
  --prepare-only

/home/cwadmin/sim-env/miniconda3/envs/autodrive/bin/python runners/scene0061_live_tick.py \
  --config /home/cwadmin/workspace/ClosedLoopBench/outputs/scene-0061-final-closure-v2/diagnostics/native_scan_original_v7_sidewalks8m_bbox_bottom_1tick_ee3760d_v12_r11/smoke_currentbound.json \
  --output-dir /home/cwadmin/workspace/ClosedLoopBench/outputs/scene-0061-final-closure-v2/diagnostics/scene0061_live_tick_r12 \
  --run-id scene0061-live-tick-r12 \
  --opendrive /home/cwadmin/workspace/ClosedLoopBench/outputs/scene-0061-final-closure-v2/runtime/road.nurec-route-extended-both-v7.sidewalks8m.bfe8fe6.xodr \
  --execute-prepared
```

`runtime_environment.json` must record the intended commit, absolute source
config path and SHA-256, sidecar SHA-256, native scan manifest SHA-256, and
OpenDRIVE SHA-256, plus the exact Python executable and protobuf version. The
prepare-only sensor-handler preflight must pass. A hash/path/interpreter mismatch
is a hard stop. Do not use the host `/usr/bin/python3`: its protobuf 3.12.4 is
incompatible with the installed NuRec generated modules.

`route_incomplete` after exactly one tick is an expected smoke termination. It
is not a passed full route. The G0 smoke passes only when all of these are
present and internally consistent: one physical CARLA frame with a BasicAgent
control, one passed NuRec trace containing six RGB and one LiDAR response, a
successful cleanup audit with every action successful, and a complete
`artifact_manifest.json`.

Pull back only these evidence files: `runtime_environment.json`,
`basic_agent_plan.json`, `runtime_result.json`, `live_tick_validation.json`,
`artifact_manifest.json`, `frame_trace.jsonl`, `nurec_multimodal_trace.jsonl`,
`metrics_trace.jsonl`, `cleanup_audit.json`, `closed_loop_report.json`, and
`run.log`. Do not treat an old output directory as evidence for the new run.
