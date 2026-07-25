# Scene-0061 provenance-bound G0 live-tick runbook

This is the approved procedure for one **new** Scene-0061 G0 diagnostic. It proves one physical CARLA tick with a BasicAgent control and real, synchronized NuRec multimodal responses. It does **not** prove a completed route, a validated LiDAR axis convention, real TransFuser++ inference, S0/S2/S4, or triplicate acceptance.

The historical r14 output is immutable evidence only. Its execution commit was `5313222`; a later read-only validation at a newer commit is not a substitute for a fresh execution on the selected commit. Never reuse or overwrite r14, or any other diagnostic directory.

## Invariants

- Run from the remote checkout with the existing `autodrive` Python interpreter; do not use the host system Python.
- The config, output directory, run ID, OpenDRIVE path, and CARLA PythonAPI path are explicit inputs to **both** phases.
- The output directory must be new. The runner rejects an existing directory.
- Do not patch a historical repro script or redirect it with `sed`.
- Preserve remote `.runtime/`, `incoming/`, and all historical outputs. Do not prune containers, volumes, images, or evidence.
- If CARLA is unavailable, use only `tools/remote_restart_carla.sh`. It delegates to the existing `env_build/start_carla.sh`; retain its server log under the new diagnostic directory.

## 1. Establish the exact runtime revision

On the local checkout, compare object IDs rather than branch counters. Do not run the diagnostic unless this reports `equal: true`.

```powershell
cd E:\code\ClosedLoopBench
git status --short
git rev-parse HEAD
python tools/scene0061_sync.py status `
  --repo . `
  --remote-host ${CLB_REMOTE_SSH_USER}@${CLB_REMOTE_SSH_HOST} `
  --remote-repo /home/cwadmin/workspace/ClosedLoopBench `
  --ssh-port ${CLB_REMOTE_SSH_PORT} `
  --require-equal
```

The remote checkout must also be clean except for the known untracked `.runtime/` and `incoming/` directories. Synchronize a reviewed commit before proceeding; never infer equality from a bundle filename.

## 2. Derive the r19 axis-bound config, then select immutable inputs

Do not edit `smoke_currentbound.json` or any historical runtime snapshot.
For r19, derive a new config from exactly the r18 source config. The derivation
tool records the absolute source path, SHA-256, byte count and the canonical
NRE-response-to-CARLA-sensor matrix; it rejects an existing output as well as
an already-derived/axis-bound source. The matrix is a candidate from r18
analysis, **not** coordinate proof: r19 must still satisfy the raw-to-normalized
payload replay and independent CARLA same-frame anchor gate.

```bash
set -euo pipefail
REPO=/home/cwadmin/workspace/ClosedLoopBench
PY=/home/cwadmin/sim-env/miniconda3/envs/autodrive/bin/python
SOURCE_CONFIG=$REPO/outputs/scene-0061-final-closure-v2/diagnostics/native_scan_original_v7_sidewalks8m_bbox_bottom_1tick_ee3760d_v12_r11/smoke_currentbound.json
DERIVED_CONFIG=$REPO/outputs/scene-0061-final-closure-v2/runtime/scene0061_r19_nre_lidar_axis_bound.json
test -f "$SOURCE_CONFIG"
test ! -e "$DERIVED_CONFIG"
"$PY" runners/derive_scene0061_lidar_axis_config.py \
  --source-config "$SOURCE_CONFIG" \
  --output "$DERIVED_CONFIG"
"$PY" -m json.tool "$DERIVED_CONFIG"
```

The result must contain `config_derivation.source_config.sha256` for the
selected source and `nurec_runtime.lidar_axis_normalization` with direction
`NRE response -> CARLA sensor`. The latter has `response_to_sensor`
`[0,0,-1,0,0,-1,0,0,-1,0,0,0,0,0,0,1]`. Do not change this document after
the `prepare-only` phase; create a new derived config/run instead.

Then select a fresh run identity.

Run the rest on the remote checkout. The following paths identify the validated scene inputs. Choose a previously unused numeric suffix (shown as `r19`); the commit prefix must come from the checkout that will execute.

```bash
set -euo pipefail
REPO=/home/cwadmin/workspace/ClosedLoopBench
PY=/home/cwadmin/sim-env/miniconda3/envs/autodrive/bin/python
CONFIG=$DERIVED_CONFIG
XODR=$REPO/outputs/scene-0061-final-closure-v2/runtime/road.nurec-route-extended-both-v7.sidewalks8m.bfe8fe6.xodr
CARLA_PYTHON_API=/home/cwadmin/sim-env/data/CARLA_0.9.16/PythonAPI/carla
RUN_LABEL="$(git -C "$REPO" rev-parse --short=7 HEAD)-r19"
OUT=$REPO/outputs/scene-0061-final-closure-v2/diagnostics/scene0061_live_tick_${RUN_LABEL}
RUN_ID=scene0061-live-tick-${RUN_LABEL}
test -x "$PY"
test -f "$CONFIG"
test -f "$XODR"
test -f "$CARLA_PYTHON_API/agents/navigation/basic_agent.py"
test ! -e "$OUT"
```

The selected config must retain its declared actor-binding and native-scan manifest hashes. Preparation verifies those identities, snapshots the config byte-for-byte as `runtime_run_config.json`, records commit and interpreter/protobuf identity, and preflights the explicit CARLA BasicAgent plus NuRec handler.

## 3. Prepare, inspect, then execute the exact snapshot

Preparation performs no CARLA tick. It is the stop point for any config, sidecar, native-scan, interpreter/protobuf, BasicAgent, or NuRec-handler failure. Do not execute after one of these failures.

```bash
cd "$REPO"
"$PY" runners/scene0061_live_tick.py \
  --config "$CONFIG" \
  --output-dir "$OUT" \
  --run-id "$RUN_ID" \
  --opendrive "$XODR" \
  --carla-python-api "$CARLA_PYTHON_API" \
  --prepare-only

"$PY" -m json.tool "$OUT/runtime_environment.json"

"$PY" runners/scene0061_live_tick.py \
  --config "$CONFIG" \
  --output-dir "$OUT" \
  --run-id "$RUN_ID" \
  --opendrive "$XODR" \
  --carla-python-api "$CARLA_PYTHON_API" \
  --execute-prepared
```

Before executing, require `status=prepared`; the selected remote `git_commit`; recorded absolute config, sidecar, native-scan manifest, and OpenDRIVE identities; `python_runtime` for `$PY`; and passed `carla_basic_agent` and `sensor_handler_preflight` records. `--execute-prepared` rechecks each path/hash, the byte-identical runtime snapshot, interpreter/protobuf identity, and BasicAgent source SHA before opening CARLA.

## 4. Interpret results without overclaiming

For one tick, `runtime_result.json` may be `status=failed` with `route_incomplete:` detail. That is the expected smoke termination, not a physical failure and not a full-route pass. The command is successful only when `live_tick_validation.json` is `passed` and its `completion_class` is `one_tick_physical_multimodal_smoke`.

Accept G0 only when the fresh output proves all of the following:

- exactly one physical CARLA frame and BasicAgent control in `frame_trace.jsonl`;
- exactly one matching NuRec frame, six passed RGB records, one passed LiDAR record, `SensorsimService/26.04` dispatch, and rehashable materialized JPEG/XYZI payloads;
- native-scan alignment and runtime-scene binding in the NuRec trace;
- a successful cleanup audit with every action succeeded; and
- `artifact_manifest.json` has `status=complete` and contains all required traces/reports.

If any condition fails, preserve the directory and stop at the earliest actual failure. Never change metadata or manufacture axis evidence. An LiDAR payload marked `coordinate_frame=unverified` or `axis_convention=unverified` remains a blocker for the formal TransFuser++ path even if G0 passes.

## 5. Recover only core evidence

Copy only these files into a new local evidence directory, then rehash every manifest-managed artifact against the remote manifest:

```text
runtime_environment.json
runtime_run_config.json
basic_agent_plan.json
runtime_result.json
live_tick_validation.json
artifact_manifest.json
frame_trace.jsonl
nurec_multimodal_trace.jsonl
metrics_trace.jsonl
cleanup_audit.json
closed_loop_report.json
run.log
```

Include `carla.log` and `carla.pid` only if this run restarted CARLA. Do not copy the entire `outputs` tree.

## What follows G0

The next formal gate is real LiDAR-coordinate evidence from same-frame NRE XYZI and independent CARLA-side/scene ground truth, validated by `runtime/scene0061_lidar_axis_gate.py`. A calibration field alone is not proof. Separately, TransFuser++ still requires an official pinned `leaderboard_2` source revision, checkpoint/config provenance, immutable CUDA-capable image, real checkpoint load, and real intermediate evidence. Only after both gates pass should S0/S2/S4 begin, first as single attempts and then as triplicates.
