# ClosedLoopBench Global Goal v2

This document is the current planning authority for the Global Goal. It
replaces the old design in which replay, evaluation, ROS control, NuRec
reconstruction, and interactive actors formed one blocking chain.

The project is organized as **stages of independently deliverable products**.
A stage may consume an earlier stage's artifact, but it does not import the
earlier implementation or wait for unrelated evidence. Adapters and immutable
fixtures are the boundaries between stages.

## Global Goal

Provide a reproducible closed-loop experiment in which:

1. a defined scene can be replayed in CARLA;
2. physical outcomes can be evaluated consistently;
3. an external ego algorithm can send controls;
4. RGB/LiDAR observations can be bound to the same world state;
5. declared actors can be made interactive; and
6. replay and counterfactual experiments can be compared with attributable
   evidence.

The first five capabilities are useful and reportable on their own. The final
integration stage is the only place where all required capabilities become a
single promotion gate.

## Cross-cutting observability: O1

The BEV/CARLA state view plus the six-camera Pygame grid is a cross-cutting
observability product, not a separate algorithm or sensor-closure Goal. It is
fed by one synchronized `FramePacket` and must preserve the same frame ID,
timestamp, ego pose, actor identity, and CARLA/NuRec mapping in both views.

| Use | Supports | Does not prove |
|---|---|---|
| CARLA BEV/state view | S1 replay, S2 metric debugging, S5 actor behavior | semantic perception or NuRec geometry |
| six-camera NuRec grid | S4 RGB transport/geometry debugging | LiDAR-world consistency or detector success |
| combined BEV + six-camera window | S6 qualitative evidence and incident diagnosis | collision absence, TF++ inference, or M8 promotion by itself |

The viewer's exit condition is only display integrity: common frame/timestamp,
non-overlapping camera layout, correct labels, and rehashable screenshots. A
synthetic or local screenshot remains visualization evidence, not live closed-loop
acceptance. The existing implementation is `runners/run_scene0061_dual_window.py`
and the six-camera grid patch is
`tools/patch_carla_nurec_six_camera_grid.py`.

## Anti-coupling rules

- A milestone owns one contract and one primary deliverable. “Implemented” is
  not inferred from another milestone's report.
- Every milestone has a local fixture. A CARLA, ROS2, GPU, or NuRec fixture is
  never required to validate a pure schema/evaluator/plugin milestone.
- Runtime evidence is append-only and includes input, config, source commit,
  frame-set, and report hashes. Historical evidence is never relabelled.
- Stages exchange `scene_package`, `runtime_trace`, `evaluation_report`,
  `control_trace`, and `sensor_transaction` artifacts through adapters; they do
  not reach into each other's internal state.
- A product may be `passed` while an integration product is blocked. For
  example, an evaluator can pass on a native CARLA trace while NuRec LiDAR
  remains failed.
- A successful RPC, zero collision count, or video is not a promotion by
  itself. Missing truth is fail-closed.

## Stage 0: Scene and evidence contract

**Outcome:** all other stages can consume the same versioned scene and trace
contracts.

| Milestone | Product | Independent exit gate | Current status |
|---|---|---|---|
| S0.M1 | Scenario IR, Scene Package, CARLA run-config schemas | valid/invalid fixtures and path/hash checks pass | `offline_ready` |
| S0.M2 | Tick/frame/control identity schema | frame ID, timestamp, run ID, actor identity, and source-control fields validate | `offline_ready` |
| S0.M3 | Immutable evidence manifest | artifact references are complete and no output is overwritten | `offline_ready` |

Stage 0 is contract readiness, not a live simulator or sensor acceptance.

## Stage 1: CARLA replay and physical truth

**Outcome:** a deterministic physical CARLA trace exists without NuRec, ROS2,
or a learned algorithm.

| Milestone | Product | Independent exit gate | Current status |
|---|---|---|---|
| S1.M1 | Complete scene-object registry | all safety-relevant dynamic/static objects, parked obstacles, and road boundary have identity and collision policy | `partial` |
| S1.M2 | Synchronous replay runner | three repeated runs match trace identity and leave no actor/sensor/settings leak | `not_evidenced` |
| S1.M3 | Physical truth probes | forced collision, lane departure, stationary ego, and actor lifecycle are visible in raw trace | `partial` |
| S1.M4 | Runtime trace package | every tick has ego/actor pose, lane, collision, control, and cleanup records | `partial` |

The current registry and one-tick probes support S1.M1/S1.M3, but a complete
three-run replay evidence bundle is not present. Therefore Stage 1 is not
fully passed. Replay actors, including the planned lead vehicle and
pedestrian, remain replay-only in this stage.

## Stage 2: Independent evaluation

**Outcome:** one evaluator scores any compatible trace, regardless of whether
the control source is replay, BasicAgent, ROS2, or a learned model.

| Milestone | Product | Independent exit gate | Current status |
|---|---|---|---|
| S2.M1 | Trace reader and availability policy | missing collision/lane/pose truth fails closed | `offline_ready` |
| S2.M2 | Safety/comfort/progress metrics | collision, lane, TTC, distance, route progress, speed, acceleration, hard-brake, jerk, latency, and safe-stop expose source metadata | `offline_ready` |
| S2.M3 | Deterministic report builder | same trace/config produces identical report hash | `offline_ready` |
| S2.M4 | Fault-injection fixtures | known collision, lane departure, TTC, stale control, and timeout cases fail as expected | `partial` |

Stage 2 consumes immutable traces and does not control CARLA or require RGB,
LiDAR, NuRec, ROS2, or TF++.

## Stage 3: Ego control and ROS integration

**Outcome:** a control producer can observe one tick and return one bounded
control, with safe-stop on failure, while producing a Stage 2-compatible trace.

| Milestone | Product | Independent exit gate | Current status |
|---|---|---|---|
| S3.M1 | Plugin lifecycle | initialize/reset/predict/health/close and capability checks pass | `offline_ready` |
| S3.M2 | Control validation/safe-stop | stale, mismatched, timeout, exception, NaN/Inf, and range errors produce full brake | `offline_ready` |
| S3.M3 | Native CARLA control | BasicAgent/Pure Pursuit controls synchronous CARLA and writes a Stage 2 trace | `not_evidenced` |
| S3.M4 | ROS2 external ego | passive bridge observation/control topics and accepted/safe-stop evidence pass | `not_evidenced` |
| S3.M5 | Real algorithm binding | pinned checkpoint/container identity and controls are reproducible | `evidence-dependent` |

S3 can use CARLA-native sensors. It must not wait for NuRec reconstruction.
Offline plugin replay is `offline_conformance`, never a real ego closed-loop
claim.

### TF++ status interpretation

"TF++ connected" is not one acceptance state. Use these separate labels:

| TF++ evidence | Stage classification | Claim allowed |
|---|---|---|
| adapter/module, ROS2 topics, container and lifecycle tests | S3.M1-S3.M2 implementation | TF++ integration boundary implemented |
| real checkpoint/config/repository/image hashes bound, but no live control trace | S3.M5 `runtime_ready` | prepared for remote validation |
| real checkpoint loaded and frame-matched controls reach CARLA using native sensors | S3.M5 `runtime_passed`; eligible for S6.M1 | native-sensor control-only run |
| same run uses NuRec RGB/LiDAR with valid paired transaction | S4.M5 + S6.M2 | multimodal control-only run |
| three repeated TF++ replay runs with common evaluator and no unclassified fallback | S6.M3 / historical M9 | TF++ replay baseline |
| sensor/world and perception gates also pass | S6.M2 plus perception evidence | perception-eligible multimodal experiment |

An `algorithm_id` in a run config is identity metadata only. It does not prove
that the TF++ backend was selected, that a checkpoint loaded, or that TF++
produced the control. The runtime report must show the actual ego driver,
backend identity, checkpoint hash, accepted frame-matched controls, and
fallback counts.

## Stage 4: Sensor bridge and world consistency

**Outcome:** the neural renderer's sensor transactions refer to the same frame,
pose, actor identity, and world coordinate as CARLA. RGB and LiDAR are separate
products inside this stage.

| Milestone | Product | Independent exit gate | Current status |
|---|---|---|---|
| S4.M1 | RGB gRPC transport | six-camera responses, dimensions, calibration, frame/timestamp identity, and hashes pass | `partial` |
| S4.M2 | RGB geometry | CARLA boxes project correctly; A/A/B target change changes RGB and preserves background | `partial` |
| S4.M3 | LiDAR gRPC transport | `render_lidar` returns validated non-empty XYZI with device/axis/unit/time/extrinsic evidence | `partial` |
| S4.M4 | LiDAR-world consistency | expected-visible objects have same-tick occupancy and valid world projection | `failed` |
| S4.M5 | Paired RGB/LiDAR transaction | both modalities share frame ID, actor digest, pose interval, and artifact identity | `blocked_by_S4.M4` |

The current retained Scene-0061 probe passes collision, lane, and RGB geometry,
but LiDAR-world support fails for expected objects. This blocks S4.M5 and the
formal M8 promotion only; it does not change S1-S3 status.

The complete CARLA registry remains the physical audit scope. A bounded NuRec
candidate can reduce render cost, but it cannot delete collision bodies or
silently classify protected tracks as background.

## Stage 5: Interactive actors

**Outcome:** only declared experimental actors react to the ego; all context
actors remain attributable replay.

| Milestone | Product | Independent exit gate | Current status |
|---|---|---|---|
| S5.M1 | Full-scene replay context | non-experimental actors remain deterministic replay | `offline_ready` |
| S5.M2 | Lead-vehicle behavior | trigger, issued policy, physical response, and evaluator trace are repeatable | `not_evidenced` |
| S5.M3 | Pedestrian behavior | bounded pause/yield/abort along source corridor is physically evidenced | `not_evidenced` |
| S5.M4 | Actor attribution | replay/reactive policy and actor identity remain explicit in every report | `partial` |

Stage 5 does not require every registered actor to become controllable. The
controlled set remains the lead vehicle and pedestrian.

## Stage 6: Integrated experiments

Stage 6 is an integration consumer, not a place to hide missing component work.
It creates release-level results from already-passed products:

| Milestone | Product | Required inputs | Current status |
|---|---|---|---|
| S6.M1 | Native-sensor control baseline | S1 + S2 + S3 | `evidence-dependent` |
| S6.M2 | NuRec multimodal baseline | S1 + S2 + S3 + S4.M5 | `blocked_by_S4` |
| S6.M3 | TF++ replay baseline | S1 + S2 + S3, pinned checkpoint, three repeat runs; S4 is required only for the multimodal variant | `evidence-dependent` |
| S6.M4 | Lead/pedestrian counterfactual matrix | S5 + selected S6 baseline | `not_started` |
| S6.M5 | Global Goal report | complete bounded evidence for selected cases | `not_started` |

S6 is the only stage with a multi-capability gate. Its failure cannot be used
to mark any component stage as failed.

## Historical M-series mapping

Historical M evidence is preserved; it is not renumbered or retroactively
passed:

| Historical milestone | Stage 2 mapping | Current interpretation |
|---|---|---|
| M1-M5 | S0/S3 | operational and offline contract evidence; not full physical acceptance |
| M6 | S1.M1 | registry/physical presence probe passed for the recorded scope |
| M7 | S4.M2 | pose-reference correction requires rerun |
| M8.1 | S1.M3/S2.M2 | collision/lane probe valid, repetition gate pending |
| M8.2 | S4.M4 | failed LiDAR-world consistency |
| M8.3 | S4.M1/S4.M2 | RGB geometry evidence, no detector claim |
| M9 | S6.M3 | integration boundary prepared; real three-run TF++ evidence not yet established |
| M10 | S5.M2/S5.M3 | not started for acceptance |
| M11 | S6.M4/S6.M5 | not started |

## Current summary

```text
S0  offline_ready
S1  partial
S2  offline_ready
S3  offline_ready (live CARLA/ROS2 pending)
S4  partial, with S4.M4 failed
S5  partial schemas, runtime acceptance not evidenced
S6  not_started / S6.M2 blocked by S4
```

The next local work should improve S1-S3 independently with native or
synthetic fixtures. When the remote host returns, run S1 replay and S3 native
control before attempting S4 reconstruction. Formal 40k reconstruction is
allowed only after S4.M4/S4.M5 pass on a common non-empty frame set.
