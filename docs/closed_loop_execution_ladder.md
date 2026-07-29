# ClosedLoopBench Execution Ladder (Legacy Ordering)

> **Planning note:** The capability-independent plan in
> [`global_goal_capability_milestones.md`](global_goal_capability_milestones.md)
> is now authoritative. This file is retained as a dependency view for the
> final integrated path; it must not be used to block G1/G2/G3 work on the
> current G4 LiDAR evidence failure.

This document defines the implementation order for the Global Goal. It is an
execution track, not a renumbering of the historical M-series. The M-series
remains the evidence and promotion vocabulary; this ladder makes each producer
independently testable so a failed reconstruction does not hide a replay,
metric, or control-contract defect.

## Global dependency

```text
L0 contract
  -> L1 physical CARLA replay
    -> L2 deterministic evaluator
      -> L3 algorithm no-op/control transport
        -> L4 RGB closure
          -> L5 LiDAR/world closure
            -> L6 M8 combined promotion
              -> L7 TF++ replay baseline
                -> L8 controlled actors
                  -> L9 Goal experiment matrix
```

Only the preceding layer is a hard dependency for the next layer. A layer
must produce a new immutable evidence directory containing its manifest,
source/config/commit hashes, frame IDs, and an explicit
`evidence_classification`. Historical evidence is never relabelled as a new
layer pass.

## L0: Contract and scene package

Freeze the Scenario IR, complete scene package, CARLA run config, actor
identity rules, tick/frame schema, control schema, and report schema. Validate
hashes and path portability without CARLA, NuRec, ROS2, or a model checkpoint.

**Exit:** schema and negative tests pass; the package is immutable and can be
consumed by both replay and evaluation.

## L1: Physical deterministic replay

Run the full CARLA registry without NuRec and without an external algorithm.
All 228 Scene-0061 objects remain represented: dynamic actors are physical
replay actors, static roadside obstacles have collision bodies, and the road
boundary comes from CARLA/OpenDRIVE. Use synchronous ticks and record every
tick's ego pose, actor pose/identity, lane state, collision events, controls,
and cleanup state.

Run the same fixture three times and compare the trace hashes. Replay is a
physical truth baseline, not an ego closed-loop or perception result. The
lead vehicle and pedestrian remain replayed here; they become controllable
only at L8.

**Exit:** three deterministic runs, no actor/sensor/settings leak, and forced
collision plus lane-departure fixtures are observable in the trace.

## L2: Deterministic evaluation harness

Consume only the immutable L1 trace. Compute collision, lane invasion,
route progress, TTC, distance, hard-brake, jerk, latency, and availability
metrics. Every metric declares its source and missing-value policy; missing
truth fails closed instead of becoming zero. Running the evaluator twice on
the same trace must produce the same report hash.

This layer proves the evaluator, not an algorithm. It can use a CARLA native
reference trace or a deterministic fixture and must not depend on NuRec.

**Exit:** injected known collision/lane/TTC failures are detected and a
complete immutable evaluation report is reproducible.

## L3: Algorithm no-op and control transport

Exercise `reset -> observation -> predict_control -> CARLA tick -> report`
with a safe-stop/hold-brake plugin or BasicAgent. Validate frame identity,
timestamp ordering, control ownership, timeout, stale-input handling,
invalid-control rejection, and cleanup. Offline plugin replay is classified as
`offline_conformance`; only a real CARLA loop can claim `control_only` or
`ego_closed_loop`.

No RGB, LiDAR, detector, or real TF++ checkpoint is required at this layer.
The result isolates clock and control transport before sensor reconstruction.

**Exit:** valid controls reach CARLA, failures produce safe-stop, and the
trace/report records the exact source frame and latency.

## L4: RGB-only closure

Add the NuRec six-camera payload on the same L1 tick set. Verify payload
identity, calibration, CARLA-to-camera pose binding, timestamp/frame matching,
and calibrated projection of the ego plus protected actors. A/A/B probes must
show that a declared actor change changes the RGB payload while the background
remains stable.

The full CARLA registry and collision audit remain active. A bounded NuRec
render candidate is allowed for cost control, but candidate selection never
deletes a CARLA actor or collision proxy. This layer proves RGB geometry, not
semantic detection or LiDAR support.

**Exit:** same-tick six-camera geometry and projection pass on a non-empty
frame set; report classification is `rgb_geometry` (or `rgb_control_only`).

## L5: LiDAR/world closure

On the same artifact and frame contract, add native LiDAR timestamps,
sensor-to-world transforms, units/axis checks, occlusion-aware expected
support, per-object ROI occupancy, and A/A/B content change. The protected
lead vehicle and pedestrian require a continuous three-frame local LiDAR
editable window. Sparse objects are marked `low_lidar_support`; they are not
silently removed or converted to background.

Ego-corridor and point-cloud quality may select NuRec candidates, while the
complete CARLA registry remains the physical audit scope. Static parked cars,
road boundaries, and other declared-observable objects still need an explicit
collision/geometry policy.

**Exit:** all declared-observable objects have support on a common non-empty
same-tick frame set, and the LiDAR-to-world audit passes. Failure here blocks
M8 promotion but does not invalidate L1-L4 evidence.

## L6: M8 combined promotion

Run the formal M8 promotion workflow using one candidate artifact, one scene
registry hash, and one common frame set. Collision, lane, calibrated
visibility, and LiDAR-world streams must all pass, along with candidate
source/config smoke and editable-window checks. Only a report with
`formal_reconstruction_allowed=true` may authorize a 40k reconstruction.

## L7-L9: Algorithm, actors, and Goal

- **L7 / M9:** run the real TF++ replay baseline three times on the promoted
  artifact. Keep context, lead, and pedestrian in replay mode so algorithm
  behavior is attributable.
- **L8 / M10:** promote only the lead vehicle and pedestrian to scripted or
  reactive control; retain full-scene registry and replay background.
- **L9 / M11:** execute the seeded Goal matrix and compare replay and declared
  counterfactuals using the same KPI/evidence contract.

## Historical M-series mapping

Do not mark historical M6-M8 as passed because a new ladder layer passes. Use
the following traceability aliases:

| Ladder | Primary historical evidence |
|---|---|
| L0 | M1-M5 contracts and exchange artifacts |
| L1 | M6 registry and M8.1 physical truth |
| L2 | M8.1 evaluator/report evidence |
| L3 | M1-M5 plugin/ROS2 contract evidence |
| L4 | M7 pose binding and M8.3 RGB geometry |
| L5 | M8.2 LiDAR-world evidence |
| L6 | formal M8 promotion |
| L7 | M9 |
| L8 | M10 |
| L9 | M11 / Global Goal |

The current Scene-0061 state is L0-L3 offline-ready, L4-L6 blocked on the
remote NuRec/source-content artifact, and L7-L9 not started. Remote host
availability is required for claims about real CARLA or NuRec execution.
