# ClosedLoopBench: CARLA/ROS Open-Loop Evaluation

Status snapshot: 2026-08-07
Current status: `open-loop complete` / `closed-loop blocked`

## Current Delivery: CARLA/ROS Open Loop

Despite the repository name, the current delivered product is the **CARLA/ROS
open-loop evaluation path**. It consumes a pinned Scenario IR, compiles
portable scenario artifacts, replays a logged trajectory with deterministic
time ownership, publishes synchronized observations through the ROS/algorithm
boundary, and scores the outputs against the same scene contract.

This means the CARLA scene handoff, GT replay, ROS observation boundary,
TransFuser++ evaluation, and evidence reports are connected and reproducible.
It does not mean that model control drives the next simulator state. The
current report is an open-loop evaluation result, not a closed-loop driving
score.

```text
TriggerEngine Scenario IR
        |
        v
Contract validation + identity hashes
        |
        +--> OpenSCENARIO / OpenDRIVE / CARLA run config
        |
        v
GT replay host and synchronized observation transaction
        |
        +--> native CARLA route
        +--> reconstructed NuRec route
        +--> optional Harmonizer RGB route
        |
        v
Algorithm boundary -> intermediate trace -> open-loop report
```

## Evaluation Contract

| Layer | ClosedLoopBench owns | Current claim |
| --- | --- | --- |
| Scenario | IR validation, identity, timing, actor roles, metrics | Scenario intent remains traceable |
| Exchange | `.xosc`, `.xodr`, CARLA run config, shared package references | Portable artifacts can be generated and checked offline |
| Replay | Fixed frame ids, GT ego/actor poses, sensor provenance | The logged trajectory owns the next pose |
| Observation | Native/reconstructed route binding, ROS boundary, hashes, latency | Inputs are comparable when provenance gates pass |
| Evaluation | Per-frame trace, actor-aware BEV boxes, route comparison, fail-closed report | Metrics describe the recorded trajectory, not a policy-caused future |

TriggerEngine remains the source of mined events and Scenario IR. NeuralSceneBridge
remains the optional reconstruction and sensor-rendering provider. ClosedLoopBench
owns the evaluation boundary between those artifacts and an eventual simulator
or ego-policy runtime.

## The Open-Loop Transaction

For every scored timestamp:

1. Read the pinned Scenario IR and validate its identity and frame contract.
2. Advance the replay host to the requested timestamp.
3. Set ego and replay actors to their Scenario IR reference poses.
4. Materialize native or reconstructed observations with a shared `frame_id`.
5. Send observations to the algorithm boundary and record response hashes,
   latency, controls, and intermediate heads.
6. Score predictions against the original actor trajectory and write a report.
7. Ignore model control when selecting the next scored ego pose.

That last rule is the defining boundary. A prediction can be logged and
evaluated, but it cannot change the next pose in the current formal route.

## LiDAR Problem Discovery and Debug

LiDAR debug is a diagnosis produced by the evaluation pipeline, not a separate
project claim. M8 keeps the same Scenario IR, 39 scored frames, 68 dynamic
actors, and shared actor manifest while comparing three input routes:

| Route | RGB | LiDAR | Result |
| --- | --- | --- | --- |
| `raw_original` | CARLA native | CARLA native | Reference route: 66 predictions, 39 matches, vehicle AP50 0.547, mAP50 0.274 |
| `reconstructed` | NuRec RGB | NuRec LiDAR | 35 predictions, 0 matches, mAP50 0.0 |
| `harmonized` | Harmonizer RGB | Same NuRec LiDAR | 23 predictions, 1 match, mAP50 0.0016 |

The debug sequence is deliberately causal:

1. Use native CARLA RGB/LiDAR as the reference route.
2. Run the reconstructed and harmonized routes with the same frame and actor
   contract.
3. Validate the LiDAR axis matrix and sensor-height compensation; geometry
   improves, but detection does not recover.
4. Swap one modality at a time to separate RGB quality from LiDAR quality.
5. Probe the live NRE dynamic-LiDAR path after the cross-input result points to
   LiDAR.

The cross-input experiment isolates the cause:

- NuRec RGB + raw LiDAR: 21 matches, mAP50 0.130.
- Raw RGB + NuRec LiDAR: 0 matches, mAP50 0.0.

The evidence clears reconstructed RGB as the main bottleneck and identifies
reconstructed LiDAR as the failing modality. The live NRE 26.04 probe further
shows that dynamic vehicle returns are emitted at stored offset positions
instead of following each track's cuboid pose. This is recorded as an
upstream/server-path limitation, not as a ClosedLoopBench coordinate fix.

## Closed-Loop Vision: Blocked

The intended next stage is an interactive CARLA/ROS loop in which Ego control
changes the next simulator state, reactive actors respond, and the same
evaluation contract scores the resulting run. That vision is currently
**blocked** by acceptance gates, not by a missing document-level architecture:

- the reconstructed dynamic LiDAR path must place returns at the track pose;
- synchronous CARLA `world.tick()` must be owned by the runtime;
- one physical actor must be shown to react to Ego behavior;
- a real ROS2/GPU Ego policy must pass timeout, safe-stop, and provenance gates.

Until those gates pass, the following are explicitly not claimed:

- Ego control changes the next simulator state.
- Scripted or TrafficManager actors react to ego behavior in a formal run.
- A real ROS2/GPU/model stack has passed environment acceptance.
- Reconstructed RGB/LiDAR is a perception-ready sensor stream.
- The current XODR fit is a map-faithful lane-driving environment.
- Any report in the retained M8 set is a closed-loop score.

The CARLA GUI smoke is observability evidence only. Fake runtimes and dry-run
reports validate contracts and cleanup behavior; they do not substitute for a
real CARLA/ROS2 acceptance run.

## Path To Closure

The next implementation sequence is deliberately incremental:

1. Freeze one Scene Package, actor-binding set, timestamp clock, and sensor
   calibration contract.
2. Pass a real CARLA BasicAgent replay with synchronous `world.tick()` ownership.
3. Add one physical actor controller and prove that its state changes in
   response to ego behavior.
4. Connect one external ego policy through the ROS2/TCP plugin boundary with
   timeout and safe-stop evidence.
5. Re-run the same-frame actor-aware RGB/LiDAR gate and three-run acceptance
   matrix.

Only after these gates pass should the project title describe a closed-loop
evaluation result. Until then, `evidence_classification: open_loop_multimodal`
is the canonical status for the M8 route comparison.

## Code Map

- `runners/build_nuscenes_exchange.py`: compiles the portable scene package.
- `runners/run_open_loop_transfuserpp_triplicate.py`: runs the three-route
  replay/evaluation procedure.
- `runners/compare_open_loop_transfuserpp_triplicate.py`: binds route
  provenance and comparison evidence.
- `metrics/transfuserpp_m8.py`: evaluates intermediate and actor-aware metrics.
- `runners/validate_multimodal_closed_loop.py`: future fail-closed sensor gate;
  its presence is not proof that the real gate has passed.
- `docs/open_loop_multimodal_eval.md`: pinned inputs, route contracts, and
  reproducible details.
- `docs/open_loop_m8_debug_log.md`: failed attempts, root-cause isolation, and
  retained evidence disposition.
