# ClosedLoopBench: Open-Loop Testing and LiDAR Editability Diagnosis

Status snapshot: 2026-08-07
Current status: `open-loop test complete` / `closed-loop blocked`

## 1. Delivered: CARLA/ROS2 Open-Loop Testing

ClosedLoopBench is currently a test bench for driving algorithms. It consumes
`Scenario IR` from TriggerEngine, builds the CARLA scene and exchange artifacts,
replays the logged trajectory with GT pose ownership, publishes observations
through the ROS2 boundary, runs the real TransFuser++ inference path, and writes
frame-matched evaluation evidence.

The key open-loop rule is explicit: model control is recorded and scored, but
it does not choose the next Ego pose. The next pose always comes from the
Scenario IR reference trajectory. This makes the current result reproducible
and testable without claiming interactive driving.

```text
TriggerEngine Scenario IR
        |
        v
CARLA scene + synchronized GT replay
        |
        v
ROS2 observation boundary -> real TransFuser++ inference
        |
        v
frame trace + metrics + provenance -> open-loop report
```

## 2. Real Inference and Evaluation Evidence

The project is more than a compiler or a fake runtime. The open-loop path has
been exercised with the real algorithm stack and retained evidence:

| Stage | What ran | Evidence |
| --- | --- | --- |
| Native CARLA | CARLA RGB/LiDAR through the ROS2/TF++ path | Stage-A inference and intermediate traces |
| NuRec multimodal | NuRec RGB/LiDAR at pinned GT poses | 39-frame full run, zero fallback and frame mismatch |
| M7 acceptance | Same open-loop contract over three seeds | 39/39 frames per seed, deterministic report identity |
| M8 comparison | Native, reconstructed, and Harmonizer RGB routes | 39 scored frames, 68 dynamic actors, formal actor-aware bbox evaluation |

The evaluation keeps one Scenario IR, one actor manifest, one frame clock, and
one GT trajectory across routes. Available outputs include waypoint ADE/FDE,
latency and synchronization health, provenance hashes, and actor-aware BEV
bbox metrics. They are metrics for the recorded trajectory, not a policy-caused
future state.

## 3. M8: Find the Failure on the Same Scene

M8 compares three sensor routes over the same 39 scored frames and 68 dynamic
actors:

| Route | RGB | LiDAR | Result |
| --- | --- | --- | --- |
| `raw_original` | CARLA native | CARLA native | 66 predictions / 39 matches, vehicle AP50 0.547, mAP50 0.274 |
| `reconstructed` | NuRec RGB | NuRec LiDAR | 35 predictions / 0 matches, mAP50 0.0 |
| `harmonized` | Harmonizer RGB | Same NuRec LiDAR | 23 predictions / 1 match, mAP50 0.0016 |

The native route is the reference. The reconstructed route is not rejected by
frame synchronization or report validation; it runs inference and produces a
valid, comparable result. Its failure is therefore useful evidence rather than
an execution error.

## 4. A/B and Scale-Up Debug: LiDAR Is Not Editable

The diagnosis proceeds from inexpensive checks to causal tests:

1. Correct the LiDAR axis matrix and compensate the measured sensor-height
   offset. Point-cloud geometry improves, but detection does not recover.
2. Swap one modality at a time on the same 39 frames. `NuRec RGB + raw LiDAR`
   produces 21 matches and mAP50 0.130; `raw RGB + NuRec LiDAR` produces zero
   matches and mAP50 0.0. RGB is not the main bottleneck.
3. Repeat the comparison at scale: three routes, 39/39 matched frames, zero
   fallback, one shared actor manifest, and 68 dynamic actors.
4. Probe the live NRE dynamic-LiDAR service with target-only, all-minus-target,
   and moved-pose variants. RGB follows the edited actor pose, while LiDAR
   returns remain at stored offset positions instead of following the track's
   cuboid pose.

The conclusion is specific: the dynamic LiDAR path is not currently
**track-editable**. The server emits dynamic vehicle returns from canonical
offsets rather than applying the per-track pose. This is an upstream
NRE/SensorsimService limitation, not a ClosedLoopBench coordinate patch.

The same-frame visual evidence uses `frame_00000018` from the raw and
reconstructed routes. It demonstrates that the comparison is same-index
observation evidence, not a closed-loop consequence of model control.

## 5. Closed-Loop Vision: Blocked

The intended next stage is an interactive CARLA/ROS2 loop: Ego control changes
the next simulator state, reactive actors respond, and the same evidence
contract scores the resulting run. This vision is currently **blocked** until:

- the dynamic LiDAR path applies edits at the true track pose;
- one runtime owns synchronous CARLA `world.tick()` and control application;
- a physical actor is shown to react to Ego behavior;
- a real ROS2/GPU Ego policy passes timeout, safe-stop, and provenance gates.

Until those gates pass, no retained M8 report is a closed-loop score. The
canonical evidence classification is `open_loop_multimodal`.

## Code Map

- `runners/run_open_loop_transfuserpp_triplicate.py`: runs the three-route
  replay and real TF++ inference procedure.
- `runners/compare_open_loop_transfuserpp_triplicate.py`: binds route
  provenance and comparison evidence.
- `metrics/transfuserpp_m8.py`: computes intermediate and actor-aware metrics.
- `tools/diagnose_nurec_dynamic_vehicle_lidar.py`: reproduces the dynamic
  LiDAR editability probe.
- `docs/open_loop_multimodal_eval.md`: pinned inputs and reproducible evidence.
- `docs/open_loop_m8_debug_log.md`: debug history and root-cause evidence.
