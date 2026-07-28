# Scene-0061 Phase 2 Milestones

## Phase 2 at a Glance (Chinese)

全局 Goal 是最终验收方向：在同一个因果一致的 CARLA/NuRec 场景中，
算法可见的车、人、静态障碍物和道路边界必须同时具有可追溯的物理、
渲染和安全约束。M 系列是实现该 Goal 的顺序门禁，而不是用 Goal 替代
里程碑。

| Milestone | Stage gate | Current Scene-0061 status | Promotion dependency |
|---|---|---|---|
| M1-M5 | operational vertical slice: CARLA, NuRec, ROS2/TF++ and portable evidence | retained as Phase 1 record | none; not a physics/safety pass |
| M6 | full scene-object registry and physical CARLA representation | passed | M7 |
| M7 | same-tick CARLA physical pose to NuRec render-pose binding | passed | M8 |
| M8.1 | CARLA collision, lane and physical-box runtime truth on every tick | implemented; three-tick probe valid | M8.2 |
| M8.2 | occlusion-aware expected LiDAR support and LiDAR-to-world audit | failed on the three-tick probe; NuRec LiDAR investigation required | M8.3 |
| M8.3 | calibrated six-camera geometric visibility records | required as geometry evidence; no detector dependency | M9 |
| M9 | three-run TF++ replay baseline in the audited scene | blocked by M8 | M10 |
| M10 | controlled lead vehicle and controlled pedestrian counterfactuals | not started | M11 |
| M11 | seeded Goal-level experiment matrix and attributable comparison | not started | Goal acceptance |

The working order is strict:

```text
M6 complete registry
  -> M7 same-tick pose binding
    -> M8.1 runtime collision/lane/box truth
      -> M8.2 expected-scannable LiDAR support
        -> M8.3 calibrated RGB geometric visibility records
          -> M9 TF++ replay baseline (three runs)
            -> M10 lead vehicle / pedestrian controls
              -> M11 seeded experiment matrix / Global Goal acceptance
```

`background_replay` remains replay-only throughout this sequence.  Only the
predeclared lead vehicle and pedestrian may move from replay to controlled
behavior in M10; full registration never authorizes control of every actor.

An independent RGB detector is explicitly deferred.  It is not an M8 or M9
promotion dependency, and may be added after M11 as an optional assurance
study when semantic image-detection evidence becomes necessary.  It must not
be presented as a CARLA collision requirement.

## Global Goal

ClosedLoopBench must evaluate an external driving algorithm in one causally
consistent scene: the objects the algorithm can observe through NuRec RGB and
LiDAR must correspond to objects that exist at the same pose, time, lane and
collision boundary in CARLA. The evaluation must then distinguish replayed
context from the two deliberately controlled actors.

This is a global Goal, not a replacement for the M-series. Milestones are
evidence gates that progressively make the Goal true.

## Phase 1 Record

M1-M5 established an operational vertical slice: CARLA execution, ROS2 ego
control, NuRec RGB/LiDAR requests, TF++ control messages, repeated short runs,
and portable evidence. They do not close the physical-consistency Goal.

The Phase 1 evidence showed these gaps:

- a visible roadside parked vehicle was not represented by a CARLA collision
  object;
- only a vehicle and a pedestrian were explicitly bound as dynamic actors;
- the dynamic vehicle's physical CARLA pose and NRE render pose diverged;
- collision checks covered CARLA contact only, not NRE-visible contact;
- no lane-boundary, object-visibility, or LiDAR-to-world consistency gate was
  required.

M1-M5 remain reproducible operational evidence. They must not be labelled as
safe-driving, full-physics, or visual-physics acceptance.

## Representation Rules

Every object in the safety-relevant scene corridor has exactly one registry
record and one of these roles:

| Role | CARLA representation | NRE representation | Runtime control |
|---|---|---|---|
| `static_obstacle` | static collision body or validated map collision mesh | source-scene appearance | none |
| `background_replay` | physical actor | matching dynamic render pose | replay only |
| `controlled_lead_vehicle` | physical vehicle actor | matching dynamic render pose | replay, then scripted/reactive |
| `controlled_pedestrian` | physical walker actor | matching dynamic render pose | replay, then scripted/reactive |
| `road_boundary` | OpenDRIVE/CARLA lane topology | camera/LiDAR projection target | none |

Only the lead vehicle and target pedestrian become experimental controls in
Phase 2. Registering every relevant object does not make every object
controllable.

For Scene-0061, the control candidates are pinned to the immutable
counterfactual matrix: lead vehicle `c1958768d48640948f6053d04cffd35b` and
pedestrian `71603dd1a2ba4e9daf095535e38310ac`.  Their M6 registry roles are
`controlled_lead_vehicle` and `controlled_pedestrian`, respectively, while
their runtime mode remains `replay` until M10.  Every other dynamic record is
`background_replay` and is never promoted by this designation.

## M6: Safety-Relevant Scene Object Registry

**Objective:** build a versioned inventory of all vehicles, pedestrians,
parked/static obstacles, road boundaries and traffic-control objects that can
affect the ego within the selected scene horizon.

**Deliverables:**

- `scene_object_registry.v1.json` with identity, class, source provenance,
  temporal interval, CARLA representation, NRE representation, collision
  policy and control role for every object;
- a static-obstacle inventory that explicitly includes the visible roadside
  parked vehicle and any equivalent collision proxy/mesh;
- an actor coverage audit that reports unregistered safety-relevant NRE-visible
  objects rather than silently ignoring them;
- CARLA spawn/collision audit proving each required object exists.

**Exit gate:** no required registry object is missing a CARLA collision policy;
the parked vehicle is either collidable or explicitly rejected as an
unmodelled limitation. The latter fails promotion to M7.

### M6 Implementation Contract

The M6 builder is `runners/build_scene_object_registry.py`.  It writes two
immutable sidecars:

- `scene_object_registry.v1.json`: full dynamic Scenario IR catalog, curated
  static-object inventory, and the CARLA road-boundary representation;
- `scene_object_coverage_audit.v1.json`: every NRE-visible safety object
  reconciled to the registry and its CARLA collision/topology policy.

The Scenario IR catalog is the first static-object inventory: its traffic
cones, barriers and parked/zero-speed tracked vehicles are registered as
collision proxies directly.  An optional static-object manifest is deliberate
ground-truth annotation, not a model guess, for objects visible only in the
NuRec reconstruction. Every extra entry requires an id, provenance,
scene-local `x/y/z/yaw` placement and a CARLA collision proxy. The initial
Scene-0061 audit must classify the roadside parked vehicle visible at the
start of the M5 recording from one of these two sources.
The visibility manifest records each observed NRE safety object by `object_id`
or source track id.  Omission of this manifest fails closed, so a run cannot
promote merely because it produced RGB/LiDAR frames.

For Scene-0061, M6 visibility evidence is produced from an actual complete
six-camera NRE payload set, the same CARLA world tick, recorded camera
extrinsics, and the source `calibrated_sensor` 3x3 intrinsics. Each geometric
observation records the NRE payload SHA-256, source calibration token and
projected 3D-box bounds. The calibration capture must bind the live NRE camera
identity to the exact source calibration-table hash. This is a coverage gate,
not a semantic-detection claim: pixel-level object evidence and LiDAR
occupancy remain independently required by M8.

`runners/attach_scene_object_registry.py` derives a new CARLA run config from
the registry. It embeds each static collision proxy and the exact registry
file hash, requires source-pose spawning with no map fallback, and emits
`static_obstacle_runtime_evidence.v1` in the run report. It is a fresh Phase 2
config, never an in-place edit of M1-M5 evidence.

With `--include-dynamic-replay --scenario-ir`, the same derivation adds every
registered vehicle, pedestrian and two-wheeler as a physical replay actor.
The existing lead-vehicle/pedestrian bindings are retained; added actors are
not controllable and are not yet claimed as NuRec pose-bound. That per-tick
claim remains the M7 gate.

For this M6 physical-presence probe only, a failed source-origin spawn may
retry vertically in `0.05 m` increments up to `0.50 m` while preserving source
XY and yaw. The selected adjustment is recorded in runtime spawn evidence.
It is not an alignment pass: M7 audits that correction against the dynamic
physical/render threshold before any TF++ result can be interpreted.

Only `controlled_lead_vehicle` and `controlled_pedestrian` records are marked
as potentially controllable.  Their M6 control mode remains `replay`; all
other dynamic records remain `background_replay`, and static objects remain
uncontrolled collision proxies.

### Scene-0061 M6 Evidence

The first passing M6 probe is a one-tick, fail-closed CARLA/NuRec run bound to
the role-pinned registry. It proves 89 dynamic replay actors, 138 static
collision proxies and one road-topology record have unique CARLA runtime
identities while all six RGB cameras and LiDAR return successfully. The
payload-bound calibrated visibility manifest and coverage audit pass for that
same run. This closes M6 physical presence and coverage only; it does not
waive the per-tick pose thresholds or independent visual/LiDAR checks in
M7-M8.

## M7: Per-Tick Physical and Render Pose Binding

**Objective:** bind CARLA physical state to the exact NRE dynamic-object pose
used for the same sensor transaction.

**Deliverables:**

- `actor_pose_audit.v1.jsonl`, one record per bound object and tick;
- pose-reference, bounding-box-centre offset, coordinate transform, timestamps,
  CARLA actor id, NRE track id, translation and yaw error in every record;
- an A/A/B RGB and LiDAR pose-probe for each controllable actor;
- a static-obstacle placement audit against the NRE image/scene reference.

**Exit gate:** dynamic vehicle translation error is at most `0.50 m`, pedestrian
translation error is at most `0.30 m`, and yaw error is at most `5 deg`, unless
a tighter object-specific contract is declared. Any missing pose or exceeded
threshold fails the run before TF++ KPI interpretation.

### Scene-0061 M7 Evidence

M7 derives a new replay-preserving binding set whose NuRec request poses come
from the CARLA runtime rather than the source trajectory. The explicit
`replay_render_pose_mode=carla_runtime_physical` contract keeps replay as the
actor behaviour policy while making CARLA/NuRec pose disagreement measurable.
The 22-tick run has 22 successful RGB/LiDAR transactions and a 44-row pose
audit: 23 active actor/tick rows pass, 21 pedestrian rows before its source
annotation window are `not_applicable`, and no row fails. At the first common
window both the lead vehicle and pedestrian pass their respective
`0.50 m`/`0.30 m` and `5 deg` gates. Both actors also pass RGB and LiDAR A/A/B
pose probes (stable A/A, changed B after a 1 m target shift).

The static placement audit joins the immutable registry to CARLA static runtime
identity and calibrated 3D-box projections into the same NuRec six-camera
payload set: all 138 required static collision proxies are present and 135 are
geometrically observed. This is a calibrated placement/scene-reference gate;
it deliberately does not claim semantic image detection or LiDAR occupancy,
which remain M8 requirements.

## M8: Unified Geometry and Safety Audits

**Objective:** make physical contact, visual contact, lane state and LiDAR
geometry independently inspectable at every tick.

**Deliverables:**

- `collision_audit.v1.jsonl`: CARLA collision-sensor events, all registered
  object clearance distances, 3D bounding-box overlap, and collision source;
- `lane_audit.v1.jsonl`: lane id, on-road state, lane-invasion events, signed
  centreline/lane-boundary distance, route progress and off-road duration;
- `visibility_audit.v1.jsonl`: calibrated projection of CARLA 3D boxes into
  all six NRE cameras and expected geometric visibility;
- `lidar_world_audit.v1.jsonl`: point/occupancy support for registered actors,
  static obstacles and road boundary, separate from the existing LiDAR-axis
  regression.

**Exit gate:** all required audit rows exist for every tick. A CARLA/NRE
visibility contradiction, unmodelled obstacle contact, lane departure, or
missing LiDAR support is a failed safety result, not an unavailable metric.

### M8 Implementation Status

The fail-closed audit contract and immutable four-stream writer are implemented
in `adapters/scene_safety_audit.py` and `runners/audit_scene_safety.py`.  They
produce `collision_audit.v1.jsonl`, `lane_audit.v1.jsonl`,
`visibility_audit.v1.jsonl`, and `lidar_world_audit.v1.jsonl`; any missing
per-tick source is a failure.  In particular, the writer will not promote a
successful NuRec RPC, a geometric projection, or `collision_count == 0` into
semantic image detection or LiDAR-world evidence.

Scene-0061 has not passed M8.  The CARLA/NuRec runner must still persist these
raw inputs in the same world frame before a remote M8 run is valid:

- ego and every active registry object's CARLA bounding-box centre and extent,
  plus collision sensor event counterpart identity;
- CARLA map waypoint/lane topology and lane-invasion events;
- projected 3D-box rows joined to calibrated NRE payload identity and the
  physical CARLA box state; independent RGB detection is optional post-M11
  assurance, not an M8 gate;
- LiDAR point/occupancy support for every object declared observable in the
  scan, including the road-boundary representation.

### Scene-0061 M8 Runtime Probe

The first valid CARLA/NuRec M8 truth probe used the `sidewalks8m` OpenDRIVE
variant.  The visually similar `scene-right-handed` variant is retained as a
failed preflight: it cannot spawn a later-appearing full-scene replay actor and
therefore produces zero sensor frames.  The valid three-tick probe produces a
continuous CARLA frame identity, 227 non-road physical object states per tick,
a live collision sensor, a live lane-invasion sensor, six RGB payloads and one
NuRec LiDAR payload on every tick.

It does **not** pass M8. The raw sensor-local envelopes are anomalous for a
scene scan: tick one
has 2,759 points confined to approximately `x=0.23..1.31 m`,
`y=-3.02..-0.20 m`; ticks two and three have only 20 and 18 points respectively
in narrow `y=-36 m` bands.  This is recorded in
`m8_lidar_occupancy_with_envelope.v1.jsonl`.

The occlusion-aware physical expectation is now materialized in
`m8_expected_lidar_support.v1.jsonl`. It samples every CARLA OBB surface from
the same-tick LiDAR origin, requires a first-hit ray and uses an explicit 80 m
maximum range. It correctly excludes occluded and out-of-range objects. Even
under that conservative gate, frames 6120/6121/6122 have respectively
113/112/112 expected-scannable objects and zero of them has a LiDAR point in
its physical CARLA box. The immutable four-stream audit bundle
`m8_safety_audit_occlusion_20260727/` passes collision, lane and calibrated
visibility for all three frames, but fails `lidar_world` for all three. This is
a formal M8 failure, not an incomplete metric: successful RGB/LiDAR RPC status
does not compensate for the NuRec LiDAR/world contradiction. Resolve the
NuRec LiDAR response unit, coordinate, time-window or source-scene-content
contradiction before a TF++ baseline.

That physical expected-visible derivation is now available as
`m8_expected_visibility.v1.json`: it joins the three CARLA truth ticks to the
exact six-camera NuRec payloads and records 814 calibrated 3D-box projections
across 224 registered objects.  This is the geometric M8 candidate set.  It is
not an occlusion-aware LiDAR expectation, so LiDAR support may not be inferred
from it.  Semantic image detection is intentionally deferred and is not an M8
or M9 promotion requirement.
The current remote runtime has no installed detector stack (`torch` and
`ultralytics` are absent) and no vehicle/pedestrian detection weight. This is
not a blocker: independent detector evidence is deferred until after M11 as
an optional assurance study. The implemented `rgb_detector_evidence.v1`
contract is retained for that later study; it does not contribute to an M8 or
M9 pass.

### M8 Targeted NuRec Controllability Probe (2026-07-28)

The NuRec 26.04 source confirms that post-hoc `sequence_tracks.json` flag edits
cannot create movable Gaussian content: `CONTROLLABLE` is assigned from tracks
actually present in the `CompositeModel`. Attempt 002 stopped before training
because the Hydra override omitted the `+` required for a new `tracks.ids`
field. Attempt 003 used the corrected override and produced a 100-step
candidate artifact at
`outputs/nurec_scene0061_m8_targeted_track_smoke_attempt_003/.../artifacts/last.usdz`.
That candidate put all six requested IDs under `dynamic_deformables`; the layer
only accepts pedestrian labels, so the three vehicle IDs were registered in
the config but did not acquire vehicle Gaussian content. It is evidence of a
layer/class mismatch, not a successful physical-scene repair, and it did not
replace the formal artifact.

The immutable runtime probe
`outputs/phase2_m8/scene0061_m8_targeted_track_probe_20260728_attempt003/`
then tested four of the unverified tracks against the formal artifact. All four
failed LiDAR A/A/B content change; three also failed RGB content change. The
three-tick geometry diagnostic found the identity CARLA-sensor interpretation
best on ticks two and three; a different axis permutation improves only tick
one and still leaves most expected boxes unsupported. No single axis
interpretation closes all three ticks, and the time windows are already within
0.53 ms of the native scan midpoints. This is therefore a source-scene content
failure, not a justified coordinate reinterpretation. The formal artifact
remains unchanged and M8 remains failed. A fourth candidate was trained with
vehicle IDs explicitly assigned to `dynamic_rigids` and the pedestrian ID
assigned to `dynamic_deformables`; its artifact was provisional until the same
A/A/B RGB and LiDAR probe passed. Attempt 004 did generate a
USDZ, but its NuRec service log reported `0 available tracks`, empty LiDAR
frames, and zero dynamic particles. The scene manifest's eligible tracks are
labelled `EXTERNAL`, while the training recipe still selected `AUTOLABEL`; this
source mismatch explains the empty layers. Attempt 005 reran the same
layer-correct recipe with `TRACK_LABEL_SOURCES=EXTERNAL`: it found 210 source
tracks and selected the requested IDs, but failed at dynamic LiDAR point-cloud
initialization with `NoPointsFoundException: No point clouds were found in the
dataset`. This proves that configuration/source-label repair alone cannot
invent LiDAR support for these actors. The failed log and parsed config are
preserved under
`outputs/phase2_m8/scene0061_m8_external_track_smoke_attempt_005/`; M8 remains
failed. The reproducible coordinate check is recorded in
`outputs/phase2_m8/m8_lidar_geometry_diagnostic.v1.json`.

Attempt 006 used one already verified vehicle track
`4005437c730645c2b628dc1da999e06a` and one already verified pedestrian track
`00cfe5312e5e469bb97d7b64245a98e3` as a minimal pipeline control. The 100-step
training completed and produced a candidate USDZ with SHA-256
`6145eb736ca6f4541419b3878480aefb29a403e5286010c7f09aa0d1554ad83a`. The
isolated runtime A/A/B probe is preserved at
`outputs/phase2_m8/scene0061_m8_verified_track_probe_20260728_attempt006b/`:
the pedestrian passed RGB+LiDAR, while the vehicle passed RGB but failed
`lidar_render_unchanged`. This separates a working dynamic training/RPC path
from the missing vehicle LiDAR content; it does not promote the four control
tracks or close M8.

### M8 Source-Content Coverage Audit

The full M6 registry is now joined to the formal loaded-track inventory by
`runners/audit_nurec_source_content.py`. It does not delete or downgrade
objects when the artifact lacks them: each dynamic object is classified as
`verified`, `unverified`, or `missing_from_artifact`, while static obstacles
remain `unverified` until independent source-scene content evidence is attached.
The formal audit is preserved at
`outputs/phase2_m8/scene0061_nurec_source_content_audit.v1.json` and reports
60 verified objects, 157 unverified required objects, and 10 dynamic tracks
missing from the loaded artifact. All four candidate control/replay tracks are
explicitly unverified. This audit is an M8 prerequisite and cannot be replaced
by successful RGB/LiDAR RPC status alone.

The next M8 repair is consequently on the artifact/data boundary: either
rebuild the renderable artifact from a dataset window that contains LiDAR
support for every CARLA object declared observable, or explicitly remove those
objects from the physical registry before the run. A client-side axis change
or a relaxed occupancy tolerance is not an admissible repair.

## M9: Consistent Replay TF++ Baseline

**Objective:** repeat the M2-M5 baseline only after M6-M8 close the scene
representation gap.

**Execution:** all context remains `static_obstacle` or `background_replay`;
the lead vehicle and pedestrian remain replayed. TF++ receives the same
six-camera/LiDAR transaction that is audited against CARLA state.

**Exit gate:** three independent runs use identical runtime identity and pass
M6-M8, multimodal closure, matched ego controls, physical and visual collision
checks, lane checks, and LiDAR-world checks. The report must state whether the
result is a short-horizon stability result or a full-route result; a three-second
run cannot claim route completion.

## M10: Controlled Lead Vehicle and Pedestrian

**Objective:** promote only the planned lead vehicle and pedestrian from replay
to ego-responsive behavior while retaining full-scene context.

**Execution order:**

1. controlled lead vehicle with replay, then scripted/reactive braking or
   gap-response behavior;
2. controlled pedestrian with replay, then bounded pause/yield/abort behavior
   along its source corridor;
3. no free-space pedestrian path editing and no uncontrolled background actor
   promotion.

**Exit gate:** every control transition has trigger, ego state, issued control,
actual CARLA response, NRE pose and safety-audit evidence. A counterfactual is
valid only when it differs from replay because of the declared behavior policy,
not because the physical and rendered scenes diverged.

## M11: Goal-Level Experiment Matrix

**Objective:** evaluate TF++ only after the scene is causally consistent and
the two controlled actors are available.

**Scope:** S0 replay baseline plus lead-vehicle and pedestrian counterfactuals,
multiple pinned seeds, and repeatable per-case evidence bundles.

**Exit gate:** every case passes M6-M10; comparison reports include collision,
TTC, lane violations, object-visibility failures, pose-error distributions,
LiDAR-world consistency, control latency and route outcome. Claims must be
limited to the covered scenarios and seeds.

## Promotion Policy

No later milestone can compensate for an earlier missing gate. In particular,
an M5-style video, `collision_count == 0`, or a successful RGB/LiDAR RPC does
not promote a run past M6-M8. Existing Phase 1 artifacts are retained for
debugging and regression comparison, while Phase 2 creates new immutable run
directories and evidence archives.
