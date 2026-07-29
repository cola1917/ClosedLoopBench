# M8 Candidate Recommendation

This is the recommended recovery path for Scene-0061 after the remote
reconstruction host becomes reachable. It is intentionally fail-closed: a
small NuRec candidate is allowed to reduce training cost, but it never reduces
the CARLA physical world or the M8 audit scope.

## Current decision

Do not start a formal 40k reconstruction from the retained artifact021
artifact. The local evidence is not promotable:

- the editable LiDAR window manifest is `failed` because the sampled frames
  are approximately `0.05`, `0.10`, and `0.15` seconds, while the protected
  pedestrian lifecycle starts at approximately `1.050097` seconds;
- the retained four-stream summary has collision, lane, and calibrated
  visibility passing, but `lidar_world` fails on all three ticks;
- the old render selection is `failed` with 43 repair blockers and includes
  static candidates without a declared NuRec geometry path;
- the candidate config derivation correctly rejects those static candidates.

The old USDZ is useful as diagnostic evidence only. It is not a passing
candidate and must not authorize a formal run.

## Candidate versus physical scope

The candidate NuRec layer should initially be bounded to the protected dynamic
interaction tracks:

- controlled lead vehicle: `c1958768d48640948f6053d04cffd35b`;
- controlled pedestrian: `71603dd1a2ba4e9daf095535e38310ac`;
- any additional dynamic or static object only when it has a real, auditable
  NuRec representation in the selected artifact;
- road boundary only through a declared map/geometry layer, never through a
  fabricated dynamic track.

This is a render-candidate scope, not a reduced scene definition. The complete
CARLA registry remains authoritative: 228 objects (87 dynamic, 140 static,
and the road boundary), with all actors, static collision proxies, lane state,
and collision state retained. The same complete registry must be supplied to
the four M8 audits.

The roadside parked vehicle must be identified from the registry/source pose
and retained as a CARLA collision object. It may enter the NuRec candidate only
after its source or static geometry path is explicit. Do not infer its identity
from a frame index or silently turn it into background.

## Recovery sequence

1. Obtain native NCore frame-end timestamps inside the pedestrian lifecycle.
   Use the actual sequence-store timestamps; do not shift the old three frame
   timestamps. Collect at least three consecutive same-tick frames, expected
   near `1.10`, `1.15`, and `1.20` seconds only as a search range.
2. Re-run dynamic and static source LiDAR audits using those native frames.
   Build a new `lidar_quality_window_manifest.v1.json` in which both protected
   tracks have a continuous three-frame editable window. Preserve the complete
   CARLA registry hash in the evidence.
3. Produce a passed render selection. Every selected static object must carry
   a real NuRec geometry layer/path. Never use
   `generate_static_rigid_cuboid_tracks=true` as a substitute for missing
   source geometry.
4. Derive the bounded candidate config with at most 1000 samples per epoch
   and one epoch, six cameras, `lidar_top`, sequence tracks enabled,
   `TRACK_LABEL_SOURCES=EXTERNAL`, vehicles in `dynamic_rigids`, and the
   pedestrian in `dynamic_deformables`.
5. Run the source/config smoke gate. If it fails, stop before any artifact
   training. A passing source/config smoke is necessary but not sufficient.
6. Train/render only the bounded candidate. Run a same-tick CARLA/NuRec probe
   on the three native frames and regenerate all four independent streams:
   collision, lane, calibrated visibility, and LiDAR-world.
7. Run `audit_m8_formal_promotion.py`. Promotion is valid only when the
   candidate smoke, editable windows, artifact, all four streams, and their
   common non-empty frame set are `passed`. Any missing occupancy, including a
   static or replay object expected to be observable, stops the process.
8. Only a promotion report with
   `formal_reconstruction_allowed=true` may derive or start the formal 40k
   reconstruction.

## Why this is the lowest-risk recommendation

The current failure is object-level NuRec/LiDAR coverage, not a total sensor
point-count failure or a justified coordinate-axis issue. Narrowing the
candidate reduces VRAM and training time, and fixes the pedestrian timing
mistake, but it cannot hide missing world occupancy. The full-registry audit
therefore remains the deciding test for M8.

