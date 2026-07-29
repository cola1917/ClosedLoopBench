# M8 Candidate Smoke Runbook

The M8 order is deliberately two-stage:

1. Build a low-budget candidate from a complete CARLA registry. The NuRec
   render selection may contain only objects with an auditable ego-corridor
   and LiDAR-quality basis, but it never removes a CARLA actor or collision
   proxy.
2. Run the candidate artifact at the same CARLA ticks and regenerate the four
   immutable streams: collision, lane, calibrated visibility, and LiDAR-world.
3. Run `runners/audit_m8_formal_promotion.py`. A formal reconstruction is
   allowed only when the candidate smoke, required editable quality windows,
   artifact, and all four streams pass on the same non-empty frame set.

## Remote recovery sequence

After the remote VM is reachable:

1. Pull the local branch from GitHub. Do not copy an old bundle over the
   worktree.
2. Read native NCore frame-end timestamps covering the pedestrian lifecycle;
   do not infer them from the first three smoke frames. Collect at least three
   consecutive same-tick frames while the pedestrian is active (the expected
   interval is around 1.10, 1.15, and 1.20 seconds, subject to native frame
   timestamps).
3. Re-run dynamic and static source LiDAR audits and build a new immutable
   `lidar_quality_window_manifest.v1.json`. Both the lead vehicle and
   pedestrian must have at least three editable frames.
4. Build a passed render selection and derive a fresh candidate config with
   at most 1000 samples per epoch, one epoch, six cameras, `lidar_top`,
   sequence tracks enabled, `TRACK_LABEL_SOURCES=EXTERNAL`,
   `dynamic_rigids` for vehicles and `dynamic_deformables` for the pedestrian.
5. Run source/config smoke only. Do not start the 40k reconstruction if this
   gate fails.
6. If smoke passes, render the candidate, run a new same-tick CARLA/NuRec
   three-tick probe, and regenerate expected visibility, expected LiDAR
   support, independent occupancy, and all four M8 streams.
7. Run the promotion gate. Only a `passed` report with
   `formal_reconstruction_allowed=true` authorizes deriving a formal 40k
   config.

The current local evidence is not promotable: candidate quality window v4 is
failed because the pedestrian is outside the three collected frames, and the
retained four-stream summary is failed because LiDAR-world fails all three
frames.
