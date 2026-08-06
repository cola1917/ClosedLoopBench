# NRE `render_lidar` dynamic objects: bug report (forum-ready draft)

Status: prepared 2026-08-06 for posting to the NVIDIA Developer Forums
(https://forums.developer.nvidia.com/, category "NVIDIA Omniverse NuRec", tags:
`camera`, `lidar`). The text below is the draft post; the "Background and
attachments" section is repo-internal context that is not part of the post.

---

## Draft post (English, ready to paste)

**Title:** `[Bug] NuRec 26.04 + nurec-grpc:0.2.0: dynamic objects render in RGB
but are absent from `render_lidar` point clouds at their true positions

**Environment**

- Server: `nvcr.io/nvidia/nre/nre-ga:26.04` (`serve-grpc --artifact-glob
  /scenes/last.usdz --host 127.0.0.1 --port 46443 --test-scenes-are-valid
  --enable-editing-actors`)
- Client: `carlasimulator/nvidia-nurec-grpc:0.2.0` gRPC protos,
  `SensorsimService.render_lidar` (device type PANDAR128)
- Artifact: a NuRec 26.04 reconstruction of nuScenes mini scene-0061
  (`nurec_scene0061_renderable_lidar_v3_6cam_40k_formal_attempt_001`); the
  server startup log reports
  `LayerTrackIds: Initializing track filtering from 223 available tracks`
- GPU: NVIDIA (A100-class), CUDA 12.x

**Summary**

Dynamic objects (10 controllable vehicles in a 223-track scene) render
correctly in `render_rgb` at their requested poses, but `render_lidar` emits
essentially no returns at those poses. Live A/B experiments against the running
server show the LiDAR dynamic-object path is broken internally, and the client
request is provably correct.

**Reproduction sketch**

1. Serve the artifact with the command above.
2. For one frame (e.g. timestamp 1532402928498150, 50 ms window), call
   `render_lidar(lidar_id="lidar_top", device_type="PANDAR128",
   frame_start_us=..., frame_end_us=..., sensor_pose=..., dynamic_objects=[
   {"track_id": "c1958768d48640948f6053d04cffd35b", "pose_pair": {...}}])`.
3. Compare the returned cloud with a render whose `dynamic_objects` is empty:
   the number of returns within 5 m of the requested track pose is identical
   (no vehicle points). The same request list passed to `render_rgb` shows the
   vehicle at the requested pose (edits to the pose move the RGB pixels).

**Evidence**

1. **Return counts near the track pose are pose-invariant.** Rendering the
   scene with `all 37 controllable objects`, `empty`, `target only`, and
   `all minus target` produces the same return count within 5 m of the
   server-reported target ROI (21 at frame 0, 40 at frame 18); the static
   background explains all of it. The target contributes 0 returns at its true
   position.
2. **Each vehicle rendered alone lands at a fixed scatter, not its pose.**
   Every one of the 9 vehicles yields ~110-140 scattered returns whose centroid
   sits ~12 m forward at ground level; none matches the interpolated track pose
   (checked over the whole trajectory; no time-shift matches either).
3. **The scattered points lie on the sensor's own ray grid.** All 193 extra
   points from a target-only render are on fixed azimuth/elevation rays from
   the sensor, and their depths plane-match the checkpoint's per-track gaussians
   at `canonical_position + lidar_extra_signal` in the response frame (no track
   transform applied). The checkpoint's `dynamic_rigids` layer is healthy:
   10 cuboids, ~50 k gaussians, vehicle-sized extents (3.9-6.0 x 1.7-2.3 x
   1.6-2.6 m).
4. **LiDAR output ignores the requested pose for distant objects.** Rendering
   the target-only request with the pose placed at 34.7 m vs 100 m ahead yields
   byte-identical extras (136 cells each); the same pose edit changes the RGB
   image at the true target pixels.
5. **RGB applies the pose, LiDAR does not.** Moving the target by (+5, +3, 0) m
   changes the RGB image at the target's pixels (edited-vs-original diff at
   x 358-392, y 229-261 for frame 18) but changes the LiDAR cloud only in 24
   scattered cells at lateral 20-60 m, never near the true or edited position.

**Call-correctness argument**

- The proto contract for dynamic objects carries only `track_id` + `pose_pair`;
  there is no mesh or material a client could misplace.
- The identical request lists render correctly in RGB, so the server receives
  and applies the poses.
- The server accepts the requests without error and reports 223 available
  tracks at startup.
- The request pattern matches the CARLA NuRec integration flow and the
  recorded frame data (frame-0 point count 18714 reproduces exactly).

**Related observations**

- `render_lidar` is not wrapped by the official CARLA NuRec examples
  (carla-simulator/carla#9734, discussion #9732); it appears to be an
  internal/experimental path.
- nurec-grpc:0.2.0 officially targets NuRec 25.07, while NGC currently ships
  26.x; per forum thread
  https://forums.developer.nvidia.com/t/nurec-please-make-nre-ga-25-07-available-on-ngc/372696
  the 26.04 CARLA integration was still in progress on 2026-06-10. Our
  combination (26.04 + 0.2.0) loads and renders, but the LiDAR dynamic path
  appears only partially wired.
- Related thread on silent dynamic-entity failures (fixed by ensuring a
  CuboidsComponent is present in the NCore data):
  https://forums.developer.nvidia.com/t/nurec-usage-issues/368592

**Questions**

1. Is the dynamic-object LiDAR path (`lidar_dynamic_points.method:
   dynamic_tracks`) known to be incomplete in nre-ga 26.04?
2. Is there a workaround (config, server flag, or client-side convention) that
   makes `render_lidar` place per-track gaussians at the requested pose?
3. Would NVIDIA staff like access to the artifact + a minimal repro client?
   We can provide the USDZ and the exact requests.

---

## Background and attachments (repo-internal, not part of the post)

- Full investigation log: `docs/open_loop_m8_debug_log.md` (r6 rows:
  cross-modal check, GT match, K mismatch, cross-input mix, NRE LiDAR root
  cause, external validation).
- Live A/B scripts (kept in /tmp/opencode during the investigation):
  `ab_dynamic_lidar.py` (all/empty/target-only/all-minus-target),
  `per_track_lidar.py` (per-vehicle renders), `multi_frame_target.py`,
  `pose_invariance_proof.py` (RGB vs LiDAR under a pose edit),
  `pose_invariance_test2.py` (pose placed at 3/34.7/100 m),
  `three_meter_test.py`, `near3_check.py`, `compare_3m_35m.py`,
  `pose_transform_match.py` (candidate transform search against checkpoint
  gaussians), `dump_extra.py`, `cluster_target_diff.py`, `diff_dynamic_lidar.py`.
- Checkpoint inspection (torch.load of `checkpoint.ckpt` inside the server
  image with a Dummy-`find_class` unpickler): `dynamic_rigids` layer summary in
  `inspect_rigids.py`; `lidar_extra_signal` magnitude analysis in
  `layers_compare.py` and `check_extra_signal.py`.
- Key numbers for the post:
  - scene-0061, 223 tracks, 74 controllable (10 vehicles + 64 others).
  - `dynamic_rigids`: 10 cuboids, 50,334 gaussians; per-cuboid extent
    3.9-6.0 x 1.7-2.3 x 1.6-2.6 m; `lidar_extra_signal` max abs 45.5 m.
  - frame 18 (1532402928498150): target ROI (34.73, -1.75, 0.50); all variants
    report 40 returns within 5 m; target-only adds 0 there.
  - pose-invariance: target at 34.7 m vs 100 m -> identical 136-cell extras;
    at 3 m -> 3,313 extras incl. 92 genuinely new cells at the requested spot.
