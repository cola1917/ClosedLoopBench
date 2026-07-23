# scene-0061 CARLA state display

## Purpose

The CARLA window is a synchronized **world-state explainer**. It is not a
camera sensor and must never be presented as NuRec output. The independent
NuRec window remains the source of the six rendered camera streams.

The display deliberately uses the loaded `road.xodr` rather than a CARLA Town
or a synthetic grid. scene-0061's OpenDRIVE conversion is a local lane map,
not a city mesh; the renderer therefore shows only simple lane surfaces,
boundaries, and centre marks. Buildings, road texture, and sky are outside its
contract.

## Visual contract

Every rendered CARLA state frame contains:

- width-derived OpenDRIVE lane surfaces and boundaries near the synchronized
  ego and controlled actor;
- ego, vehicle, and pedestrian 3D bbox proxies with a height projection and
  heading arrow;
- a two-line label next to each key bbox with CARLA actor ID, NuRec track ID,
  actor type, and speed, without repeating those long labels in the road area;
- a distinct orange controlled actor with its recent dashed reference trace;
- a compact fixed upper-left HUD containing the frame ID, simulation timestamp,
  zero-error shared-frame contract, map location/source/hash prefix, CARLA
  actor ID, NuRec track ID, actor type, speed, and bbox dimensions;
- `EGO`, `CTRL`, or `ACTOR` markers at each proxy, but no repeated long labels
  across the road surface.

The source Scenario IR declares yaw in degrees. The renderer converts it to
radians only for the bbox and heading drawing, keeping report values in the
source unit.

## Synchronization and modes

`run_scene0061_dual_window.py` renders the CARLA state canvas and the NuRec
camera grid from the same `FramePacket`. Both therefore use the same
`frame_id`, timestamp, ego/actor state, and NuRec-to-CARLA mapping. The two
windows do not maintain separate clocks.

- `formal_acceptance`: six cameras in front-left, front, front-right,
  back-left, back, back-right order; saves raw frames and both screenshots.
- `preview_debug`: one or three cameras; no image save requirement; optional
  NuRec RGB debug overlay only when `--overlay` is explicitly supplied.

The CARLA state view may be captured for evidence only after a remote runtime
has verified the actor mapping, live LiDAR, and synchronization gates. A local
unit test or synthetic renderer frame is not formal closed-loop evidence.

## Remote capture check

Use a new output directory and the normal formal invocation. Confirm the
report records `window_contract.carla_window` with lane surfaces and compact
HUD, `map_validation.status=matched`, `same_frame_packet_drives_both_windows`,
and screenshots for the same `frame_id`/`timestamp_us`.
