# Route-aligned OpenDRIVE diagnostic

This artifact is intentionally a single-road Ego control corridor. It is not a
repair of the nuScenes map and must not replace the multi-road topology
artifact used for scene exchange or map inspection.

The historical multi-road result for scene-0061 is
`outputs/scene0061_xodr_repair/scene0061_topology_route_v1.xodr`. The v2
topology implementation adds explicit junction connector roads; generate its
map-only and route-aware artifacts with
`runners/build_nuscenes_topology_opendrive.py` before using them as current
evidence.

The repaired path is generated from the Scenario IR Ego trajectory:

```powershell
python runners/build_route_aligned_opendrive.py `
  --scenario-ir outputs/alignment-validation/cc8c0bf57f984915a77078b10eb33198/scene_ir.json `
  --output outputs/scene0061_xodr_repair/scene0061_route_aligned_v1.xodr `
  --lane-width-m 3.7 `
  --extension-m 10 `
  --sample-spacing-m 2
```

`adapters/ir_to_opendrive.py` uses the same generator for the generic MVP
fallback. Its single road is expected there because Scenario IR alone does not
contain a recoverable map graph. CARLA waypoint continuity, actor spawning,
physics, collision events, and NuRec RGB/LiDAR consistency remain runtime
acceptance checks.

Local static corridor comparison and visualization:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\validate_xodr_route.ps1 `
  -BaselineXodr .\outputs\alignment-validation\cc8c0bf57f984915a77078b10eb33198\road.xodr `
  -FixedXodr .\outputs\scene0061_xodr_repair\scene0061_route_aligned_v1.xodr `
  -ScenarioIr .\outputs\alignment-validation\cc8c0bf57f984915a77078b10eb33198\scene_ir.json `
  -RoadScope corridor `
  -Output .\outputs\scene0061_xodr_repair\xodr_route_static_validation.json
```

The scene-0061 corridor reaches 100% Ego inside-lane coverage, 0.018 m
centerline-distance P95, and 2.258 degree heading-error P95 in the local
static check. Those numbers describe only corridor alignment; they do not
measure map reconstruction and do not replace a CARLA runtime report.
