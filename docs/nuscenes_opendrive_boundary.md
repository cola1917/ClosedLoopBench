# nuScenes Local OpenDRIVE Boundary

## Source decision

The current NeuralSceneBridge NuRec artifact contract validates only:

- `artifacts/last.usdz` (or `usd-out/last.usdz`)
- `config/parsed.yaml`
- `checkpoints/last.ckpt`

It does not guarantee an OpenDRIVE file or CARLA waypoint topology. A NuRec
visual asset must therefore not be treated as an `.xodr` road network.

## Implemented scope

`runners/build_nuscenes_opendrive.py` reads the nuScenes map matching a Scene,
selects lane polygons near Ego and Actor reference trajectories, reconstructs a
centerline from each polygon's from/to edges, and writes OpenDRIVE 1.4 XML.

The output is intentionally local and limited:

- one OpenDRIVE road and one driving lane per selected nuScenes lane;
- only nuScenes lanes whose `lane_type` is `CAR` are converted;
- piecewise-linear plan-view geometry;
- width estimated from the lane polygon and clamped to a plausible range;
- unambiguous end-to-start predecessor/successor links only;
- coordinates transformed into the Scene IR local frame.

Example:

```powershell
python runners/build_nuscenes_opendrive.py `
  --dataroot E:/code/nuscenes-mini `
  --scene scene-0061 `
  --radius-m 35 `
  --output outputs/scene-0061/road.xodr
```

An already generated Scenario IR can be supplied with `--scenario-ir` instead
of `--scene`.

## Explicit limitations

This is not a complete city-scale HD-map conversion. It does not currently
encode junction objects, traffic lights, stop lines, crosswalks, elevation,
superelevation, lane-change markings, or ambiguous branching topology. Curved
lane boundaries are approximated by short straight OpenDRIVE geometries.

The output is suitable for XML contract checks and esmini smoke work. CARLA
import, waypoint quality, junction behavior, and TrafficManager routing remain
environment integration gates and must be validated before claiming runtime
map fidelity.
