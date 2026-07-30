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
centerline from each polygon's from/to edges, infers directed transitions, and
writes a topology-aware OpenDRIVE 1.4 XML. Where the source map contains a
road-block or intersection region but no explicit lane connector, it adds
separately named `inferred_route_*` roads for the missing recorded-route span.
The complete exchange entry point `runners/build_nuscenes_exchange.py` uses the
same writer.

The output is intentionally local and limited:

- one OpenDRIVE road and one driving lane per selected nuScenes lane;
- inferred junction-local connector roads for ambiguous/turning transitions;
- source-labelled inferred route roads for source-map geometry gaps;
- reciprocal road/lane links only for one-to-one transitions;
- a declared mixed `route_path` for the recorded Ego route, ordered across
  map lanes, map connectors, and only the remaining source-gap roads;
- explicit route access connectors at source-gap/map boundaries;
- an optional, separately named `ego_route_corridor` only when explicitly requested;
- only nuScenes lanes whose `lane_type` is `CAR` are converted;
- piecewise-linear plan-view geometry;
- width estimated from the lane polygon and clamped to a plausible range;
- endpoint and heading-based transition inference with fail-closed topology validation;
- explicit source-boundary metadata for every isolated map lane, with a fail-closed
  boundary audit in the exchange builder;
- raw-lane and complete-network alignment are reported as separate scopes;
- coordinates transformed into the Scene IR local frame.

Example:

```powershell
python runners/build_nuscenes_opendrive.py `
  --dataroot E:/code/nuscenes-mini `
  --scene scene-0061 `
  --radius-m 50 `
  --include-route-inference `
  --output outputs/scene-0061/road.xodr
```

An already generated Scenario IR can be supplied with `--scenario-ir` instead
of `--scene`.

## Explicit limitations

This is not a complete city-scale HD-map conversion. It does not currently
encode traffic lights, stop lines, crosswalks, elevation, superelevation, or
full lane-change markings. Curved lane boundaries are approximated by short
straight OpenDRIVE geometries. Ambiguous branches are represented by explicit
junction connections and connector geometry; CARLA waypoint continuity at
those branches remains a runtime gate.

`runners/build_route_aligned_opendrive.py` remains a separate single-road
control-corridor diagnostic. It is not a valid replacement for the topology
writer. The route-aware topology artifact declares an ordered mixed path over
map lanes, map connectors, and source-gap roads, and joins source gaps to the
map graph through explicit route access junctions. `ego_route_corridor` is
optional and explicitly named, and the map/network gate excludes it.

The output is suitable for XML contract checks and esmini smoke work. CARLA
import, waypoint quality, junction behavior, and TrafficManager routing remain
environment integration gates and must be validated before claiming runtime
map fidelity. Run `tools/audit_carla_xodr_runtime.py` against the exact XODR
SHA used by CARLA to check waypoint continuity, driving-lane membership, and
branch transitions before collecting collision or multimodal evidence.
