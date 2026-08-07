# Multi-road OpenDRIVE topology repair

The previous `road.xodr` contained independent lane-strip roads with incomplete
connectivity. The single-road route-only artifact is useful for diagnosing Ego
replay alignment, but it is not a scene road network and is never the canonical
exchange map.

The topology repair entry point is:

```powershell
python runners/build_nuscenes_topology_opendrive.py `
  --dataroot E:/code/nuscenes-mini `
  --scenario-ir outputs/alignment-validation/cc8c0bf57f984915a77078b10eb33198/scene_ir.json `
  --radius-m 50 `
  --connection-tolerance-m 8 `
  --boundary-connection-tolerance-m 20 `
  --junction-tolerance-m 20 `
  --include-route-inference `
  --output outputs/scene0061_xodr_repair/scene0061_topology_route_v2.xodr
```

For the complete portable exchange bundle, use the topology-specific wrapper
after generating or supplying the Scenario IR:

```powershell
python runners/build_nuscenes_topology_exchange.py `
  --dataroot E:/code/nuscenes-mini `
  --scene scene-0061 `
  --output-dir outputs/scene0061_topology_exchange_v2
```

The exchange wrapper emits a declared mixed Ego route path by default, so
replay/control does not fall back to the diagnostic single-road corridor. The separately
named `ego_route_corridor` is diagnostic-only and requires explicit
`--include-ego-corridor`. Route inference is limited to source gaps; use
`--no-route-inference` only for raw-lane diagnostics.
Route inference remains enabled in both modes; use `--no-route-inference` only
for raw-lane diagnostics.

`runners/build_nuscenes_opendrive.py` and
`runners/build_nuscenes_exchange.py` now use the topology writer as their
canonical implementation. The explicit `runners/build_nuscenes_topology_*`
entries remain available when the map/corridor scope must be visible in the
command itself. All exchange writers fail closed on a single-road or
corridor-only artifact.

Against the current Singapore map export, a fresh default scene-0061 build
contains 67 selected or boundary-closure nuScenes lane roads, 88 map
junction-local connector roads, one source-gap route road, and 15 inferred
junctions. The source intersection boundary allowance is 20 m, while
geometry-only connections remain capped at 8 m unless source edge-line or
intersection evidence exists. Its declared route path is a mixed chain of one
source-gap road, map lanes, and map connectors, and it covers all 39 Ego
reference samples. The source-gap is explicit and auditable; it is not silently
counted as map-lane coverage. These counts are tied to the external map export
and must be regenerated when that input changes.
CARLA waypoint continuity and generated-world lane membership remain runtime
gates. When explicitly requested it additionally contains one diagnostic
`ego_route_corridor` road (road id `1000`); that road is never used as proof of
map topology.

The previous `*_v1.xodr` files remain immutable historical candidates. The
current canonical mixed-path exchange contains 156 roads (67 lane roads, 88
connector roads, and one source-gap route road) and no Ego corridor for the
current map export. Verify generated counts from the CLI rather than treating
ignored output directories or historical artifacts as the source of truth.

The current formal artifacts are:

- `outputs/scene0061_xodr_repair/scene0061_topology_v2.xodr`: historical
  map-only candidate; regenerate it when a standalone repair artifact is
  needed.
- `outputs/scene0061_xodr_repair/scene0061_topology_route_v2.xodr`: historical
  route-aware candidate with an explicitly named Ego corridor.
- `outputs/scene0061_exchange_v2/road.xodr`: canonical exchange output used by
  the replay entry point, with the declared mixed route path joined to the map
  graph and no diagnostic corridor.
- `outputs/scene0061_topology_exchange_v2/road.xodr`: scene-generated
  topology-wrapper exchange bundle with the same mixed-path default scope.

The three currently isolated map lane roads are explicit source-map boundaries,
not silently accepted dangling links. Each carries `userData` identifying its
source lane/block/segment and `topology_boundary_reason`; the contract reports
`isolated_map_lane_unclassified_count=0`. A future generator change that
creates an isolated lane without that metadata fails the exchange build.

The replay entry points now default to the multi-road exchange map at
`outputs/scene0061_exchange_v2/road.xodr`. `tools/run_scene0061_nurec_replay.sh`,
`tools/remote_run_pure_pursuit.sh`, and the topology Scene Package loader reject
a corridor-only or structurally invalid legacy XODR. The validator also reports
network connectivity separately: the current scene-local export has four
components because it selects lanes around all recorded tracks and preserves
source-map boundaries, so it is not a claim of one globally connected CARLA
road graph. The contract also reports
route-road provenance and route-to-map linkage; a passed route-chain check
alone must not be read as map-lane coverage. The one-road artifacts remain
valid only when explicitly selected for control-corridor diagnostics.

When esmini is available, run the local geometry-materialization audit as an
additional check that every XML road is sampled by the simulator's OpenDRIVE
road model:

```powershell
python tools/audit_esmini_xodr_runtime.py `
  --xodr outputs/scene0061_exchange_v2/road.xodr `
  --expected-sha256 eb117dd99f84cdd8072e13aaacc502702dd815658ed4b53e81a00ace931b109e
```

This proves esmini road materialization only. CARLA waypoint membership,
junction traversal, physics, and collision evidence remain separate runtime
gates.

Validate a scene map with both topology gates enabled:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\validate_xodr_topology.ps1 `
  -Xodr .\outputs\scene0061_xodr_repair\scene0061_topology_v2.xodr `
  -RequireMapTopology `
  -RequireJunctionTopology `
  -RequireBoundaryAudit `
  -Output .\outputs\scene0061_xodr_repair\xodr_topology_v2_validation.json
```


nuScenes mini's JSON export does not include explicit lane predecessor and
successor records. The converter binds each lane to its source `road_block`,
rejects same-block parallel-lane links, matches lanes one-to-one within each
source block pair, and adds only one-hop endpoint-closed CAR lanes at the local
selection boundary. Directed transitions are then inferred from endpoint
proximity and centerline heading. Straight transitions with coincident
endpoints use reciprocal road and lane links. Branching and turning transitions
use junction-local connector roads with OpenDRIVE junction connections: every
incoming/outgoing map road references the junction, and every connector carries
one explicit predecessor and successor movement. Connections beyond 8 m are
accepted only when the midpoint is covered by a source intersection polygon or
the source lane edge-line records share an endpoint node; the connector records
that evidence in `userData.topology_evidence`. This avoids both Cartesian
parallel-lane links and dropping branch continuations merely because one map
road cannot encode multiple successors. This is still a local scene graph, not
a city-scale reconstruction.

Local evidence:

- `xodr_topology_route_static_validation.json`: the v1 report is historical;
  its 100% Ego coverage includes the explicit corridor and is not map-only
  evidence.
- `xodr_topology_route_validation.json`: the historical artifact contains 50
  roads, 6 junctions, 35 junction connections, and no dangling references.
  After connector-road generation, regenerate this report before using it as
  evidence; it must include junction-local connector roads and connector
  predecessor/successor links.
- `esmini_topology_route_smoke.json`: the historical v1
  OpenDRIVE/OpenSCENARIO load passed with zero topology warnings. Regenerate
  this smoke for the connector-aware v2 artifact; the validator now rejects
  non-reciprocal road links that esmini reports as reversed-link warnings.
- Use `tools/validate_xodr_route.ps1 -RoadScope map` to measure only the raw
  `nuscenes_lane_*` roads. Use `-RoadScope network` to measure the complete
  multi-road network (raw lanes, connectors, and inferred route roads) while
  excluding the corridor. Use `-RoadScope corridor` to measure only the
  separately named diagnostic corridor. The `all` scope is diagnostic only and must not
  be used to claim network alignment.
- `xodr_topology_route_comparison.png`: visual comparison with the old lane
  strips and the multi-road route-aware result.

The raw-lane `map` report is intentionally separate from the `network` report:
raw nuScenes lane alignment remains lower where the source export has no
explicit connector geometry, while the declared mixed route path records which
map lanes/connectors are used and which source-gap remains. The source-gap
remains explicit even when its access junction shares a component with the map
graph. This is static
geometry evidence, not a claim that nuScenes geometry has been perfectly
reconstructed. The v2 static report distinguishes
structural topology from network connectivity. CARLA waypoint continuity,
generated-world lane membership, actor physics, collision callbacks, and
RGB/LiDAR synchronization remain separate runtime acceptance gates.

For the CARLA-specific map gate, run the following in the CARLA Python
environment against the exact XODR copied into the runtime:

```powershell
python tools/audit_carla_xodr_runtime.py `
  --xodr outputs/scene0061_exchange_v2/road.xodr `
  --scenario-ir outputs/scene0061_exchange_v2/scene_ir.json `
  --expected-sha256 eb117dd99f84cdd8072e13aaacc502702dd815658ed4b53e81a00ace931b109e `
  --carla-python-api <CARLA>/PythonAPI/carla `
  --generate-world `
  --output outputs/scene0061_exchange_v2/carla_xodr_runtime_audit.json
```

This gate checks all 39 Ego samples against CARLA driving waypoints, calls
`Waypoint.next()` for consecutive-sample continuity, and requires road/lane
changes to be explained by a junction branch. It does not replace physical
collision, lane-invasion, or RGB/LiDAR same-tick evidence.
