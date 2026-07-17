# NuRec RGB/LiDAR Actor Closed Loop

## Final Boundary

The final project is not a general neural-scene editor.

- Ego is fully parameterized by the driving algorithm.
- One primary vehicle is ego-responsive in CARLA and its current physical root
  pose is sent to NuRec.
- One pedestrian may be ego-responsive only through speed, pause, yield, and
  abort along the recorded source corridor. Free-space path edits and skeleton
  animation edits are not supported.
- Background actors remain source-trajectory replay.
- RGB and LiDAR must be generated from the same frame, pose interval, and
  dynamic-object payload. Radar is not part of the current NuRec acceptance
  claim. IMU, odometry, and ego kinematics remain CARLA-derived signals.

Cosmos Transfer remains an optional offline RGB appearance/data-augmentation
stage. It receives videos after acceptance, not the USDZ and not live CARLA
poses. It must not hide a missing/ghosted dynamic track. See
`docs/cosmos_offline_derivation.md`.

## Identity Chain

`actor_binding_set.v1` freezes one identity across four systems:

```text
nuScenes instance_token
  = Scenario IR source_track_id
  = NuRec dynamic-object track_id
  -> deterministic CARLA role_name
  -> measured CARLA runtime actor id
```

An asset-level track name is not sufficient. A formal inventory promotes a
track only after the loaded NuRec runtime accepts a non-zero dynamic pose probe
for RGB and LiDAR with the same actor digest.

Relevant code:

- `adapters/actor_catalog.py`: repairs event references missing from a mined
  actor subset and ranks source candidates.
- `adapters/actor_binding.py`: builds and validates cross-runtime identity.
- `adapters/nurec_inventory.py`: records loaded runtime tracks and RGB/LiDAR
  dynamic-pose probes.
- `adapters/nurec_multimodal.py`: builds the common frame/dynamic-object payload.
- `adapters/nurec_grpc_dispatch.py`: enforces the version-specific protobuf
  encoder boundary and records response hashes/latency.
- `adapters/nurec_260_client.py`: maps the SDK-neutral transaction to the
  installed NRE 26.04 `SensorsimService` RGB/LiDAR protobufs, verifies JPEG
  dimensions and LiDAR XYZ/intensity counts, and keeps the runtime sequence ID
  separate from the canonical nuScenes scene token.
- `adapters/nurec_runtime_handler.py`: connects CARLA frame contexts to NuRec.
- `runners/validate_multimodal_closed_loop.py`: final fail-closed evidence gate.

The binding, runtime inventory, synchronized frame request, and response
evidence are canonical SceneExchangeContracts artifacts. They remain
independently inspectable outside the runner, and the gRPC dispatch metadata is
validated again after a version-specific encoder returns.

## Frame Transaction

For every synchronous CARLA snapshot:

1. Sample ego and bound physical actor poses.
2. Form start/end pose pairs from the previous and current snapshot.
   CARLA `elapsed_seconds` timestamps the sensor transaction, while the
   run-relative scenario clock indexes replay trajectories; they are never
   treated as the same clock.
3. Convert scene-local simulator poses into NuRec/nuScenes global coordinates
   through a runtime-validated Scene Package alignment.
4. Compose each camera/LiDAR `sensor_to_ego` calibration.
5. Canonicalize and hash one shared dynamic-object list.
6. Send every RGB and LiDAR RPC through an encoder that must echo the same frame
   id, modality, and dynamic-object hash before dispatch.
7. Validate the returned JPEG dimensions or LiDAR XYZ/intensity cardinality,
   then hash each serialized RPC response and record latency and typed response
   metadata.
8. Fail the run if a required response is missing, cross-frame, over the latency
   threshold, or references a different actor hash.

The report sets `runtime.multimodal_sensor.sensor_closed_loop=true` only when all
frames pass. `interactive_closed_loop` by itself remains a CARLA physics claim,
not a NuRec sensor claim.

## scene-0061 Candidate Preparation

The complete local nuScenes catalog contains 227 annotated instances, while the
existing mined Scenario IR contains only 12 selected actors. Its event evidence
references the front-car track
`c1958768d48640948f6053d04cffd35b`, but that track is not present in its
`actors[]`; `prepare_actor_catalog.py` repairs this inconsistency from the source
metadata.

Source-geometry ranking currently recommends:

- primary vehicle: `c1958768d48640948f6053d04cffd35b` (39 keyframes,
  initially about 39 m ahead and 2.2 m lateral, source closest approach about
  8.8 m);
- bounded pedestrian: `71603dd1a2ba4e9daf095535e38310ac`
  (37 keyframes, source closest approach about 4.9 m);
- early high-risk pedestrian alternative:
  `0e79e7bed1d543c9a00acfbf90ff60b3` (22 keyframes, about 2.4 m at 2 s).

These are candidates, not runtime proof. See
`examples/scene0061_actor_selection.v1.json`.

```bash
python -m runners.prepare_actor_catalog \
  --dataroot /path/to/nuscenes-mini \
  --scene scene-0061 \
  --scenario-ir /path/to/mined/scene_ir.json \
  --include-actor c1958768d48640948f6053d04cffd35b \
  --include-actor 71603dd1a2ba4e9daf095535e38310ac \
  --output /path/to/scene_ir.actor-ready.json \
  --audit-output /path/to/actor_candidate_audit.json
```

After the NuRec runtime probe inventory exists:

```bash
python -m runners.build_actor_bindings \
  --scenario-ir /path/to/scene_ir.actor-ready.json \
  --actor-id c1958768d48640948f6053d04cffd35b \
  --actor-id 71603dd1a2ba4e9daf095535e38310ac \
  --control-mode c1958768d48640948f6053d04cffd35b=scripted \
  --control-mode 71603dd1a2ba4e9daf095535e38310ac=scripted \
  --nurec-track-inventory /path/to/nurec_runtime_track_inventory.json \
  --require-ready \
  --output /path/to/actor_bindings.json
```

## Dynamic Ghosting Rebuild

The NCore converter now has two explicit cuboid cadences:

- `keyframes` is the compatibility default;
- `lidar-sweeps` interpolates translation and rotation at each LiDAR sweep.

The NeuralSceneBridge formal conversion script defaults to `lidar-sweeps`.
This is a reconstruction-input fix, not a runtime scene edit. The current 40k
USDZ must be rebuilt/retrained before it can benefit, and the rebuilt primary
tracks must still pass the runtime pose probe.

## Remaining Environment Work

The concrete NRE 26.04 encoder/client and local evidence gates are implemented.
The installed protobuf exposes `render_rgb` and `render_lidar`; its LiDAR model
is limited to `PANDAR128` or `AT128`. Therefore the acceptance claim is a
verified NuRec LiDAR sensor loop, not exact nuScenes HDL-32E emulation.

The handler factory reads these fields from the normal run configuration:

```json
{
  "nurec_runtime": {
    "python_api_path": "/path/to/CARLA/PythonAPI/examples/nvidia/nurec",
    "target": "127.0.0.1:46435",
    "runtime_scene_id": "scene-0061",
    "scene_start_us": 0,
    "scene_package": "/path/to/scene_package.runtime-validated.json",
    "actor_bindings": "/path/to/actor_bindings.json",
    "camera_specs": [],
    "lidar_specs": []
  }
}
```

`scene_start_us` is required even when its measured value is zero. Each camera
ID must be returned by `get_available_cameras`; for scene-0061 the observed
logical IDs include `camera_front`, `camera_front_left`, and
`camera_front_right`. Every camera and LiDAR spec must contain a measured
`sensor_to_ego` 4x4 transform. Do not use an identity placeholder in formal
acceptance.

The three-run command binds the concrete implementation explicitly:

```bash
python -m runners.run_carla_acceptance_triplicate \
  --run-config /path/to/run_config.json \
  --output-root /path/to/acceptance \
  --opendrive /path/to/scene.xodr \
  --sensor-handler-factory adapters.nurec_260_client:build_nurec_260_handler \
  --require-multimodal
```

The following items still require the NVIDIA/CARLA server and cannot be claimed
from local tests:

1. Rebuild scene-0061 NCore with dense cuboids and compare front-car ghosting.
2. Run the non-zero pose probes for both selected tracks and export the runtime
   inventory.
3. Promote Scene Package alignment to `runtime_validated` using measured
   landmarks.
4. Run the CARLA handler with six RGB cameras and at least one verified NuRec
   LiDAR request on every frame.
5. Pass `validate_multimodal_closed_loop.py`, then three consecutive acceptance
   runs with `--require-multimodal`.

Until these pass, the accurate status is: CARLA actor/multimodal integration
implemented and locally tested; real NuRec actor RGB/LiDAR closure pending
server evidence.
