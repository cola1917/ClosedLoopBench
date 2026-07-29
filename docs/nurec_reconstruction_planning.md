# NuRec Reconstruction Planning and Validation

ClosedLoopBench can validate and plan against an existing NeuralSceneBridge
training result before any CARLA process is started. The gate verifies:

- Reconstruction Package scene identity and artifact SHA-256/size inventory
- the requested camera set in `parsed.yaml`
- the requested `n_samples_per_epoch` and `max_epochs`
- the requested `last.ckpt.global_step` without unpickling executable checkpoint data

Run the gate with a Reconstruction Package produced by NeuralSceneBridge:

```bash
python runners/plan_reconstruction_integration.py \
  --scenario-ir /path/to/scene_ir.json \
  --reconstruction-package /path/to/reconstruction_package.json \
  --output outputs/scene-0061/reconstruction_integration_plan.json
```

The defaults retain the three-camera, 1000-step smoke gate. A formal 40k
result must explicitly pass all six `--expected-camera-id` arguments together
with `--expected-global-step 40000 --expected-samples-per-epoch 40000`; this
prevents a smoke artifact from being promoted as the formal reconstruction.

Before starting any formal training, run the source/config smoke gate against
the intended NuRec config, the M6 scene-object registry, and the source track
manifest:

```bash
python runners/audit_nurec_reconstruction_smoke.py \
  --config /path/to/parsed_config.yaml \
  --scene-object-registry /path/to/scene_object_registry.v1.json \
  --source-track-manifest /path/to/sequence_tracks.json \
  --expected-camera-id camera_front \
  --expected-camera-id camera_front_left \
  --expected-camera-id camera_front_right \
  --expected-camera-id camera_back \
  --expected-camera-id camera_back_left \
  --expected-camera-id camera_back_right \
  --output outputs/scene-0061/nurec_reconstruction_smoke.v1.json
```

The gate is intentionally cheap and fail-closed. It rejects a formal training
budget, missing registered dynamic tracks, disabled sequence-track export, and
a safety-relevant static registry when neither
`dataset.generate_static_rigid_cuboid_tracks.enabled` is true nor every static
object has an explicitly configured NuRec track-layer representation. Passing
this gate proves only that the planned source/config boundary is eligible for
a formal run; the four-stream M8 audit remains mandatory after rendering.

Then build the portable motion/map/scenario bundle with the same package:

```bash
python runners/build_nuscenes_exchange.py \
  --dataroot /path/to/nuscenes \
  --scenario-ir /path/to/scene_ir.json \
  --reconstruction-package /path/to/reconstruction_package.json \
  --output-dir outputs/scene-0061-integrated
```

The resulting Scene Package carries the NuRec USDZ, checkpoint, and immutable
Reconstruction Package alongside Scenario IR, OpenDRIVE, and OpenSCENARIO.

## Runtime boundary

Passing this gate proves artifact integrity, exact training-step completion,
configuration, and scene identity. It does **not** prove that CARLA displays the
NuRec reconstruction. CARLA 0.9.16 cannot load the USDZ as a native map or
sensor renderer. A NuRec renderer adapter plus at least three measured,
non-collinear runtime landmarks are required before
`runtime_alignment_evidence.v1` may promote the Scene Package to
`runtime_validated`.
