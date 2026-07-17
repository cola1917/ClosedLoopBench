# NuRec Reconstruction Planning and Validation

ClosedLoopBench can validate and plan against an existing NeuralSceneBridge
training result before any CARLA process is started. The gate verifies:

- Reconstruction Package scene identity and artifact SHA-256/size inventory
- the three requested cameras in `parsed.yaml`
- `n_samples_per_epoch = 1000` and `max_epochs = 1`
- `last.ckpt.global_step = 1000` without unpickling executable checkpoint data

Run the gate with a Reconstruction Package produced by NeuralSceneBridge:

```bash
python runners/plan_reconstruction_integration.py \
  --scenario-ir /path/to/scene_ir.json \
  --reconstruction-package /path/to/reconstruction_package.json \
  --output outputs/scene-0061/reconstruction_integration_plan.json
```

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
