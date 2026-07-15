# Strict Runtime Acceptance

Offline and fake-CARLA tests are regression checks only. They cannot close the
real runtime gates.

## CARLA BasicAgent triplicate

Start a matching CARLA 0.9.16 server, install its Python API in the runner
environment, then run:

```powershell
python runners/run_carla_acceptance_triplicate.py `
  --run-config outputs/scene-1077/carla_run_config.json `
  --output-root E:/sim-data/runs/basic-agent-triplicate `
  --host 127.0.0.1 --port 2000 --max-ticks 600
```

The output root must not already contain any of the three attempt directories.
Every attempt fails closed unless all of the following are present:

- synchronous, consecutive CARLA frame identities;
- a real collision sensor and non-null collision KPI source;
- physical route progress of at least 0.95;
- frame and metric JSONL traces;
- observable displacement for every Actor claimed as interactive;
- successful TrafficManager, sensor, Actor, Ego, weather and world-settings
  cleanup.

Only `acceptance_triplicate.json` with `status: passed` closes the three-run
gate. Three manually selected reports do not.

## NuRec/simulator registration

Capture at least three non-collinear landmarks visible in both nuScenes global
coordinates and the loaded NuRec/simulator runtime. The observations file must
use `runtime_alignment_observations.v1` and record the simulator, renderer,
capture method and NuRec artifact SHA-256.

```powershell
python runners/validate_runtime_alignment.py `
  --scene-package path/to/scene_package.json `
  --observations path/to/runtime_alignment_observations.json `
  --evidence-output path/to/runtime_alignment_evidence.json `
  --promoted-package-output path/to/scene_package.runtime-validated.json
```

Defaults are 0.25 m horizontal error, 0.25 m vertical error and 2 degrees yaw
error. A package can become `runtime_validated` only from passed evidence; the
validated package must be published as a new immutable scene version.
