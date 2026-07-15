# Native Host CARLA Orchestration

CARLA 0.9.16 runs natively on the Ubuntu 22.04 host prepared by
`E:/code/env_build`; it is not part of Docker Compose. TriggerEngine,
NeuralSceneBridge, and ClosedLoopBench exchange immutable scene versions through
the shared data root. Per-tick algorithm traffic remains on ROS 2.

## Startup Order

1. Publish a complete scene version to `E:/sim-data`.
2. Start native CARLA with `E:/code/env_build/start_carla.sh`.
3. Start CARLA ROS Bridge in passive mode for an external algorithm run.
4. Start the algorithm-only container and wait for its healthy status.
5. Execute the ClosedLoopBench host run.

ClosedLoopBench spawns Ego and owns synchronous `world.tick()`. Do not run the
separate `carla_spawn_objects` ego launch in this mode, and do not let ROS Bridge
advance the world.

## BasicAgent Baseline

```powershell
python runners/run_host_closed_loop.py `
  --exchange-root E:/sim-data `
  --scene-id cc8c0bf57f984915a77078b10eb33198 --version v001 `
  --run-dir E:/sim-data/runs/basic-agent-001 `
  --carla-map Town04 --execute
```

## External TCP Baseline

Start the algorithm-only Compose profile first, then run:

```powershell
python runners/run_host_closed_loop.py `
  --exchange-root E:/sim-data `
  --scene-id cc8c0bf57f984915a77078b10eb33198 --version v001 `
  --run-dir E:/sim-data/runs/tcp-001 `
  --carla-map Town04 `
  --algorithm-id tcp --algorithm-version <commit-or-checkpoint> `
  --ego-driver ros2_control `
  --execute
```

Before touching the world, the runner validates the Scene Package, probes the
native CARLA endpoint, requires CARLA 0.9.16, and checks the live algorithm ready
file written by the container. Missing ROS control still causes the existing
full-brake safe stop.

The nuScenes-derived OpenDRIVE file remains a portable/local validation artifact.
Until its CARLA topology gate is proven, the first host smoke uses a native CARLA
town and `--snap-to-map`; this limitation must not be reported as faithful map
reconstruction.
