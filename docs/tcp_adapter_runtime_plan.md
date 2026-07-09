# TCP Adapter Runtime Plan

ClosedLoopBench selects TCP as the first classic end-to-end ego algorithm integration target because it is easy to explain in a simulation-testing interview: it consumes driving observations and route intent, predicts trajectory/control, and is evaluated in CARLA closed-loop settings.

## Why TCP

TCP is a compact classic E2E baseline compared with larger modern stacks. It is suitable as the first learned ego adapter because the benchmark can explain it as:

```text
CARLA current tick sensors + speed + route command
  -> TCP adapter/backend
  -> vehicle control
  -> CARLA ego vehicle
  -> next tick observation
```

ClosedLoopBench does not vendor TCP model code. The external TCP repository or container owns model dependencies, checkpoint loading, preprocessing details, and inference.

## Implemented Locally

The local implementation provides a model-free runtime shell:

- `agents.tcp_runtime_adapter.build_tcp_runtime_plan`
- `agents.tcp_runtime_adapter.TcpRuntimeAdapter`
- `runners/plan_tcp_adapter.py`

This is enough to test:

- runtime plan generation
- required front-camera observation packaging
- ego speed/state and route command handoff
- backend invocation with a fake TCP backend
- vehicle-control validation
- safe fallback when backend, sensor, or control output is invalid

## Not Implemented Locally

These are intentionally left for remote/runtime integration:

- importing the real TCP repo
- loading TCP checkpoints
- camera preprocessing matched to the original TCP training setup
- ROS2 node lifecycle
- publishing to `/carla/ego_vehicle/vehicle_control_cmd`
- running TCP inside CARLA 0.9.16

## Planning Command

```powershell
python runners/plan_tcp_adapter.py `
  --scenario-id scene-1077-integrated `
  --runtime-path E:\models\TCP `
  --checkpoint-path E:\models\TCP\tcp.pth `
  --output outputs\scene-1077-integrated\tcp_runtime_plan.json
```

The plan is valid even if the paths do not exist yet. Path validation belongs to the remote integration step.

## Remote Debug Order

1. Verify CARLA and ScenarioRunner with BasicAgent first.
2. Clone/install TCP separately on the remote machine.
3. Confirm the checkpoint can be loaded by a tiny TCP smoke script outside ClosedLoopBench.
4. Implement a TCP backend object with `predict_control(observation)`.
5. Plug that backend into `TcpRuntimeAdapter`.
6. Add ROS2 bridge publishing only after fake-backend control validation is stable.

This keeps TCP explainable as an ego plugin, not as a second project hidden inside ClosedLoopBench.
