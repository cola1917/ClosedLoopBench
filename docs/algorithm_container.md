# External Algorithm Container

ClosedLoopBench keeps CARLA 0.9.16 on the Ubuntu 22.04 host prepared by
`E:/code/env_build`. The algorithm image is based on ROS 2 Humble and does not
install, launch, or own CARLA. ClosedLoopBench remains the only owner of the
CARLA synchronous tick.

## Process Boundary

```text
host CARLA :2000 <-- ClosedLoopBench / passive CARLA ROS bridge
                               ^
                               | ROS 2 sensor, route and control topics
                               v
                   external algorithm container

TriggerEngine / NeuralSceneBridge / ClosedLoopBench
                   <--> E:/sim-data
```

The shared disk carries immutable scene packages, run requests, reports, and
logs. It is not the per-tick control channel. ROS 2 DDS carries observations and
`/carla/ego_vehicle/vehicle_control_cmd` during a run.

The verified model-free route loop publishes a JSON `std_msgs/String` on
`/closed_loop/ego/observation`. Every message contains a unique
`observation_id`, current CARLA ego pose/speed/velocity/acceleration, and route
waypoints/target. The plugin copies that ID into
`CarlaEgoVehicleControl.header.frame_id`; the host rejects stale or mismatched
commands and applies full braking on timeout.

On the target Linux host, use `network_mode=host` so DDS discovery works without
maintaining a large UDP port map. `CARLA_HOST=127.0.0.1` is only plugin metadata;
the preferred design is that the algorithm talks to ROS 2 and never ticks CARLA.
Set `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` in both the algorithm container and the
host runner environment. FastDDS otherwise discovers the host endpoint but can
select shared-memory transport across Docker IPC namespaces, making the topic
visible while delivering no samples. Use a dedicated `ROS_DOMAIN_ID` per run
family to avoid stale ROS2 daemon/discovery state.
For Docker Desktop, host networking and DDS behavior must be validated; a
configurable `ALGORITHM_NETWORK_MODE` and `CARLA_HOST=host.docker.internal` are
available, but this is not the production baseline.

## Plugin Contract

The mounted algorithm repository must expose a factory selected with
`ALGORITHM_PLUGIN=module:factory`:

```python
def create_backend(config):
    return Backend(config)

class Backend:
    def health_check(self):
        return {"status": "ready"}

    def predict_control(self, observation):
        ...

    def run(self):
        # Blocking lifecycle, normally rclpy.spin(node).
        ...
```

The backend owns model-specific imports, preprocessing, checkpoint loading,
ROS 2 subscriptions, and publication. `predict_control()` must return the
existing ClosedLoopBench vehicle-control contract. `run()` or `run_forever()`
must be a real blocking transport loop.

The container deliberately has no sample inference backend. Startup fails when
the repository, checkpoint, plugin factory, `predict_control()`, health check,
or blocking lifecycle is absent. This prevents a placeholder from being
reported as a successful TCP run.

## Configure And Start

Create a local env file from `docker/algorithm.env.example`, pointing it at an
external TCP repository and checkpoint. Checkpoints and model source remain
outside this repository.

```powershell
docker compose `
  --env-file docker/algorithm.env.local `
  -f docker/compose.algorithm.yml `
  build

docker compose `
  --env-file docker/algorithm.env.local `
  -f docker/compose.algorithm.yml `
  run --rm ego-algorithm preflight

docker compose `
  --env-file docker/algorithm.env.local `
  -f docker/compose.algorithm.yml `
  up ego-algorithm
```

The Compose file starts only `ego-algorithm`. Start host CARLA, the passive ROS
bridge, and ClosedLoopBench separately. GPU/CUDA/PyTorch layers are intentionally
the responsibility of the selected algorithm image or a derived Dockerfile,
because TCP checkpoints are tied to their upstream repository versions.

The container health check refreshes `/sim-data/runtime/ego-algorithm.ready.json`.
The host runner rejects a missing, mismatched, or stale heartbeat before it
spawns Ego.

## Integration Gate

Before calling an algorithm integrated, capture all of the following:

1. `preflight` reports the exact external repository and checkpoint mounts.
2. The ROS 2 plugin receives current-tick camera, speed, and route data.
3. ClosedLoopBench accepts at least one fresh control command.
4. Stale or missing commands still trigger `Ros2ControlDriver` full braking.
5. A fixed scene completes and produces a report with algorithm commit,
   checkpoint hash, CARLA 0.9.16, ROS 2 Humble, seed, and KPI values.

The repository includes four deterministic reference plugins under
`examples/reference_algorithm_plugins`:

- two fixed-throttle transport baselines, used to verify Docker mounting,
  discovery, safe-stop, real CARLA actuation, and report comparison;
- two pure-pursuit variants, which consume current-tick ego/route observations
  and return frame-matched steering/throttle controls.

On scene-0061 v7, both pure-pursuit variants completed more than 99% route
progress without collision. They prove the state/route observation-control
round trip, but are not learned camera policies. The remaining TCP/TransFuser
gate is to add synchronized current-tick camera tensors to the same observation
ID and load a real upstream checkpoint.

BasicAgent remains the first host-side baseline. TCP is the first external
learned baseline; adding TransFuser later should require only another mounted
plugin and derived dependency image, not a change to the closed-loop clock.
