# Environment Dependency Backlog

This file is the hard boundary between completed offline work and claims that
require CARLA, ROS 2, a GPU, or a real model. None of the items below can be
closed by additional fake-runtime tests.

## Requires Native CARLA 0.9.16

- Connect the matching Python API and verify the server version.
- Load the selected native town, snap/spawn Ego and Actors, and validate map
  alignment rather than only successful API calls.
- Complete one fixed BasicAgent route three consecutive times.
- Observe real collision callback timing, physical route progress, acceleration,
  jerk and cleanup of settings, vehicles and sensors.
- Prove TrafficManager Actor motion and scripted Actor braking/continuation in
  physical trajectories, not only control-call evidence.
- Confirm deterministic seeds and TrafficManager synchronous behavior.

## Requires ROS 2 Humble And CARLA ROS Bridge

- Spawn the real CARLA RGB camera actor and read its actual intrinsic/extrinsic
  calibration.
- Bind typed `sensor_msgs/Image`, `CameraInfo`, odometry, route and
  `CarlaEgoVehicleControl` messages with production QoS.
- Verify passive bridge mode while ClosedLoopBench remains the only owner of
  `world.tick()`.
- Measure observation-to-control latency, missing-frame handling and safe-stop
  timing under synchronous stepping.

## Requires A Model Repository, Checkpoint And GPU

- Choose one TCP or TransFuser revision compatible with CARLA 0.9.16, or record
  the compatibility exception explicitly.
- Build the derived CUDA/PyTorch image and load the real checkpoint.
- Validate model-specific preprocessing, route command encoding and output
  conversion against the generic plugin contract.
- Run the complete BasicAgent versus learned-policy matrix over at least three
  seeds and preserve image tag, commit and checkpoint digest.

## Shared Contract Work Completed Locally

The three sibling projects now consume the canonical `SceneExchangeContracts`
package. NeuralSceneBridge emits Reconstruction Package/result inventory and
TriggerEngine consumes evaluation feedback without copying protocol schemas.

## Acceptance Evidence

The environment phase is complete only when reports pass the experiment matrix
coverage gate with no missing, duplicate, malformed, unknown-KPI or
non-interactive runs. Console screenshots alone are not acceptance evidence.

The implementation order, code changes, stop conditions, commands, and required
evidence are defined in
`docs/environment_integration_verification_plan.md`.
