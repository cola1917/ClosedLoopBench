# Sensor / Ego Adapter Contract

## Scope

This contract keeps ClosedLoopBench focused on simulation benchmarking. It
defines the sensor and ego-adapter boundary used by TCP-like external policies,
but it does not install, vendor, or execute TCP, TransFuser, InterFuser, UniAD,
or any other autonomy model.

## Source Boundary

nuScenes six-camera data is a dataset and reconstruction source:

- `CAM_FRONT`
- `CAM_FRONT_LEFT`
- `CAM_FRONT_RIGHT`
- `CAM_BACK`
- `CAM_BACK_LEFT`
- `CAM_BACK_RIGHT`

Closed-loop ego policies must not consume recorded nuScenes images as runtime
observations. During closed-loop evaluation, ego input comes from the current
CARLA tick. This keeps the benchmark closed-loop: ego observes the state caused
by its own previous controls, not a fixed log frame.

## CARLA Camera Profiles

ClosedLoopBench provides model-free camera profiles:

- `tcp_front`: front-camera profile for TCP-style adapters.
- `multi_view`: six-camera CARLA profile for stacks that expect surround views.

The CARLA role names are:

- `rgb_front`
- `rgb_front_left`
- `rgb_front_right`
- `rgb_back`
- `rgb_back_left`
- `rgb_back_right`

For an ego role named `ego_vehicle`, the front-camera topic is:

```text
/carla/ego_vehicle/rgb_front/image
```

## Ego Adapter IO

A TCP-style adapter receives:

- sensor profile from the current CARLA tick
- ego state with `speed_mps`, pose, velocity, and acceleration
- route data with `route_waypoints`, `route_command`, and `target_point`

It publishes:

- vehicle control with `throttle`, `steer`, `brake`, `hand_brake`, and `reverse`
- optional debug data such as predicted waypoints

ClosedLoopBench owns this IO contract and benchmark reporting. The external
adapter owns model preprocessing, checkpoint loading, inference, and any
framework-specific runtime.

## Non Goals

- Do not use nuScenes camera frames as closed-loop ego observations.
- Do not download model dependencies for contract tests.
- Do not make TCP or any other ego model a core dependency.
- Do not implement model-specific image preprocessing in ClosedLoopBench.
