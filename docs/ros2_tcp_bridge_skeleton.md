# ROS2 TCP Bridge Skeleton

This bridge is the runtime-facing shell between ClosedLoopBench and a TCP-style external ego adapter.

## Scope

Implemented locally:

- ROS2 topic wiring as a plan artifact.
- Injectable node/publisher/subscriber shell for fake tests.
- Sensor, ego-state, and route message cache.
- `TcpRuntimeAdapter.tick(...)` handoff.
- Safe-stop publish on missing or stale observations.

Not implemented locally:

- Real `rclpy` node lifecycle.
- Real ROS2 message classes.
- TCP repo import, torch import, checkpoint loading, or image preprocessing.
- CARLA ROS bridge process management.

## Data Flow

```text
/carla/ego_vehicle/rgb_front/image
/closed_loop/ego/state
/closed_loop/route
  -> Ros2TcpBridge cache
  -> TcpRuntimeAdapter.tick(...)
  -> /carla/ego_vehicle/vehicle_control_cmd
```

The bridge only owns topic wiring and message handoff. TCP inference belongs to the injected backend behind `TcpRuntimeAdapter`.

## Planning Command

```powershell
python runners/plan_ros2_tcp_bridge.py `
  --scenario-id scene-1077-integrated `
  --role-name ego_vehicle `
  --timeout-sec 0.5 `
  --output outputs\scene-1077-integrated\ros2_tcp_bridge_plan.json
```

The generated plan is valid without ROS2 installed. Runtime integration later binds the same plan to a real `rclpy` node and CARLA ROS bridge topics.
