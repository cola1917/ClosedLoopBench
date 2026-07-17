# CARLA vehicle and pedestrian actors

The CARLA runner now materializes both recorded and ego-responsive scene actors.
NuRec pixels are not treated as physical actors: collision, TTC, distance, and
runtime tracks come only from actors declared in `carla_run_config.json`.

## Supported execution

| Actor | `replay` | `scripted` | `traffic_manager_reactive` |
|---|---|---|---|
| Vehicle | physical trajectory follower | target-speed/TTC control, optional plugin target | CARLA TrafficManager |
| Pedestrian | physical `WalkerControl` trajectory follower | TTC/yield/abort `WalkerControl` | rejected: TrafficManager is vehicle-only |

Pedestrians are spawned from `walker.pedestrian.*`, have `is_invincible=false`
when the blueprint exposes that attribute, and are not projected onto driving
lanes by `--snap-to-map`. Vehicles continue to use `vehicle.*` and may be
projected to a driving waypoint.

## Configuration

See `examples/vehicle_pedestrian_actors.example.json`. Copy its `actors` and
`actor_control` objects into a generated `carla_run_config.json` after applying
the same scene-to-CARLA alignment used by the ego v7 route.

Run with:

```bash
python runners/run_carla_basic_agent.py \
  --run-config carla_run_config.json \
  --execute --snap-to-map --debug-draw --follow-ego
```

`--snap-to-map` intentionally affects only vehicles. A failed pedestrian spawn
does not fall back to a random road spawn; the run fails closed instead of
silently moving the pedestrian away from the source event.

## Behavior plugin

Set `actor_control.behavior_plugin` to `module:function`. The callable uses the
same arguments as `actors.reactive_actor.plan_reactive_actor_control`:

```python
def plan(actor_state, ego_state, *, style, reference_speed_mps):
    return {
        "desired_speed_mps": 1.2,
        "brake": False,
        "should_yield": False,
        "should_abort": False,
        "ttc_sec": None,
        "distance_m": ego_state["distance_m"],
        "target_point": {"x": 30.0, "y": 4.0},
    }
```

`target_point` is optional. When present it changes vehicle steering or walker
direction and therefore changes the physical runtime track. The immutable
source track remains under `reference_trajectory`.

Every `frame_trace.jsonl` actor row records:

- `actor_type`;
- actual `pose` and `speed_mps`;
- time-aligned `reference_pose`;
- `reference_error_m`.

Replay actors are physical and contribute distance/TTC, but do not make the run
interactive. Scripted and TrafficManager actors are ego-responsive and can make
the result `interactive_closed_loop` after physical response is observed.

## NuRec consistency

Recorded vehicles and pedestrians may already be baked into the NuRec camera
appearance. Spawning their CARLA counterparts makes them visible and physical in
the CARLA spectator, but does not automatically insert them into NuRec images.
A camera-policy evaluation must later bind the same actor pose to NuRec dynamic
rendering or remove/mask the baked actor to avoid visual duplicates.
