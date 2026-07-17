# Reference ROS2 algorithm plugins

These deterministic reference algorithms prove that an external Docker plugin can
publish `CarlaEgoVehicleControl`, own the ego control channel, and participate in
the same report/metrics flow. They are transport baselines, not learned models.

- `reference_cruise_035`: constant throttle 0.35
- `reference_cruise_055`: constant throttle 0.55
- `reference_pure_pursuit_short`: current-tick ego/route observation with short lookahead
- `reference_pure_pursuit_long`: current-tick ego/route observation with long lookahead

The pure-pursuit variants are deterministic route-following algorithms, not learned
camera policies. Unlike the cruise transport baselines, they prove an actual
observation -> algorithm -> frame-matched control round trip.

Both use `reference_plugins:create_backend` and the same immutable reference
checkpoint manifest. A real TCP or TransFuser integration replaces only the
mounted plugin repository, checkpoint, and derived dependency image.
