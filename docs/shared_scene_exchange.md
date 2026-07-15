# Shared Scene Exchange

This Scene Package mechanism is the artifact layer beneath the complete
[`shared_exchange_protocol.v1`](shared_exchange_protocol_v1.md).

The shared disk is an offline handoff boundary. TriggerEngine selects a scene,
NeuralSceneBridge may add reconstruction assets, and ClosedLoopBench consumes a
complete Scene Package. Per-tick observations and controls still use ROS2 or a
network transport; they do not use this directory.

## Layout

```text
<exchange-root>/
  scenes/
    cc8c0bf57f984915a77078b10eb33198/  # nuScenes scene token
      v001/
        scene_ir.json
        road.xodr
        scenario.xosc
        scene_package.json
        READY.json
```

For nuScenes, `scene_id` is the native scene token; the readable scene name is
retained in `scene_package.json` source metadata. Scene IDs and versions are
safe single path segments. A published version is
immutable; producers must use a new version instead of replacing `v001`.
Artifact references in `scene_package.json` must be relative, contained by the
version directory, and present at publication time. `SceneExchangeContracts`
owns the canonical `closed_loop_scene_package.v1` Schema; ClosedLoopBench
imports it instead of keeping a local copy.

## Atomic Publication

Publish an already built bundle with:

```powershell
python runners/publish_scene_exchange.py `
  --bundle-dir outputs/p1_scene_0061 `
  --exchange-root E:/sim-data `
  --version v001
```

The publisher validates the bundle, copies it to a hidden sibling staging
directory, validates the copy, writes manifest and complete artifact inventory
digests to `READY.json`, and
renames the complete directory into place. Consumers ignore hidden staging
directories and any version without a valid READY marker. Competing publishers
for the same version cannot overwrite the winner.

The exchange root and staging directory must be on the same filesystem so the
final directory rename is atomic.

## Consumption

```powershell
python runners/consume_scene_exchange.py `
  --exchange-root E:/sim-data `
  --scene-id cc8c0bf57f984915a77078b10eb33198 `
  --version v001 `
  --output outputs/resolved_scene.json
```

Omit `--version` to select the lexically latest valid READY version. The command
revalidates the marker, Scene Package schema, scene ID, path containment, and
referenced-file existence, then returns absolute artifact paths for the local
runtime. A missing, incomplete, altered, or unsafe version fails closed.
