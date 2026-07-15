# ClosedLoopBench

ClosedLoopBench consumes Scenario IR from TriggerEngine and compiles it into executable CARLA / ScenarioRunner closed-loop evaluations.

Architecture and contract boundary: [`docs/architecture.md`](docs/architecture.md).

Core CARLA/ROS2 environment handoff: [`docs/core_closed_loop_integration.md`](docs/core_closed_loop_integration.md).

Shared-disk publication: [`docs/shared_scene_exchange.md`](docs/shared_scene_exchange.md).

Three-project message and job protocol:
[`docs/shared_exchange_protocol_v1.md`](docs/shared_exchange_protocol_v1.md).

Native CARLA and algorithm-container orchestration:
[`docs/native_host_orchestration.md`](docs/native_host_orchestration.md).

Environment-only remaining work:
[`docs/environment_dependency_backlog.md`](docs/environment_dependency_backlog.md).

Step-by-step environment coding and acceptance plan:
[`docs/environment_integration_verification_plan.md`](docs/environment_integration_verification_plan.md).

Algorithm/ODD/seed matrix planning:
[`docs/offline_experiment_planning.md`](docs/offline_experiment_planning.md).

## Role

ClosedLoopBench is the interactive evaluation layer: it turns Scenario IR into CARLA run config, ScenarioRunner/OpenSCENARIO/OpenDRIVE exchange artifacts, and comparable closed-loop reports.

It runs:

- ego closed-loop policies
- replay or reactive actor policies
- CARLA physics and world stepping
- closed-loop safety, comfort, rule, and progress metrics
- optional NuRec / Cosmos sensor realism integration from NeuralSceneBridge

## Main Input

Required:

- `Scenario IR` from Project 1 / TriggerEngine

Optional:

- `Reconstruction Package` from NeuralSceneBridge
- Cosmos Transfer1 configuration from NeuralSceneBridge

## Main Output

- CARLA ScenarioRunner config/scripts
- closed-loop evaluation reports

See `docs/data_contract.md` and `schemas/closed_loop_run.mvp.schema.json`.


## Exchange validation

Build the complete portable exchange package directly from one nuScenes scene:

```bash
python runners/build_nuscenes_exchange.py --dataroot E:/code/nuscenes-mini --version v1.0-mini --scene scene-0061 --output-dir outputs/p1_scene_0061
```

This produces `scene_ir.json`, `road.xodr`, `scenario.xosc`, and
`scene_package.json`. The local nuScenes-to-OpenDRIVE converter intentionally
has a limited topology scope; see
[`docs/nuscenes_opendrive_boundary.md`](docs/nuscenes_opendrive_boundary.md).

Publish the completed bundle as an immutable shared-disk version:

```bash
python runners/publish_scene_exchange.py --bundle-dir outputs/p1_scene_0061 --exchange-root E:/sim-data --version v001
```

Prepare the native-host CARLA BasicAgent run without executing it:

```bash
python runners/run_host_closed_loop.py --exchange-root E:/sim-data --scene-id cc8c0bf57f984915a77078b10eb33198 --version v001 --run-dir E:/sim-data/runs/basic-agent-001
```

The external algorithm Compose file contains only the algorithm service; CARLA
0.9.16 remains on the host. See
[`docs/algorithm_container.md`](docs/algorithm_container.md).

Run every acceptance gate that requires no CARLA, ROS 2, GPU, network, or model:

```bash
python runners/run_offline_acceptance.py --output outputs/offline_acceptance.json
```

Build ScenarioRunner-oriented config:

```bash
python runners/build_carla_config.py --scenario-ir E:/code/TriggerEngine/outputs/scenario_ir/scene-1077.mvp.json --output outputs/scene-1077/carla_run_config.json
```

Build exchange artifacts without CARLA:

```bash
python runners/build_opendrive.py --scenario-ir E:/code/TriggerEngine/outputs/scenario_ir/scene-1077.mvp.json --output outputs/scene-1077/road.xodr
python runners/build_openscenario.py --scenario-ir E:/code/TriggerEngine/outputs/scenario_ir/scene-1077.mvp.json --output outputs/scene-1077/scenario.xosc --road-file road.xodr
python runners/esmini_smoke.py --xosc outputs/scene-1077/scenario.xosc
```

The esmini smoke test skips cleanly when esmini is not installed. Set `ESMINI_BIN` to enable it.

## Closed-loop dry run

Generate the MVP closed-loop report without launching CARLA:

```bash
python runners/run_closed_loop.py --run-config outputs/scene-1077/carla_run_config.json --output outputs/scene-1077/closed_loop_report.json
```

This dry-run runner is the handoff point for the future real CARLA runner. Today it validates that a `carla_run_config.json` can produce a `closed_loop_report.json` with the expected summary, metric trace slot, and artifacts. When CARLA execution is added, the runner should keep the same report contract and replace the dry-run `not_run` status with the actual execution status and per-tick metrics.

## Implementation scope

The current implementation scope is documented in `docs/implementation_scope.md`. In short: ClosedLoopBench owns CARLA/ScenarioRunner evaluation and reports, treats UniAD as an optional ego-policy plugin, and uses replay/ghost/TrafficManager/scripted actors as progressive actor-model stages.

Ego policy adapters are configured through `agents.ego_policy.build_ego_policy_config()`. Classic E2E stacks should start with TransFuser, InterFuser, or TCP through the ROS2 bridge boundary; UniAD is documented as an optional showcase plugin rather than required runtime.

Local esmini is installed under `tools/esmini/dist/esmini/bin/esmini.exe` and is discovered automatically by `tools.esmini.find_esmini()`.
