# ClosedLoopBench Delivery Boundary Gate

This document keeps the pre-remote-development work explainable and bounded. ClosedLoopBench is a closed-loop evaluation harness. It is not a model zoo, a reconstruction system, a scenario mining system, or a full CARLA fork.

## Core Delivery Before Remote CARLA Access

The local-only delivery is complete when the repo can do all of the following without CARLA, ROS2, TCP, NuRec, or Cosmos installed:

- compile Scenario IR into CARLA/OpenSCENARIO/OpenDRIVE artifacts
- validate exchange artifacts with esmini when available
- build CARLA probe and ScenarioRunner command plans
- build a BasicAgent ego closed-loop execution plan
- build sensor and ego-adapter contracts for external stacks
- build actor runtime plans for replay, scripted, and TrafficManager modes
- aggregate mock tick metrics into the same closed-loop report shape
- document the exact remote integration runbook

## What Must Stay Optional

These integrations are adapters, not core dependencies:

- TCP, TransFuser, InterFuser, UniAD, or any learned ego stack
- CARLA ROS2 bridge runtime
- ScenarioRunner process execution
- NuRec/Cosmos reconstructed sensor rendering
- real CARLA actor controllers and sensor callbacks

The core tests must pass without all of them.

## Boundary Rules

1. Scenario IR remains the source of scenario intent.
2. OpenSCENARIO remains the primary portable scenario artifact.
3. Python ScenarioRunner code is only a runtime extension when OpenSCENARIO is not expressive enough.
4. BasicAgent is the first real ego closed-loop runner because it has the fewest dependencies.
5. TCP is an optional external ego adapter. ClosedLoopBench owns topic, sensor, timeout, fallback, and report contracts; the TCP repo/container owns model inference.
6. nuScenes camera images are data-mining/reconstruction inputs. Closed-loop ego algorithms consume CARLA or reconstructed runtime observations, not frozen nuScenes log images.
7. Actor controllers expose three levels only: replay, scripted, and TrafficManager/reactive. Avoid a bespoke rule engine in this project.
8. Metrics collectors record primitive per-tick facts and reuse the report contract. Avoid simulator-specific metrics logic in the report layer.

## Stop Conditions

Do not add new subsystems unless one of these is true:

- it is needed to run the next CARLA integration step
- it is needed to keep a contract testable without CARLA
- it clarifies a handoff to TriggerEngine or NeuralSceneBridge
- it reduces ambiguity in an interview explanation

If a feature requires model checkpoints, external containers, CARLA server execution, ROS2 nodes, or GPU runtime, it belongs behind an adapter boundary and should be documented as a remote integration step.
