# Shared Exchange Protocol v1

`shared_exchange_protocol.v1` is the environment-independent handoff contract
between TriggerEngine, NeuralSceneBridge, and ClosedLoopBench. It uses one shared
filesystem for immutable messages and artifacts. ROS 2 remains the per-tick
runtime channel; it is not part of this protocol.

## Ownership

| Project | Publishes | Consumes |
| --- | --- | --- |
| TriggerEngine | `scene.selection.request`, optional reconstruction request | closed-loop result for review/feedback |
| NeuralSceneBridge | reconstruction result and Reconstruction Package assets | selection/reconstruction jobs |
| ClosedLoopBench | final Scene Package, evaluation request/result, and run artifacts | Scenario IR, Reconstruction Packages, evaluation jobs |

The minimum TriggerEngine handoff contains only `scene_id` and
`source.dataset`. Scenario IR, selection score, trigger tags, and reconstruction
window are optional enrichments.

## Directory Layout

```text
<exchange-root>/
  datasets/                         # optional shared source data
  requests/                         # immutable business request artifacts
  messages/
    <message-type>/<message-id>/
      message.json
      READY.json
  claims/
    shared.job.request/<request-message-id>/attempt-0001/
      claim.json
      READY.json
  scenes/<scene-id>/<version>/      # Scene Package protocol
  runs/<run-id>/                    # evaluation artifacts
```

IDs that become directory names use `[A-Za-z0-9._-]`; colons, slashes and path
traversal are forbidden. Artifact references use POSIX paths relative to the
exchange root even when the disk is mounted on Windows.

## Message Flow

```text
TriggerEngine
  scene.selection.request
    -> reconstruction.build.request
    -> shared.job.request(target=NeuralSceneBridge)

NeuralSceneBridge
  shared.job.claim(attempt=N)
    -> reconstruction.build.result
    -> shared.job.result
    -> immutable Reconstruction Package assets

ClosedLoopBench
  Scenario IR + optional Reconstruction Package
    -> immutable scenes/<scene>/<version>/Scene Package

ClosedLoopBench
  evaluation.run.request
    -> shared.job.request(target=ClosedLoopBench)
    -> shared.job.claim(attempt=N)
    -> evaluation.run.result
    -> shared.job.result
    -> immutable runs/<run-id>/report and traces
```

Every message carries `producer`, `correlation`, and `idempotency`. The initial
selection is the root message. Later messages preserve its `correlation_id` and
`root_message_id`, while `causation_message_id` points to the direct predecessor.

## Atomicity And Integrity

Messages and files are written to hidden staging paths and atomically renamed
into place. Consumers accept them only when `READY.json` exists and its digest
matches. Every artifact reference includes path, media type, role, byte size and
SHA-256. Existing message IDs, artifact paths, scene versions and run IDs are
never overwritten.

Concurrent workers claim a specific attempt with one atomic directory rename;
exactly one wins. A claim is immutable and is not proof of completion. A
terminal `shared.job.result` must reference the winning claim message.

## Lease And Retry

- `claimed_at < lease_expires_at` is mandatory.
- An attempt number starts at one and cannot exceed `max_attempts`.
- A worker publishes exactly one terminal result for its claimed attempt.
- A retry uses the next attempt and a new claim/result message ID.
- Replaying the same idempotency key must resolve to the existing job; it must
  not create a second run or overwrite output.
- Operators may start a later attempt only after the previous lease expired and
  no terminal result exists. Automatic clock-skew recovery is intentionally not
  inferred from file modification times.

## Result Semantics

Terminal job status is `succeeded`, `failed`, or `cancelled`. Reconstruction
also supports `partial`, with per-product status and warnings. Failed/cancelled
results require a structured error code, message, retryability flag, and optional
details. Missing KPI values remain JSON `null`; they are never converted to
passing zeros.

## Schemas

Canonical Draft 2020-12 schemas live in
`SceneExchangeContracts/src/scene_exchange_contracts/schemas/shared_exchange_protocol/`.
Each schema contains a validated example:

- common envelope and immutable artifact reference
- shared job request, claim and result
- scene selection request
- reconstruction request/result
- evaluation run request/result

Concrete schema changes that reject an existing v1 message require a new
concrete `schema_version`. Changes to directory atomicity, identity or envelope
semantics require `shared_exchange_protocol.v2`. Consumers must reject unknown
major versions rather than guessing.

## Reference Commands

Validate a message and all referenced files:

```powershell
python runners/validate_shared_protocol.py `
  --document requests/reconstruction-scene-0061-v001/request.json `
  --exchange-root E:/sim-data
```

Publish and inspect messages:

```powershell
python runners/shared_message_exchange.py --exchange-root E:/sim-data `
  publish-message --message request.json

python runners/shared_message_exchange.py --exchange-root E:/sim-data `
  list-messages --message-type shared.job.request
```

Publish a request file before referencing it from a job message:

```powershell
python runners/shared_message_exchange.py --exchange-root E:/sim-data `
  publish-artifact --source request.json `
  --path requests/reconstruction-scene-0061-v001/request.json `
  --role job_request --media-type application/json
```

Claim and complete a job with Schema-valid envelopes:

```powershell
python runners/shared_message_exchange.py --exchange-root E:/sim-data `
  claim-job --request-message-id msg-job-request-001 --claim claim.json

python runners/shared_message_exchange.py --exchange-root E:/sim-data `
  complete-job --request-message-id msg-job-request-001 --result result.json
```

The reference implementation has no CARLA, ROS 2, Docker, GPU, network, or
third-party Python dependency.
