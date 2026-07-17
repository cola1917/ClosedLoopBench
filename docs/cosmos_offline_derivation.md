# Cosmos Offline Derivation

Cosmos is optional and deliberately outside the closed-loop acceptance path.

## Architecture

```text
last.usdz
  -> NuRec gRPC (pose-to-RGB / pose-to-LiDAR)
  -> accepted CARLA + NuRec run
  -> synchronized RGB MP4 + edge/depth/vis/seg MP4
  -> Cosmos Transfer2.5 HTTP /v1/infer
  -> presentation or offline augmentation video
```

NuRec, not Cosmos, consumes the USDZ. NVIDIA's NuRec gRPC server launches with
`--artifact-glob .../last.usdz`. The documented Cosmos Transfer2.5 NIM accepts
an input video plus at least one `edge`, `depth`, `vis`, or `seg` control. Its
public request is a video inference job, not a per-CARLA-frame physics RPC.

Official references:

- [NuRec gRPC API guide](https://docs.nvidia.com/nurec/api/grpc_api_guide.html)
- [Cosmos Transfer2.5](https://docs.nvidia.com/cosmos/latest/transfer2.5/index.html)
- [Cosmos NIM API reference](https://docs.nvidia.com/nim/cosmos/latest/api-reference.html)
- [Cosmos NIM sampling inputs](https://docs.nvidia.com/nim/cosmos/latest/sampling-params.html)

## Package Boundary

`cosmos_transfer_job.v1` can only be built from a runtime result that already
passes `validate_multimodal_closed_loop.py`. It hashes the accepted result, RGB
video, and control videos, and freezes these non-negotiable fields:

- `execution.realtime=false`;
- `execution.consumes_usdz=false`;
- `boundary.part_of_control_loop=false`;
- `boundary.part_of_sensor_acceptance=false`;
- Cosmos output cannot be used as closed-loop metrics, safety evidence, or
  RGB/LiDAR consistency evidence.

Example:

```bash
python -m runners.build_cosmos_transfer_job \
  --accepted-run outputs/run/runtime-result.json \
  --rgb-video outputs/run/camera-front.mp4 \
  --control edge=outputs/run/camera-front-edge.mp4 \
  --prompt "Preserve road users and geometry; improve photorealism." \
  --frame-count 93 \
  --fps 16 \
  --width 1280 \
  --height 720 \
  --resolution 720 \
  --output outputs/run/cosmos-transfer-job.json
```

The 93--480 frame limit follows the current documented Transfer2.5 NIM input
range. The package records declared video metadata; the eventual NIM launch
must still decode and verify the real MP4 streams before submission.
