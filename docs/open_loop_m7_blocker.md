# Open-loop M7 remote gate audit

Date: 2026-08-04  
Branch: `feat/open-loop-multimodal`  
Source commit: `7cd7145`

This is an environment blocker audit, not open-loop score evidence. It must not
be used as a formal M5, M6, or M7 pass report.

## Verified

- TF++ runtime image: `closed-loop-bench/transfuserpp-v5:open-loop`
- Image digest: `sha256:b45d7d5dadef95db6aad6bf558afce8c39bf94ea59c668c7ac5d6b2deb707a80`
- Image-reported size: `3708144863` bytes
- Torch: `2.5.0+cu124`
- CUDA: available on `NVIDIA GeForce RTX 4080 SUPER`
- CARLA Python package inside the image: `0.9.15`
- `carla_msgs` builds and imports after sourcing `/opt/algorithm-msgs/install/setup.bash`
- Agents and runner modules compile inside the image

## Missing M5 prerequisites

The exact host paths declared by `docker/transfuserpp.env.example` were
checked on `xt167` and are absent:

| Required input | Status |
|---|---|
| `/home/cwadmin/workspace/external/carla_garage` | Missing |
| `/home/cwadmin/workspace/checkpoints/model_0030_0.pth` | Missing |
| `/home/cwadmin/CARLA_0.9.15/PythonAPI/carla` | Missing |

The host has CARLA `0.9.16` at
`/home/cwadmin/sim-env/data/CARLA_0.9.16`, which is not a substitute for the
pinned `0.9.15` Python API mount. The existing NRE/NuRec images and stopped
containers do not provide a usable TF++ checkpoint/repository binding.

## Consequence

The image can pass its GPU and ROS import checks, but the real checkpoint
preflight cannot be run. Therefore M5 Stage A, M6 NuRec Stage B, and the M7
three-seed freeze remain blocked. No model inference, ADE report, or triplicate
acceptance claim is valid until the three inputs above are mounted and hashed.

## Unlock sequence

1. Provide the exact CARLA Garage checkout, pinned revision, model config, and
   `model_0030_0.pth` under the declared host mounts.
2. Provide the matching CARLA `0.9.15/PythonAPI/carla` directory.
3. Run the runtime manifest/preflight and bind its immutable image, repository,
   checkpoint, and config hashes.
4. Run M5 Stage A, then M6 Stage B, then seeds `41`, `43`, and `47`; freeze
   `open_loop_multimodal_report.v1` only after all frame and sensor gates pass.
