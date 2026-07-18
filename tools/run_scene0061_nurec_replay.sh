#!/usr/bin/env bash
set -euo pipefail

# Recorded scene replay only. Algorithmic ego control is intentionally not
# enabled here; add --enable-ego-control explicitly in a later control phase.
CARLA_ROOT="${CARLA_ROOT:-/home/cwadmin/sim-env/data/CARLA_0.9.16}"
PYTHON_BIN="${PYTHON_BIN:-/home/cwadmin/sim-env/miniconda3/envs/autodrive/bin/python}"
NUREC_EXAMPLE_DIR="${NUREC_EXAMPLE_DIR:-${CARLA_ROOT}/PythonAPI/examples/nvidia/nurec}"
# The standalone nvidia-nurec-grpc:0.2.0 image cannot load 26.04 artifacts;
# formal replay always uses nre-ga:26.04 with the serve-grpc subcommand.
NUREC_IMAGE="${NUREC_IMAGE:-nvcr.io/nvidia/nre/nre-ga:26.04}"
NUREC_IMAGE_COMMAND="${NUREC_IMAGE_COMMAND:-serve-grpc}"
NUREC_USDZ="${NUREC_USDZ:-/home/cwadmin/workspace/NeuralSceneBridge/outputs/nurec_formal_scene0061_6cam_40k/nB3fGDTuUz5ptMbZzCXnjS/artifacts/last.usdz}"
NUREC_XODR_PATH="${NUREC_XODR_PATH:-/home/cwadmin/workspace/ClosedLoopBench/outputs/scene-0061-1000step/road.xodr}"
OUTPUT_DIR="${OUTPUT_DIR:-/home/cwadmin/workspace/ClosedLoopBench/outputs/scene-0061-40k-nurec-replay/images}"
CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2000}"
NUREC_PORT="${NUREC_PORT:-46435}"
NUREC_RESOLUTION_RATIO="${NUREC_RESOLUTION_RATIO:-0.5}"
NUREC_CAMERA_CONFIG="${NUREC_CAMERA_CONFIG:-/home/cwadmin/workspace/ClosedLoopBench/configs/scene0061_nurec_cameras.yaml}"
DISPLAY="${DISPLAY:-:1}"
LOG_FILE="${LOG_FILE:-}"
OVERLAP_LOG="${OVERLAP_LOG:-}"
ACTOR_MAPPING_LOG="${ACTOR_MAPPING_LOG:-}"

for path in "${PYTHON_BIN}" "${NUREC_USDZ}" "${NUREC_XODR_PATH}" "${NUREC_CAMERA_CONFIG}"; do
  if [[ ! -e "${path}" ]]; then
    echo "Required path does not exist: ${path}" >&2
    exit 1
  fi
done
if ! docker image inspect "${NUREC_IMAGE}" >/dev/null 2>&1; then
  echo "NuRec gRPC image is not installed: ${NUREC_IMAGE}" >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"
export DISPLAY NUREC_IMAGE NUREC_IMAGE_COMMAND NUREC_XODR_PATH
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla${PYTHONPATH:+:${PYTHONPATH}}"

cd "${NUREC_EXAMPLE_DIR}"
replay_args=(
  --host "${CARLA_HOST}" \
  --port "${CARLA_PORT}" \
  --nurec-port "${NUREC_PORT}" \
  --usdz-filename "${NUREC_USDZ}" \
  --output-dir "${OUTPUT_DIR}" \
  --resolution-ratio "${NUREC_RESOLUTION_RATIO}" \
  --camera-config "${NUREC_CAMERA_CONFIG}" \
  --saveimages \
  --move-spectator
)
if [[ -n "${OVERLAP_LOG}" ]]; then
  replay_args+=(--overlap-log "${OVERLAP_LOG}")
fi
if [[ -n "${ACTOR_MAPPING_LOG}" ]]; then
  replay_args+=(--actor-mapping-log "${ACTOR_MAPPING_LOG}")
fi

if [[ -n "${LOG_FILE}" ]]; then
  mkdir -p "$(dirname "${LOG_FILE}")"
  "${PYTHON_BIN}" example_nurec_replay_save_images.py "${replay_args[@]}" \
    2>&1 | tee "${LOG_FILE}"
  exit "${PIPESTATUS[0]}"
fi

exec "${PYTHON_BIN}" example_nurec_replay_save_images.py "${replay_args[@]}"
