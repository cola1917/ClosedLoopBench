#!/usr/bin/env bash
set -eo pipefail

algorithm_id="${1:?algorithm id required}"
attempt_name="${2:?attempt name required}"
root=/home/cwadmin/workspace/ClosedLoopBench
base="$root/outputs/scene-0061-1000step/runtime/attempt-024-reference-cruise-035-udp/carla_run_config.json"
attempt="$root/outputs/scene-0061-1000step/runtime/$attempt_name"
container="clb-${algorithm_id//_/-}"

mkdir -p "$attempt"
jq --arg run "scene0061-${algorithm_id}-${attempt_name}" --arg alg "$algorithm_id" \
  '.run_id=$run | .ego.algorithm_id=$alg | .experiment.algorithm_id=$alg' \
  "$base" > "$attempt/carla_run_config.json"

cleanup() {
  docker rm -f "$container" >/dev/null 2>&1 || true
}
trap cleanup EXIT
cleanup

docker run -d --rm \
  --name "$container" \
  --network host \
  -e ALGORITHM_ID="$algorithm_id" \
  -e ALGORITHM_PLUGIN=reference_plugins:create_backend \
  -e ALGORITHM_REPO_PATH=/opt/algorithm/repo \
  -e ALGORITHM_CHECKPOINT_PATH=/opt/algorithm/checkpoints/reference.json \
  -e SIM_DATA_PATH=/sim-data \
  -e ALGORITHM_READY_FILE="/sim-data/scene-0061-1000step/runtime/$attempt_name/algorithm.ready.json" \
  -e ROS_DOMAIN_ID=61 \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e CONTROL_TOPIC=/carla/ego_vehicle/vehicle_control_cmd \
  -e OBSERVATION_TOPIC=/closed_loop/ego/observation \
  -v "$root/examples/reference_algorithm_plugins:/opt/algorithm/repo:ro" \
  -v "$root/examples/reference_algorithm_plugins/checkpoints:/opt/algorithm/checkpoints:ro" \
  -v "$root/outputs:/sim-data" \
  closed-loop-bench/ego-algorithm:humble run \
  > "$attempt/container.id"

for _ in $(seq 1 30); do
  if test -f "$attempt/algorithm.ready.json"; then
    break
  fi
  sleep 0.2
done
test -f "$attempt/algorithm.ready.json"

source /opt/ros/humble/setup.bash
source /home/cwadmin/sim-env/carla-ros2-ws/install/setup.bash
source /home/cwadmin/sim-env/miniconda3/etc/profile.d/conda.sh
conda activate autodrive
export ROS_DOMAIN_ID=61
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

python "$root/runners/run_carla_basic_agent.py" \
  --run-config "$attempt/carla_run_config.json" \
  --output "$attempt/ros2_observation_control_plan.json" \
  --host 127.0.0.1 \
  --port 2000 \
  --timeout-sec 20 \
  --max-ticks 1200 \
  --execute \
  --ego-driver ros2_observation_control \
  --control-topic /carla/ego_vehicle/vehicle_control_cmd \
  --observation-topic /closed_loop/ego/observation \
  --control-timeout-sec 0.5 \
  --acceptance-evidence \
  --physics-smoke \
  --opendrive "$root/outputs/scene-0061-1000step/road.nurec-route-extended-both-v7.xodr" \
  > "$attempt/runner.stdout.json" \
  2> "$attempt/runner.stderr.log"

docker logs "$container" > "$attempt/algorithm.log" 2>&1 || true
sha256sum "$attempt"/* > "$attempt/artifact_sha256.txt"
