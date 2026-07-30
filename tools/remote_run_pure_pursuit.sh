#!/usr/bin/env bash
set -eo pipefail

algorithm_id="${1:?algorithm id required}"
attempt_name="${2:?attempt name required}"
root=/home/cwadmin/workspace/ClosedLoopBench
PYTHON_BIN="${PYTHON_BIN:-/home/cwadmin/sim-env/miniconda3/envs/autodrive/bin/python}"
base="$root/outputs/scene-0061-1000step/runtime/attempt-024-reference-cruise-035-udp/carla_run_config.json"
attempt="$root/outputs/scene-0061-1000step/runtime/$attempt_name"
container="clb-${algorithm_id//_/-}"
XODR_PATH="${XODR_PATH:-$root/outputs/scene0061_exchange_v2/road.xodr}"
SCENARIO_IR_PATH="${SCENARIO_IR_PATH:-$root/outputs/scene0061_exchange_v2/scene_ir.json}"
ALLOW_CORRIDOR_ONLY_XODR="${ALLOW_CORRIDOR_ONLY_XODR:-0}"
EXPECTED_CANONICAL_XODR_SHA256="eb117dd99f84cdd8072e13aaacc502702dd815658ed4b53e81a00ace931b109e"

test -f "$XODR_PATH"
test -f "$SCENARIO_IR_PATH"
test -x "$PYTHON_BIN"
validator_args=(
  --xodr "$XODR_PATH"
)
if test "$ALLOW_CORRIDOR_ONLY_XODR" = "1"; then
  echo "WARNING: explicitly allowing corridor-only control evidence" >&2
  validator_args+=(--no-map-topology --expected-ego-corridor-count 1)
else
  validator_args+=(
    --expected-sha256 "$EXPECTED_CANONICAL_XODR_SHA256"
    --expected-ego-corridor-count 0
    --require-junction-topology
    --require-boundary-audit
    --require-connector-evidence
    --require-route-chain
    --require-route-map-integration
    --require-route-source-audit
    --scenario-ir "$SCENARIO_IR_PATH"
    --require-ego-route-coverage
  )
fi
PYTHONPATH="$root${PYTHONPATH:+:${PYTHONPATH}}" \
  "$PYTHON_BIN" -m adapters.opendrive_contract "${validator_args[@]}"

mkdir -p "$attempt"
jq --arg run "scene0061-${algorithm_id}-${attempt_name}" --arg alg "$algorithm_id" \
  '.run_id=$run | .ego.algorithm_id=$alg | .experiment.algorithm_id=$alg' \
  "$base" > "$attempt/carla_run_config.json"

expected_runtime_xodr_args=()
runtime_topology_args=(--allow-corridor-only-opendrive)
if test "$ALLOW_CORRIDOR_ONLY_XODR" != "1"; then
  expected_runtime_xodr_args=(
    --expected-opendrive-sha256 "$EXPECTED_CANONICAL_XODR_SHA256"
  )
  runtime_topology_args=(--require-topology-map)
fi

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

"$PYTHON_BIN" "$root/runners/run_carla_basic_agent.py" \
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
  --opendrive "$XODR_PATH" \
  --scenario-ir "$SCENARIO_IR_PATH" \
  "${expected_runtime_xodr_args[@]}" \
  "${runtime_topology_args[@]}" \
  > "$attempt/runner.stdout.json" \
  2> "$attempt/runner.stderr.log"

docker logs "$container" > "$attempt/algorithm.log" 2>&1 || true
sha256sum "$attempt"/* > "$attempt/artifact_sha256.txt"
