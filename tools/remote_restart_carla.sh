#!/usr/bin/env bash
set -euo pipefail

CARLA_ROOT="${CARLA_ROOT:-/home/cwadmin/sim-env/data/CARLA_0.9.16}"
RUN_DIR="${RUN_DIR:?RUN_DIR must name a unique server evidence directory}"
ENV_BUILD_ROOT="${ENV_BUILD_ROOT:-/home/cwadmin/workspace/env_build}"
pattern="$CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"

mapfile -t pids < <(pgrep -f "$pattern" || true)
if ((${#pids[@]})); then
  kill -TERM "${pids[@]}"
  for _ in $(seq 1 20); do
    if ! pgrep -f "$pattern" >/dev/null; then
      break
    fi
    sleep 0.5
  done
fi
if pgrep -f "$pattern" >/dev/null; then
  mapfile -t stuck_pids < <(pgrep -f "$pattern" || true)
  echo "CARLA did not stop after SIGTERM; killing stuck PIDs: ${stuck_pids[*]}" >&2
  kill -KILL "${stuck_pids[@]}"
  sleep 1
fi
if pgrep -f "$pattern" >/dev/null; then
  echo "CARLA processes remain after SIGKILL" >&2
  exit 2
fi

mkdir -p "$RUN_DIR"
log_file="$RUN_DIR/carla.log"
cd "$ENV_BUILD_ROOT"
nohup bash start_carla.sh >"$log_file" 2>&1 < /dev/null &
carla_pid=$!
echo "$carla_pid" > "$RUN_DIR/carla.pid"
echo "Started CARLA through $ENV_BUILD_ROOT/start_carla.sh (pid=$carla_pid, log=$log_file)"
