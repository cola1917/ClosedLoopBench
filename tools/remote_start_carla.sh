#!/usr/bin/env bash
set -euo pipefail

CARLA_ROOT="${CARLA_ROOT:-/home/cwadmin/sim-env/data/CARLA_0.9.16}"
RUN_DIR="${RUN_DIR:-/home/cwadmin/workspace/ClosedLoopBench/outputs/scene-0061-1000step/runtime}"
DISPLAY="${DISPLAY:-:1}"
XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
export DISPLAY XAUTHORITY

mkdir -p "$RUN_DIR"
if pgrep -f "^${CARLA_ROOT}/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping( |$)" >/dev/null; then
  echo "CARLA is already running"
  exit 0
fi

nohup "$CARLA_ROOT/CarlaUE4.sh" \
  -carla-port=2000 \
  -quality-level=Low \
  -nosound \
  >"$RUN_DIR/carla_server.log" 2>&1 </dev/null &
echo "$!" >"$RUN_DIR/carla_server.pid"
echo "started pid=$! display=$DISPLAY log=$RUN_DIR/carla_server.log"
