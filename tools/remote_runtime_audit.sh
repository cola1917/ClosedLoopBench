#!/usr/bin/env bash
set -u

echo "hostname=$(hostname)"
echo "timestamp=$(date --iso-8601=seconds)"
echo "home=$HOME"
echo "display=${DISPLAY:-unset}"
echo "carla_root=${CARLA_ROOT:-unset}"

nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader || true
command -v CarlaUE4.sh || true
find /home/cwadmin -path '*/PythonAPI/examples/nvidia/nurec' -type d 2>/dev/null | head -n 10
find /home/cwadmin -maxdepth 5 -name CarlaUE4.sh -type f 2>/dev/null
pgrep -af 'CarlaUE4|CARLA' || true
ss -ltnp | grep -E ':(2000|2001|8000)' || true

git -C /home/cwadmin/workspace/ClosedLoopBench status --short || true
git -C /home/cwadmin/workspace/ClosedLoopBench log -1 --oneline || true

for candidate in \
  /home/cwadmin/sim-env/carla \
  /home/cwadmin/CARLA_0.9.16 \
  /home/cwadmin/carla; do
  if [[ -d "$candidate" ]]; then
    echo "carla_candidate=$candidate"
    find "$candidate/PythonAPI/examples" -maxdepth 3 -path '*/nvidia/nurec' -type d 2>/dev/null
  fi
done
