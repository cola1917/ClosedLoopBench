#!/usr/bin/env bash
set -u

CARLA_ROOT=/home/cwadmin/sim-env/data/CARLA_0.9.16
NUREC_DIR="$CARLA_ROOT/PythonAPI/examples/nvidia/nurec"
PYTHON=/home/cwadmin/sim-env/miniconda3/envs/autodrive/bin/python
BUNDLE=/home/cwadmin/workspace/ClosedLoopBench/outputs/scene-0061-1000step

echo "x_sockets"
ls -la /tmp/.X11-unix 2>/dev/null || true
echo "display_processes"
pgrep -af 'Xorg|Xwayland|gnome-shell' || true
echo "nurec_requirements"
cat "$NUREC_DIR/requirements.txt" 2>/dev/null || true
find "$NUREC_DIR" -maxdepth 4 -type f -path '*/grpc/*' -printf '%P %s bytes\n' 2>/dev/null | head -n 100
echo "python_packages"
"$PYTHON" -m pip show carla grpcio pygame imageio numpy scipy pyyaml 2>/dev/null || true
echo "bundle_inventory"
find "$BUNDLE" -maxdepth 3 -type f -printf '%p %s bytes\n' | sort
echo "bundle_json_summary"
"$PYTHON" - "$BUNDLE" <<'PY'
import json, sys
from pathlib import Path

root = Path(sys.argv[1])
for name in (
    "reconstruction_integration_plan.json",
    "reconstruction_package.json",
    "scene_package.json",
    "carla_run_config.planned.json",
):
    path = root / name
    if not path.exists():
        continue
    data = json.loads(path.read_text())
    print(name)
    if name.startswith("carla_run_config"):
        print(json.dumps({
            "scenario_id": data.get("scenario_id"),
            "carla": data.get("carla"),
            "ego_initial": (data.get("ego") or {}).get("initial_state"),
            "ego_reference_count": len((data.get("ego") or {}).get("reference_trajectory") or []),
            "actor_count": len(data.get("actors") or []),
            "reconstruction_package": data.get("reconstruction_package"),
        }, indent=2))
    else:
        print(json.dumps(data, indent=2)[:12000])
PY
