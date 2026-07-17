#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/algorithm-msgs/install/setup.bash

exec python3 -m runners.run_algorithm_container "$@"
