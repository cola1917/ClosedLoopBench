FROM ros:humble-ros-base-jammy

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       libgl1 libglib2.0-0 python3-colcon-common-extensions python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY docker/transfuserpp/requirements.runtime.txt /tmp/requirements.runtime.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.runtime.txt

COPY docker/algorithm/carla_msgs /opt/algorithm-msgs/src/carla_msgs
RUN . /opt/ros/humble/setup.sh \
    && cd /opt/algorithm-msgs \
    && colcon build --packages-select carla_msgs

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    SIM_DATA_PATH=/sim-data \
    ALGORITHM_REPO_PATH=/opt/algorithm/carla_garage \
    ALGORITHM_CHECKPOINT_PATH=/opt/algorithm/checkpoints/model.pth \
    ALGORITHM_READY_FILE=/sim-data/runtime/transfuserpp.ready.json

WORKDIR /opt/closed-loop-bench
COPY agents/__init__.py agents/algorithm_backend.py agents/plugin_contract.py ./agents/
COPY agents/transfuserpp_contract.py agents/transfuserpp_runtime.py ./agents/
COPY agents/transfuserpp_plugin.py agents/transfuserpp_ros2_backend.py ./agents/
COPY runners/run_algorithm_container.py ./runners/
COPY runners/build_transfuserpp_runtime_manifest.py ./runners/
COPY runners/run_transfuserpp_cuda_preflight.py ./runners/
COPY docker/transfuserpp/Dockerfile docker/transfuserpp/requirements.runtime.txt ./docker/transfuserpp/
COPY docker/compose.transfuserpp.yml ./docker/
COPY docker/algorithm/entrypoint.sh ./docker/algorithm/
COPY docker/algorithm/entrypoint.sh /usr/local/bin/closed-loop-algorithm-entrypoint
RUN chmod +x /usr/local/bin/closed-loop-algorithm-entrypoint

# The official CARLA Garage checkout is mounted read-only from the host.  Its
# host UID need not match the container user, so Git otherwise rejects the
# checkout before the runtime can verify its pinned revision, origin, clean
# worktree, and content hash.  Trust only this fixed, read-only mount path; do
# not use a wildcard safe.directory.  Keep this terminal layer so that a
# rebuild can reuse the previously verified dependency layers rather than
# accepting a changed package under the same pinned requirement.
RUN git config --system --add safe.directory /opt/algorithm/carla_garage

ENTRYPOINT ["/usr/local/bin/closed-loop-algorithm-entrypoint"]
CMD ["run"]
