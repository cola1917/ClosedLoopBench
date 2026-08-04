FROM ros:humble-ros-base-jammy

ARG APT_MIRROR=https://mirrors.aliyun.com/ubuntu
ARG ROS_APT_MIRROR=https://mirrors.aliyun.com/ros2/ubuntu
ARG PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple

RUN set -eux; \
    sed -i \
      -e "s|http://archive.ubuntu.com/ubuntu|${APT_MIRROR}|g" \
      -e "s|http://security.ubuntu.com/ubuntu|${APT_MIRROR}|g" \
      /etc/apt/sources.list; \
    sed -i "s|http://packages.ros.org/ros2/ubuntu|${ROS_APT_MIRROR}|g" \
      /usr/share/ros-apt-source/ros2.sources; \
    apt-get -o Acquire::Retries=3 update \
    && apt-get install -y --no-install-recommends \
       libgl1 libglib2.0-0 python3-colcon-common-extensions python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY docker/transfuserpp/requirements.runtime.txt /tmp/requirements.runtime.txt
RUN --mount=type=bind,source=docker/transfuserpp/vendor,target=/tmp/tfpp-vendor,ro \
    set -eux; \
    if [ -f /tmp/tfpp-vendor/torch-2.5.0-cp310-cp310-manylinux1_x86_64.whl ]; then \
      python3 -m pip install --no-cache-dir \
        --index-url "${PIP_INDEX_URL}" \
        --find-links /tmp/tfpp-vendor \
        /tmp/tfpp-vendor/torch-2.5.0-cp310-cp310-manylinux1_x86_64.whl; \
    fi; \
    python3 -m pip install --no-cache-dir \
      --index-url "${PIP_INDEX_URL}" \
      --find-links /tmp/tfpp-vendor \
      -r /tmp/requirements.runtime.txt

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
