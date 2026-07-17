FROM ros:humble-ros-base-jammy

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY docker/algorithm/carla_msgs /opt/algorithm-msgs/src/carla_msgs
RUN . /opt/ros/humble/setup.sh \
    && cd /opt/algorithm-msgs \
    && colcon build --packages-select carla_msgs

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    SIM_DATA_PATH=/sim-data \
    ALGORITHM_REPO_PATH=/opt/algorithm/repo \
    ALGORITHM_CHECKPOINT_PATH=/opt/algorithm/checkpoints/model.pth \
    ALGORITHM_READY_FILE=/tmp/algorithm.ready

WORKDIR /opt/closed-loop-bench
COPY agents/__init__.py agents/algorithm_backend.py ./agents/
COPY runners/run_algorithm_container.py ./runners/
COPY docker/algorithm/entrypoint.sh /usr/local/bin/closed-loop-algorithm-entrypoint
RUN chmod +x /usr/local/bin/closed-loop-algorithm-entrypoint

ENTRYPOINT ["/usr/local/bin/closed-loop-algorithm-entrypoint"]
CMD ["run"]
