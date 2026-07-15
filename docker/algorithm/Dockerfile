FROM ros:humble-ros-base-jammy

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    SIM_DATA_PATH=/sim-data \
    ALGORITHM_REPO_PATH=/opt/algorithm/repo \
    ALGORITHM_CHECKPOINT_PATH=/opt/algorithm/checkpoints/model.pth \
    ALGORITHM_READY_FILE=/tmp/algorithm.ready

WORKDIR /opt/closed-loop-bench
COPY agents/__init__.py agents/algorithm_backend.py ./agents/
COPY runners/run_algorithm_container.py ./runners/

ENTRYPOINT ["python3", "-m", "runners.run_algorithm_container"]
CMD ["run"]
