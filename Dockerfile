## syntax=docker/dockerfile:1.7
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    set -eux; \
    for attempt in 1 2 3 4 5; do \
        rm -rf /var/lib/apt/lists/*; \
        apt-get clean; \
        if apt-get update \
            -o Acquire::Retries=5 \
            -o Acquire::By-Hash=force \
            -o Acquire::http::No-Cache=true \
            -o Acquire::https::No-Cache=true \
        && apt-get install -y --no-install-recommends \
            python3-colcon-common-extensions \
            python3-pip \
            python3-setuptools \
            ros-jazzy-navigation2 \
            ros-jazzy-apriltag-ros \
            ros-jazzy-rmw-cyclonedds-cpp \
            ros-jazzy-rqt-robot-steering \
            ros-jazzy-slider-publisher \
            ros-jazzy-robot-state-publisher \
            ros-jazzy-ros-gz \
            ros-jazzy-rviz2 \
            ros-jazzy-slam-toolbox \
            ros-jazzy-tf2-ros \
            ros-jazzy-yasmin \
            ros-jazzy-yasmin-viewer \
            ros-jazzy-xacro; then \
            break; \
        fi; \
        if [ "$attempt" -eq 5 ]; then \
            exit 1; \
        fi; \
        echo "apt attempt $attempt failed; retrying in 15s"; \
        sleep 15; \
    done; \
    python3 -m pip install --break-system-packages --no-cache-dir simple-websocket; \
    python3 -c 'from pathlib import Path; old = "SocketIO(app, cors_allowed_origins=\"*\")"; new = "SocketIO(app, cors_allowed_origins=\"*\", manage_session=False)"; paths = list(Path("/opt/ros/jazzy/lib").glob("python*/site-packages/yasmin_viewer/yasmin_viewer_node.py")) + [Path("/opt/ros/jazzy/lib/yasmin_viewer/yasmin_viewer_node")]; assert paths; [p.write_text(p.read_text().replace(old, new)) for p in paths if p.exists()]'; \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
