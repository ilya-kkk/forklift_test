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
            python3-setuptools \
            ros-jazzy-navigation2 \
            ros-jazzy-rmw-cyclonedds-cpp \
            ros-jazzy-rqt-robot-steering \
            ros-jazzy-slider-publisher \
            ros-jazzy-robot-state-publisher \
            ros-jazzy-ros-gz \
            ros-jazzy-rviz2 \
            ros-jazzy-slam-toolbox \
            ros-jazzy-tf2-ros \
            ros-jazzy-xacro; then \
            break; \
        fi; \
        if [ "$attempt" -eq 5 ]; then \
            exit 1; \
        fi; \
        echo "apt attempt $attempt failed; retrying in 15s"; \
        sleep 15; \
    done; \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
