## syntax=docker/dockerfile:1.7
FROM osrf/ros:humble-desktop

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
            ros-humble-navigation2 \
            ros-humble-rmw-cyclonedds-cpp \
            ros-humble-rqt-robot-steering \
            ros-humble-robot-state-publisher \
            ros-humble-ros-gz \
            ros-humble-rviz2 \
            ros-humble-slam-toolbox \
            ros-humble-tf2-ros \
            ros-humble-xacro; then \
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
