FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-setuptools \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-robot-state-publisher \
    ros-humble-ros-gz \
    ros-humble-rqt \
    ros-humble-rqt-robot-steering \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-tf2-ros \
    ros-humble-xacro \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY src /ws/src
COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh \
 && source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
