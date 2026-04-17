#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /ws/install/setup.bash ]; then
    if grep -q "/opt/ros/${ROS_DISTRO:-jazzy}" /ws/install/setup.bash; then
        source /ws/install/setup.bash
    else
        echo "Skipping /ws/install/setup.bash: workspace overlay was built against another ROS distro"
    fi
fi

exec "$@"
