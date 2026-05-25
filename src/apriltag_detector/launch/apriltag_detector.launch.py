import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    detector_share = get_package_share_directory("apriltag_detector")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gate_params = os.path.join(detector_share, "config", "image_gate.yaml")
    direct_detector_params = os.path.join(detector_share, "config", "direct_detector.yaml")
    detection_monitor_params = os.path.join(
        detector_share, "config", "detection_monitor.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="apriltag_detector",
                executable="image_gate",
                name="apriltag_image_gate",
                output="screen",
                parameters=[gate_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="apriltag_detector",
                executable="direct_detector",
                name="apriltag_direct_detector",
                output="screen",
                parameters=[direct_detector_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="apriltag_detector",
                executable="detection_monitor",
                name="apriltag_detection_monitor",
                output="screen",
                parameters=[detection_monitor_params, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
