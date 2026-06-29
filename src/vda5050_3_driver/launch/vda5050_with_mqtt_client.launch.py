from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("vda5050_3_driver")
    vda_config_path = os.path.join(package_share, "config", "vda5050_3_driver.yaml")
    mqtt_config_path = os.path.join(
        package_share, "config", "mqtt_client_vda5050_v3.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="vda5050_3_driver",
                executable="vda5050_3_bridge",
                name="vda5050_3_bridge",
                output="screen",
                parameters=[vda_config_path],
            ),
            Node(
                package="mqtt_client",
                executable="mqtt_client",
                name="mqtt_client",
                output="screen",
                parameters=[mqtt_config_path],
            ),
        ]
    )
