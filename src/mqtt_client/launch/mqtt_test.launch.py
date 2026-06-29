import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mqtt_params_file = LaunchConfiguration('mqtt_params_file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'mqtt_params_file',
        default_value=os.path.join(
            get_package_share_directory('mqtt_client'),
            'config',
            'params.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for the mqtt_client node'
    )

    mqtt_client_node = Node(
        package='mqtt_client',
        executable='mqtt_client',
        name='mqtt_client',
        output='screen',
        parameters=[mqtt_params_file],
    )

    return LaunchDescription([
        declare_params_file_cmd,
        mqtt_client_node
    ])
