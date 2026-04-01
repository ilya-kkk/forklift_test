from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("forklift_demo_description")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")
    path_horizontal_length = LaunchConfiguration("path_horizontal_length")
    path_vertical_length = LaunchConfiguration("path_vertical_length")

    robot_xacro = PathJoinSubstitution(
        [pkg_share, "urdf", "forklift_demo.urdf.xacro"]
    )
    gazebo_model = PathJoinSubstitution(
        [pkg_share, "models", "forklift_demo", "model.sdf"]
    )
    bridge_config = PathJoinSubstitution([pkg_share, "config", "bridge_config.yaml"])
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])
    slam_params = PathJoinSubstitution([pkg_share, "config", "slam_toolbox.yaml"])
    control_params = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "demo.rviz"])

    robot_description = Command(
        ["xacro ", robot_xacro, " use_sim_time:=", use_sim_time]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-s -r -v 2 ", world]}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "forklift_demo",
            "-world",
            "demo_room",
            "-file",
            gazebo_model,
            "-x",
            x,
            "-y",
            y,
            "-z",
            "0.18",
            "-Y",
            yaw,
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="forklift_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": robot_description,
                    "publish_frequency": 30.0,
                }
            ],
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="forklift_demo_control",
            executable="cmd_vel_to_tricycle",
            name="cmd_vel_to_tricycle",
            output="screen",
            parameters=[control_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="forklift_demo_control",
            executable="hardcoded_route_sender",
            name="hardcoded_route_sender",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "start_x": x,
                    "start_y": y,
                    "horizontal_length": path_horizontal_length,
                    "vertical_length": path_vertical_length,
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(launch_rviz),
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [pkg_share, "worlds", "square_room.sdf"]
                ),
            ),
            DeclareLaunchArgument("x", default_value="-1.0"),
            DeclareLaunchArgument("y", default_value="-1.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("path_horizontal_length", default_value="4.0"),
            DeclareLaunchArgument("path_vertical_length", default_value="4.0"),
            gazebo,
            bridge,
            TimerAction(period=2.0, actions=[spawn_robot]),
            *nodes,
        ]
    )
