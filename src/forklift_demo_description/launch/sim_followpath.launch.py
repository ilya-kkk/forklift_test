from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("forklift_demo_description")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    run_demo_loop = LaunchConfiguration("run_demo_loop")
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")
    rounded_corner_radius = LaunchConfiguration("rounded_corner_radius")
    rear_entry_extension = LaunchConfiguration("rear_entry_extension")

    robot_xacro = PathJoinSubstitution(
        [pkg_share, "urdf", "forklift_demo.urdf.xacro"]
    )
    gazebo_model = PathJoinSubstitution(
        [pkg_share, "models", "forklift_demo", "model.sdf"]
    )
    pallet_model = PathJoinSubstitution(
        [pkg_share, "models", "euro_pallet", "model.sdf"]
    )
    bridge_config = PathJoinSubstitution([pkg_share, "config", "bridge_config.yaml"])
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])
    slam_params = PathJoinSubstitution([pkg_share, "config", "slam_toolbox.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "demo.rviz"])

    robot_description = Command(
        ["xacro ", robot_xacro, " use_sim_time:=", use_sim_time]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r -s --headless-rendering -v 2 ", world]}.items(),
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

    spawn_pallet = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "euro_pallet_point_6",
            "-world",
            "demo_room",
            "-file",
            pallet_model,
            "-x",
            "0.0",
            "-y",
            "0.64",
            "-z",
            "0.0",
            "-Y",
            "1.57079632679",
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
            executable="cmd_vel_to_motors",
            name="cmd_vel_to_motors",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": robot_description,
                    "cmd_vel_frame": "tracking_link",
                }
            ],
        ),
        Node(
            package="forklift_demo_control",
            executable="scan_sector_filter",
            name="scan_sector_filter",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "input_scan_topic": "/scan_raw",
                    "output_scan_topic": "/scan",
                    "blind_sector_center_deg": 0.0,
                    "blind_sector_half_width_deg": 32.0,
                }
            ],
        ),
        Node(
            package="forklift_demo_control",
            executable="depth_image_to_pointcloud",
            name="depth_image_to_pointcloud",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "input_depth_topic": "/fork_depth",
                    "input_camera_info_topic": "/fork_depth/camera_info",
                    "output_pointcloud_topic": "/fork_depth/points",
                    "output_frame_id": "fork_depth_camera_link",
                    "rotate_x_deg": -90.0,
                    "rotate_y_deg": 0.0,
                    "rotate_z_deg": -90.0,
                }
            ],
        ),
        Node(
            package="forklift_demo_control",
            executable="map_service",
            name="map_service",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="forklift_demo_control",
            executable="route_service",
            name="route_service",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "rounded_corner_radius": ParameterValue(
                        rounded_corner_radius, value_type=float
                    ),
                    "rear_entry_extension": ParameterValue(
                        rear_entry_extension, value_type=float
                    ),
                }
            ],
        ),
        Node(
            package="forklift_demo_control",
            executable="demo_route_loop",
            name="demo_route_loop",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "base_frame_id": "tracking_link",
                }
            ],
            condition=IfCondition(run_demo_loop),
        ),
        Node(
            package="forklift_demo_control",
            executable="rviz_teleop_marker",
            name="rviz_teleop_marker",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "frame_id": "tracking_link",
                }
            ],
            condition=IfCondition(launch_rviz),
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
            DeclareLaunchArgument("run_demo_loop", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [pkg_share, "worlds", "square_room.sdf"]
                ),
            ),
            DeclareLaunchArgument("x", default_value="3.0"),
            DeclareLaunchArgument("y", default_value="3.5"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("rounded_corner_radius", default_value="0.0"),
            DeclareLaunchArgument("rear_entry_extension", default_value="0.0"),
            gazebo,
            bridge,
            TimerAction(period=2.0, actions=[spawn_robot, spawn_pallet]),
            *nodes,
        ]
    )
