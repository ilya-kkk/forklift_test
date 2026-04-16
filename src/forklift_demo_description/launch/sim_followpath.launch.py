from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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

    graph_filepath = "/tmp/forklift_demo_route_graph.geojson"

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
    route_server_params = PathJoinSubstitution(
        [pkg_share, "config", "route_server_params.yaml"]
    )
    collision_monitor_params = PathJoinSubstitution(
        [pkg_share, "config", "collision_monitor_params.yaml"]
    )
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

    route_graph_builder = Node(
        package="forklift_demo_control",
        executable="route_graph_builder",
        name="route_graph_builder",
        output="screen",
        parameters=[
            {
                "map_service_name": "/robot_data/map/get_map",
                "graph_filepath": graph_filepath,
                "graph_frame_id": "map",
            }
        ],
    )

    route_server = Node(
        package="nav2_route",
        executable="route_server",
        name="route_server",
        output="screen",
        parameters=[
            route_server_params,
            {
                "use_sim_time": use_sim_time,
                "graph_filepath": graph_filepath,
            },
        ],
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[collision_monitor_params, {"use_sim_time": use_sim_time}],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    route_service = Node(
        package="forklift_demo_control",
        executable="route_service",
        name="route_service",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    start_navigation_after_graph = RegisterEventHandler(
        OnProcessExit(
            target_action=route_graph_builder,
            on_exit=[
                route_server,
                collision_monitor,
                lifecycle_manager,
                route_service,
            ],
        )
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
            remappings=[("/cmd_vel", "/cmd_vel_raw")],
        ),
        planner_server,
        behavior_server,
        bt_navigator,
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
            executable="map_service",
            name="map_service",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
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
            DeclareLaunchArgument("x", default_value="3.0"),
            DeclareLaunchArgument("y", default_value="3.5"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            gazebo,
            bridge,
            TimerAction(period=2.0, actions=[spawn_robot, spawn_pallet]),
            route_graph_builder,
            start_navigation_after_graph,
            *nodes,
        ]
    )
