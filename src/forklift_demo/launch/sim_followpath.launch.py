import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    demo_share = get_package_share_directory("forklift_demo")
    navigation_share = get_package_share_directory("navigation_forklift")
    motors_share = get_package_share_directory("cmd_vel_to_motors")
    fork_share = get_package_share_directory("fork_manager")
    map_share = get_package_share_directory("map_service")
    collision_share = get_package_share_directory("collision_monitor")
    rviz_share = get_package_share_directory("rviz")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_gz_gui = LaunchConfiguration("launch_gz_gui")
    enable_cmd_vel_to_motors = LaunchConfiguration("enable_cmd_vel_to_motors")
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")

    robot_xacro = os.path.join(demo_share, "urdf", "forklift_demo.urdf.xacro")
    bridge_config = os.path.join(demo_share, "config", "bridge_config.yaml")
    nav2_params = os.path.join(navigation_share, "config", "nav2_params.yaml")
    route_params = os.path.join(navigation_share, "config", "route_service.yaml")
    slam_params = os.path.join(navigation_share, "config", "slam_toolbox.yaml")
    cmd_vel_to_motors_params = os.path.join(
        motors_share, "config", "cmd_vel_to_motors.yaml"
    )
    fork_params = os.path.join(fork_share, "config", "fork_position_controller.yaml")
    map_params = os.path.join(map_share, "config", "map_service.yaml")
    collision_monitor_params = os.path.join(
        collision_share, "config", "collision_monitor_params.yaml"
    )
    scan_filter_params = os.path.join(
        collision_share, "config", "scan_sector_filters.yaml"
    )
    json_map_visualizer_params = os.path.join(
        rviz_share, "config", "json_map_visualizer.yaml"
    )
    up_lidar_marker_params = os.path.join(
        rviz_share, "config", "up_lidar_marker_service.yaml"
    )
    cmd_vel_activity_params = os.path.join(
        rviz_share, "config", "cmd_vel_activity_service.yaml"
    )
    cmd_vel_twist_stamper_params = os.path.join(
        rviz_share, "config", "cmd_vel_twist_stamper.yaml"
    )
    rviz_config = os.path.join(rviz_share, "rviz", "demo.rviz")

    robot_description = ParameterValue(
        Command(["xacro ", robot_xacro, " use_sim_time:=", use_sim_time]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                PythonExpression(
                    [
                        "'-r -v 2 ' if '",
                        launch_gz_gui,
                        "' == 'true' else '-r -s --headless-rendering -v 2 '",
                    ]
                ),
                world,
            ]
        }.items(),
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
            "-topic",
            "robot_description",
            "-x",
            x,
            "-y",
            y,
            "-z",
            "0.05",
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

    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
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
        slam_toolbox,
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            remappings=[("/cmd_vel", "/cmd_vel_raw")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            remappings=[("/cmd_vel", "/cmd_vel_raw")],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[collision_monitor_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="navigation_forklift",
            executable="route_service",
            name="route_service",
            output="screen",
            parameters=[route_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="cmd_vel_to_motors",
            executable="cmd_vel_to_motors",
            name="cmd_vel_to_motors",
            output="screen",
            parameters=[
                cmd_vel_to_motors_params,
                {
                    "use_sim_time": use_sim_time,
                    "enabled": enable_cmd_vel_to_motors,
                    "robot_description": robot_description,
                },
            ],
        ),
        Node(
            package="fork_manager",
            executable="fork_position_controller",
            name="fork_position_controller",
            output="screen",
            parameters=[fork_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="collision_monitor",
            executable="scan_sector_filter",
            name="scan_sector_filter",
            output="screen",
            parameters=[scan_filter_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="collision_monitor",
            executable="scan_sector_filter",
            name="scan_left_sector_filter",
            output="screen",
            parameters=[scan_filter_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="collision_monitor",
            executable="scan_sector_filter",
            name="scan_right_sector_filter",
            output="screen",
            parameters=[scan_filter_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="collision_monitor",
            executable="scan_sector_filter",
            name="scan_rear_sector_filter",
            output="screen",
            parameters=[scan_filter_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="map_service",
            executable="map_service",
            name="map_service",
            output="screen",
            parameters=[map_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz",
            executable="json_map_visualizer",
            name="json_map_visualizer",
            output="screen",
            parameters=[json_map_visualizer_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz",
            executable="up_lidar_marker_service",
            name="up_lidar_marker_service",
            output="screen",
            parameters=[up_lidar_marker_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz",
            executable="cmd_vel_activity_service",
            name="cmd_vel_activity_service",
            output="screen",
            parameters=[cmd_vel_activity_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz",
            executable="cmd_vel_twist_stamper",
            name="cmd_vel_twist_stamper",
            output="screen",
            parameters=[cmd_vel_twist_stamper_params, {"use_sim_time": use_sim_time}],
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    output="screen",
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": use_sim_time}],
                    condition=IfCondition(launch_rviz),
                )
            ],
        ),
    ]

    configure_slam_toolbox = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_slam_toolbox = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("launch_gz_gui", default_value="true"),
            DeclareLaunchArgument("enable_cmd_vel_to_motors", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(demo_share, "worlds", "square_room.sdf"),
            ),
            DeclareLaunchArgument("x", default_value="3.0"),
            DeclareLaunchArgument("y", default_value="3.5"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(demo_share)),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", os.path.join(demo_share, "models")
            ),
            AppendEnvironmentVariable("SDF_PATH", os.path.join(demo_share, "models")),
            gazebo,
            bridge,
            TimerAction(period=2.0, actions=[spawn_robot]),
            *nodes,
            activate_slam_toolbox,
            configure_slam_toolbox,
        ]
    )
