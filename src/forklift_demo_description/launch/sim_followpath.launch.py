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
    pkg_share = get_package_share_directory("forklift_demo_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_gz_gui = LaunchConfiguration("launch_gz_gui")
    enable_cmd_vel_to_motors = LaunchConfiguration("enable_cmd_vel_to_motors")
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")

    robot_xacro = os.path.join(pkg_share, "urdf", "forklift_demo.urdf.xacro")
    bridge_config = os.path.join(pkg_share, "config", "bridge_config.yaml")
    nav2_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    collision_monitor_params = os.path.join(
        pkg_share, "config", "collision_monitor_params.yaml"
    )
    slam_params = os.path.join(pkg_share, "config", "slam_toolbox.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "demo.rviz")

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

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[collision_monitor_params, {"use_sim_time": use_sim_time}],
    )

    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params,
            {
                "use_sim_time": use_sim_time,
                "use_lifecycle_manager": False,
            },
        ],
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
        collision_monitor,
        lifecycle_manager,
        route_service,
        Node(
            package="forklift_demo_control",
            executable="cmd_vel_to_motors",
            name="cmd_vel_to_motors",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "enabled": enable_cmd_vel_to_motors,
                    "robot_description": robot_description,
                    "cmd_vel_topic": "/cmd_vel",
                    "cmd_vel_frame": "base_link",
                    "steering_joint_name": "right_steering_joint",
                    "drive_wheel_joint_name": "right_wheel_joint",
                    "steering_cmd_topic": "/forklift/right_steering_cmd",
                    "wheel_cmd_topic": "/forklift/right_wheel_cmd",
                    "drive_wheel_radius": 0.125,
                    "drive_wheel_velocity_sign": -1.0,
                }
            ],
        ),
        Node(
            package="forklift_demo_control",
            executable="fork_position_controller",
            name="fork_position_controller",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "joint_name": "fork_joint",
                    "position_cmd_topic": "/forklift/fork_cmd",
                    "velocity_cmd_topic": "/forklift/fork_velocity_cmd",
                    "invert_position_command": False,
                    "position_lower_limit": 0.0,
                    "position_upper_limit": 1.0,
                    "max_velocity": 0.35,
                    "p_gain": 2.5,
                    "deadband": 0.002,
                    "publish_rate_hz": 30.0,
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
            executable="scan_sector_filter",
            name="scan_left_sector_filter",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "input_scan_topic": "/scan_left",
                    "output_scan_topic": "/scan_left_limited",
                    # Keep [-90, 180] by blinding [-180, -90].
                    "blind_sector_center_deg": -135.0,
                    "blind_sector_half_width_deg": 45.0,
                }
            ],
        ),
        Node(
            package="forklift_demo_control",
            executable="scan_sector_filter",
            name="scan_right_sector_filter",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "input_scan_topic": "/scan_right",
                    "output_scan_topic": "/scan_right_limited",
                    # Keep [-180, 90] by blinding [90, 180].
                    "blind_sector_center_deg": 135.0,
                    "blind_sector_half_width_deg": 45.0,
                }
            ],
        ),
        Node(
            package="forklift_demo_control",
            executable="scan_sector_filter",
            name="scan_rear_sector_filter",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "input_scan_topic": "/scan_raw",
                    "output_scan_topic": "/scan_rear_limited",
                    # Keep [-45, 45] by blinding everything else.
                    "blind_sector_center_deg": 180.0,
                    "blind_sector_half_width_deg": 135.0,
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
            executable="json_map_visualizer",
            name="json_map_visualizer",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="forklift_demo_control",
            executable="up_lidar_marker_service",
            name="up_lidar_marker_service",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "frame_id": "up_lidar_link",
                    "marker_topic": "/debug/up_lidar_marker",
                    "moving_topic": "/debug/up_lidar_marker/is_moving",
                    "service_name": "/robot_data/marker/up_lidar/control",
                }
            ],
        ),
        Node(
            package="forklift_demo_control",
            executable="cmd_vel_activity_service",
            name="cmd_vel_activity_service",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "wheel_velocity_topic": "/forklift/right_wheel_cmd",
                    "moving_topic": "/debug/up_lidar_marker/is_moving",
                    "service_name": "/robot_data/marker/cmd_vel_watch/control",
                }
            ],
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
                default_value=os.path.join(pkg_share, "worlds", "square_room.sdf"),
            ),
            DeclareLaunchArgument("x", default_value="3.0"),
            DeclareLaunchArgument("y", default_value="3.5"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.path.dirname(pkg_share),
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.path.join(pkg_share, "models"),
            ),
            AppendEnvironmentVariable(
                "SDF_PATH",
                os.path.join(pkg_share, "models"),
            ),
            gazebo,
            bridge,
            TimerAction(period=2.0, actions=[spawn_robot]),
            *nodes,
            activate_slam_toolbox,
            configure_slam_toolbox,
        ]
    )
