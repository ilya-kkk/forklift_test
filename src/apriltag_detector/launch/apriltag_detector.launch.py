import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    detector_share = get_package_share_directory("apriltag_detector")
    use_sim_time = LaunchConfiguration("use_sim_time")
    configure_detector = LaunchConfiguration("configure_detector")
    activate_detector = LaunchConfiguration("activate_detector")
    launch_detection_monitor = LaunchConfiguration("launch_detection_monitor")

    direct_detector_params = os.path.join(
        detector_share, "config", "direct_detector.yaml"
    )
    detection_monitor_params = os.path.join(
        detector_share, "config", "detection_monitor.yaml"
    )

    direct_detector = LifecycleNode(
        package="apriltag_detector",
        executable="direct_detector",
        name="apriltag_direct_detector",
        namespace="",
        output="screen",
        parameters=[direct_detector_params, {"use_sim_time": use_sim_time}],
    )

    configure_direct_detector = TimerAction(
        period=0.5,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(direct_detector),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                ),
                condition=IfCondition(configure_detector),
            )
        ],
    )

    activate_direct_detector = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=direct_detector,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(direct_detector),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                    condition=IfCondition(activate_detector),
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("configure_detector", default_value="true"),
            DeclareLaunchArgument("activate_detector", default_value="false"),
            DeclareLaunchArgument("launch_detection_monitor", default_value="false"),
            direct_detector,
            activate_direct_detector,
            configure_direct_detector,
            Node(
                package="apriltag_detector",
                executable="detection_monitor",
                name="apriltag_detection_monitor",
                output="screen",
                parameters=[detection_monitor_params, {"use_sim_time": use_sim_time}],
                condition=IfCondition(launch_detection_monitor),
            ),
        ]
    )
