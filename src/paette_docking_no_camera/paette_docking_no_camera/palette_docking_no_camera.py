import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from forklift_interfaces.srv import StringWithJson
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from nav2_msgs.action import DriveOnHeading
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


IDLE = "IDLE"
WAIT_FOR_TAG = "WAIT_FOR_TAG"
RETREAT = "RETREAT"
PLANNING = "PLANNING"
APPROACH_PREFINAL = "APPROACH_PREFINAL"
ALIGN_TO_PALLET = "ALIGN_TO_PALLET"
FINAL_DOCK = "FINAL_DOCK"
DONE = "DONE"
FAILED = "FAILED"
CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PalletObservation:
    frame_id: str
    tag_x: float
    tag_y: float
    tag_z: float
    distance: float
    angle: float
    age_sec: float


@dataclass(frozen=True)
class DockPlan:
    tag_frame: str
    pallet_x: float
    pallet_y: float
    prefinal_x: float
    prefinal_y: float
    prefinal_yaw: float
    segment_yaw: float
    segment_distance: float
    final_drive_distance: float
    retreat_required: bool


class DockingCancelled(Exception):
    pass


class PaletteDockingNoCamera(Node):
    def __init__(self) -> None:
        super().__init__("palette_docking_no_camera")

        self.declare_parameter("service_name", "/palette_docking_no_camera/control")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("tag_frame", "")
        self.declare_parameter("tag_frame_candidates", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("dock_motion_mode", "reverse")
        self.declare_parameter("lookup_timeout_sec", 0.05)
        self.declare_parameter("tag_timeout_sec", 0.5)
        self.declare_parameter("wait_for_tag_timeout_sec", 5.0)
        self.declare_parameter("transform_timeout_sec", 1.0)
        self.declare_parameter("close_distance_threshold_m", 0.8)
        self.declare_parameter("retreat_distance_m", 0.45)
        self.declare_parameter("prefinal_distance_from_pallet_m", 1.0)
        self.declare_parameter("final_drive_distance_m", 1.0)
        self.declare_parameter("max_initial_angle_rad", 1.2)
        self.declare_parameter("prefinal_xy_tolerance_m", 0.05)
        self.declare_parameter("yaw_tolerance_rad", 0.12)
        self.declare_parameter("yaw_timeout_acceptance_rad", 0.35)
        self.declare_parameter("dock_longitudinal_tolerance_m", 0.15)
        self.declare_parameter("dock_lateral_tolerance_m", 0.10)
        self.declare_parameter("dock_pose_yaw_tolerance_rad", 0.10)
        self.declare_parameter("dock_camera_frame", "camera_link")
        self.declare_parameter("dock_target_x_m", 0.10)
        self.declare_parameter("dock_target_y_m", 0.0)
        self.declare_parameter("dock_tf_x_tolerance_m", 0.05)
        self.declare_parameter("dock_tf_y_tolerance_m", 0.05)
        self.declare_parameter("final_dock_timeout_sec", 120.0)
        self.declare_parameter("final_dock_lateral_gain", 1.0)
        self.declare_parameter("final_dock_initial_lateral_gain", 0.1)
        self.declare_parameter("final_dock_gain_ramp_retreat_m", 0.50)
        self.declare_parameter("final_dock_max_angular_radps", 0.30)
        self.declare_parameter("final_dock_alignment_speed_mps", 0.20)
        self.declare_parameter("final_dock_max_alignment_retreat_m", 0.60)
        self.declare_parameter("final_dock_abort_lateral_error_m", 0.25)
        self.declare_parameter("final_dock_tf_rate_hz", 5.0)
        self.declare_parameter("final_dock_max_tf_lookups", 600)
        self.declare_parameter("drive_on_heading_action_name", "drive_on_heading")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_pallet_docking")
        self.declare_parameter(
            "cmd_vel_orchestrator_service", "/cmd_vel_arcestrator/control"
        )
        self.declare_parameter("cmd_vel_docking_source", "pallet_docking")
        self.declare_parameter("cmd_vel_restore_source", "navigation")
        self.declare_parameter("path_topic", "/palette_docking_no_camera/path")
        self.declare_parameter("marker_topic", "/palette_docking_no_camera/markers")
        self.declare_parameter("action_server_timeout_sec", 5.0)
        self.declare_parameter("spin_timeout_sec", 20.0)
        self.declare_parameter("drive_timeout_sec", 20.0)
        self.declare_parameter("yaw_angular_speed_radps", 0.35)
        self.declare_parameter("yaw_linear_assist_mps", 0.03)
        self.declare_parameter("yaw_control_frequency_hz", 20.0)
        self.declare_parameter("retreat_speed_mps", 0.12)
        self.declare_parameter("prefinal_drive_speed_mps", 0.20)
        self.declare_parameter("final_drive_speed_mps", 0.16)
        self.declare_parameter("marker_lifetime_sec", 0.0)
        self.declare_parameter("start_enabled", False)

        service_name = str(self.get_parameter("service_name").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._global_frame = str(self.get_parameter("global_frame").value)
        self._robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self._configured_tag_frame = str(self.get_parameter("tag_frame").value).strip()
        self._tag_frame_candidates = [
            str(frame).strip()
            for frame in self.get_parameter("tag_frame_candidates").value
            if str(frame).strip()
        ]
        self._motion_mode = str(self.get_parameter("dock_motion_mode").value).lower()
        if self._motion_mode not in {"forward", "reverse"}:
            raise ValueError("dock_motion_mode must be 'forward' or 'reverse'")
        self._motion_sign = -1.0 if self._motion_mode == "reverse" else 1.0

        self._lookup_timeout = self._param_float("lookup_timeout_sec", minimum=0.0)
        self._tag_timeout = self._param_float("tag_timeout_sec", minimum=0.0)
        self._wait_for_tag_timeout = self._param_float(
            "wait_for_tag_timeout_sec", minimum=0.0
        )
        self._transform_timeout = self._param_float(
            "transform_timeout_sec", minimum=0.0
        )
        self._default_close_distance = self._param_float(
            "close_distance_threshold_m", minimum=0.0
        )
        self._default_retreat_distance = self._param_float(
            "retreat_distance_m", minimum=0.0
        )
        self._default_prefinal_distance = self._param_float(
            "prefinal_distance_from_pallet_m", minimum=0.0
        )
        self._default_final_drive_distance = self._param_float(
            "final_drive_distance_m", minimum=0.0
        )
        self._default_max_initial_angle = self._param_float(
            "max_initial_angle_rad", minimum=0.0
        )
        self._prefinal_xy_tolerance = self._param_float(
            "prefinal_xy_tolerance_m", minimum=0.0
        )
        self._yaw_tolerance = self._param_float("yaw_tolerance_rad", minimum=0.0)
        self._yaw_timeout_acceptance = self._param_float(
            "yaw_timeout_acceptance_rad", minimum=self._yaw_tolerance
        )
        self._dock_longitudinal_tolerance = self._param_float(
            "dock_longitudinal_tolerance_m", minimum=0.0
        )
        self._dock_lateral_tolerance = self._param_float(
            "dock_lateral_tolerance_m", minimum=0.0
        )
        self._dock_pose_yaw_tolerance = self._param_float(
            "dock_pose_yaw_tolerance_rad", minimum=0.0
        )
        self._dock_camera_frame = str(
            self.get_parameter("dock_camera_frame").value
        ).strip()
        self._dock_target_x = float(self.get_parameter("dock_target_x_m").value)
        self._dock_target_y = float(self.get_parameter("dock_target_y_m").value)
        self._dock_tf_x_tolerance = self._param_float(
            "dock_tf_x_tolerance_m", minimum=0.0
        )
        self._dock_tf_y_tolerance = self._param_float(
            "dock_tf_y_tolerance_m", minimum=0.0
        )
        self._final_dock_timeout = self._param_float(
            "final_dock_timeout_sec", minimum=0.0
        )
        self._final_dock_lateral_gain = self._param_float(
            "final_dock_lateral_gain", minimum=0.0
        )
        self._final_dock_initial_lateral_gain = self._param_float(
            "final_dock_initial_lateral_gain", minimum=0.0
        )
        self._final_dock_gain_ramp_retreat = self._param_float(
            "final_dock_gain_ramp_retreat_m", minimum=0.01
        )
        self._final_dock_max_angular = self._param_float(
            "final_dock_max_angular_radps", minimum=0.0
        )
        self._final_dock_alignment_speed = self._param_float(
            "final_dock_alignment_speed_mps", minimum=0.01
        )
        self._final_dock_max_alignment_retreat = self._param_float(
            "final_dock_max_alignment_retreat_m", minimum=0.0
        )
        self._final_dock_abort_lateral_error = self._param_float(
            "final_dock_abort_lateral_error_m",
            minimum=self._dock_tf_y_tolerance,
        )
        self._final_dock_tf_rate = self._param_float(
            "final_dock_tf_rate_hz", minimum=1.0
        )
        self._final_dock_max_tf_lookups = max(
            1, int(self.get_parameter("final_dock_max_tf_lookups").value)
        )
        self._action_server_timeout = self._param_float(
            "action_server_timeout_sec", minimum=0.0
        )
        self._spin_timeout = self._param_float("spin_timeout_sec", minimum=0.0)
        self._drive_timeout = self._param_float("drive_timeout_sec", minimum=0.0)
        self._yaw_angular_speed = self._param_float(
            "yaw_angular_speed_radps", minimum=0.01
        )
        self._yaw_linear_assist = self._param_float(
            "yaw_linear_assist_mps", minimum=0.0
        )
        self._yaw_control_frequency = self._param_float(
            "yaw_control_frequency_hz", minimum=1.0
        )
        self._retreat_speed = self._param_float("retreat_speed_mps", minimum=0.01)
        self._prefinal_drive_speed = self._param_float(
            "prefinal_drive_speed_mps", minimum=0.01
        )
        self._final_drive_speed = self._param_float(
            "final_drive_speed_mps", minimum=0.01
        )
        self._marker_lifetime = self._param_float("marker_lifetime_sec", minimum=0.0)

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._path_publisher = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), path_qos
        )
        self._cmd_vel_publisher = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self._cmd_vel_docking_source = str(
            self.get_parameter("cmd_vel_docking_source").value
        )
        self._cmd_vel_restore_source = str(
            self.get_parameter("cmd_vel_restore_source").value
        )
        self._orchestrator_client = self.create_client(
            StringWithJson,
            str(self.get_parameter("cmd_vel_orchestrator_service").value),
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), marker_qos
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._drive_client = ActionClient(
            self,
            DriveOnHeading,
            str(self.get_parameter("drive_on_heading_action_name").value),
        )

        self._lock = threading.Lock()
        self._state = IDLE
        self._active_thread: Optional[threading.Thread] = None
        self._active_goal_handle = None
        self._cancel_requested = False
        self._failure_reason = ""
        self._last_result: Dict[str, Any] = {"state": "idle"}
        self._last_observation: Optional[PalletObservation] = None
        self._last_plan: Optional[DockPlan] = None

        self.create_service(StringWithJson, service_name, self._handle_control)

        if bool(self.get_parameter("start_enabled").value):
            self._start({})

        self.get_logger().info(
            "palette_docking_no_camera ready: service=%s target_frame=%s "
            "global_frame=%s base=%s motion=%s tag=%s candidates=%d"
            % (
                service_name,
                self._target_frame,
                self._global_frame,
                self._robot_base_frame,
                self._motion_mode,
                self._configured_tag_frame or "<auto>",
                len(self._tag_frame_candidates),
            )
        )

    def _handle_control(self, request, response):
        try:
            payload = self._decode_payload(request.message or "{}")
        except ValueError as exc:
            response.success = False
            response.message = json.dumps({"error": "invalid_json", "details": str(exc)})
            return response

        command = str(payload.get("command", "")).strip().lower()
        if "enabled" in payload:
            command = "start" if bool(payload["enabled"]) else "cancel"

        if command in {"start", "dock", "enable"}:
            response.success = self._start(payload)
            response.message = json.dumps(self._status(), ensure_ascii=False)
            return response
        if command in {"cancel", "stop", "disable"}:
            self._request_cancel("control cancel")
            response.success = True
            response.message = json.dumps(self._status(), ensure_ascii=False)
            return response
        if command == "reset":
            response.success = self._reset()
            response.message = json.dumps(self._status(), ensure_ascii=False)
            return response
        if command in {"", "status"}:
            response.success = True
            response.message = json.dumps(self._status(), ensure_ascii=False)
            return response

        response.success = False
        response.message = json.dumps(
            {
                "error": "invalid_command",
                "details": "expected start|dock|cancel|status|reset or enabled bool",
            },
            ensure_ascii=False,
        )
        return response

    def _start(self, payload: Dict[str, Any]) -> bool:
        with self._lock:
            if self._active_thread is not None and self._active_thread.is_alive():
                self._last_result = {
                    "state": "busy",
                    "details": "docking already in progress",
                }
                return False
            self._cancel_requested = False
            self._failure_reason = ""
            self._last_result = {"state": "running"}
            self._state = WAIT_FOR_TAG
            self._last_observation = None
            self._last_plan = None

        worker = threading.Thread(
            target=self._run_docking_sequence,
            args=(dict(payload),),
            daemon=True,
        )
        with self._lock:
            self._active_thread = worker
        worker.start()
        return True

    def _reset(self) -> bool:
        with self._lock:
            active = self._active_thread is not None and self._active_thread.is_alive()
            if active:
                return False
            self._state = IDLE
            self._failure_reason = ""
            self._last_result = {"state": "idle"}
            self._last_observation = None
            self._last_plan = None
        self._publish_delete_all_markers()
        return True

    def _request_cancel(self, reason: str) -> None:
        goal_handle = None
        with self._lock:
            self._cancel_requested = True
            self._failure_reason = reason
            goal_handle = self._active_goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:  # pragma: no cover - defensive logging
                self.get_logger().warn("Failed to cancel active Nav2 goal: %s" % exc)

    def _run_docking_sequence(self, payload: Dict[str, Any]) -> None:
        result: Dict[str, Any] = {"state": "failed", "error": "unknown"}
        try:
            options = self._options_from_payload(payload)
            observation = self._wait_for_observation(options)
            self._store_observation(observation)
            self._check_cancelled()
            self._validate_initial_observation(observation, options)
            initial_tf_eval = self._evaluate_final_tag_tf(observation.frame_id)
            self._select_cmd_vel_source(self._cmd_vel_docking_source)
            self._set_state(FINAL_DOCK, "camera/tag closed-loop docking")
            final_tf_eval = self._drive_to_final_tag_tf(observation.frame_id)
            self.get_logger().info(
                "final camera/tag check: frame=%s x=%.3f y=%.3f z=%.3f "
                "x_error=%.3f y_error=%.3f accepted=%s"
                % (
                    observation.frame_id,
                    final_tf_eval["x"],
                    final_tf_eval["y"],
                    final_tf_eval["z"],
                    final_tf_eval["x_error_m"],
                    final_tf_eval["y_error_m"],
                    str(final_tf_eval["accepted"]).lower(),
                )
            )

            self._set_state(DONE, "docking sequence finished")
            result = {
                "state": "finished",
                "tag_frame": observation.frame_id,
                "retreat_required": False,
                "prefinal_distance_m": 0.0,
                "final_drive_distance_m": max(
                    0.0, initial_tf_eval["x"] - final_tf_eval["x"]
                ),
                "initial_tf_eval": initial_tf_eval,
                "final_tf_eval": final_tf_eval,
                "final_pose_eval": None,
            }
        except DockingCancelled:
            self._set_state(CANCELLED, "docking cancelled")
            result = {"state": "cancelled", "reason": self._failure_reason or "cancelled"}
        except Exception as exc:  # pragma: no cover - defensive logging
            self._failure_reason = str(exc)
            self._set_state(FAILED, str(exc))
            result = {"state": "failed", "error": str(exc)}
            self.get_logger().error("Docking sequence failed: %s" % exc)
        finally:
            self._publish_stop()
            try:
                self._select_cmd_vel_source(self._cmd_vel_restore_source)
            except Exception as exc:  # pragma: no cover - best effort handoff
                self.get_logger().error(
                    "Failed to restore cmd_vel source %s: %s"
                    % (self._cmd_vel_restore_source, exc)
                )
            with self._lock:
                self._active_goal_handle = None
                self._active_thread = None
                self._last_result = result

    def _options_from_payload(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        max_initial_angle = self._payload_float(
            payload, "max_initial_angle_rad", self._default_max_initial_angle
        )
        if "max_initial_angle_deg" in payload:
            max_initial_angle = math.radians(float(payload["max_initial_angle_deg"]))
        return {
            "tag_frame": str(payload.get("tag_frame", self._configured_tag_frame)).strip(),
            "close_distance": self._payload_float(
                payload, "close_distance_threshold_m", self._default_close_distance
            ),
            "retreat_distance": max(
                0.0,
                self._payload_float(
                    payload, "retreat_distance_m", self._default_retreat_distance
                ),
            ),
            "prefinal_distance": max(
                0.0,
                self._payload_float(
                    payload,
                    "prefinal_distance_from_pallet_m",
                    self._default_prefinal_distance,
                ),
            ),
            "final_drive_distance": max(
                0.0,
                self._payload_float(
                    payload, "final_drive_distance_m", self._default_final_drive_distance
                ),
            ),
            "max_initial_angle": max(0.0, float(max_initial_angle)),
        }

    def _wait_for_observation(self, options: Dict[str, Any]) -> PalletObservation:
        self._set_state(WAIT_FOR_TAG, "waiting for AprilTag TF")
        deadline = time.monotonic() + self._wait_for_tag_timeout
        while rclpy.ok():
            self._check_cancelled()
            observation = self._lookup_pallet_observation(options)
            if observation is not None:
                self.get_logger().info(
                    "Detected pallet TF: tag=%s distance=%.3f angle=%.3f rad age=%.3fs"
                    % (
                        observation.frame_id,
                        observation.distance,
                        observation.angle,
                        observation.age_sec,
                    )
                )
                return observation

            if self._wait_for_tag_timeout > 0.0 and time.monotonic() > deadline:
                raise ValueError("no valid pallet tag TF")
            # The detector publishes at 5 Hz. Polling faster only repeats the
            # same transform and multiplies candidate-frame lookups.
            time.sleep(0.2)

        raise DockingCancelled()

    def _lookup_pallet_observation(
        self, options: Dict[str, Any]
    ) -> Optional[PalletObservation]:
        observations = []
        for frame_id in self._candidate_frames(options):
            transform = self._lookup_tag_transform(frame_id)
            if transform is None:
                continue
            observation = self._observation_from_transform(frame_id, transform)
            if observation is not None:
                observations.append(observation)

        if not observations:
            return None
        return min(observations, key=lambda item: abs(item.angle))

    def _candidate_frames(self, options: Dict[str, Any]) -> List[str]:
        tag_frame = str(options.get("tag_frame", "")).strip()
        if tag_frame:
            return [tag_frame]
        return list(self._tag_frame_candidates)

    def _lookup_tag_transform(self, source_frame: str) -> Optional[TransformStamped]:
        try:
            if self._lookup_timeout > 0.0:
                return self._tf_buffer.lookup_transform(
                    self._target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self._lookup_timeout),
                )
            return self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                Time(),
            )
        except TransformException:
            return None

    def _observation_from_transform(
        self, frame_id: str, transform: TransformStamped
    ) -> Optional[PalletObservation]:
        age_sec = self._transform_age_sec(transform)
        if self._tag_timeout > 0.0 and age_sec >= 0.0 and age_sec > self._tag_timeout:
            return None

        translation = transform.transform.translation
        tag_x = float(translation.x)
        tag_y = float(translation.y)
        approach_x = self._motion_sign * tag_x
        approach_y = self._motion_sign * tag_y
        return PalletObservation(
            frame_id=frame_id,
            tag_x=tag_x,
            tag_y=tag_y,
            tag_z=float(translation.z),
            distance=math.hypot(approach_x, approach_y),
            angle=math.atan2(approach_y, approach_x),
            age_sec=age_sec,
        )

    def _validate_initial_observation(
        self, observation: PalletObservation, options: Dict[str, Any]
    ) -> None:
        max_initial_angle = float(options["max_initial_angle"])
        if max_initial_angle > 0.0 and abs(observation.angle) > max_initial_angle:
            raise ValueError(
                "required initial turn %.3f rad exceeds max_initial_angle %.3f rad"
                % (observation.angle, max_initial_angle)
            )

    def _lookup_robot_pose(self) -> RobotPose:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._robot_base_frame,
                Time(),
                timeout=Duration(seconds=self._transform_timeout),
            )
        except TransformException as exc:
            raise ValueError(
                "robot pose TF %s -> %s is not available: %s"
                % (self._global_frame, self._robot_base_frame, exc)
            ) from exc

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return RobotPose(
            x=float(translation.x),
            y=float(translation.y),
            yaw=self._yaw_from_quaternion(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ),
        )

    def _build_plan(
        self,
        observation: PalletObservation,
        robot_pose: RobotPose,
        options: Dict[str, Any],
        retreat_required: bool,
    ) -> DockPlan:
        pallet_x, pallet_y = self._base_point_to_global(
            robot_pose, observation.tag_x, observation.tag_y
        )
        axis_x = pallet_x - robot_pose.x
        axis_y = pallet_y - robot_pose.y
        axis_length = math.hypot(axis_x, axis_y)
        if axis_length <= 1e-6:
            axis_x = self._motion_sign * math.cos(robot_pose.yaw)
            axis_y = self._motion_sign * math.sin(robot_pose.yaw)
        else:
            axis_x /= axis_length
            axis_y /= axis_length

        prefinal_distance = float(options["prefinal_distance"])
        prefinal_x = pallet_x - axis_x * prefinal_distance
        prefinal_y = pallet_y - axis_y * prefinal_distance
        segment_dx = prefinal_x - robot_pose.x
        segment_dy = prefinal_y - robot_pose.y
        segment_distance = math.hypot(segment_dx, segment_dy)
        segment_yaw = (
            math.atan2(segment_dy, segment_dx)
            if segment_distance > 1e-6
            else robot_pose.yaw
        )
        prefinal_yaw = self._normalize_angle(
            math.atan2(axis_y, axis_x)
            + (math.pi if self._motion_mode == "reverse" else 0.0)
        )

        return DockPlan(
            tag_frame=observation.frame_id,
            pallet_x=pallet_x,
            pallet_y=pallet_y,
            prefinal_x=prefinal_x,
            prefinal_y=prefinal_y,
            prefinal_yaw=prefinal_yaw,
            segment_yaw=segment_yaw,
            segment_distance=segment_distance,
            final_drive_distance=float(options["final_drive_distance"]),
            retreat_required=retreat_required,
        )

    def _build_prefinal_visual_path(
        self, robot_pose: RobotPose, plan: DockPlan
    ) -> Path:
        path = Path()
        path.header.frame_id = self._global_frame
        path.header.stamp = self.get_clock().now().to_msg()

        start_pose = self._pose_stamped(
            robot_pose.x,
            robot_pose.y,
            (
                self._motion_yaw(plan.segment_yaw)
                if plan.segment_distance > 1e-6
                else robot_pose.yaw
            ),
            path.header,
        )
        prefinal_pose = self._pose_stamped(
            plan.prefinal_x,
            plan.prefinal_y,
            plan.prefinal_yaw,
            path.header,
        )
        path.poses.extend([start_pose, prefinal_pose])
        return path

    def _refined_pallet_point(
        self, robot_pose: RobotPose, plan: DockPlan, options: Dict[str, Any]
    ) -> Tuple[float, float]:
        observation = self._lookup_pallet_observation(options)
        if observation is None:
            return plan.pallet_x, plan.pallet_y
        self._store_observation(observation)
        return self._base_point_to_global(
            robot_pose, observation.tag_x, observation.tag_y
        )

    def _docking_yaw(
        self, robot_pose: RobotPose, pallet_x: float, pallet_y: float
    ) -> float:
        dx = pallet_x - robot_pose.x
        dy = pallet_y - robot_pose.y
        if math.hypot(dx, dy) <= 1e-6:
            return robot_pose.yaw
        yaw_to_pallet = math.atan2(dy, dx)
        if self._motion_mode == "reverse":
            yaw_to_pallet += math.pi
        return self._normalize_angle(yaw_to_pallet)

    def _motion_yaw(self, travel_yaw: float) -> float:
        if self._motion_mode == "reverse":
            travel_yaw += math.pi
        return self._normalize_angle(travel_yaw)

    def _turn_to_yaw(self, target_yaw: float, current_yaw: float, label: str) -> None:
        delta_yaw = self._normalize_angle(target_yaw - current_yaw)
        if abs(delta_yaw) <= self._yaw_tolerance:
            self.get_logger().info(
                "%s yaw turn skipped: delta=%.3f rad tol=%.3f"
                % (label, delta_yaw, self._yaw_tolerance)
            )
            return

        self.get_logger().info(
            "%s: yaw turning by %.3f rad angular=%.3f linear_assist=%.3f "
            "timeout=%.1fs"
            % (
                label,
                delta_yaw,
                self._yaw_angular_speed,
                self._yaw_linear_assist,
                self._spin_timeout,
            )
        )
        deadline = (
            time.monotonic() + self._spin_timeout if self._spin_timeout > 0.0 else None
        )
        period = 1.0 / self._yaw_control_frequency
        last_progress_log_at = 0.0
        try:
            while rclpy.ok():
                self._check_cancelled()
                robot_pose = self._lookup_robot_pose()
                delta_yaw = self._normalize_angle(target_yaw - robot_pose.yaw)
                if abs(delta_yaw) <= self._yaw_tolerance:
                    self.get_logger().info(
                        "%s yaw reached: current=%.3f target=%.3f delta=%.3f"
                        % (label, robot_pose.yaw, target_yaw, delta_yaw)
                    )
                    return

                if deadline is not None and time.monotonic() > deadline:
                    if abs(delta_yaw) <= self._yaw_timeout_acceptance:
                        self.get_logger().warn(
                            "%s yaw turn timed out near target; accepting "
                            "current=%.3f target=%.3f delta=%.3f acceptance=%.3f"
                            % (
                                label,
                                robot_pose.yaw,
                                target_yaw,
                                delta_yaw,
                                self._yaw_timeout_acceptance,
                            )
                        )
                        return
                    raise ValueError(
                        "%s yaw turn timed out: current=%.3f target=%.3f delta=%.3f"
                        % (label, robot_pose.yaw, target_yaw, delta_yaw)
                    )

                twist = Twist()
                twist.linear.x = -self._motion_sign * self._yaw_linear_assist
                twist.angular.z = math.copysign(self._yaw_angular_speed, delta_yaw)
                now = time.monotonic()
                if now - last_progress_log_at >= 1.0:
                    self.get_logger().info(
                        "%s yaw progress: current=%.3f target=%.3f delta=%.3f "
                        "linear=%.3f angular=%.3f"
                        % (
                            label,
                            robot_pose.yaw,
                            target_yaw,
                            delta_yaw,
                            twist.linear.x,
                            twist.angular.z,
                        )
                    )
                    last_progress_log_at = now
                self._cmd_vel_publisher.publish(twist)
                time.sleep(period)
        finally:
            self._publish_stop()

        raise DockingCancelled()

    def _drive_on_heading(
        self, distance_m: float, direction_sign: float, speed_mps: float, *, label: str
    ) -> None:
        distance_m = max(0.0, float(distance_m))
        if distance_m <= 1e-6:
            return
        if not self._drive_client.wait_for_server(
            timeout_sec=self._action_server_timeout
        ):
            raise ValueError("DriveOnHeading action server is not available")

        drive_sign = 1.0 if direction_sign >= 0.0 else -1.0
        goal = DriveOnHeading.Goal()
        goal.target.x = float(distance_m * drive_sign)
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = float(abs(speed_mps) * drive_sign)
        timeout = self._drive_action_timeout(distance_m, abs(speed_mps))
        self._set_duration(goal.time_allowance, timeout)
        self.get_logger().info(
            "%s: DriveOnHeading distance=%.3f sign=%.0f speed=%.3f timeout=%.1fs"
            % (label, distance_m, drive_sign, goal.speed, timeout)
        )
        self._send_action_goal_and_wait(
            self._drive_client, goal, timeout_sec=timeout, label="DriveOnHeading"
        )

    def _final_drive_distance_after_alignment(
        self, plan: DockPlan, options: Dict[str, Any]
    ) -> float:
        distance = plan.final_drive_distance
        observation = self._lookup_pallet_observation(options)
        if observation is None:
            return distance

        self._store_observation(observation)
        refreshed = max(distance, observation.distance)
        if refreshed > distance + 1e-3:
            self.get_logger().info(
                "final pallet drive distance adjusted from %.3f to %.3f using "
                "latest tag distance"
                % (distance, refreshed)
            )
        return refreshed

    def _refresh_plan_from_observation(
        self, options: Dict[str, Any], retreat_required: bool, label: str
    ) -> DockPlan:
        observation = self._lookup_pallet_observation(options)
        if observation is None:
            with self._lock:
                current_plan = self._last_plan
            if current_plan is None:
                raise ValueError("cannot refresh docking plan: no pallet tag TF")
            self.get_logger().warn(
                "%s: pallet TF unavailable after yaw turn; using previous plan"
                % label
            )
            return current_plan

        self._store_observation(observation)
        robot_pose = self._lookup_robot_pose()
        plan = self._build_plan(observation, robot_pose, options, retreat_required)
        self._store_plan(plan)
        visual_path = self._build_prefinal_visual_path(robot_pose, plan)
        self._publish_visualization(robot_pose, plan, visual_path)
        self.get_logger().info(
            "%s: refreshed docking geometry distance=%.3f angle=%.3f"
            % (label, plan.segment_distance, observation.angle)
        )
        return plan

    def _send_action_goal_and_wait(
        self,
        client: ActionClient,
        goal,
        *,
        timeout_sec: float,
        label: str,
    ):
        goal_response_event = threading.Event()
        result_event = threading.Event()
        state: Dict[str, Any] = {}

        def _goal_response_callback(future) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:  # pragma: no cover - defensive logging
                state["goal_error"] = str(exc)
                goal_response_event.set()
                result_event.set()
                return

            state["goal_handle"] = goal_handle
            goal_response_event.set()
            if goal_handle is None or not goal_handle.accepted:
                result_event.set()
                return

            with self._lock:
                self._active_goal_handle = goal_handle
            if self._is_cancel_requested():
                goal_handle.cancel_goal_async()
            goal_handle.get_result_async().add_done_callback(_result_callback)

        def _result_callback(future) -> None:
            try:
                state["result_response"] = future.result()
            except Exception as exc:  # pragma: no cover - defensive logging
                state["result_error"] = str(exc)
            result_event.set()

        client.send_goal_async(goal).add_done_callback(_goal_response_callback)

        if not self._wait_event_or_cancel(goal_response_event, timeout_sec):
            raise ValueError("%s goal response timed out" % label)
        if "goal_error" in state:
            raise ValueError("%s goal send failed: %s" % (label, state["goal_error"]))

        goal_handle = state.get("goal_handle")
        if goal_handle is None or not goal_handle.accepted:
            raise ValueError("%s goal was rejected" % label)

        if not self._wait_event_or_cancel(result_event, timeout_sec):
            try:
                goal_handle.cancel_goal_async()
            except Exception:  # pragma: no cover - defensive logging
                pass
            raise ValueError("%s result timed out" % label)

        with self._lock:
            if self._active_goal_handle is goal_handle:
                self._active_goal_handle = None

        if "result_error" in state:
            raise ValueError(
                "%s result retrieval failed: %s" % (label, state["result_error"])
            )

        result_response = state.get("result_response")
        if result_response is None:
            raise ValueError("%s returned no result" % label)
        if result_response.status == GoalStatus.STATUS_CANCELED:
            raise DockingCancelled()
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            raise ValueError(
                "%s finished with status %s" % (label, result_response.status)
            )
        return result_response.result

    def _wait_event_or_cancel(self, event: threading.Event, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec if timeout_sec > 0.0 else None
        while rclpy.ok():
            if event.wait(timeout=0.05):
                self._check_cancelled()
                return True
            self._check_cancelled()
            if deadline is not None and time.monotonic() > deadline:
                return False
        raise DockingCancelled()

    def _check_cancelled(self) -> None:
        if self._is_cancel_requested():
            raise DockingCancelled()

    def _is_cancel_requested(self) -> bool:
        with self._lock:
            return bool(self._cancel_requested)

    def _set_state(self, state: str, reason: str) -> None:
        with self._lock:
            old_state = self._state
            self._state = state
        if old_state != state:
            self.get_logger().info(
                "palette docking no camera state: %s -> %s (%s)"
                % (old_state, state, reason)
            )

    def _store_observation(self, observation: PalletObservation) -> None:
        with self._lock:
            self._last_observation = observation

    def _store_plan(self, plan: DockPlan) -> None:
        with self._lock:
            self._last_plan = plan

    def _status(self) -> Dict[str, Any]:
        with self._lock:
            active = self._active_thread is not None and self._active_thread.is_alive()
            state = self._state
            last_result = dict(self._last_result)
            failure_reason = self._failure_reason
            observation = self._last_observation
            plan = self._last_plan
        return {
            "active": active,
            "state": state,
            "result": last_result,
            "failure_reason": failure_reason,
            "target_frame": self._target_frame,
            "global_frame": self._global_frame,
            "robot_base_frame": self._robot_base_frame,
            "motion_mode": self._motion_mode,
            "last_observation": self._observation_status(observation),
            "last_plan": self._plan_status(plan),
        }

    def _observation_status(
        self, observation: Optional[PalletObservation]
    ) -> Optional[Dict[str, Any]]:
        if observation is None:
            return None
        return {
            "frame_id": observation.frame_id,
            "tag_x": observation.tag_x,
            "tag_y": observation.tag_y,
            "tag_z": observation.tag_z,
            "distance": observation.distance,
            "angle": observation.angle,
            "age_sec": observation.age_sec,
        }

    def _plan_status(self, plan: Optional[DockPlan]) -> Optional[Dict[str, Any]]:
        if plan is None:
            return None
        return {
            "tag_frame": plan.tag_frame,
            "pallet": {"x": plan.pallet_x, "y": plan.pallet_y},
            "prefinal": {
                "x": plan.prefinal_x,
                "y": plan.prefinal_y,
                "yaw": plan.prefinal_yaw,
            },
            "segment_yaw": plan.segment_yaw,
            "segment_distance": plan.segment_distance,
            "final_drive_distance": plan.final_drive_distance,
            "retreat_required": plan.retreat_required,
        }

    def _publish_visualization(
        self, robot_pose: RobotPose, plan: DockPlan, visual_path: Path
    ) -> None:
        self._path_publisher.publish(visual_path)
        markers = MarkerArray()
        markers.markers.append(self._delete_all_marker())
        markers.markers.append(
            self._sphere_marker(
                "palette_docking_no_camera_points",
                1,
                plan.pallet_x,
                plan.pallet_y,
                0.12,
                (0.95, 0.18, 0.14, 0.95),
                0.18,
            )
        )
        markers.markers.append(
            self._sphere_marker(
                "palette_docking_no_camera_points",
                2,
                plan.prefinal_x,
                plan.prefinal_y,
                0.12,
                (0.10, 0.72, 0.34, 0.95),
                0.18,
            )
        )
        markers.markers.append(
            self._sphere_marker(
                "palette_docking_no_camera_points",
                3,
                robot_pose.x,
                robot_pose.y,
                0.12,
                (0.18, 0.45, 0.95, 0.95),
                0.14,
            )
        )
        markers.markers.append(self._visual_path_marker(visual_path))
        markers.markers.append(
            self._arrow_marker(
                "palette_docking_no_camera_heading",
                10,
                plan.prefinal_x,
                plan.prefinal_y,
                plan.prefinal_yaw,
                0.65,
                (0.98, 0.77, 0.16, 0.95),
            )
        )
        markers.markers.append(
            self._label_marker(
                "palette_docking_no_camera_labels",
                20,
                plan.pallet_x,
                plan.pallet_y,
                0.42,
                "pallet tag",
            )
        )
        markers.markers.append(
            self._label_marker(
                "palette_docking_no_camera_labels",
                21,
                plan.prefinal_x,
                plan.prefinal_y,
                0.42,
                "prefinal",
            )
        )
        self._marker_publisher.publish(markers)

    def _publish_delete_all_markers(self) -> None:
        marker_array = MarkerArray()
        marker_array.markers.append(self._delete_all_marker())
        self._marker_publisher.publish(marker_array)

    def _delete_all_marker(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        return marker

    def _base_marker(self, namespace: str, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        if self._marker_lifetime > 0.0:
            self._set_duration(marker.lifetime, self._marker_lifetime)
        return marker

    def _sphere_marker(
        self,
        namespace: str,
        marker_id: int,
        x: float,
        y: float,
        z: float,
        color: Tuple[float, float, float, float],
        scale: float,
    ) -> Marker:
        marker = self._base_marker(namespace, marker_id)
        marker.type = Marker.SPHERE
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.scale.x = float(scale)
        marker.scale.y = float(scale)
        marker.scale.z = float(scale)
        self._set_color(marker, color)
        return marker

    def _visual_path_marker(self, path: Path) -> Marker:
        marker = self._base_marker("palette_docking_no_camera_path", 1)
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.045
        self._set_color(marker, (0.20, 0.86, 0.88, 0.95))
        for pose in path.poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = 0.05
            marker.points.append(point)
        return marker

    def _arrow_marker(
        self,
        namespace: str,
        marker_id: int,
        x: float,
        y: float,
        yaw: float,
        length: float,
        color: Tuple[float, float, float, float],
    ) -> Marker:
        marker = self._base_marker(namespace, marker_id)
        marker.type = Marker.ARROW
        marker.scale.x = 0.05
        marker.scale.y = 0.08
        marker.scale.z = 0.10
        start = Point()
        start.x = float(x)
        start.y = float(y)
        start.z = 0.16
        end = Point()
        end.x = float(x + math.cos(yaw) * length)
        end.y = float(y + math.sin(yaw) * length)
        end.z = 0.16
        marker.points.extend([start, end])
        self._set_color(marker, color)
        return marker

    def _label_marker(
        self, namespace: str, marker_id: int, x: float, y: float, z: float, text: str
    ) -> Marker:
        marker = self._base_marker(namespace, marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.scale.z = 0.22
        marker.text = text
        self._set_color(marker, (0.92, 0.96, 1.0, 0.95))
        return marker

    def _set_color(
        self, marker: Marker, color: Tuple[float, float, float, float]
    ) -> None:
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

    def _pose_stamped(
        self, x: float, y: float, yaw: float, header
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = self._quaternion_from_yaw(yaw)
        return pose

    def _base_point_to_global(
        self, robot_pose: RobotPose, base_x: float, base_y: float
    ) -> Tuple[float, float]:
        cos_yaw = math.cos(robot_pose.yaw)
        sin_yaw = math.sin(robot_pose.yaw)
        return (
            robot_pose.x + cos_yaw * base_x - sin_yaw * base_y,
            robot_pose.y + sin_yaw * base_x + cos_yaw * base_y,
        )

    def _drive_action_timeout(self, distance_m: float, speed_mps: float) -> float:
        if distance_m <= 0.0:
            return self._drive_timeout
        # In this stack the effective reverse DriveOnHeading progress is much
        # slower than the requested speed because steering, wheel conversion,
        # and collision filtering all reduce the realized motion. Use a
        # conservative envelope so we do not cancel a still-progressing dock.
        nominal = distance_m / max(0.01, speed_mps) * 4.0 + 5.0
        if self._drive_timeout <= 0.0:
            return nominal
        return max(self._drive_timeout, nominal)

    def _evaluate_final_dock_pose(self, plan: DockPlan) -> Dict[str, Any]:
        robot_pose = self._lookup_robot_pose()
        axis_x = plan.pallet_x - plan.prefinal_x
        axis_y = plan.pallet_y - plan.prefinal_y
        axis_length = math.hypot(axis_x, axis_y)
        if axis_length <= 1e-6:
            axis_x = math.cos(plan.segment_yaw)
            axis_y = math.sin(plan.segment_yaw)
            axis_length = math.hypot(axis_x, axis_y)
        axis_x /= axis_length
        axis_y /= axis_length

        dx = robot_pose.x - plan.pallet_x
        dy = robot_pose.y - plan.pallet_y
        longitudinal_error = dx * axis_x + dy * axis_y
        lateral_error = -dx * axis_y + dy * axis_x
        yaw_error = abs(self._normalize_angle(robot_pose.yaw - plan.prefinal_yaw))
        accepted = (
            abs(longitudinal_error) <= self._dock_longitudinal_tolerance
            and abs(lateral_error) <= self._dock_lateral_tolerance
            and yaw_error <= self._dock_pose_yaw_tolerance
        )
        return {
            "accepted": accepted,
            "robot_x": robot_pose.x,
            "robot_y": robot_pose.y,
            "robot_yaw": robot_pose.yaw,
            "longitudinal_error_m": longitudinal_error,
            "lateral_error_m": lateral_error,
            "yaw_error_rad": yaw_error,
        }

    def _evaluate_final_tag_tf(self, tag_frame: str) -> Dict[str, Any]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._dock_camera_frame,
                tag_frame,
                Time(),
                timeout=Duration(seconds=self._transform_timeout),
            )
        except TransformException as exc:
            raise ValueError(
                "final tag TF %s -> %s is not available: %s"
                % (self._dock_camera_frame, tag_frame, exc)
            ) from exc

        age_sec = self._transform_age_sec(transform)
        if self._tag_timeout > 0.0 and age_sec >= 0.0 and age_sec > self._tag_timeout:
            raise ValueError(
                "final tag TF is stale: frame=%s age=%.3fs limit=%.3fs"
                % (tag_frame, age_sec, self._tag_timeout)
            )

        translation = transform.transform.translation
        x = float(translation.x)
        y = float(translation.y)
        x_error = x - self._dock_target_x
        y_error = y - self._dock_target_y
        return {
            "accepted": (
                abs(x_error) <= self._dock_tf_x_tolerance
                and abs(y_error) <= self._dock_tf_y_tolerance
            ),
            "camera_frame": self._dock_camera_frame,
            "tag_frame": tag_frame,
            "x": x,
            "y": y,
            "z": float(translation.z),
            "x_target_m": self._dock_target_x,
            "y_target_m": self._dock_target_y,
            "x_error_m": x_error,
            "y_error_m": y_error,
            "x_tolerance_m": self._dock_tf_x_tolerance,
            "y_tolerance_m": self._dock_tf_y_tolerance,
            "age_sec": age_sec,
        }

    def _select_cmd_vel_source(self, source: str) -> None:
        if not self._orchestrator_client.wait_for_service(timeout_sec=3.0):
            raise ValueError("cmd_vel orchestrator service is not available")
        request = StringWithJson.Request()
        request.message = json.dumps(
            {"command": "select_source", "source": str(source)}
        )
        future = self._orchestrator_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise ValueError("cmd_vel orchestrator request timed out")
        response = future.result()
        if response is None or not response.success:
            details = response.message if response is not None else "no response"
            raise ValueError("cmd_vel orchestrator rejected source: %s" % details)
        self.get_logger().info("cmd_vel source selected: %s" % source)

    @staticmethod
    def _rotate_vector_by_quaternion(
        vector: Tuple[float, float, float],
        quaternion: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float]:
        vx, vy, vz = vector
        qx, qy, qz, qw = quaternion
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return (
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        )

    def _anchor_tag_in_global(self, current: Dict[str, Any]) -> Tuple[float, float, float]:
        try:
            camera_pose = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._dock_camera_frame,
                Time(),
                timeout=Duration(seconds=self._lookup_timeout),
            )
        except TransformException as exc:
            raise ValueError("camera pose is not available for tag anchoring: %s" % exc) from exc
        translation = camera_pose.transform.translation
        rotation = camera_pose.transform.rotation
        offset = self._rotate_vector_by_quaternion(
            (float(current["x"]), float(current["y"]), float(current["z"])),
            (float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)),
        )
        return (
            float(translation.x) + offset[0],
            float(translation.y) + offset[1],
            float(translation.z) + offset[2],
        )

    def _evaluate_anchored_tag(
        self,
        tag_frame: str,
        anchor: Tuple[float, float, float],
        anchor_age_sec: float,
    ) -> Dict[str, Any]:
        try:
            camera_pose = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._dock_camera_frame,
                Time(),
                timeout=Duration(seconds=self._lookup_timeout),
            )
        except TransformException as exc:
            raise ValueError("camera pose is not available for anchored docking: %s" % exc) from exc
        translation = camera_pose.transform.translation
        rotation = camera_pose.transform.rotation
        global_delta = (
            anchor[0] - float(translation.x),
            anchor[1] - float(translation.y),
            anchor[2] - float(translation.z),
        )
        camera_delta = self._rotate_vector_by_quaternion(
            global_delta,
            (-float(rotation.x), -float(rotation.y), -float(rotation.z), float(rotation.w)),
        )
        x, y, z = camera_delta
        x_error = x - self._dock_target_x
        y_error = y - self._dock_target_y
        return {
            "accepted": (
                abs(x_error) <= self._dock_tf_x_tolerance
                and abs(y_error) <= self._dock_tf_y_tolerance
            ),
            "camera_frame": self._dock_camera_frame,
            "tag_frame": tag_frame,
            "x": x,
            "y": y,
            "z": z,
            "x_target_m": self._dock_target_x,
            "y_target_m": self._dock_target_y,
            "x_error_m": x_error,
            "y_error_m": y_error,
            "x_tolerance_m": self._dock_tf_x_tolerance,
            "y_tolerance_m": self._dock_tf_y_tolerance,
            "age_sec": anchor_age_sec,
            "measurement_source": "map_anchored_tag",
        }

    def _drive_to_final_tag_tf(self, tag_frame: str) -> Dict[str, Any]:
        deadline = (
            time.monotonic() + self._final_dock_timeout
            if self._final_dock_timeout > 0.0
            else None
        )
        period = 1.0 / self._final_dock_tf_rate
        last_log_at = 0.0
        last_eval: Optional[Dict[str, Any]] = None
        tf_lookup_count = 0
        anchored_control_cycles = 0
        tag_anchor: Optional[Tuple[float, float, float]] = None
        tag_anchor_at = 0.0
        use_tag_anchor = False
        alignment_start_x: Optional[float] = None
        last_control_mode = "starting"
        last_cmd_linear = 0.0
        last_cmd_angular = 0.0
        last_alignment_retreat = 0.0
        approach_progress_x: Optional[float] = None
        approach_progress_at = 0.0
        try:
            while rclpy.ok():
                self._check_cancelled()
                now = time.monotonic()
                if not use_tag_anchor and tf_lookup_count >= self._final_dock_max_tf_lookups:
                    raise ValueError(
                        "final camera/tag TF lookup limit reached: %d"
                        % tf_lookup_count
                    )
                try:
                    if use_tag_anchor:
                        anchored_control_cycles += 1
                        current = self._evaluate_anchored_tag(
                            tag_frame,
                            tag_anchor,
                            max(0.0, now - tag_anchor_at),
                        )
                    else:
                        tf_lookup_count += 1
                        current = self._evaluate_final_tag_tf(tag_frame)
                        current["measurement_source"] = "live_apriltag_tf"
                        tag_anchor = self._anchor_tag_in_global(current)
                        tag_anchor_at = now
                    current["tf_lookup_count"] = tf_lookup_count
                    current["anchored_control_cycles"] = anchored_control_cycles
                    last_eval = current
                except ValueError as exc:
                    if not use_tag_anchor and tag_anchor is not None:
                        use_tag_anchor = True
                        self.get_logger().warn(
                            "live AprilTag TF unavailable; continuing from map anchor "
                            "after %d tag lookups: %s" % (tf_lookup_count, exc)
                        )
                        continue
                    self._publish_stop()
                    if deadline is not None and now > deadline:
                        raise
                    time.sleep(period)
                    continue

                if current["accepted"]:
                    self.get_logger().info(
                        "final camera/tag accepted: frame=%s x=%.3f y=%.3f "
                        "source=%s lookups=%d anchored_cycles=%d"
                        % (
                            tag_frame,
                            current["x"],
                            current["y"],
                            current.get("measurement_source", "unknown"),
                            tf_lookup_count,
                            anchored_control_cycles,
                        )
                    )
                    return current

                if deadline is not None and now > deadline:
                    raise ValueError(
                        "final camera/tag target timed out: x=%.3f target=%.3f "
                        "y=%.3f target=%.3f lookups=%d"
                        % (
                            current["x"],
                            self._dock_target_x,
                            current["y"],
                            self._dock_target_y,
                            tf_lookup_count,
                        )
                    )

                twist = Twist()
                x_error = float(current["x_error_m"])
                y_error = float(current["y_error_m"])
                if abs(y_error) > self._final_dock_abort_lateral_error:
                    raise ValueError(
                        "final camera/tag lateral error diverged: y=%.3f "
                        "target=%.3f limit=%.3f lookups=%d"
                        % (
                            current["y"],
                            self._dock_target_y,
                            self._final_dock_abort_lateral_error,
                            tf_lookup_count,
                        )
                    )

                control_mode = "stop"
                alignment_retreat = 0.0
                if abs(y_error) > self._dock_tf_y_tolerance:
                    approach_progress_x = None
                    # Correct lateral error with a bounded steering sweep while
                    # moving away from the pallet. The schedule is tied to the
                    # measured retreat distance, so a bad steering state cannot
                    # make the robot keep driving away without limit.
                    if alignment_start_x is None:
                        alignment_start_x = float(current["x"])
                    alignment_retreat = float(current["x"]) - alignment_start_x
                    if alignment_retreat > self._final_dock_max_alignment_retreat:
                        raise ValueError(
                            "lateral alignment retreat limit reached: y=%.3f "
                            "retreat=%.3f limit=%.3f"
                            % (
                                current["y"],
                                alignment_retreat,
                                self._final_dock_max_alignment_retreat,
                            )
                        )
                    twist.linear.x = self._final_dock_alignment_speed
                    twist.angular.z = self._alignment_sweep_angular(
                        y_error,
                        max(0.0, alignment_retreat),
                    )
                    control_mode = "align_sweep"
                elif abs(x_error) > self._dock_tf_x_tolerance:
                    alignment_start_x = None
                    if approach_progress_x is None:
                        approach_progress_x = float(current["x"])
                        approach_progress_at = now
                    elif float(current["x"]) < approach_progress_x - 0.03:
                        approach_progress_x = float(current["x"])
                        approach_progress_at = now
                    speed = self._final_drive_speed
                    if abs(x_error) < 0.30:
                        speed = max(0.04, speed * 0.5)
                    twist.linear.x = self._motion_sign * math.copysign(speed, x_error)
                    angular = self._final_dock_lateral_gain * y_error
                    twist.angular.z = max(
                        -self._final_dock_max_angular,
                        min(self._final_dock_max_angular, angular),
                    )
                    control_mode = "approach"
                    if now - approach_progress_at > 4.0:
                        wiggle = min(self._final_dock_max_angular, 0.04)
                        phase = int((now - approach_progress_at) / 2.0) % 2
                        direction = -1.0 if phase == 0 else 1.0
                        if abs(y_error) > 0.5 * self._dock_tf_y_tolerance:
                            direction = 1.0 if y_error > 0.0 else -1.0
                        twist.angular.z = direction * wiggle
                        control_mode = "approach_wiggle"
                last_control_mode = control_mode
                last_cmd_linear = float(twist.linear.x)
                last_cmd_angular = float(twist.angular.z)
                last_alignment_retreat = alignment_retreat
                if now - last_log_at >= 1.0:
                    self.get_logger().info(
                        "final camera/tag progress: frame=%s x=%.3f y=%.3f "
                        "x_error=%.3f y_error=%.3f age=%.3f source=%s "
                        "mode=%s cmd_x=%.3f cmd_w=%.3f align_retreat=%.3f "
                        "lookups=%d anchored_cycles=%d"
                        % (
                            tag_frame,
                            current["x"],
                            current["y"],
                            current["x_error_m"],
                            current["y_error_m"],
                            current["age_sec"],
                            current.get("measurement_source", "unknown"),
                            last_control_mode,
                            last_cmd_linear,
                            last_cmd_angular,
                            last_alignment_retreat,
                            tf_lookup_count,
                            anchored_control_cycles,
                        )
                    )
                    last_log_at = now
                self._cmd_vel_publisher.publish(twist)
                time.sleep(period)
        finally:
            self._publish_stop()

        if last_eval is not None:
            return last_eval
        raise DockingCancelled()

    def _alignment_sweep_angular(self, y_error: float, retreat_m: float) -> float:
        if self._final_dock_max_angular <= 0.0:
            return 0.0
        sign = 1.0 if y_error > 0.0 else -1.0
        stages = (
            (0.52, 1.00),
            (0.40, 0.75),
            (0.28, 0.50),
            (0.16, 0.25),
            (0.08, 0.00),
            (0.00, -0.25),
        )
        multiplier = stages[-1][1]
        for threshold, value in stages:
            if retreat_m >= threshold:
                multiplier = value
                break
        return sign * multiplier * self._final_dock_max_angular

    def _publish_stop(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    def _transform_age_sec(self, transform: TransformStamped) -> float:
        now = self.get_clock().now()
        if now.nanoseconds <= 0:
            return -1.0
        stamp = Time.from_msg(transform.header.stamp)
        return max(0.0, (now.nanoseconds - stamp.nanoseconds) / 1e9)

    def _decode_payload(self, message: str) -> Dict[str, Any]:
        try:
            payload = json.loads(message or "{}")
        except json.JSONDecodeError as exc:
            raise ValueError(str(exc)) from exc
        if not isinstance(payload, dict):
            raise ValueError("request message must be a JSON object")
        return payload

    def _payload_float(
        self, payload: Dict[str, Any], key: str, default: float
    ) -> float:
        if key not in payload:
            return default
        return float(payload[key])

    def _param_float(self, name: str, *, minimum: Optional[float] = None) -> float:
        value = float(self.get_parameter(name).value)
        if minimum is not None:
            value = max(minimum, value)
        return value

    def _set_duration(self, duration_msg, seconds: float) -> None:
        seconds = max(0.0, float(seconds))
        whole_seconds = int(seconds)
        duration_msg.sec = whole_seconds
        duration_msg.nanosec = int((seconds - whole_seconds) * 1e9)

    def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _quaternion_from_yaw(self, yaw: float) -> Tuple[float, float, float, float]:
        half_yaw = yaw * 0.5
        return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PaletteDockingNoCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._request_cancel("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
