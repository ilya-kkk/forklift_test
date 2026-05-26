import json
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from forklift_interfaces.srv import StringWithJson
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


IDLE = "IDLE"
WAIT_FOR_TAG = "WAIT_FOR_TAG"
COMPENSATE_Y = "COMPENSATE_Y"
ALIGN_TO_TAG = "ALIGN_TO_TAG"
APPROACH_STANDOFF = "APPROACH_STANDOFF"
DOCK_UNDER_PALLET = "DOCK_UNDER_PALLET"
BACK_OUT = "BACK_OUT"
DONE = "DONE"
FAILED = "FAILED"


@dataclass(frozen=True)
class PalletPose:
    frame_id: str
    x: float
    y: float
    z: float
    angle: float
    age_sec: float


class PalletDockingController(Node):
    def __init__(self) -> None:
        super().__init__("pallet_docking_controller")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("service_name", "/palette_docking/control")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tag_frame", "")
        self.declare_parameter("tag_frame_candidates", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("start_enabled", False)
        self.declare_parameter("lookup_timeout_sec", 0.02)
        self.declare_parameter("tag_timeout_sec", 0.5)
        self.declare_parameter("wait_for_tag_timeout_sec", 5.0)
        self.declare_parameter("compensate_y_timeout_sec", 10.0)
        self.declare_parameter("align_timeout_sec", 8.0)
        self.declare_parameter("approach_timeout_sec", 15.0)
        self.declare_parameter("dock_timeout_sec", 12.0)
        self.declare_parameter("back_out_timeout_sec", 6.0)
        self.declare_parameter("standoff_distance_m", 1.0)
        self.declare_parameter("approach_extra_drive_m", 0.6)
        self.declare_parameter("final_drive_distance_m", 1.0)
        self.declare_parameter("back_out_distance_m", 0.45)
        self.declare_parameter("y_tolerance_m", 0.05)
        self.declare_parameter("y_reacquire_tolerance_m", 0.12)
        self.declare_parameter("angle_tolerance_rad", 0.04)
        self.declare_parameter("x_tolerance_m", 0.05)
        self.declare_parameter("max_turn_angle_rad", 0.75)
        self.declare_parameter("lateral_linear_speed_mps", 0.12)
        self.declare_parameter("align_linear_speed_mps", 0.0)
        self.declare_parameter("approach_max_linear_speed_mps", 0.25)
        self.declare_parameter("approach_min_linear_speed_mps", 0.06)
        self.declare_parameter("dock_linear_speed_mps", 0.16)
        self.declare_parameter("back_out_linear_speed_mps", -0.12)
        self.declare_parameter("max_angular_speed_radps", 0.45)
        self.declare_parameter("y_angular_gain", 0.8)
        self.declare_parameter("angle_angular_gain", 1.2)
        self.declare_parameter("approach_x_gain", 0.5)
        self.declare_parameter("approach_y_angular_gain", 0.35)
        self.declare_parameter("approach_angle_angular_gain", 0.8)
        self.declare_parameter("dock_y_angular_gain", 0.0)
        self.declare_parameter("dock_angle_angular_gain", 0.0)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        service_name = str(self.get_parameter("service_name").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._configured_tag_frame = str(self.get_parameter("tag_frame").value).strip()
        self._tag_frame_candidates = [
            str(frame)
            for frame in self.get_parameter("tag_frame_candidates").value
            if str(frame).strip()
        ]
        self._lookup_timeout = max(
            0.0, float(self.get_parameter("lookup_timeout_sec").value)
        )
        self._tag_timeout = max(
            0.0, float(self.get_parameter("tag_timeout_sec").value)
        )
        self._wait_for_tag_timeout = max(
            0.0, float(self.get_parameter("wait_for_tag_timeout_sec").value)
        )
        self._state_timeouts = {
            COMPENSATE_Y: max(
                0.0, float(self.get_parameter("compensate_y_timeout_sec").value)
            ),
            ALIGN_TO_TAG: max(0.0, float(self.get_parameter("align_timeout_sec").value)),
            APPROACH_STANDOFF: max(
                0.0, float(self.get_parameter("approach_timeout_sec").value)
            ),
            DOCK_UNDER_PALLET: max(
                0.0, float(self.get_parameter("dock_timeout_sec").value)
            ),
            BACK_OUT: max(0.0, float(self.get_parameter("back_out_timeout_sec").value)),
        }

        self._default_standoff_distance = max(
            0.0, float(self.get_parameter("standoff_distance_m").value)
        )
        self._default_approach_extra_drive = max(
            0.0, float(self.get_parameter("approach_extra_drive_m").value)
        )
        self._default_final_drive_distance = max(
            0.0, float(self.get_parameter("final_drive_distance_m").value)
        )
        self._default_max_turn_angle = max(
            0.0, float(self.get_parameter("max_turn_angle_rad").value)
        )
        self._standoff_distance = self._default_standoff_distance
        self._approach_extra_drive = self._default_approach_extra_drive
        self._final_drive_distance = self._default_final_drive_distance
        self._max_turn_angle = self._default_max_turn_angle
        self._active_tag_frame = self._configured_tag_frame

        self._back_out_distance = max(
            0.0, float(self.get_parameter("back_out_distance_m").value)
        )
        self._y_tolerance = max(0.0, float(self.get_parameter("y_tolerance_m").value))
        self._y_reacquire_tolerance = max(
            self._y_tolerance,
            float(self.get_parameter("y_reacquire_tolerance_m").value),
        )
        self._angle_tolerance = max(
            0.0, float(self.get_parameter("angle_tolerance_rad").value)
        )
        self._x_tolerance = max(0.0, float(self.get_parameter("x_tolerance_m").value))

        self._lateral_linear_speed = abs(
            float(self.get_parameter("lateral_linear_speed_mps").value)
        )
        self._align_linear_speed = float(
            self.get_parameter("align_linear_speed_mps").value
        )
        self._approach_max_linear_speed = abs(
            float(self.get_parameter("approach_max_linear_speed_mps").value)
        )
        self._approach_min_linear_speed = min(
            self._approach_max_linear_speed,
            abs(float(self.get_parameter("approach_min_linear_speed_mps").value)),
        )
        self._dock_linear_speed = abs(
            float(self.get_parameter("dock_linear_speed_mps").value)
        )
        # In rear-docking mode forward approach is commanded with negative linear.x,
        # so back-out must use positive linear.x to move away from the pallet.
        self._back_out_linear_speed = abs(
            float(self.get_parameter("back_out_linear_speed_mps").value)
        )
        self._max_angular_speed = abs(
            float(self.get_parameter("max_angular_speed_radps").value)
        )
        self._y_angular_gain = float(self.get_parameter("y_angular_gain").value)
        self._angle_angular_gain = float(
            self.get_parameter("angle_angular_gain").value
        )
        self._approach_x_gain = float(self.get_parameter("approach_x_gain").value)
        self._approach_y_angular_gain = float(
            self.get_parameter("approach_y_angular_gain").value
        )
        self._approach_angle_angular_gain = float(
            self.get_parameter("approach_angle_angular_gain").value
        )
        self._dock_y_angular_gain = float(
            self.get_parameter("dock_y_angular_gain").value
        )
        self._dock_angle_angular_gain = float(
            self.get_parameter("dock_angle_angular_gain").value
        )

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_service(StringWithJson, service_name, self._handle_control)

        self._state = IDLE
        self._state_started_at = self.get_clock().now()
        self._last_tick_at = self._state_started_at
        self._final_drive_traveled = 0.0
        self._back_out_traveled = 0.0
        self._failure_reason = ""
        self._last_pose: Optional[PalletPose] = None

        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.create_timer(1.0 / rate, self._control_tick)

        if bool(self.get_parameter("start_enabled").value):
            self._start({})

        self.get_logger().info(
            "pallet_docking_controller ready: cmd_vel=%s service=%s target_frame=%s "
            "tag_frame=%s candidates=%d state=%s"
            % (
                cmd_vel_topic,
                service_name,
                self._target_frame,
                self._configured_tag_frame or "<auto>",
                len(self._tag_frame_candidates),
                self._state,
            )
        )

    def _handle_control(self, request, response):
        try:
            payload = json.loads(request.message or "{}")
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = json.dumps({"error": "invalid_json", "details": str(exc)})
            return response

        command = str(payload.get("command", "")).strip().lower()
        if "enabled" in payload:
            command = "start" if bool(payload["enabled"]) else "stop"
        if command in {"start", "enable", "dock"}:
            self._start(payload)
            response.success = True
        elif command in {"stop", "disable", "cancel"}:
            self._set_state(IDLE, "control stop")
            response.success = True
        elif command in {"", "status"}:
            response.success = True
        elif command == "reset":
            self._failure_reason = ""
            self._set_state(IDLE, "control reset")
            response.success = True
        else:
            response.success = False
            response.message = json.dumps(
                {
                    "error": "invalid_command",
                    "details": "expected start|stop|status|reset or enabled bool",
                }
            )
            return response

        response.message = json.dumps(self._status())
        return response

    def _start(self, payload: dict) -> None:
        self._standoff_distance = _payload_float(
            payload, "standoff_distance_m", self._default_standoff_distance
        )
        self._approach_extra_drive = max(
            0.0,
            _payload_float(
                payload,
                "approach_extra_drive_m",
                self._default_approach_extra_drive,
            ),
        )
        self._final_drive_distance = _payload_float(
            payload, "final_drive_distance_m", self._default_final_drive_distance
        )
        self._max_turn_angle = _payload_float(
            payload, "max_turn_angle_rad", self._default_max_turn_angle
        )
        if "max_turn_angle_deg" in payload:
            self._max_turn_angle = math.radians(float(payload["max_turn_angle_deg"]))
        self._active_tag_frame = str(
            payload.get("tag_frame", self._configured_tag_frame)
        ).strip()
        self._failure_reason = ""
        self._last_pose = None
        self._set_state(WAIT_FOR_TAG, "control start")

    def _control_tick(self) -> None:
        now = self.get_clock().now()
        dt = self._seconds_since(self._last_tick_at, now)
        self._last_tick_at = now

        if self._state in {IDLE, DONE, FAILED}:
            return

        if self._state == BACK_OUT:
            self._run_back_out(dt)
            return

        pose = self._lookup_pallet_pose()
        if pose is None:
            self._publish_stop()
            if self._state == WAIT_FOR_TAG:
                if (
                    self._wait_for_tag_timeout > 0.0
                    and self._state_elapsed(now) > self._wait_for_tag_timeout
                ):
                    self._fail("no valid tag transform")
                return
            if self._state == DOCK_UNDER_PALLET:
                self._run_final_dock(dt, None)
                return
            if self._state_timed_out(now):
                self._begin_back_out("tag transform lost in %s" % self._state)
            return

        self._last_pose = pose
        if self._state == WAIT_FOR_TAG:
            self._set_state(COMPENSATE_Y, "tag acquired: %s" % pose.frame_id)
            self._publish_stop()
            return

        if self._state_timed_out(now):
            self._begin_back_out("state timeout in %s" % self._state)
            return

        if self._state != DOCK_UNDER_PALLET and not self._turn_is_allowed(pose):
            return

        if self._state == COMPENSATE_Y:
            self._run_compensate_y(pose)
        elif self._state == ALIGN_TO_TAG:
            self._run_align_to_tag(pose)
        elif self._state == APPROACH_STANDOFF:
            self._run_approach_standoff(pose)
        elif self._state == DOCK_UNDER_PALLET:
            self._run_final_dock(dt, pose)

    def _run_compensate_y(self, pose: PalletPose) -> None:
        if abs(pose.y) <= self._y_tolerance:
            self._set_state(ALIGN_TO_TAG, "y compensated")
            self._publish_stop()
            return

        angular = self._clamp_angular(self._y_angular_gain * pose.y)
        self._publish_twist(self._toward_pallet_linear(self._lateral_linear_speed), angular)

    def _run_align_to_tag(self, pose: PalletPose) -> None:
        if abs(pose.angle) <= self._angle_tolerance:
            self._set_state(APPROACH_STANDOFF, "angle aligned")
            self._publish_stop()
            return

        angular = self._clamp_angular(self._angle_angular_gain * pose.angle)
        self._publish_twist(self._toward_pallet_linear(self._align_linear_speed), angular)

    def _run_approach_standoff(self, pose: PalletPose) -> None:
        if abs(pose.y) > self._y_reacquire_tolerance:
            self._set_state(COMPENSATE_Y, "y drifted during approach")
            self._publish_stop()
            return

        target_x = self._standoff_distance - self._approach_extra_drive
        x_error = pose.x - target_x
        if x_error <= self._x_tolerance:
            self._set_state(DOCK_UNDER_PALLET, "standoff reached")
            self._publish_stop()
            return

        linear = self._approach_x_gain * x_error
        linear = max(self._approach_min_linear_speed, min(self._approach_max_linear_speed, linear))
        angular = self._clamp_angular(
            self._approach_y_angular_gain * pose.y
            + self._approach_angle_angular_gain * pose.angle
        )
        self._publish_twist(self._toward_pallet_linear(linear), angular)

    def _run_final_dock(self, dt: float, pose: Optional[PalletPose]) -> None:
        if self._final_drive_traveled >= self._final_drive_distance:
            self._set_state(DONE, "final drive distance reached")
            return

        angular = 0.0
        if pose is not None:
            angular = self._clamp_angular(
                self._dock_y_angular_gain * pose.y
                + self._dock_angle_angular_gain * pose.angle
            )
        linear = self._toward_pallet_linear(self._dock_linear_speed)
        self._final_drive_traveled += abs(linear) * max(0.0, dt)
        self._publish_twist(linear, angular)

    def _run_back_out(self, dt: float) -> None:
        if self._state_timed_out(self.get_clock().now()):
            self._fail("back_out timeout: %s" % self._failure_reason)
            return
        if self._back_out_traveled >= self._back_out_distance:
            self._fail(self._failure_reason or "back_out complete")
            return

        self._back_out_traveled += abs(self._back_out_linear_speed) * max(0.0, dt)
        self._publish_twist(self._back_out_linear_speed, 0.0)

    def _lookup_pallet_pose(self) -> Optional[PalletPose]:
        frames = self._candidate_frames()
        if not frames:
            return None

        poses = []
        for frame_id in frames:
            transform = self._lookup_transform(frame_id)
            if transform is None:
                continue
            pose = self._pose_from_transform(frame_id, transform)
            if pose is not None:
                poses.append(pose)

        if not poses:
            return None
        return min(poses, key=lambda pose: abs(pose.angle))

    def _lookup_transform(self, source_frame: str) -> Optional[TransformStamped]:
        try:
            if self._active_tag_frame and self._lookup_timeout > 0.0:
                return self._buffer.lookup_transform(
                    self._target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self._lookup_timeout),
                )
            return self._buffer.lookup_transform(
                self._target_frame,
                source_frame,
                Time(),
            )
        except TransformException:
            return None

    def _pose_from_transform(
        self, frame_id: str, transform: TransformStamped
    ) -> Optional[PalletPose]:
        now = self.get_clock().now()
        stamp = Time.from_msg(transform.header.stamp)
        age_sec = self._seconds_since(stamp, now)
        if self._tag_timeout > 0.0 and age_sec > self._tag_timeout:
            return None

        translation = transform.transform.translation
        # Rear-docking mode: interpret pallet pose in a frame rotated by pi
        # relative to base_link, so tags behind the robot become "forward".
        x = -float(translation.x)
        y = -float(translation.y)
        z = float(translation.z)
        return PalletPose(
            frame_id=frame_id,
            x=x,
            y=y,
            z=z,
            angle=math.atan2(y, x),
            age_sec=age_sec,
        )

    def _candidate_frames(self) -> list[str]:
        if self._active_tag_frame:
            return [self._active_tag_frame]
        return self._tag_frame_candidates

    def _turn_is_allowed(self, pose: PalletPose) -> bool:
        if abs(pose.angle) <= self._max_turn_angle:
            return True
        self._begin_back_out(
            "required angle %.3f rad exceeds max_turn_angle %.3f rad"
            % (pose.angle, self._max_turn_angle)
        )
        return False

    def _begin_back_out(self, reason: str) -> None:
        self._failure_reason = reason
        self._set_state(BACK_OUT, reason)

    def _fail(self, reason: str) -> None:
        self._failure_reason = reason
        self._set_state(FAILED, reason)

    def _set_state(self, state: str, reason: str) -> None:
        if state == self._state and state not in {WAIT_FOR_TAG, BACK_OUT}:
            return
        self.get_logger().info(
            "palette docking state: %s -> %s (%s)" % (self._state, state, reason)
        )
        self._state = state
        self._state_started_at = self.get_clock().now()
        if state == DOCK_UNDER_PALLET:
            self._final_drive_traveled = 0.0
        if state == BACK_OUT:
            self._back_out_traveled = 0.0
        if state in {IDLE, DONE, FAILED}:
            self._publish_stop()

    def _state_elapsed(self, now: Time) -> float:
        return self._seconds_since(self._state_started_at, now)

    def _state_timed_out(self, now: Time) -> bool:
        timeout = self._state_timeouts.get(self._state, 0.0)
        return timeout > 0.0 and self._state_elapsed(now) > timeout

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = self._clamp_angular(angular_z)
        self._cmd_vel_publisher.publish(twist)

    def _publish_stop(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    def _toward_pallet_linear(self, value: float) -> float:
        return -abs(float(value))

    def _clamp_angular(self, value: float) -> float:
        return max(-self._max_angular_speed, min(self._max_angular_speed, value))

    def _seconds_since(self, earlier: Time, later: Time) -> float:
        seconds = (later - earlier).nanoseconds / 1_000_000_000.0
        if seconds <= 0.0:
            return 0.0
        return seconds

    def _status(self) -> dict:
        pose = None
        if self._last_pose is not None:
            pose = {
                "frame_id": self._last_pose.frame_id,
                "x": self._last_pose.x,
                "y": self._last_pose.y,
                "z": self._last_pose.z,
                "angle": self._last_pose.angle,
                "age_sec": self._last_pose.age_sec,
            }
        return {
            "state": self._state,
            "target_frame": self._target_frame,
            "tag_frame": self._active_tag_frame or "",
            "standoff_distance_m": self._standoff_distance,
            "final_drive_distance_m": self._final_drive_distance,
            "max_turn_angle_rad": self._max_turn_angle,
            "final_drive_traveled_m": self._final_drive_traveled,
            "back_out_traveled_m": self._back_out_traveled,
            "failure_reason": self._failure_reason,
            "last_pose": pose,
        }


def _payload_float(payload: dict, key: str, default: float) -> float:
    if key not in payload:
        return default
    return float(payload[key])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PalletDockingController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
