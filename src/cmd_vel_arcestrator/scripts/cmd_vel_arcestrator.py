#!/usr/bin/env python3
import json
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from forklift_interfaces.srv import StringWithJson
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String


SOURCES = ("pallet_docking", "charging_docking", "manual", "navigation")
SOURCE_ALIASES = {
    "pallet": "pallet_docking",
    "palette_docking": "pallet_docking",
    "pallet_docking": "pallet_docking",
    "charging": "charging_docking",
    "charging_docking": "charging_docking",
    "manual": "manual",
    "nav": "navigation",
    "navigation": "navigation",
}


@dataclass
class SourceState:
    topic: str
    last_msg: Optional[Twist] = None
    last_seen_at: Optional[Time] = None


class CmdVelArcestrator(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_arcestrator")

        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("control_service", "/cmd_vel_arcestrator/control")
        self.declare_parameter("status_topic", "/cmd_vel_arcestrator/status")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("source_timeout_sec", 0.25)
        self.declare_parameter("status_publish_period_sec", 1.0)
        self.declare_parameter("default_source", "")
        self.declare_parameter("max_linear_speed_mps", 1.0)
        self.declare_parameter("max_angular_speed_radps", 0.8)
        self.declare_parameter("pallet_docking_topic", "/cmd_vel_pallet_docking")
        self.declare_parameter("charging_docking_topic", "/cmd_vel_charging_docking")
        self.declare_parameter("manual_topic", "/cmd_vel_manual")
        self.declare_parameter("navigation_topic", "/cmd_vel_nav")

        output_topic = str(self.get_parameter("output_topic").value)
        control_service = str(self.get_parameter("control_service").value)
        status_topic = str(self.get_parameter("status_topic").value)
        publish_rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self._source_timeout = max(
            0.0, float(self.get_parameter("source_timeout_sec").value)
        )
        self._status_period = max(
            0.0, float(self.get_parameter("status_publish_period_sec").value)
        )
        self._default_max_linear_speed = self._non_negative_parameter(
            "max_linear_speed_mps"
        )
        self._default_max_angular_speed = self._non_negative_parameter(
            "max_angular_speed_radps"
        )
        self._max_linear_speed = self._default_max_linear_speed
        self._max_angular_speed = self._default_max_angular_speed

        self._sources = {
            "pallet_docking": SourceState(
                str(self.get_parameter("pallet_docking_topic").value)
            ),
            "charging_docking": SourceState(
                str(self.get_parameter("charging_docking_topic").value)
            ),
            "manual": SourceState(str(self.get_parameter("manual_topic").value)),
            "navigation": SourceState(
                str(self.get_parameter("navigation_topic").value)
            ),
        }
        default_source = self._default_source_from_parameter()
        self._selected_source: Optional[str] = default_source
        self._stopped = default_source is None
        self._last_output_source = "stop"
        self._last_output_limited = False
        self._last_status_at: Optional[Time] = None

        self._cmd_vel_pub = self.create_publisher(Twist, output_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_service(StringWithJson, control_service, self._handle_control)

        for source_name, source in self._sources.items():
            self.create_subscription(
                Twist,
                source.topic,
                self._make_source_callback(source_name),
                10,
            )

        self.create_timer(1.0 / publish_rate, self._control_tick)

        self.get_logger().info(
            "cmd_vel_arcestrator ready: output=%s control=%s status=%s "
            "sources=%s timeout=%.3fs default_source=%s stopped=%s "
            "max_linear=%.3fm/s max_angular=%.3frad/s"
            % (
                output_topic,
                control_service,
                status_topic,
                ", ".join(
                    "%s:%s" % (name, state.topic)
                    for name, state in self._sources.items()
                ),
                self._source_timeout,
                self._selected_source or "<none>",
                self._stopped,
                self._max_linear_speed,
                self._max_angular_speed,
            )
        )

    def _make_source_callback(self, source_name: str):
        def callback(msg: Twist) -> None:
            source = self._sources[source_name]
            source.last_msg = msg
            source.last_seen_at = self.get_clock().now()

        return callback

    def _handle_control(self, request, response):
        try:
            payload = json.loads(request.message or "{}")
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = json.dumps(
                {"error": "invalid_json", "details": str(exc)}
            )
            return response

        command = str(payload.get("command", "status")).strip().lower()
        if command in {"", "status"}:
            response.success = True
        elif command in {"select", "select_source", "set_source"}:
            source = self._normalize_source(payload.get("source"))
            if source is None:
                response.success = False
                response.message = json.dumps(
                    {
                        "error": "invalid_source",
                        "details": "expected one of: %s" % ", ".join(SOURCES),
                    }
                )
                return response
            self._selected_source = source
            self._stopped = False
            self._publish_stop()
            response.success = True
            self.get_logger().info("cmd_vel source selected: %s" % source)
        elif command in {"stop", "disable", "emergency_stop", "estop"}:
            self._stopped = True
            self._publish_stop()
            response.success = True
            self.get_logger().warn(
                "cmd_vel arcestrator stopped; selected_source=%s"
                % (self._selected_source or "<none>")
            )
        elif command in {"release", "enable"}:
            if self._selected_source is None:
                response.success = False
                response.message = json.dumps(
                    {
                        "error": "no_source_selected",
                        "details": "select a source before release",
                    }
                )
                return response
            self._stopped = False
            self._publish_stop()
            response.success = True
            self.get_logger().info(
                "cmd_vel arcestrator released: source=%s" % self._selected_source
            )
        elif command in {"set_limits", "set_speed_limits", "set_speed_limit"}:
            try:
                self._update_speed_limits(payload)
            except ValueError as exc:
                response.success = False
                response.message = json.dumps(
                    {"error": "invalid_limits", "details": str(exc)}
                )
                return response
            response.success = True
            self.get_logger().info(
                "cmd_vel speed limits updated: max_linear=%.3fm/s "
                "max_angular=%.3frad/s"
                % (self._max_linear_speed, self._max_angular_speed)
            )
        elif command in {"reset_limits", "reset_speed_limits"}:
            self._max_linear_speed = self._default_max_linear_speed
            self._max_angular_speed = self._default_max_angular_speed
            response.success = True
            self.get_logger().info(
                "cmd_vel speed limits reset: max_linear=%.3fm/s "
                "max_angular=%.3frad/s"
                % (self._max_linear_speed, self._max_angular_speed)
            )
        else:
            response.success = False
            response.message = json.dumps(
                {
                    "error": "invalid_command",
                    "details": (
                        "expected select_source|stop|release|status|"
                        "set_limits|reset_limits"
                    ),
                }
            )
            return response

        response.message = json.dumps(self._status())
        self._publish_status(force=True)
        return response

    def _control_tick(self) -> None:
        if self._stopped or self._selected_source is None:
            self._last_output_source = "stop"
            self._last_output_limited = False
            self._publish_stop()
            self._publish_status()
            return

        source = self._sources[self._selected_source]
        if source.last_msg is None or self._source_is_stale(source):
            self._last_output_source = "stale:%s" % self._selected_source
            self._last_output_limited = False
            self._publish_stop()
            self._publish_status()
            return

        output_msg, limited = self._apply_speed_limits(source.last_msg)
        self._last_output_source = self._selected_source
        self._last_output_limited = limited
        self._cmd_vel_pub.publish(output_msg)
        self._publish_status()

    def _normalize_source(self, raw_source) -> Optional[str]:
        source = str(raw_source or "").strip().lower()
        return SOURCE_ALIASES.get(source)

    def _default_source_from_parameter(self) -> Optional[str]:
        raw_source = str(self.get_parameter("default_source").value or "").strip()
        if not raw_source:
            return None

        source = self._normalize_source(raw_source)
        if source is None:
            raise ValueError(
                "default_source must be empty or one of: %s"
                % ", ".join(SOURCES)
            )
        return source

    def _non_negative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError("%s must be non-negative" % name)
        return value

    def _update_speed_limits(self, payload: dict) -> None:
        next_linear = self._max_linear_speed
        next_angular = self._max_angular_speed
        changed = False

        raw_linear = self._first_present(
            payload,
            (
                "max_linear_speed_mps",
                "linear_speed_mps",
                "linear_limit_mps",
                "max_linear",
                "linear",
            ),
        )
        if raw_linear is not None:
            next_linear = self._non_negative_float(raw_linear, "max_linear_speed_mps")
            changed = True

        raw_angular = self._first_present(
            payload,
            (
                "max_angular_speed_radps",
                "angular_speed_radps",
                "angular_limit_radps",
                "max_angular",
                "angular",
            ),
        )
        if raw_angular is not None:
            next_angular = self._non_negative_float(
                raw_angular, "max_angular_speed_radps"
            )
            changed = True

        if not changed:
            raise ValueError(
                "provide max_linear_speed_mps and/or max_angular_speed_radps"
            )

        self._max_linear_speed = next_linear
        self._max_angular_speed = next_angular

    def _first_present(self, payload: dict, keys) -> Optional[object]:
        for key in keys:
            if key in payload:
                return payload[key]
        return None

    def _non_negative_float(self, raw_value, field_name: str) -> float:
        try:
            value = float(raw_value)
        except (TypeError, ValueError) as exc:
            raise ValueError("%s must be a number" % field_name) from exc
        if value < 0.0:
            raise ValueError("%s must be non-negative" % field_name)
        return value

    def _apply_speed_limits(self, msg: Twist) -> tuple[Twist, bool]:
        limited = False
        output = Twist()
        output.linear.x = float(msg.linear.x)
        output.linear.y = float(msg.linear.y)
        output.linear.z = float(msg.linear.z)
        output.angular.x = float(msg.angular.x)
        output.angular.y = float(msg.angular.y)
        output.angular.z = float(msg.angular.z)

        linear_speed = math.hypot(output.linear.x, output.linear.y)
        if linear_speed > self._max_linear_speed:
            scale = 0.0
            if linear_speed > 0.0 and self._max_linear_speed > 0.0:
                scale = self._max_linear_speed / linear_speed
            output.linear.x *= scale
            output.linear.y *= scale
            limited = True

        angular_z = self._clamp_scalar(
            output.angular.z, self._max_angular_speed
        )
        if angular_z != output.angular.z:
            output.angular.z = angular_z
            limited = True

        return output, limited

    def _clamp_scalar(self, value: float, limit: float) -> float:
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def _source_is_stale(self, source: SourceState) -> bool:
        if self._source_timeout <= 0.0:
            return False
        if source.last_seen_at is None:
            return True
        return (
            self._seconds_since(source.last_seen_at, self.get_clock().now())
            > self._source_timeout
        )

    def _publish_stop(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _publish_status(self, *, force: bool = False) -> None:
        if not force and self._status_period <= 0.0:
            return
        now = self.get_clock().now()
        if (
            not force
            and self._last_status_at is not None
            and self._seconds_since(self._last_status_at, now) < self._status_period
        ):
            return
        self._last_status_at = now
        msg = String()
        msg.data = json.dumps(self._status())
        self._status_pub.publish(msg)

    def _status(self) -> dict:
        return {
            "stopped": self._stopped,
            "selected_source": self._selected_source or "",
            "output_source": self._last_output_source,
            "output_limited": self._last_output_limited,
            "source_timeout_sec": self._source_timeout,
            "speed_limits": {
                "max_linear_speed_mps": self._max_linear_speed,
                "max_angular_speed_radps": self._max_angular_speed,
            },
            "sources": {
                name: {
                    "topic": source.topic,
                    "fresh": source.last_msg is not None
                    and not self._source_is_stale(source),
                    "age_sec": self._source_age(source),
                }
                for name, source in self._sources.items()
            },
        }

    def _source_age(self, source: SourceState) -> Optional[float]:
        if source.last_seen_at is None:
            return None
        return self._seconds_since(source.last_seen_at, self.get_clock().now())

    def _seconds_since(self, earlier: Time, later: Time) -> float:
        seconds = (later - earlier).nanoseconds / 1_000_000_000.0
        return max(0.0, seconds)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelArcestrator()
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


if __name__ == "__main__":
    main()
