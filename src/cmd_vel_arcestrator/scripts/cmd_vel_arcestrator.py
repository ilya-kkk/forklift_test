#!/usr/bin/env python3
import json
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
        self._selected_source: Optional[str] = None
        self._stopped = True
        self._last_output_source = "stop"
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
            "sources=%s timeout=%.3fs"
            % (
                output_topic,
                control_service,
                status_topic,
                ", ".join(
                    "%s:%s" % (name, state.topic)
                    for name, state in self._sources.items()
                ),
                self._source_timeout,
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
        else:
            response.success = False
            response.message = json.dumps(
                {
                    "error": "invalid_command",
                    "details": "expected select_source|stop|release|status",
                }
            )
            return response

        response.message = json.dumps(self._status())
        self._publish_status(force=True)
        return response

    def _control_tick(self) -> None:
        if self._stopped or self._selected_source is None:
            self._last_output_source = "stop"
            self._publish_stop()
            self._publish_status()
            return

        source = self._sources[self._selected_source]
        if source.last_msg is None or self._source_is_stale(source):
            self._last_output_source = "stale:%s" % self._selected_source
            self._publish_stop()
            self._publish_status()
            return

        self._last_output_source = self._selected_source
        self._cmd_vel_pub.publish(source.last_msg)
        self._publish_status()

    def _normalize_source(self, raw_source) -> Optional[str]:
        source = str(raw_source or "").strip().lower()
        return SOURCE_ALIASES.get(source)

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
            "source_timeout_sec": self._source_timeout,
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
