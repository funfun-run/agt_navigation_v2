#!/usr/bin/env python3

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def project_track_speeds(
    linear: float, angular: float, track_width: float, max_track_speed: float
) -> tuple[float, float]:
    left = linear - angular * track_width * 0.5
    right = linear + angular * track_width * 0.5
    peak = max(abs(left), abs(right))
    if max_track_speed > 0.0 and peak > max_track_speed:
        scale = max_track_speed / peak
        linear *= scale
        angular *= scale
    return linear, angular


def slew(current: float, target: float, rise_rate: float, fall_rate: float, dt: float) -> float:
    rate = rise_rate if abs(target) > abs(current) else fall_rate
    return current + clamp(target - current, -rate * dt, rate * dt)


class TrackedSafetyController(Node):
    def __init__(self) -> None:
        super().__init__("agt_tracked_safety_controller")
        self._nav_timeout = self.declare_parameter("navigation_timeout", 0.5).value
        self._manual_timeout = self.declare_parameter("manual_timeout", 0.35).value
        self._publish_rate = self.declare_parameter("publish_rate", 20.0).value
        self._max_forward = self.declare_parameter("max_forward_speed", 0.5).value
        self._max_reverse = self.declare_parameter("max_reverse_speed", 0.25).value
        self._max_angular = self.declare_parameter("max_angular_speed", 0.6).value
        self._max_track = self.declare_parameter("max_track_speed", 0.65).value
        self._track_width = self.declare_parameter("effective_track_width", 0.62).value
        self._linear_accel = self.declare_parameter("max_linear_acceleration", 0.35).value
        self._linear_decel = self.declare_parameter("max_linear_deceleration", 0.7).value
        self._angular_accel = self.declare_parameter("max_angular_acceleration", 0.8).value
        self._angular_decel = self.declare_parameter("max_angular_deceleration", 1.2).value
        self._motion_enabled = self.declare_parameter("startup_motion_enabled", False).value

        self._nav_cmd = Twist()
        self._manual_cmd = Twist()
        self._nav_stamp = float("-inf")
        self._manual_stamp = float("-inf")
        self._physical_estop = False
        self._estop_latched = False
        self._linear_out = 0.0
        self._angular_out = 0.0
        self._last_tick = time.monotonic()
        self._reason = "startup_disabled" if not self._motion_enabled else "input_timeout"

        self._publisher = self.create_publisher(Twist, "/agt/safety/cmd_vel", 10)
        self._status_publisher = self.create_publisher(
            DiagnosticArray, "/agt/safety/status", 10
        )
        self.create_subscription(
            Twist, "/agt/navigation/cmd_vel", self._navigation_callback, 10
        )
        self.create_subscription(
            Twist, "/agt/cmd_vel_manual", self._manual_callback, 10
        )
        self.create_subscription(
            Bool, "/agt/safety/emergency_stop", self._estop_callback, 10
        )
        self.create_service(
            SetBool, "/agt/safety/set_motion_enabled", self._set_motion_enabled
        )
        self.create_service(
            Trigger, "/agt/safety/reset_emergency_stop", self._reset_estop
        )
        self.create_timer(1.0 / self._publish_rate, self._tick)

    @staticmethod
    def _valid(cmd: Twist) -> bool:
        values = (
            cmd.linear.x,
            cmd.linear.y,
            cmd.linear.z,
            cmd.angular.x,
            cmd.angular.y,
            cmd.angular.z,
        )
        return all(math.isfinite(value) for value in values)

    def _navigation_callback(self, msg: Twist) -> None:
        if self._valid(msg):
            self._nav_cmd = msg
            self._nav_stamp = time.monotonic()
        else:
            self._nav_stamp = float("-inf")
            self.get_logger().error("rejected non-finite navigation command")

    def _manual_callback(self, msg: Twist) -> None:
        if self._valid(msg):
            self._manual_cmd = msg
            self._manual_stamp = time.monotonic()
        else:
            self._manual_stamp = float("-inf")
            self.get_logger().error("rejected non-finite manual command")

    def _estop_callback(self, msg: Bool) -> None:
        self._physical_estop = msg.data
        if msg.data:
            self._estop_latched = True
            self._motion_enabled = False

    def _set_motion_enabled(self, request: SetBool.Request, response: SetBool.Response):
        if request.data and (self._physical_estop or self._estop_latched):
            response.success = False
            response.message = "clear the emergency stop before enabling motion"
            return response
        self._motion_enabled = request.data
        response.success = True
        response.message = "motion enabled" if request.data else "motion disabled"
        return response

    def _reset_estop(self, _request: Trigger.Request, response: Trigger.Response):
        if self._physical_estop:
            response.success = False
            response.message = "physical emergency-stop input is still active"
            return response
        self._estop_latched = False
        response.success = True
        response.message = "emergency stop latch cleared; motion remains explicitly controlled"
        return response

    def _target(self, now: float) -> tuple[float, float, str, bool]:
        if self._physical_estop or self._estop_latched:
            return 0.0, 0.0, "emergency_stop", True
        if not self._motion_enabled:
            return 0.0, 0.0, "motion_disabled", True
        if now - self._manual_stamp <= self._manual_timeout:
            cmd = self._manual_cmd
            source = "manual"
        elif now - self._nav_stamp <= self._nav_timeout:
            cmd = self._nav_cmd
            source = "navigation"
        else:
            return 0.0, 0.0, "input_timeout", True

        linear = clamp(cmd.linear.x, -self._max_reverse, self._max_forward)
        angular = clamp(cmd.angular.z, -self._max_angular, self._max_angular)
        linear, angular = project_track_speeds(
            linear, angular, self._track_width, self._max_track
        )
        return linear, angular, source, False

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self._last_tick, 0.0), 0.2)
        self._last_tick = now
        target_linear, target_angular, self._reason, immediate_stop = self._target(now)
        if immediate_stop:
            self._linear_out = 0.0
            self._angular_out = 0.0
        else:
            self._linear_out = slew(
                self._linear_out,
                target_linear,
                self._linear_accel,
                self._linear_decel,
                dt,
            )
            self._angular_out = slew(
                self._angular_out,
                target_angular,
                self._angular_accel,
                self._angular_decel,
                dt,
            )

        output = Twist()
        output.linear.x = self._linear_out
        output.angular.z = self._angular_out
        self._publisher.publish(output)
        self._publish_status()

    def _publish_status(self) -> None:
        status = DiagnosticStatus()
        status.name = "agt_safety/tracked_controller"
        status.hardware_id = "bunker"
        stopped = self._reason in ("emergency_stop", "motion_disabled", "input_timeout")
        status.level = DiagnosticStatus.WARN if stopped else DiagnosticStatus.OK
        status.message = self._reason
        status.values = [
            KeyValue(key="motion_enabled", value=str(self._motion_enabled).lower()),
            KeyValue(key="estop_latched", value=str(self._estop_latched).lower()),
            KeyValue(key="linear_output", value=f"{self._linear_out:.4f}"),
            KeyValue(key="angular_output", value=f"{self._angular_out:.4f}"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status_publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackedSafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
