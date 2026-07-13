#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def clamp(value: float, limit: float) -> float:
    return min(max(value, -limit), limit)


class ChassisCommandGuard(Node):
    def __init__(self) -> None:
        super().__init__("agt_chassis_command_guard")
        input_topic = self.declare_parameter(
            "input_topic", "/agt/safety/cmd_vel"
        ).value
        output_topic = self.declare_parameter(
            "output_topic", "/agt/chassis/cmd_vel"
        ).value
        self._timeout = self.declare_parameter("command_timeout", 0.20).value
        self._publish_rate = self.declare_parameter("publish_rate", 30.0).value
        self._max_linear = self.declare_parameter("hard_max_linear_speed", 0.60).value
        self._max_angular = self.declare_parameter("hard_max_angular_speed", 0.70).value
        self._last_command = Twist()
        self._last_stamp = float("-inf")
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._callback, 10)
        self.create_timer(1.0 / self._publish_rate, self._tick)

    def _callback(self, msg: Twist) -> None:
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self._last_stamp = float("-inf")
            self.get_logger().error("rejected non-finite safety command")
            return
        self._last_command = msg
        self._last_stamp = time.monotonic()

    def _tick(self) -> None:
        output = Twist()
        if time.monotonic() - self._last_stamp <= self._timeout:
            output.linear.x = clamp(self._last_command.linear.x, self._max_linear)
            output.angular.z = clamp(self._last_command.angular.z, self._max_angular)
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisCommandGuard()
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
