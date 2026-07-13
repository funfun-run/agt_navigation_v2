#!/usr/bin/env python3

import math
import time

import rclpy
from bunker_msgs.msg import BunkerStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


class BunkerStatusBridge(Node):
    def __init__(self) -> None:
        super().__init__("agt_bunker_status_bridge")
        raw_topic = self.declare_parameter(
            "raw_status_topic", "/agt/chassis/status/raw"
        ).value
        self._timeout = self.declare_parameter("status_timeout", 0.50).value
        self._last_status = float("-inf")
        self._status = None
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/agt/chassis/status", 10
        )
        self._battery_pub = self.create_publisher(BatteryState, "/battery", 10)
        self._connected_pub = self.create_publisher(
            Bool, "/agt/chassis/connected", 10
        )
        self.create_subscription(BunkerStatus, raw_topic, self._callback, 10)
        self.create_timer(0.2, self._tick)

    def _callback(self, msg: BunkerStatus) -> None:
        self._status = msg
        self._last_status = time.monotonic()

        battery = BatteryState()
        battery.header = msg.header
        battery.voltage = float(msg.battery_voltage)
        battery.percentage = math.nan
        battery.present = True
        self._battery_pub.publish(battery)

    def _tick(self) -> None:
        connected = time.monotonic() - self._last_status <= self._timeout
        self._connected_pub.publish(Bool(data=connected))

        status = DiagnosticStatus()
        status.name = "agt_chassis/bunker"
        status.hardware_id = "bunker_can"
        if not connected or self._status is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "status_timeout"
        else:
            status.level = (
                DiagnosticStatus.ERROR
                if self._status.error_code != 0
                else DiagnosticStatus.OK
            )
            status.message = (
                f"error_0x{self._status.error_code:04x}"
                if self._status.error_code
                else "connected"
            )
            status.values = [
                KeyValue(key="vehicle_state", value=str(self._status.vehicle_state)),
                KeyValue(key="control_mode", value=str(self._status.control_mode)),
                KeyValue(key="battery_voltage", value=f"{self._status.battery_voltage:.2f}"),
                KeyValue(key="linear_velocity", value=f"{self._status.linear_velocity:.3f}"),
                KeyValue(key="angular_velocity", value=f"{self._status.angular_velocity:.3f}"),
            ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostic_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BunkerStatusBridge()
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
