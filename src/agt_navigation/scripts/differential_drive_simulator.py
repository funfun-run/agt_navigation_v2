#!/usr/bin/env python3

import math
import struct

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class DifferentialDriveSimulator(Node):
    def __init__(self) -> None:
        super().__init__("agt_offline_base_simulator")
        self._rate = float(self.declare_parameter("publish_rate", 30.0).value)
        self._max_linear = float(self.declare_parameter("max_linear_speed", 0.5).value)
        self._max_angular = float(self.declare_parameter("max_angular_speed", 0.6).value)
        self._x = float(self.declare_parameter("initial_x", 0.0).value)
        self._y = float(self.declare_parameter("initial_y", 0.0).value)
        self._yaw = float(self.declare_parameter("initial_yaw", 0.0).value)
        self._linear = 0.0
        self._angular = 0.0
        self._obstacle_enabled = bool(
            self.declare_parameter("synthetic_obstacle_enabled", False).value
        )
        self._obstacle_x = float(self.declare_parameter("synthetic_obstacle_x", 0.7).value)
        self._obstacle_y = float(self.declare_parameter("synthetic_obstacle_y", 0.0).value)
        self._obstacle_z = float(self.declare_parameter("synthetic_obstacle_z", 0.3).value)
        self._last_time = self.get_clock().now()
        self._odom = self.create_publisher(Odometry, "/agt/mapping/odometry", 20)
        self._cloud = self.create_publisher(PointCloud2, "/agt/perception/obstacle_cloud", 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "/agt/safety/cmd_vel", self._on_cmd, 10)
        self.create_timer(1.0 / self._rate, self._tick)

    def _on_cmd(self, message: Twist) -> None:
        self._linear = max(-self._max_linear, min(message.linear.x, self._max_linear))
        self._angular = max(-self._max_angular, min(message.angular.z, self._max_angular))

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = min(max((now - self._last_time).nanoseconds * 1.0e-9, 0.0), 0.1)
        self._last_time = now
        self._yaw = math.atan2(
            math.sin(self._yaw + self._angular * dt),
            math.cos(self._yaw + self._angular * dt),
        )
        self._x += self._linear * math.cos(self._yaw) * dt
        self._y += self._linear * math.sin(self._yaw) * dt
        stamp = now.to_msg()
        qx, qy, qz, qw = quaternion_from_yaw(self._yaw)

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.rotation.w = 1.0
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_footprint"
        odom_to_base.transform.translation.x = self._x
        odom_to_base.transform.translation.y = self._y
        odom_to_base.transform.rotation.x = qx
        odom_to_base.transform.rotation.y = qy
        odom_to_base.transform.rotation.z = qz
        odom_to_base.transform.rotation.w = qw
        self._tf.sendTransform([map_to_odom, odom_to_base])

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self._linear
        odom.twist.twist.angular.z = self._angular
        self._odom.publish(odom)
        self._publish_obstacle_cloud(stamp)

    def _publish_obstacle_cloud(self, stamp) -> None:
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "base_footprint"
        cloud.height = 1
        cloud.width = 1 if self._obstacle_enabled else 0
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        if self._obstacle_enabled:
            cloud.data = struct.pack(
                "<fff", self._obstacle_x, self._obstacle_y, self._obstacle_z
            )
        self._cloud.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DifferentialDriveSimulator()
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
