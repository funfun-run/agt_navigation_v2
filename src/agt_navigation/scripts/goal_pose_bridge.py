#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class GoalPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("agt_goal_pose_bridge")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._status = self.create_publisher(String, "/agt/navigation/status", 10)
        self._goal_handle = None
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)

    def _publish_status(self, value: str) -> None:
        message = String()
        message.data = value
        self._status.publish(message)

    def _on_goal(self, pose: PoseStamped) -> None:
        if not pose.header.frame_id:
            pose.header.frame_id = "map"
        if not self._client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warning("NavigateToPose action server is not ready")
            self._publish_status("action_server_unavailable")
            return
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._publish_status("goal_submitted")
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._goal_handle = None
            self._publish_status("goal_rejected")
            return
        self._publish_status("goal_accepted")
        result = self._goal_handle.get_result_async()
        result.add_done_callback(self._goal_result)

    def _goal_result(self, future) -> None:
        status = future.result().status
        self._goal_handle = None
        self._publish_status(f"goal_finished:{status}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalPoseBridge()
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
