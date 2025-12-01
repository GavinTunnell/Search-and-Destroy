#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class TabletNavBridge(Node):
    def __init__(self):
        super().__init__('tablet_nav_bridge')

        # 1) Subscribe to the tablet goal topic (DO NOT change this name)
        self.sub = self.create_subscription(
            PoseStamped,
            '/tablet_goal',
            self.on_tablet_goal,
            10
        )

        # 2) Nav2 action client (standard nav2 action name)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info('TabletNavBridge up, listening on /tablet_goal')

    def on_tablet_goal(self, msg: PoseStamped):
        # Log what we got from the tablet
        p = msg.pose.position
        o = msg.pose.orientation
        self.get_logger().info(
            f"Received tablet goal: "
            f"x={p.x:.2f}, y={p.y:.2f}, "
            f"q=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})"
        )

        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 navigate_to_pose action server not available')
            return

        goal_msg = NavigateToPose.Goal()

        # 3) Copy pose straight through, but refresh the header stamp
        goal_msg.pose.header.frame_id = msg.header.frame_id or 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = msg.pose.position
        goal_msg.pose.pose.orientation = msg.pose.orientation  # <-- KEEP ORIENTATION

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal was rejected')
            return

        self.get_logger().info('Nav2 goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Nav2 result: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = TabletNavBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
