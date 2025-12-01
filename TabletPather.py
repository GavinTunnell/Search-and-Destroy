#!/usr/bin/env python3
#
# Listens to /tablet_goal (PoseStamped from the tablet UI)
# and forwards it to Nav2's NavigateToPose action.
#

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
w
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class TabletGoalToNav2(Node):
    def __init__(self):
        super().__init__('tablet_goal_to_nav2')

        # Action client for Nav2
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to the goal published by the tablet UI
        self._tablet_goal_sub = self.create_subscription(
            PoseStamped,
            '/tablet_goal',         # must match HTML JS topic name
            self.tablet_goal_cb,
            10
        )

        self._current_goal_handle = None
        self._sending_goal = False

        self.get_logger().info('TabletGoalToNav2 node started. Waiting for /tablet_goal...')

    # Called whenever the tablet publishes a new PoseStamped
    def tablet_goal_cb(self, msg: PoseStamped):
        # If we’re already processing a goal, you can either cancel it or queue the new one.
        if self._sending_goal:
            self.get_logger().warn('Got new /tablet_goal while one is active. Cancelling old goal and sending new one.')
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()

        self._sending_goal = True

        # Ensure frame_id is valid; the HTML sets it to 'map'
        if not msg.header.frame_id:
            msg.header.frame_id = 'map'

        self.get_logger().info(
            f'Received tablet goal: frame={msg.header.frame_id}, '
            f'x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
        )

        # Wait for the Nav2 action server
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 navigate_to_pose action server not available! Is nav2_bringup running?')
            self._sending_goal = False
            return

        # Wrap PoseStamped in NavigateToPose.Goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        self.get_logger().info('Sending goal to Nav2...')
        send_goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_cb
        )
        send_goal_future.add_done_callback(self._goal_response_cb)

    # Called when the action server accepts/rejects the goal
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected.')
            self._sending_goal = False
            self._current_goal_handle = None
            return

        self.get_logger().info('Nav2 goal accepted.')
        self._current_goal_handle = goal_handle

        # Now wait for the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._nav_result_cb)

    # Optional feedback from Nav2 (current pose, distance remaining, etc.)
    def nav_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        # distance_remaining is in meters
        self.get_logger().debug(
            f'Nav2 feedback: distance_remaining={fb.distance_remaining:.2f} m'
        )

    # Called when Nav2 finishes (succeeded, aborted, canceled)
    def _nav_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        self.get_logger().info(f'Nav2 result received. Status={status}')
        # You can inspect result if needed (e.g., result.error_code).

        self._sending_goal = False
        self._current_goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = TabletGoalToNav2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
