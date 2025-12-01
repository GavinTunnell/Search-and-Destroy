#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException, TransformBroadcaster
from geometry_msgs.msg import Quaternion, Point
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quaternion(q: Quaternion) -> float:
    """
    Extract yaw from a geometry_msgs/Quaternion.
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class FakeOdomFromTF(Node):
    """
    Periodically looks up TF (map -> base_link) and publishes nav_msgs/Odometry
    on /odom so external tools (tablet) can consume a simple odom topic.

    NOTE: we are using 'map' as the fixed frame. If you later configure
    Cartographer to provide a real 'odom' frame, just change the lookup
    target_frame from 'map' to 'odom' below and set header.frame_id = 'odom'.
    """

    def __init__(self):
        super().__init__("fake_odom_from_tf")

        # TF buffer + listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry publisher (mirrors TF as /odom)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Timer at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info(
            "fake_odom_from_tf: publishing /odom from TF (map -> base_link)"
        )

    def timer_cb(self):
        try:
            # *** IMPORTANT ***
            # Using map as the fixed frame here (Option B).
            # If you later have a real 'odom' frame, change "map" -> "odom".
            t = self.tf_buffer.lookup_transform(
                "map",        # target_frame (fixed)
                "base_link",  # source_frame (robot base)
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not get TF map -> base_link: {ex}", throttle_duration_sec=5.0
            )
            return

        # Fill Odometry message
        msg = Odometry()
        msg.header.stamp = t.header.stamp
        # We keep frame_id = "map" to match the TF tree
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        # Pose from TF
        msg.pose.pose.position = Point()  # Create an instance of Point
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = t.transform.translation.z

        msg.pose.pose.orientation = t.transform.rotation  # Quaternion from transform

        # Covariances: simple reasonable defaults
        msg.pose.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.1,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.1,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1,
        ]

        # We don't have velocity info from TF, so leave twist = 0
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(msg)

        # Also broadcast a TF using fake_odom as the parent
        tf_msg = TransformStamped()
        tf_msg.header.stamp = t.header.stamp
        tf_msg.header.frame_id = "fake_odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation = t.transform.translation
        tf_msg.transform.rotation = t.transform.rotation
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomFromTF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
