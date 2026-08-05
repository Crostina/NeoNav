#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTfBridge(Node):
    def __init__(self):
        super().__init__("euroboot_odom_to_tf_bridge")
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, "/odom/unfiltered", self.odom_cb, 20)
        self.last_tf_time = 0.0

    def odom_cb(self, msg: Odometry):
        now = time.monotonic()
        if now - self.last_tf_time < 0.05:
            return
        self.last_tf_time = now

        tf = TransformStamped()
        tf.header = msg.header
        tf.header.frame_id = msg.header.frame_id or "odom"
        tf.child_frame_id = msg.child_frame_id or "base_footprint"
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomToTfBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
