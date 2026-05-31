#!/usr/bin/env python3
"""Publish odom->base_link TF from /odom topic.
   Uses node clock (sim_time when use_sim_time=True)."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__("odom_tf_publisher")
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.get_logger().info("Odom TF publisher started")

    def odom_cb(self, msg):
        t = TransformStamped()
        # Use node clock (sim_time when use_sim_time=True)
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
