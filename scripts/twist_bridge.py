#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__("twist_to_twist_stamped")
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.callback, 10)
        self.pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.get_logger().info("Bridging /cmd_vel -> /diff_drive_controller/cmd_vel (TwistStamped)")

    def callback(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = "base_link"
        stamped.twist = msg  # 直接透传，不反转
        self.pub.publish(stamped)

def main():
    rclpy.init()
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
