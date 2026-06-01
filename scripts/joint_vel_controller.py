#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

WHEEL_RADIUS = 0.08
WHEEL_SEPARATION = 0.41

class JointVelocityController(Node):
    def __init__(self):
        super().__init__("joint_vel_controller")
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        self.pubs = {}
        for j in ["front_left_wheel_joint", "rear_left_wheel_joint",
                  "front_right_wheel_joint", "rear_right_wheel_joint"]:
            self.pubs[j] = self.create_publisher(
                Float64, f"/model/service_robot_cart/joint/{j}/cmd_vel", 10)
        self.get_logger().info("Joint velocity controller ready (ROS bridge)")

    def cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        vl = (v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        vr = (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        for j in ["front_left_wheel_joint", "rear_left_wheel_joint"]:
            m = Float64(); m.data = vl; self.pubs[j].publish(m)
        for j in ["front_right_wheel_joint", "rear_right_wheel_joint"]:
            m = Float64(); m.data = vr; self.pubs[j].publish(m)

def main():
    rclpy.init()
    rclpy.spin(JointVelocityController())

if __name__ == "__main__":
    main()
