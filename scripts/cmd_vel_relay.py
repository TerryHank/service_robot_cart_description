#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/model/service_robot_cart/cmd_vel', 10)
    def cb(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(CmdVelRelay())

if __name__ == '__main__':
    main()
