#!/usr/bin/env python3
"""Publish /robot_description topic so gz_ros_control can receive it."""
import subprocess, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        self.pub = self.create_publisher(String, 'robot_description', 10)
        self.timer = self.create_timer(2.0, self.publish_once)
        self.count = 0
        self.get_logger().info('Robot description publisher started')
        
    def publish_once(self):
        try:
            xacro_path = '/home/lab/ros2_ws/install/service_robot_cart_description/share/service_robot_cart_description/urdf/service_robot_cart_gazebo.urdf.xacro'
            result = subprocess.run(['xacro', xacro_path], capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                self.get_logger().error(f'xacro failed: {result.stderr}')
                return
            msg = String()
            msg.data = result.stdout
            self.pub.publish(msg)
            self.count += 1
            self.get_logger().info(f'Published robot_description ({self.count}), {len(msg.data)} bytes')
            if self.count >= 5:
                self.timer.cancel()
                self.get_logger().info('Done publishing robot_description')
        except Exception as e:
            self.get_logger().error(f'Failed: {e}')

def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
