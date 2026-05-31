#!/usr/bin/env python3
"""Fake LaserScan publisher - 从 odom 生成模拟 scan 数据"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np


class FakeLaserScan(Node):
    def __init__(self):
        super().__init__('fake_laser_scan')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.sub = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10Hz
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.get_logger().info('Fake LaserScan publisher started')

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0

        num_readings = 360
        ranges = []
        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            world_angle = angle + self.yaw
            # 简单的房间边界模拟（10x10 房间）
            cos_a = math.cos(world_angle)
            sin_a = math.sin(world_angle)
            dist = 10.0
            # 计算到墙壁的距离
            if abs(cos_a) > 1e-6:
                t1 = (5.0 - self.x) / cos_a
                t2 = (-5.0 - self.x) / cos_a
                for t in [t1, t2]:
                    if t > 0.1:
                        dist = min(dist, t)
            if abs(sin_a) > 1e-6:
                t1 = (5.0 - self.y) / sin_a
                t2 = (-5.0 - self.y) / sin_a
                for t in [t1, t2]:
                    if t > 0.1:
                        dist = min(dist, t)
            # 添加一些噪声
            dist += np.random.normal(0, 0.02)
            ranges.append(max(scan.range_min, min(scan.range_max, dist)))

        scan.ranges = ranges
        scan.intensities = []
        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
