#!/usr/bin/env python3
"""Filter body_link points from LaserScan by masking center angular sector."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanBodyFilter(Node):
    def __init__(self):
        super().__init__('scan_body_filter')
        self.declare_parameter('mask_half_width_deg', 12.0)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.get_logger().info('Body filter: /scan -> /scan_filtered (center +-12deg masked)')

    def cb(self, msg: LaserScan):
        half = math.radians(self.get_parameter('mask_half_width_deg').value)
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = list(msg.ranges)
        filtered.intensities = list(msg.intensities) if msg.intensities else []
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) < half:
                filtered.ranges[i] = float('inf')
        self.pub.publish(filtered)

def main():
    rclpy.init()
    rclpy.spin(ScanBodyFilter())

if __name__ == '__main__':
    main()
