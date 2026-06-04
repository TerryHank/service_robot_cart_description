#!/usr/bin/env python3
"""
Clock bridge: publishes wall-clock time to ROS /clock so all nodes using
use_sim_time=true get monotonic, consistent timestamps. Avoids Gazebo
sim-time instability (backwards jumps, DART physics resets).
"""
import time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class ClockBridge(Node):
    def __init__(self):
        super().__init__("clock_bridge")
        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.last_ns = 0
        # 200 Hz wall-time clock
        self.timer = self.create_timer(0.005, self.publish_clock)
        self.get_logger().info("Clock bridge: publishing WALL TIME to /clock (monotonic)")

    def publish_clock(self):
        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 1e9)
        # Strict monotonic guard (in case of NTP adjustment)
        now_ns = sec * 1_000_000_000 + nsec
        if now_ns <= self.last_ns:
            now_ns = self.last_ns + 1
            sec = now_ns // 1_000_000_000
            nsec = now_ns % 1_000_000_000
        self.last_ns = now_ns
        msg = Clock()
        msg.clock.sec = sec
        msg.clock.nanosec = nsec
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClockBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
