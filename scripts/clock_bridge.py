#!/usr/bin/env python3
"""Efficient /clock bridge: subscribes to Gazebo via gz-transport, publishes sim time to ROS.
   Fixes ros_gz_bridge bug that forwards wall time instead of sim time."""
import threading
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from gz.transport13 import Node as GzNode
from gz.msgs10.clock_pb2 import Clock as GzClock


class ClockBridge(Node):
    def __init__(self):
        super().__init__("clock_bridge")
        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.latest_msg = None
        self.lock = threading.Lock()
        self.gz_count = 0

        # Gazebo transport subscriber
        self.gz_node = GzNode()
        self.gz_node.subscribe(GzClock, "/world/default/clock", self.on_gz_clock)
        self.get_logger().info("Clock bridge: subscribed to gz /world/default/clock")

        # Publish at fixed rate from latest received
        self.timer = self.create_timer(0.005, self.publish_clock)  # 200Hz publish
        self.get_logger().info("Clock bridge started (gz-transport -> ROS /clock, sim time)")

    def on_gz_clock(self, msg):
        with self.lock:
            self.latest_msg = (msg.sim.sec, msg.sim.nsec)
            self.gz_count += 1

    def publish_clock(self):
        with self.lock:
            data = self.latest_msg
            count = self.gz_count
        if data is None:
            return
        # Log once every 1000 publishes
        if count == 1 and not hasattr(self, "_logged_first"):
            self._logged_first = True
            self.get_logger().info(f"First GZ clock received: sim=({data[0]},{data[1]})")
        msg = Clock()
        msg.clock.sec = data[0]
        msg.clock.nanosec = data[1]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClockBridge()
    # Run rclpy.spin in background thread so gz transport callbacks work
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        # Main thread just waits
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
