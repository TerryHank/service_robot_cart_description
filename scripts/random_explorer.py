#!/usr/bin/env python3
"""Random Explorer with obstacle avoidance for SLAM mapping.
   One param 'speed' controls everything. Hot-modify via: ros2 param set /random_explorer speed <value>"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import random


class RandomExplorer(Node):
    def __init__(self):
        super().__init__("random_explorer")
        self.declare_parameter("speed", 1.2)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.map_data = None
        self.front_dist = 10.0  # default far away
        self.left_dist = 10.0
        self.right_dist = 10.0
        self.timer = self.create_timer(0.5, self.move_step)
        self.state = "forward"
        self.state_timer = 0
        self.get_logger().info("Random explorer started! Change speed: ros2 param set /random_explorer speed <value>")

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 10.0
        ranges[np.isnan(ranges)] = 10.0
        n = len(ranges)
        # front: ±15 degrees
        front_start = n * 7 // 12
        front_end = n * 5 // 12
        front = np.concatenate([ranges[:front_end], ranges[front_start:]])
        self.front_dist = float(np.min(front))
        # left and right sides
        self.left_dist = float(np.min(ranges[n*3//12:n*5//12]))
        self.right_dist = float(np.min(ranges[n*7//12:n*9//12]))

    def move_step(self):
        s = self.get_parameter("speed").value
        msg = Twist()
        self.state_timer += 1
        stop_dist = 0.5  # stop and turn if obstacle within 0.5m

        if self.state == "forward":
            if self.front_dist < stop_dist:
                # obstacle ahead, turn immediately
                self.state = "turn_away"
                self.state_timer = 0
                self.turn_away_duration = random.randint(6, 14)
                # turn toward the more open side
                if self.left_dist > self.right_dist:
                    self.turn_away_dir = 1  # turn left
                else:
                    self.turn_away_dir = -1  # turn right
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                msg.linear.x = s
                msg.angular.z = 0.0
                if self.state_timer > random.randint(6, 16):
                    self.state = "turn"
                    self.state_timer = 0
                    self.turn_duration = random.randint(4, 12)
                    self.turn_direction = random.choice([-1, 1])
        elif self.state == "turn":
            msg.linear.x = 0.0
            msg.angular.z = s * 1.5 * self.turn_direction
            if self.state_timer > self.turn_duration:
                self.state = "forward"
                self.state_timer = 0
        elif self.state == "turn_away":
            msg.linear.x = 0.0
            msg.angular.z = s * 2.0 * self.turn_away_dir
            if self.state_timer > self.turn_away_duration:
                self.state = "forward"
                self.state_timer = 0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
