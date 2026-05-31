#!/usr/bin/env python3
"""Frontier Explorer - auto exploration with blacklist and obstacle filtering"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import numpy as np
import math
from scipy import ndimage

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.map_data = None
        self.map_info = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.navigating = False
        self.explored_count = 0
        self.total_goals = 50
        self.failed_goals = set()
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.timer = self.create_timer(5.0, self.explore_step)
        self.get_logger().info("Frontier explorer started, waiting for map...")

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def explore_step(self):
        if self.map_data is None:
            self.get_logger().info("Waiting for map...")
            return
        if self.navigating:
            return
        if self.explored_count >= self.total_goals:
            self.get_logger().info("Exploration complete! Navigated %d times" % self.explored_count)
            self.timer.cancel()
            return

        frontiers = self.find_frontiers_fast()
        if len(frontiers) == 0:
            self.get_logger().info("No more frontiers, map may be complete!")
            self.timer.cancel()
            return

        best = self.select_best_frontier(frontiers)
        if best is None:
            self.get_logger().info("No reachable frontier")
            return
        self.send_goal(best[0], best[1])

    def find_frontiers_fast(self):
        """Optimized frontier detection using numpy/scipy"""
        unknown = (self.map_data == -1).astype(np.uint8)
        free = (self.map_data == 0).astype(np.uint8)
        
        # Dilate free space by 1 pixel
        kernel = np.ones((3, 3), dtype=np.uint8)
        free_dilated = ndimage.binary_dilation(free, structure=kernel).astype(np.uint8)
        
        # Frontiers = unknown pixels adjacent to free space
        frontier_mask = unknown & free_dilated
        
        # Get frontier coordinates
        ys, xs = np.where(frontier_mask)
        if len(xs) == 0:
            return []
        
        # Convert to world coordinates
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        world_xs = origin_x + xs * resolution
        world_ys = origin_y + ys * resolution
        
        # Sample frontiers (max 1000 for performance)
        if len(xs) > 1000:
            indices = np.random.choice(len(xs), 1000, replace=False)
            return list(zip(world_xs[indices], world_ys[indices]))
        return list(zip(world_xs, world_ys))

    def select_best_frontier(self, frontiers):
        """Select nearest frontier cluster, avoiding blacklisted goals and obstacles"""
        if not frontiers:
            return None
        
        pts = np.array(frontiers)
        # Simple grid-based clustering
        grid_size = 1.0  # 1m grid
        grid_ids = np.floor(pts / grid_size).astype(int)
        grid_keys = grid_ids[:, 0] * 10000 + grid_ids[:, 1]
        
        unique_keys, counts = np.unique(grid_keys, return_counts=True)
        # Get largest clusters
        top_indices = np.argsort(-counts)[:20]
        
        best_score = -1
        best_point = None
        
        for idx in top_indices:
            key = unique_keys[idx]
            mask = grid_keys == key
            cluster_pts = pts[mask]
            cx, cy = cluster_pts.mean(axis=0)
            
            # Check if this point is blacklisted
            grid_key = (round(cx), round(cy))
            if grid_key in self.failed_goals:
                continue
            
            # Check if point is too close to obstacles
            if self.is_near_obstacle(cx, cy):
                continue
            
            dist = math.sqrt((cx - self.robot_x)**2 + (cy - self.robot_y)**2)
            size = counts[idx]
            
            if dist > 0.5:  # Ignore too close
                score = size / (dist + 0.1)
                if score > best_score:
                    best_score = score
                    best_point = (cx, cy)
        
        if best_point:
            self.get_logger().info("Frontiers: %d pts, target: (%.1f, %.1f)" % (len(frontiers), best_point[0], best_point[1]))
        return best_point

    def is_near_obstacle(self, x, y):
        """Check if point is too close to obstacles"""
        if self.map_data is None:
            return False
        
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        
        # Convert world to pixel
        px = int((x - origin_x) / resolution)
        py = int((y - origin_y) / resolution)
        
        # Check bounds
        h, w = self.map_data.shape
        if px < 0 or px >= w or py < 0 or py >= h:
            return True
        
        # Check if point is occupied (100) or unknown (-1)
        if self.map_data[py, px] == 100 or self.map_data[py, px] == -1:
            return True
        
        # Check 3x3 area around point
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                ny, nx = py + dy, px + dx
                if 0 <= ny < h and 0 <= nx < w:
                    if self.map_data[ny, nx] == 100:
                        return True
        
        return False

    def send_goal(self, x, y):
        self.navigating = True
        self.current_goal = (x, y)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        self.get_logger().info("Navigating to (%.1f, %.1f)" % (x, y))
        self.nav_client.wait_for_server(timeout_sec=10.0)
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected!")
            self.navigating = False
            return
        self.get_logger().info("Goal accepted, navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == 4:
            self.get_logger().info("Reached target!")
            self.explored_count += 1
        else:
            self.get_logger().warn("Navigation failed (status=%d)" % status)
            # Blacklist this goal
            if hasattr(self, "current_goal"):
                grid_key = (round(self.current_goal[0]), round(self.current_goal[1]))
                self.failed_goals.add(grid_key)
                self.get_logger().info("Blacklisted goal (%.1f, %.1f)" % self.current_goal)
        self.navigating = False

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
