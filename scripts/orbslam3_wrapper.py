#!/usr/bin/env python3
"""
ORB-SLAM3 ROS2 Wrapper - 简化版
使用 ORB-SLAM3 库进行视觉 SLAM

用法: ros2 run service_robot_cart_description orbslam3_wrapper.py --ros-args -p vocab_file:=/path/to/ORBvoc.txt
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False

try:
    import sys
    sys.path.append('/home/lab/ORB_SLAM3/lib')
    sys.path.append('/home/lab/ORB_SLAM3')
    import pyorb_slam3  # 需要编译 Python binding
    HAS_ORB = True
except ImportError:
    HAS_ORB = False
    print("[WARN] pyorb_slam3 not available, running in mock mode")


class ORBSLAM3Wrapper(Node):
    def __init__(self):
        super().__init__('orbslam3_wrapper')

        self.declare_parameter('vocab_file', '/home/lab/ORB_SLAM3/Vocabulary/ORBvoc.txt')
        self.declare_parameter('settings_file', '')
        self.declare_parameter('input_topic', '/camera/rgb/image_raw')
        self.declare_parameter('use_viewer', False)

        vocab_file = self.get_parameter('vocab_file').value
        settings_file = self.get_parameter('settings_file').value
        input_topic = self.get_parameter('input_topic').value

        self.bridge = CvBridge() if HAS_CV else None
        self.slam = None

        if HAS_ORB:
            try:
                self.slam = pyorb_slam3.System(vocab_file, settings_file, pyorb_slam3.System.RGBD)
                self.get_logger().info(f'ORB-SLAM3 initialized with {vocab_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to init ORB-SLAM3: {e}')

        # 发布器
        self.pose_pub = self.create_publisher(PoseStamped, '/orbslam3/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/orbslam3/odom', 10)

        # 订阅器
        self.sub = self.create_subscription(Image, input_topic, self.image_callback, 10)

        self.frame_count = 0
        self.get_logger().info(f'ORB-SLAM3 wrapper started (mock={not HAS_ORB})')
        self.get_logger().info(f'  Input: {input_topic}')
        self.get_logger().info(f'  Vocab: {vocab_file}')

    def image_callback(self, msg):
        if not HAS_CV or not self.bridge:
            return

        self.frame_count += 1
        if self.frame_count % 3 != 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        if self.slam is not None:
            # 真实 ORB-SLAM3 推理
            try:
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                pose = self.slam.TrackRGBD(cv_image, None, timestamp)
                if pose is not None:
                    self._publish_pose(msg.header, pose)
            except Exception as e:
                self.get_logger().error(f'ORB-SLAM3 tracking error: {e}')
        else:
            # Mock 模式
            self._publish_mock_pose(msg.header)

    def _publish_pose(self, header, pose_matrix):
        pose_msg = PoseStamped()
        pose_msg.header = header
        # 从 4x4 矩阵提取位姿
        pose_msg.pose.position.x = float(pose_matrix[0, 3])
        pose_msg.pose.position.y = float(pose_matrix[1, 3])
        pose_msg.pose.position.z = float(pose_matrix[2, 3])
        # 四元数（简化版）
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

    def _publish_mock_pose(self, header):
        import math
        t = self.get_clock().now().nanoseconds * 1e-9
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose.position.x = 0.5 * math.sin(t * 0.1)
        pose_msg.pose.position.y = 0.5 * math.cos(t * 0.1)
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ORBSLAM3Wrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
