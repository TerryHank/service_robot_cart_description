#!/usr/bin/env python3
"""
语义分割节点 - YOLO + Robot Pose → 3D 世界坐标
无深度相机方案: 用机器人位姿 + 检测框估计物体位置
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import numpy as np
import math

class SemanticObjectMarker(Node):
    def __init__(self):
        super().__init__("semantic_object_marker")
        self.bridge = CvBridge()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.detector = None
        self.objects = {}  # label -> list of {x, y, z, score}
        self.target_labels = ["chair", "table", "bottle", "cup", "book", "tv", "laptop", "cell phone", "remote"]

        # 订阅相机图像
        self.create_subscription(Image, "/depth_camera/image", self.image_callback, 10)
        
        # 订阅里程计获取机器人位姿
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self.odom_callback, 10)

        # 发布标记
        self.marker_pub = self.create_publisher(MarkerArray, "/semantic_markers", 10)

        self.get_logger().info(f"语义分割节点启动")
        self.get_logger().info(f"检测目标: {self.target_labels}")
        self.get_logger().info("等待图像数据...")

    def load_model(self):
        try:
            from ultralytics import YOLO
            self.detector = YOLO("yolov8n.pt")
            self.get_logger().info("YOLO 模型加载成功")
        except Exception as e:
            self.get_logger().error(f"YOLO 加载失败: {e}")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg):
        if self.detector is None:
            self.load_model()
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.detector(cv_image, conf=0.5, verbose=False)

            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    label = result.names[cls_id]
                    conf = float(box.conf[0])

                    if label not in self.target_labels:
                        continue

                    # 用检测框中心水平位置估计物体方位角
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    img_h, img_w = cv_image.shape[:2]
                    center_x = (x1 + x2) / 2.0
                    # 归一化到 [-0.5, 0.5]
                    norm_x = (center_x / img_w) - 0.5
                    # 假设相机 FOV ~60度
                    angle_offset = norm_x * 1.047  # 60度 = 1.047 rad
                    # 估计距离（根据目标大小）
                    obj_width = x2 - x1
                    est_distance = max(0.5, min(3.0, 200.0 / max(obj_width, 1)))

                    # 世界坐标
                    angle = self.robot_yaw + angle_offset
                    world_x = self.robot_x + est_distance * math.cos(angle)
                    world_y = self.robot_y + est_distance * math.sin(angle)
                    world_z = 0.5  # 假设地面物体高度

                    # 更新场景图
                    if label not in self.objects:
                        self.objects[label] = []
                    self.objects[label].append({
                        "x": world_x, "y": world_y, "z": world_z,
                        "score": conf
                    })

                    self.get_logger().info(
                        f"检测到 {label} ({conf:.0%}) @ "
                        f"({world_x:.2f}, {world_y:.2f}, {world_z:.2f})")

            self.publish_markers()

        except Exception as e:
            self.get_logger().warn(f"处理失败: {e}")

    def publish_markers(self):
        markers = MarkerArray()
        idx = 0
        for label, instances in self.objects.items():
            if not instances:
                continue
            latest = instances[-1]

            # 文字标记
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "semantic_objects"
            m.id = idx
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = latest["x"]
            m.pose.position.y = latest["y"]
            m.pose.position.z = latest["z"] + 0.3
            m.scale.z = 0.2
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.text = f"{label} ({latest[score]:.0%})"
            markers.markers.append(m)

            # 球体
            s = Marker()
            s.header.frame_id = "map"
            s.header.stamp = self.get_clock().now().to_msg()
            s.ns = "semantic_spheres"
            s.id = idx
            s.type = Marker.SPHERE
            s.action = Marker.ADD
            s.pose.position.x = latest["x"]
            s.pose.position.y = latest["y"]
            s.pose.position.z = latest["z"]
            s.scale.x = s.scale.y = s.scale.z = 0.15
            s.color.r = 0.0
            s.color.g = 1.0
            s.color.b = 0.0
            s.color.a = 0.8
            markers.markers.append(s)
            idx += 1

        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticObjectMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
