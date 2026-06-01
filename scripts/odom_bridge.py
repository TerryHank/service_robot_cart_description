#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

class OdomBridge(Node):
    def __init__(self):
        super().__init__("odom_bridge")
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_pos = None; self.last_time = None; self.last_ori = None
        self.pose_data = None
        self.gz = GzNode()
        # Try world/empty first, fall back to world/default
        self.sub_count = 0
        r1 = self.gz.subscribe(Pose_V, "/world/default/pose/info", self.cb)
        r2 = self.gz.subscribe(Pose_V, "/world/empty/pose/info", self.cb)
        self.get_logger().info(f"odom_bridge: sub default={r1}, sub empty={r2}")

    def cb(self, msg: Pose_V):
        for p in msg.pose:
            if p.name == "service_robot_cart":
                self.pose_data = (p.position.x, p.position.y, p.position.z,
                                  p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w)
                break

    def spin_loop(self):
        while rclpy.ok():
            time.sleep(0.01)
            if self.pose_data is not None:
                x,y,z,qx,qy,qz,qw = self.pose_data
                self.pose_data = None
                now = self.get_clock().now()
                twist = Twist()
                if self.last_time is not None:
                    dt_ns = (now - self.last_time).nanoseconds
                    dt = dt_ns * 1e-9
                    if dt > 0.001 and dt < 1.0:
                        px,py,pz = self.last_pos
                        twist.linear.x = (x-px)/dt; twist.linear.y = (y-py)/dt
                        pw,ppx,ppy,ppz = self.last_ori
                        dqw = qw*pw+qx*ppx+qy*ppy+qz*ppz
                        dqx = -qx*pw+qw*ppx-qz*ppy+qy*ppz
                        dqy = -qy*pw+qz*ppx+qw*ppy-qx*ppz
                        dqz = -qz*pw-qy*ppx+qx*ppy+qw*ppz
                        siny = 2.0*(dqw*dqz+dqx*dqy)
                        cosy = 1.0-2.0*(dqy*dqy+dqz*dqz)
                        twist.angular.z = math.atan2(siny,cosy)/dt
                odom = Odometry()
                odom.header.stamp = now.to_msg()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.position.z = z
                odom.pose.pose.orientation.x = qx
                odom.pose.pose.orientation.y = qy
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw
                odom.twist.twist = twist
                self.odom_pub.publish(odom)
                tf = TransformStamped()
                tf.header.stamp = now.to_msg()
                tf.header.frame_id = "odom"
                tf.child_frame_id = "base_link"
                tf.transform.translation.x = x
                tf.transform.translation.y = y
                tf.transform.translation.z = z
                tf.transform.rotation.x = qx
                tf.transform.rotation.y = qy
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(tf)
                self.last_pos = (x,y,z)
                self.last_ori = (qw,qx,qy,qz)
                self.last_time = now
            rclpy.spin_once(self, timeout_sec=0.0)

def main():
    rclpy.init()
    node = OdomBridge()
    try:
        node.spin_loop()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
