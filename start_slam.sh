#!/bin/bash
# 一键启动 SLAM + Nav2 仿真
# 用法: bash start_slam.sh [slam算法]
# slam算法: toolbox(默认), cartographer, rtabmap, orbslam3

# 加载 ROS2 环境
. /opt/ros/jazzy/setup.sh
. ~/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
export GZ_IP=$(hostname -I | awk '{print $1}')

SLAM=${1:-toolbox}
PKG_PREFIX=$(ros2 pkg prefix service_robot_cart_description 2>/dev/null)
PKG_PATH=$PKG_PREFIX/share/service_robot_cart_description
XACRO=$PKG_PATH/urdf/service_robot_cart_gazebo.urdf.xacro

echo "========================================="
echo "  SLAM 仿真系统启动中..."
echo "  SLAM 算法: $SLAM"
echo "  PKG: $PKG_PATH"
echo "========================================="

# 生成 URDF 到临时文件
URDF_FILE=/tmp/robot.urdf
xacro "$XACRO" > "$URDF_FILE" 2>/dev/null
if [ ! -s "$URDF_FILE" ]; then
  echo "ERROR: xacro 解析失败"
  exit 1
fi
echo "[验证] URDF OK ($(wc -l < "$URDF_FILE") 行)"

# 创建 RSP 参数文件
cat > /tmp/rsp_params.yaml << EOF
robot_state_publisher:
  ros__parameters:
    robot_description: "$(cat "$URDF_FILE" | tr '\n' ' ' | sed 's/"/\\"/g')"
EOF

# 1. Gazebo
echo "[1/8] 启动 Gazebo..."
gz sim -s -v 1 /opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf &
GZ_PID=$!
sleep 8
kill -0 $GZ_PID 2>/dev/null && echo "  Gazebo OK" || { echo "  Gazebo 失败！"; exit 1; }

# 2. Robot State Publisher
echo "[2/8] 启动 Robot State Publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file /tmp/rsp_params.yaml &
sleep 3

# 3. 等待 robot_description
echo "[3/8] 等待 robot_description..."
for i in $(seq 1 20); do
  if timeout 2 ros2 topic echo /robot_description --once 2>/dev/null | head -1 | grep -q "robot_state"; then
    echo "  robot_description OK"
    break
  fi
  sleep 1
done

# 4. Spawn Robot
echo "[4/8] 生成机器人..."
ros2 run ros_gz_sim create -name service_robot_cart -topic robot_description -x 0.0 -y 0.0 -z 0.05 &
sleep 8

# 5. ROS-GZ Bridge
echo "[5/8] 启动 ROS-GZ Bridge..."
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /imu@sensor_msgs/msg/Imu[gz.msgs.IMU &
sleep 3

# 6. 等待 controller_manager
echo "[6/8] 等待 controller_manager..."
for i in $(seq 1 30); do
  if timeout 2 ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers 2>/dev/null | grep -q "result"; then
    echo "  controller_manager OK"
    break
  fi
  sleep 1
done

# 7. Controllers
echo "[7/8] 启动控制器..."
ros2 run controller_manager spawner joint_state_broadcaster -c /controller_manager &
sleep 3
ros2 run controller_manager spawner diff_drive_controller -c /controller_manager &
sleep 3

# 8. 辅助节点
echo "[8/8] 启动辅助节点..."
ros2 run service_robot_cart_description twist_bridge.py &
ros2 run service_robot_cart_description fake_laser_scan.py &
sleep 2

# 9. SLAM
echo "  启动 SLAM ($SLAM)..."
case $SLAM in
  toolbox)
    ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file $PKG_PATH/config/slam_toolbox.yaml -p use_sim_time:=true &
    ;;
  cartographer)
    ros2 run cartographer_ros cartographer_node -configuration_directory $PKG_PATH/config -configuration_basename cartographer.lua --ros-args -r odom:=/diff_drive_controller/odom -r imu:=/imu -r scan:=/scan &
    ros2 run cartographer_ros cartographer_occupancy_grid_node --ros-args -p resolution:=0.05 -p publish_period_sec:=1.0 &
    ;;
  rtabmap)
    ros2 run rtabmap_slam rtabmap --ros-args --params-file $PKG_PATH/config/rtabmap.yaml -r odom:=/diff_drive_controller/odom -r imu:=/imu -r scan:=/scan -p use_sim_time:=true &
    ;;
  orbslam3)
    ros2 run service_robot_cart_description orbslam3_wrapper.py &
    ;;
esac
sleep 2

# 10. Nav2
echo "  启动 Nav2..."
ros2 launch nav2_bringup navigation_launch.py params_file:=$PKG_PATH/config/nav2_params.yaml use_sim_time:=true &
sleep 5

echo ""
echo "========================================="
echo "  全部启动完成！"
echo "========================================="
echo ""
echo "控制小车: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel"
echo "Foxglove: ws://172.25.191.70:8765"
echo ""
echo "按 Ctrl+C 停止所有进程"

# 等待用户中断
wait $GZ_PID
