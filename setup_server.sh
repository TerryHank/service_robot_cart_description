#!/bin/bash
set -e
echo "============================================"
echo " Service Robot Cart - Server Setup"
echo "============================================"

WS=~/ros2_ws
SRC=/src

# 1. Create workspace
mkdir -p 
cd 

# 2. Clone this repo if not already
if [ ! -d service_robot_cart_description ]; then
  git clone https://github.com/TerryHank/service_robot_cart_description.git
fi

# 3. Install frontier exploration (if not already)
if [ ! -d /frontier_exploration_ros2 ]; then
  cd 
  git clone https://github.com/nbfields/frontier_exploration_ros2.git -b jazzy 2>/dev/null ||   git clone https://github.com/nbfields/frontier_exploration_ros2.git
fi

# 4. Build
echo "=== Building ==="
source /opt/ros/jazzy/setup.bash
cd 

# Install deps
sudo apt-get update -qq
sudo apt-get install -y -qq   ros-jazzy-nav2-bringup   ros-jazzy-slam-toolbox   ros-jazzy-ros-gz-bridge   ros-jazzy-ros-gz-sim   ros-jazzy-ros2-control   ros-jazzy-ros2-controllers   ros-jazzy-xacro   ros-jazzy-tf2-ros   ros-jazzy-robot-state-publisher   ros-jazzy-joint-state-publisher   ros-jazzy-foxglove-bridge   ros-jazzy-nav2-lifecycle-manager   ros-jazzy-teleop-twist-keyboard   ros-jazzy-rqt-tf-tree   2>&1 | tail -3

colcon build --symlink-install --packages-select service_robot_cart_description frontier_exploration_ros2 2>&1 | tail -10

echo ""
echo "============================================"
echo " SETUP COMPLETE"
echo "============================================"
echo ""
echo " Launch:"
echo "   source /install/setup.bash"
echo "   export ROS_DOMAIN_ID=42"
echo "   ros2 launch service_robot_cart_description one_click_house.launch.py"
