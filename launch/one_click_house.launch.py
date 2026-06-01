#!/usr/bin/env python3
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def _get_wsl_ip():
    try:
        return subprocess.check_output(["hostname", "-I"]).decode().strip().split()[0]
    except Exception:
        return "127.0.0.1"

def generate_launch_description():
    pkg_share = FindPackageShare("service_robot_cart_description")
    wsl_ip = _get_wsl_ip()
    pkg_path = get_package_share_directory("service_robot_cart_description")
    models_path = pkg_path + "/models"
    nav2_params = pkg_path + "/config/nav2_params.yaml"
    slam_params = pkg_path + "/config/slam_toolbox.yaml"
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "service_robot_cart_gazebo.urdf.xacro"])
    robot_description = {"robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)}
    world_file = PathJoinSubstitution([pkg_share, "worlds", "small_house.world"])
    sim_time = {"use_sim_time": True}

    gz_sim = ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "1", world_file], output="screen")
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen", parameters=[robot_description, sim_time])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=["/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist", "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"], parameters=[sim_time], output="screen")
    clock_bridge = Node(package="service_robot_cart_description", executable="clock_bridge.py", output="screen")
    # Separate bridge for joint velocity control (ROS_TO_GZ direction)
    joint_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", name="joint_bridge",
        arguments=["--ros-args", "-p", "config_file:=/home/lab/ros2_ws/install/service_robot_cart_description/share/service_robot_cart_description/config/joint_bridge.yaml"],
        output="screen", parameters=[sim_time])
    joint_ctrl = Node(package="service_robot_cart_description", executable="joint_vel_controller.py", output="screen", parameters=[sim_time])
    odom_bridge = Node(package="service_robot_cart_description", executable="odom_bridge.py", output="screen", parameters=[sim_time])
    lidar_tf = Node(package="tf2_ros", executable="static_transform_publisher", arguments=["0", "0", "0.15", "0", "0", "0", "base_link", "service_robot_cart/base_link/front_lidar"])

    slam_node = TimerAction(period=20.0, actions=[Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox", output="screen", parameters=[slam_params, sim_time, {"scan_topic": "/scan", "odom_frame": "odom", "base_frame": "base_link", "map_frame": "map"}])])
    slam_activate = TimerAction(period=30.0, actions=[ExecuteProcess(cmd=["ros2", "service", "call", "/slam_toolbox/change_state", "lifecycle_msgs/srv/ChangeState", "{transition: {id: 1}}"], output="screen"), ExecuteProcess(cmd=["ros2", "service", "call", "/slam_toolbox/change_state", "lifecycle_msgs/srv/ChangeState", "{transition: {id: 3}}"], output="screen")])

    nav2_nodes = TimerAction(period=35.0, actions=[Node(package="nav2_controller", executable="controller_server", output="screen", parameters=[nav2_params, sim_time], remappings=[("cmd_vel", "cmd_vel_nav")]), Node(package="nav2_smoother", executable="smoother_server", output="screen", parameters=[nav2_params, sim_time]), Node(package="nav2_planner", executable="planner_server", output="screen", parameters=[nav2_params, sim_time]), Node(package="nav2_behaviors", executable="behavior_server", output="screen", parameters=[nav2_params, sim_time]), Node(package="nav2_bt_navigator", executable="bt_navigator", output="screen", parameters=[nav2_params, sim_time]), Node(package="nav2_waypoint_follower", executable="waypoint_follower", output="screen", parameters=[nav2_params, sim_time]), Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother", output="screen", parameters=[nav2_params, sim_time], remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "/cmd_vel")]), Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation", output="screen", parameters=[sim_time, {"autostart": True, "node_names": ["controller_server", "smoother_server", "planner_server", "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother"]}])])

    frontier_params = pkg_path + "/config/frontier_params.yaml"
    explore = TimerAction(period=50.0, actions=[Node(package="frontier_exploration_ros2", executable="frontier_explorer", name="frontier_explorer", output="screen", parameters=[frontier_params, sim_time, {"robot_base_frame": "base_link", "autostart": True, "mrtsp_solver": "greedy", "escape_enabled": True, "frontier_selection_min_distance": 0.6, "frontier_candidate_min_goal_distance_m": 0.6, "occ_threshold": 65, "min_frontier_size_cells": 5}])])

    return LaunchDescription([SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins"), SetEnvironmentVariable("GZ_IP", wsl_ip), SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models_path), gz_sim, rsp, bridge, clock_bridge, joint_bridge, joint_ctrl, odom_bridge, lidar_tf, slam_node, slam_activate, nav2_nodes, explore])
