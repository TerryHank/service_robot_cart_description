#!/usr/bin/env python3
"""One-click: Gazebo home + odom_bridge + joint_vel + SLAM + Nav2 + frontier explore"""
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
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "service_robot_cart_gazebo.urdf.xacro"])
    robot_description = {"robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)}
    world_file = PathJoinSubstitution([pkg_share, "worlds", "home.world"])
    sim_time = {"use_sim_time": False}

    # ===== Phase 0: Core infra (t=0) =====
    gz_sim = ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "1", world_file], output="screen")
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[robot_description, sim_time])

    # Joint state publisher: publishes default joint states so RSP can
    # compute transforms for continuous joints (wheel links -> base_link)
    jsp = Node(package="joint_state_publisher", executable="joint_state_publisher",
               output="screen", parameters=[{"use_sim_time": False}])

    # Bridge: LiDAR scan + cmd_vel
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                  arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
                  parameters=[sim_time], output="screen")

    # Custom clock bridge: gz sim time -> ROS /clock (fixes ros_gz_bridge wall-time bug)
    clock_bridge = Node(package="service_robot_cart_description",
                         executable="clock_bridge.py", output="screen")

    # Joint velocity bridge: ROS Float64 -> GZ Double
    joint_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                         name="joint_bridge",
                         arguments=["--ros-args", "-p",
                                    "config_file:=" + pkg_path + "/config/joint_bridge.yaml"],
                         output="screen", parameters=[sim_time])

    # Joint velocity controller: /cmd_vel -> wheel joint velocities
    joint_ctrl = Node(package="service_robot_cart_description",
                      executable="joint_vel_controller.py",
                      output="screen", parameters=[sim_time])

    # Odom bridge: Gazebo Pose_V -> ROS /odom + odom->base_link TF
    odom_bridge = Node(package="service_robot_cart_description",
                       executable="odom_bridge.py",
                       output="screen", parameters=[sim_time])

    # LiDAR TF: base_link -> Gazebo-scoped lidar frame
    lidar_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                    arguments=["0", "0", "0.20", "3.141593", "0", "0", "base_link",
                               "service_robot_cart/base_link/front_lidar"])

    # Scan body filter: mask body_link angles from /scan_raw to /scan
    scan_filter = Node(package="service_robot_cart_description",
                       executable="scan_body_filter.py",
                       output="screen")

    # Spawn robot from URDF/xacro (t=5s, after Gazebo ready)
    spawn_robot = TimerAction(period=5.0, actions=[
        Node(package="ros_gz_sim", executable="create",
             arguments=["-name", "service_robot_cart", "-topic", "robot_description",
                        "-x", "0.0", "-y", "0.0", "-z", "0.05"],
             output="screen")
    ])

    # ===== Phase 1: SLAM (t=25s) =====
    # Inline params (YAML file loading is unreliable in ROS2)
    slam_params = {
        "use_sim_time": False,
        "solver_plugin": "solver_plugins::CeresSolver",
        "ceres_loss_function": "HuberLoss",
        "max_iterations": 5,
        "use_online_processing": True,
        "minimum_travel_distance": 0.3,
        "minimum_travel_heading": 0.3,
        "scan_buffer_size": 10,
        "scan_buffer_maximum_scan_distance": 10.0,
        "resolution": 0.05,
        "max_laser_range": 10.0,
        "minimum_scan_amplitude": 0.0,
        "transform_timeout": 2.0,
        "tf_buffer_duration": 30.0,
        "stack_size_to_use": 40000000,
        "use_scan_matching": True,
        "use_scan_barycenter": True,
        "scan_topic": "/scan_filtered",
        "odom_frame": "odom",
        "base_frame": "base_link",
        "map_frame": "map",
        "mode": "mapping",
        "scan_queue_size": 10,
    }

    slam_node = TimerAction(period=25.0, actions=[
        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
             name="slam_toolbox", output="screen",
             parameters=[slam_params])])

    # Lifecycle manager for SLAM auto-activation
    slam_lifecycle = TimerAction(period=30.0, actions=[
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_slam", output="screen",
             parameters=[{"use_sim_time": False, "autostart": True,
                          "node_names": ["slam_toolbox"]}])])

    # ===== Phase 2: Nav2 stack (t=35s) =====
    nav2_nodes = TimerAction(period=35.0, actions=[
        Node(package="nav2_controller", executable="controller_server",
             output="screen", parameters=[nav2_params, sim_time],
             remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_smoother", executable="smoother_server",
             output="screen", parameters=[nav2_params, sim_time]),
        Node(package="nav2_planner", executable="planner_server",
             output="screen", parameters=[nav2_params, sim_time]),
        Node(package="nav2_behaviors", executable="behavior_server",
             output="screen", parameters=[nav2_params, sim_time]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             output="screen", parameters=[nav2_params, sim_time]),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower",
             output="screen", parameters=[nav2_params, sim_time]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen",
             parameters=[nav2_params, sim_time],
             remappings=[("cmd_vel", "cmd_vel_nav"),
                         ("cmd_vel_smoothed", "/cmd_vel")]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[sim_time, {"autostart": True,
                        "node_names": ["controller_server", "smoother_server",
                                       "planner_server", "behavior_server",
                                       "bt_navigator", "waypoint_follower",
                                       "velocity_smoother"]}])])

    # ===== Phase 3: Frontier exploration (t=50s) =====
    frontier_params = {
        "use_sim_time": False,
        "robot_base_frame": "base_link",
        "autostart": True,
        "mrtsp_solver": "greedy",
        "escape_enabled": True,
        "frontier_selection_min_distance": 0.6,
        "frontier_candidate_min_goal_distance_m": 0.6,
        "occ_threshold": 65,
        "min_frontier_size_cells": 5,
        "navigate_to_pose_action_name": "navigate_to_pose",
    }

    explore = TimerAction(period=50.0, actions=[
        Node(package="frontier_exploration_ros2", executable="frontier_explorer",
             name="frontier_explorer", output="screen",
             parameters=[frontier_params])])

    # ===== Launch! =====
    return LaunchDescription([
        # CRITICAL: Do NOT set GZ_SIM_SYSTEM_PLUGIN_PATH with official gz-harmonic!
        # It causes plugin conflicts and physics failure.
        SetEnvironmentVariable("GZ_IP", wsl_ip),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models_path),

        # Phase 0 (t=0)
        gz_sim, rsp, jsp, bridge, clock_bridge, joint_bridge, joint_ctrl, odom_bridge, lidar_tf, scan_filter, spawn_robot,

        # Phase 1 (t=25-30s)
        slam_node, slam_lifecycle,

        # Phase 2 (t=35s)
        nav2_nodes,

        # Phase 3 (t=50s)
        explore,
    ])
