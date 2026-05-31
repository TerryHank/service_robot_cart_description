#!/usr/bin/env python3
"""Cartographer + Nav2 + Fake LaserScan"""
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def _get_wsl_ip():
    try: return subprocess.check_output(["hostname", "-I"]).decode().strip().split()[0]
    except: return "127.0.0.1"

def generate_launch_description():
    pkg_share = FindPackageShare("service_robot_cart_description")
    wsl_ip = _get_wsl_ip()
    pkg_path = get_package_share_directory("service_robot_cart_description")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "service_robot_cart_gazebo.urdf.xacro"])
    robot_description = {"robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)}
    nav2_params = pkg_path + "/config/nav2_params.yaml"
    carto_config = pkg_path + "/config/cartographer.lua"
    gz_sim = ExecuteProcess(cmd=["gz", "sim", "-s", "-v", "1", "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf"], output="screen")
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen", parameters=[robot_description])
    spawn = TimerAction(period=8.0, actions=[Node(package="ros_gz_sim", executable="create", arguments=["-name", "service_robot_cart", "-topic", "robot_description", "-x", "0.0", "-y", "0.0", "-z", "0.05"], output="screen")])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist", "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock", "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"], output="screen")
    jsb = TimerAction(period=12.0, actions=[Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "-c", "/controller_manager"], output="screen")])
    diff = TimerAction(period=15.0, actions=[Node(package="controller_manager", executable="spawner", arguments=["diff_drive_controller", "-c", "/controller_manager"], output="screen")])
    twist_bridge = Node(package="service_robot_cart_description", executable="twist_bridge.py", output="screen")
    fake_scan = Node(package="service_robot_cart_description", executable="fake_laser_scan.py", output="screen")
    cartographer = TimerAction(period=20.0, actions=[Node(package="cartographer_ros", executable="cartographer_node", name="cartographer_node", output="screen", arguments=["-configuration_directory", pkg_path + "/config", "-configuration_basename", "cartographer.lua"], remappings=[("odom", "/diff_drive_controller/odom"), ("imu", "/imu"), ("scan", "/scan")])])
    occ_grid = TimerAction(period=22.0, actions=[Node(package="cartographer_ros", executable="cartographer_occupancy_grid_node", name="occupancy_grid_node", output="screen", parameters=[{"resolution": 0.05, "publish_period_sec": 1.0}])])
    nav2 = TimerAction(period=25.0, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource("/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py"), launch_arguments={"params_file": nav2_params, "use_sim_time": "true"}.items())])
    return LaunchDescription([SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib"), SetEnvironmentVariable("GZ_IP", wsl_ip), gz_sim, rsp, spawn, bridge, jsb, diff, twist_bridge, fake_scan, cartographer, occ_grid, nav2])
