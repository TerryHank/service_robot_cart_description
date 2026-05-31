#!/usr/bin/env python3
"""Gazebo GUI + Foxglove + Twist Bridge"""
import os, subprocess
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
    domain = os.environ.get("ROS_DOMAIN_ID", "42")

    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "service_robot_cart_gazebo.urdf.xacro"])
    robot_description = {"robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)}

    gz_sim = ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "4", "empty.sdf"], output="screen")

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[robot_description])

    spawn = TimerAction(period=3.0, actions=[
        Node(package="ros_gz_sim", executable="create",
             arguments=["-name", "service_robot_cart", "-topic", "robot_description",
                        "-x", "0", "-y", "0", "-z", "0.2"], output="screen")
    ])

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                  arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                             "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                      "/front_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                      "/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",,
                  output="screen")

    jsb = TimerAction(period=8.0, actions=[
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "-c", "/controller_manager"], output="screen")
    ])

    diff = TimerAction(period=10.0, actions=[
        Node(package="controller_manager", executable="spawner",
             arguments=["diff_drive_controller", "-c", "/controller_manager"], output="screen")
    ])

    twist_bridge = Node(package="service_robot_cart_description", executable="twist_bridge.py",
                        output="screen")

    # Foxglove bridge - 显式设置 ROS_DOMAIN_ID
    foxglove = Node(
        package="foxglove_bridge", executable="foxglove_bridge",
        output="screen",
        parameters=[{"port": 8765, "address": "0.0.0.0"}],
        additional_env={"ROS_DOMAIN_ID": domain}
    )

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib"),
        SetEnvironmentVariable("GZ_IP", wsl_ip),
        gz_sim, rsp, spawn, bridge, jsb, diff, twist_bridge, foxglove,
    ])
