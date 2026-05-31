#!/usr/bin/env python3
"""一键启动：Gazebo GUI + 机器人 + 控制器 + 前置摄像头"""
import subprocess
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

    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "service_robot_cart_gazebo.urdf.xacro"])
    robot_description = {"robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)}
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "service_robot_cart.rviz"])

    # Gazebo GUI
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "4", "empty.sdf"],
        output="screen",
    )

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[robot_description])

    spawn = TimerAction(period=3.0, actions=[
        Node(package="ros_gz_sim", executable="create",
             arguments=["-name", "service_robot_cart", "-topic", "robot_description",
                        "-x", "0", "-y", "0", "-z", "0.2"], output="screen")
    ])

    # Bridge: cmd_vel, clock, front camera image + camera_info
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                  arguments=[
                      "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                      "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                      "/front_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                      "/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                  ],
                  output="screen")

    jsb = TimerAction(period=8.0, actions=[
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "-c", "/controller_manager"], output="screen")
    ])

    diff = TimerAction(period=10.0, actions=[
        Node(package="controller_manager", executable="spawner",
             arguments=["diff_drive_controller", "-c", "/controller_manager"], output="screen")
    ])

    rviz = Node(package="rviz2", executable="rviz2", name="rviz2",
                output="screen", arguments=["-d", rviz_config])

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib"),
        SetEnvironmentVariable("GZ_IP", wsl_ip),
        gz_sim, rsp, spawn, bridge, jsb, diff, rviz,
    ])
