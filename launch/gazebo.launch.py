#!/usr/bin/env python3
"""Gazebo server-only (headless) + RViz on Windows via WSLg."""
import os, subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _get_wsl_ip():
    try:
        return subprocess.check_output(["hostname", "-I"]).decode().strip().split()[0]
    except Exception:
        return "127.0.0.1"


def generate_launch_description():
    pkg_share = FindPackageShare("service_robot_cart_description")
    wsl_ip = _get_wsl_ip()

    xacro_file = PathJoinSubstitution([
        pkg_share, "urdf", "service_robot_cart_gazebo.urdf.xacro"
    ])
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]), value_type=str
        )
    }

    rviz_config = PathJoinSubstitution([
        pkg_share, "rviz", "service_robot_cart.rviz"
    ])

    # 1. Gazebo SERVER only (-s), no rendering
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "-v", "4", "empty.sdf"],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-name", "service_robot_cart",
                        "-topic", "robot_description",
                        "-x", "0", "-y", "0", "-z", "0.2"],
            output="screen",
        )],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    jsb = TimerAction(
        period=10.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            output="screen",
        )],
    )

    diff = TimerAction(
        period=12.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "-c", "/controller_manager"],
            output="screen",
        )],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib"),
        SetEnvironmentVariable("GZ_IP", wsl_ip),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([pkg_share, "meshes"])),
        gz_sim,
        rsp,
        spawn,
        bridge,
        jsb,
        diff,
        rviz,
    ])
