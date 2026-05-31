#!/usr/bin/env python3
"""Headless debug: Gazebo server + gz_ros2_control (model plugin)."""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare("service_robot_cart_description")

    xacro_file = PathJoinSubstitution([
        pkg_share, "urdf", "service_robot_cart_gazebo.urdf.xacro"
    ])
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]), value_type=str
        )
    }

    # 1. Gazebo server only (headless, empty world)
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "-v", "4", "empty.sdf"],
        output="screen",
    )

    # 2. Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 3. Spawn robot
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

    # 4. Spawn joint_state_broadcaster
    jsb = TimerAction(
        period=10.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            output="screen",
        )],
    )

    # 5. Spawn diff_drive_controller
    diff = TimerAction(
        period=12.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "-c", "/controller_manager"],
            output="screen",
        )],
    )

    # 6. Bridge cmd_vel
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
        output="screen",
    )

    return LaunchDescription([
        # CRITICAL: tell gz where to find libgz_ros2_control-system.so
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([pkg_share, "meshes"])),
        gz_sim,
        rsp,
        spawn,
        jsb,
        diff,
        bridge,
    ])
