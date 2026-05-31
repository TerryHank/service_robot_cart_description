#!/usr/bin/env python3
import os
"""SLAM Toolbox + Nav2 + Fake LaserScan (headless Gazebo)"""
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

    gz_sim = ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "1", "--gui",
        pkg_path + "/worlds/small_house.world"], output="screen")
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", parameters=[robot_description])
    spawn = TimerAction(period=8.0, actions=[
        Node(package="ros_gz_sim", executable="create",
            arguments=["-name", "service_robot_cart", "-topic", "robot_description",
                       "-x", "-2.0", "-y", "0.0", "-z", "0.05"], output="screen")])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                   "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                   "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"], output="screen")
    jsb = TimerAction(period=25.0, actions=[
        Node(package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "30"], output="screen")])
    diff = TimerAction(period=30.0, actions=[
        Node(package="controller_manager", executable="spawner",
            arguments=["diff_drive_controller", "-c", "/controller_manager", "--controller-manager-timeout", "30"], output="screen")])
    twist_bridge = Node(package="service_robot_cart_description", executable="twist_bridge.py", output="screen")
    fake_scan = Node(package="service_robot_cart_description", executable="fake_laser_scan.py",
        output="screen", parameters=[{"use_sim_time": True}])

    # SLAM with inline params and odom remapping
    slam = TimerAction(period=18.0, actions=[
        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
            name="slam_toolbox", output="screen",
            parameters=[{
                "use_sim_time": True,
                "solver_plugin": "solver_plugins::CeresSolver",
                "max_iterations": 5,
                "use_online_processing": True,
                "minimum_travel_distance": 0.3,
                "minimum_travel_heading": 0.3,
                "scan_buffer_size": 10,
                "scan_buffer_maximum_scan_distance": 10.0,
                "resolution": 0.05,
                "max_laser_range": 10.0,
                "minimum_scan_amplitude": 0.0,
                "transform_timeout": 0.2,
                "tf_buffer_duration": 10.0,
                "stack_size_to_use": 40000000,
                "use_scan_matching": True,
                "use_scan_barycenter": True,
                "scan_topic": "/scan",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "map_frame": "map",
                "mode": "mapping",
                "scan_queue_size": 10,
            }],
            remappings=[("/odom", "/diff_drive_controller/odom")])])

    slam_lifecycle = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_slam", output="screen",
        parameters=[{"use_sim_time": True, "autostart": True,
                     "node_names": ["slam_toolbox"]}])

    nav2 = TimerAction(period=22.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pkg_path + "/launch/nav2_custom.launch.py"),
            launch_arguments={"params_file": nav2_params, "use_sim_time": "true"}.items())])

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib"),
        SetEnvironmentVariable("GZ_IP", wsl_ip),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", pkg_path + "/models:" + (os.environ.get("GZ_SIM_RESOURCE_PATH", "") or "")),
        gz_sim, rsp, spawn, bridge, jsb, diff, twist_bridge, fake_scan,
        slam, slam_lifecycle, nav2,
    ])
