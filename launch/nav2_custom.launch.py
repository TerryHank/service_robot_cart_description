#!/usr/bin/env python3
"""Nav2 launch - 排除 collision_monitor"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Controller Server
        Node(package='nav2_controller', executable='controller_server',
            output='screen', parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')]),

        # Smoother Server
        Node(package='nav2_smoother', executable='smoother_server',
            output='screen', parameters=[params_file]),

        # Planner Server
        Node(package='nav2_planner', executable='planner_server',
            output='screen', parameters=[params_file]),

        # Behavior Server
        Node(package='nav2_behaviors', executable='behavior_server',
            output='screen', parameters=[params_file]),

        # BT Navigator
        Node(package='nav2_bt_navigator', executable='bt_navigator',
            output='screen', parameters=[params_file]),

        # Waypoint Follower
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
            output='screen', parameters=[params_file]),

        # Velocity Smoother
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
            output='screen', parameters=[params_file]),

        # Lifecycle Manager
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': ['controller_server',
                                       'smoother_server',
                                       'planner_server',
                                       'behavior_server',
                                       'bt_navigator',
                                       'waypoint_follower',
                                       'velocity_smoother']}]),
    ])
