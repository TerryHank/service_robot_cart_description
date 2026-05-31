import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare("service_robot_cart_description")

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    use_teleop_arg = DeclareLaunchArgument("use_teleop", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    use_rviz = LaunchConfiguration("use_rviz")
    use_teleop = LaunchConfiguration("use_teleop")
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_file = PathJoinSubstitution([
        pkg_share, "urdf", "service_robot_cart.urdf.xacro"
    ])
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]), value_type=str
        )
    }

    controller_config = os.path.join(
        get_package_share_directory("service_robot_cart_description"),
        "config", "controllers.yaml"
    )

    rviz_config = PathJoinSubstitution([
        pkg_share, "rviz", "service_robot_cart.rviz"
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config, robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel")],
        condition=IfCondition(use_teleop),
    )

    return LaunchDescription([
        use_rviz_arg,
        use_teleop_arg,
        use_sim_time_arg,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        rviz_node,
        teleop_node,
    ])
