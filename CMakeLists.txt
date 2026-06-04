cmake_minimum_required(VERSION 3.8)
project(service_robot_cart_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/twist_bridge.py scripts/fake_laser_scan.py scripts/semantic_segmentation.py scripts/orbslam3_wrapper.py scripts/random_explorer.py scripts/frontier_explorer.py scripts/publish_robot_description.py scripts/odom_tf_publisher.py scripts/clock_bridge.py scripts/odom_bridge.py scripts/scan_body_filter.py scripts/joint_vel_controller.py scripts/random_explorer.py scripts/frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
