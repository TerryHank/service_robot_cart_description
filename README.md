# Service Robot Cart Geometric URDF

This package replaces the curved concept body with a standard straight geometric body made from URDF primitives.

## Coordinate convention

- `+X`: forward, toward the tray/shelf side
- `+Y`: left
- `+Z`: up
- Units: meters

## Joint layout

- Root: `base_footprint`
- Fixed: `base_footprint_joint` → `base_link`
- Fixed: `base_to_body_joint` → `body_link`
- Fixed tray joints are parented to `body_link`.
- Four wheel joints are continuous and parented to `base_link`:
  - `front_left_wheel_joint`
  - `front_right_wheel_joint`
  - `rear_left_wheel_joint`
  - `rear_right_wheel_joint`
- Wheel axes are `0 1 0`, matching the wheel axle direction in the robot frame.
- Camera/display joints are fixed to `body_link`; bumper sensors are fixed to `base_link`.

## ROS 2 quick start

```bash
mkdir -p ~/ros2_ws/src
unzip service_robot_cart_geo_urdf_package.zip -d ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch service_robot_cart_description display.launch.py
```

See `joint_check_report.md` for the generated URDF joint audit.
