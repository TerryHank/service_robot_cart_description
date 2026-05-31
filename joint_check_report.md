# URDF Joint Check Report

Overall status: **PASS**

## Checks
- XML parse: PASS
- Unique links: PASS (24)
- Unique joints: PASS (23)
- Single root link: PASS (`base_footprint`)
- Joint graph cycle check: PASS
- Joint graph connectivity: PASS
- Wheel joint type/axis/origin check: PASS
- Inertial positivity check: PASS

## Joint table

| Joint | Type | Parent | Child | xyz | rpy | axis |
|---|---:|---|---|---|---|---|
| `base_footprint_joint` | `fixed` | `base_footprint` | `base_link` | `0 0 0` | `0 0 0` | `-` |
| `base_to_body_joint` | `fixed` | `base_link` | `body_link` | `-0.35 0 0.98` | `0 0 0` | `-` |
| `tray_1_joint` | `fixed` | `body_link` | `tray_1_link` | `0.59 0 -0.55` | `0 0 0` | `-` |
| `tray_1_surface_frame_joint` | `fixed` | `tray_1_link` | `tray_1_surface_frame` | `0 0 0.08` | `0 0 0` | `-` |
| `tray_2_joint` | `fixed` | `body_link` | `tray_2_link` | `0.59 0 -0.25` | `0 0 0` | `-` |
| `tray_2_surface_frame_joint` | `fixed` | `tray_2_link` | `tray_2_surface_frame` | `0 0 0.08` | `0 0 0` | `-` |
| `tray_3_joint` | `fixed` | `body_link` | `tray_3_link` | `0.59 0 0.05` | `0 0 0` | `-` |
| `tray_3_surface_frame_joint` | `fixed` | `tray_3_link` | `tray_3_surface_frame` | `0 0 0.08` | `0 0 0` | `-` |
| `tray_4_joint` | `fixed` | `body_link` | `tray_4_link` | `0.59 0 0.35` | `0 0 0` | `-` |
| `tray_4_surface_frame_joint` | `fixed` | `tray_4_link` | `tray_4_surface_frame` | `0 0 0.08` | `0 0 0` | `-` |
| `front_left_wheel_joint` | `continuous` | `base_link` | `front_left_wheel_link` | `0.43 0.41 0.16` | `0 0 0` | `0 1 0` |
| `front_right_wheel_joint` | `continuous` | `base_link` | `front_right_wheel_link` | `0.43 -0.41 0.16` | `0 0 0` | `0 1 0` |
| `rear_left_wheel_joint` | `continuous` | `base_link` | `rear_left_wheel_link` | `-0.49 0.41 0.16` | `0 0 0` | `0 1 0` |
| `rear_right_wheel_joint` | `continuous` | `base_link` | `rear_right_wheel_link` | `-0.49 -0.41 0.16` | `0 0 0` | `0 1 0` |
| `body_to_rear_camera_joint` | `fixed` | `body_link` | `rear_camera_link` | `-0.185 0 0.12` | `0 0 3.141593` | `-` |
| `rear_camera_optical_joint` | `fixed` | `rear_camera_link` | `rear_camera_optical_frame` | `0 0 0` | `-1.570796 0 -1.570796` | `-` |
| `body_to_left_side_camera_link_joint` | `fixed` | `body_link` | `left_side_camera_link` | `0 0.285 0.36` | `0 0 1.570796` | `-` |
| `left_side_camera_optical_frame_joint` | `fixed` | `left_side_camera_link` | `left_side_camera_optical_frame` | `0 0 0` | `-1.570796 0 -1.570796` | `-` |
| `body_to_right_side_camera_link_joint` | `fixed` | `body_link` | `right_side_camera_link` | `0 -0.285 0.36` | `0 0 -1.570796` | `-` |
| `right_side_camera_optical_frame_joint` | `fixed` | `right_side_camera_link` | `right_side_camera_optical_frame` | `0 0 0` | `-1.570796 0 -1.570796` | `-` |
| `body_to_top_display_joint` | `fixed` | `body_link` | `top_display_link` | `-0.02 0 0.735` | `0 0 0` | `-` |
| `front_sensor_link_joint` | `fixed` | `base_link` | `front_sensor_link` | `0.605 0 0.22` | `0 0 0` | `-` |
| `rear_sensor_link_joint` | `fixed` | `base_link` | `rear_sensor_link` | `-0.705 0 0.22` | `0 0 3.141593` | `-` |

## 修正说明

- 机体已改为直立标准几何盒体，不再使用上一版的曲面外壳网格。
- 货盘、顶部屏幕、后置/侧向相机均改为 `body_link` 的子关节，避免视觉模块看似在机身上但 TF 却挂在 `base_link` 下。
- 四个轮关节保持 `continuous`，父级为 `base_link`，轴向统一为 `0 1 0`，与机器人坐标系的轮轴方向一致。
- `base_footprint` 是唯一根节点；所有非根 link 都只有一个父 joint。
