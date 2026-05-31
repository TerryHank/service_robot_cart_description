#!/usr/bin/env python3
"""Generate a simplified geometric URDF/GLB package for a service robot cart.

The package intentionally uses URDF primitives for the main geometry so that the
body is straight/standard geometric rather than curved, and so that joint
origins are easy to audit.
"""
from __future__ import annotations

import math
import os
import re
import shutil
import zipfile
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import trimesh
from trimesh.transformations import euler_matrix, rotation_matrix, translation_matrix, concatenate_matrices

OUT_ROOT = Path('/mnt/data/service_robot_cart_geo_urdf_ros2')
PKG_NAME = 'service_robot_cart_description'
PKG_ROOT = OUT_ROOT / PKG_NAME
ZIP_PATH = Path('/mnt/data/service_robot_cart_geo_urdf_package.zip')
URDF_STANDALONE = Path('/mnt/data/service_robot_cart_geo.urdf')
XACRO_STANDALONE = Path('/mnt/data/service_robot_cart_geo.urdf.xacro')
GLB_ROS = Path('/mnt/data/service_robot_cart_geo_urdf_visual.glb')
GLB_YUP = Path('/mnt/data/service_robot_cart_geo_urdf_visual_yup_viewer.glb')
PREVIEW = Path('/mnt/data/service_robot_cart_geo_urdf_preview.png')
PREVIEW_3VIEWS = Path('/mnt/data/service_robot_cart_geo_urdf_preview_3views.png')
REPORT = Path('/mnt/data/service_robot_cart_geo_joint_check_report.md')

# RGBA colors for GLB rendering/export. URDF uses matching normalized colors.
COLORS: Dict[str, Tuple[int, int, int, int]] = {
    'silver': (205, 220, 215, 255),
    'dark_silver': (70, 82, 80, 255),
    'black': (8, 10, 10, 255),
    'rubber': (18, 18, 18, 255),
    'blue': (0, 132, 210, 255),
    'screen': (16, 185, 235, 230),
    'glass': (5, 45, 65, 220),
    'red': (240, 30, 22, 255),
    'white': (235, 245, 242, 255),
}

MATERIAL_RGBA = {
    name: ' '.join(f'{c / 255.0:.3f}'.rstrip('0').rstrip('.') for c in rgba)
    for name, rgba in COLORS.items()
}


def fmt(value: float) -> str:
    if abs(value) < 1e-10:
        return '0'
    return f'{float(value):.6f}'.rstrip('0').rstrip('.')


def xyz_str(v: Iterable[float]) -> str:
    return ' '.join(fmt(x) for x in v)


def tf_from_xyz_rpy(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    return concatenate_matrices(translation_matrix(xyz), euler_matrix(*rpy, axes='sxyz'))


@dataclass
class Primitive:
    kind: str  # box/cylinder
    material: str
    name: str
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    size: Optional[Tuple[float, float, float]] = None
    radius: Optional[float] = None
    length: Optional[float] = None


@dataclass
class LinkDef:
    name: str
    visuals: List[Primitive] = field(default_factory=list)
    collisions: List[Primitive] = field(default_factory=list)
    mass: Optional[float] = None
    inertia: Optional[Tuple[float, float, float]] = None
    inertial_origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class JointDef:
    name: str
    jtype: str
    parent: str
    child: str
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: Optional[Tuple[float, float, float]] = None
    dynamics: Optional[Tuple[float, float]] = None  # damping, friction


@dataclass
class RobotDef:
    links: Dict[str, LinkDef]
    joints: List[JointDef]


def inertia_box(mass: float, sx: float, sy: float, sz: float) -> Tuple[float, float, float]:
    return (
        mass / 12.0 * (sy * sy + sz * sz),
        mass / 12.0 * (sx * sx + sz * sz),
        mass / 12.0 * (sx * sx + sy * sy),
    )


def inertia_cylinder_y(mass: float, radius: float, length: float) -> Tuple[float, float, float]:
    # Solid cylinder with its spin axis along local Y.
    i_axis = 0.5 * mass * radius * radius
    i_perp = mass / 12.0 * (3.0 * radius * radius + length * length)
    return i_perp, i_axis, i_perp


def inertia_cylinder_x(mass: float, radius: float, length: float) -> Tuple[float, float, float]:
    # Solid cylinder with its spin/optical axis along local X.
    i_axis = 0.5 * mass * radius * radius
    i_perp = mass / 12.0 * (3.0 * radius * radius + length * length)
    return i_axis, i_perp, i_perp


def box(name: str, size: Tuple[float, float, float], material: str, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> Primitive:
    return Primitive('box', material, name, tuple(xyz), tuple(rpy), size=tuple(size))


def cyl(name: str, radius: float, length: float, material: str, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> Primitive:
    return Primitive('cylinder', material, name, tuple(xyz), tuple(rpy), radius=radius, length=length)


def add_link(links: Dict[str, LinkDef], link: LinkDef) -> None:
    if link.name in links:
        raise ValueError(f'duplicate link: {link.name}')
    links[link.name] = link


def build_robot() -> RobotDef:
    links: Dict[str, LinkDef] = {}
    joints: List[JointDef] = []

    # Dimensions in meters. Coordinate convention: +X forward/shelf side, +Y left, +Z up.
    base_len, base_width, base_h = 1.20, 0.70, 0.20
    upper_len, upper_width, upper_h = 1.05, 0.64, 0.08
    wheel_radius, wheel_width = 0.16, 0.09
    wheel_front_x, wheel_rear_x, wheel_y, wheel_z = 0.43, -0.49, 0.41, wheel_radius
    body_origin = (-0.35, 0.0, 0.98)
    body_size = (0.32, 0.52, 1.36)
    tray_origin_global = [(0.24, 0.0, z) for z in (0.43, 0.73, 1.03, 1.33)]
    tray_size = (0.82, 0.60, 0.04)

    # Root frame and mobile base.
    add_link(links, LinkDef('base_footprint'))
    base_visuals = [
        box('lower_chassis', (base_len, base_width, base_h), 'silver', xyz=(-0.05, 0.0, 0.18)),
        box('upper_deck', (upper_len, upper_width, upper_h), 'white', xyz=(0.00, 0.0, 0.32)),
        box('front_black_bumper', (0.10, base_width + 0.04, 0.12), 'black', xyz=(0.55, 0.0, 0.16)),
        box('rear_black_bumper', (0.10, base_width + 0.04, 0.12), 'black', xyz=(-0.65, 0.0, 0.16)),
        box('left_side_skirt', (0.96, 0.035, 0.11), 'dark_silver', xyz=(-0.03, 0.372, 0.13)),
        box('right_side_skirt', (0.96, 0.035, 0.11), 'dark_silver', xyz=(-0.03, -0.372, 0.13)),
        box('front_red_indicator_left', (0.025, 0.06, 0.025), 'red', xyz=(0.605, 0.22, 0.22)),
        box('front_red_indicator_right', (0.025, 0.06, 0.025), 'red', xyz=(0.605, -0.22, 0.22)),
        box('rear_red_indicator_left', (0.025, 0.06, 0.025), 'red', xyz=(-0.705, 0.22, 0.22)),
        box('rear_red_indicator_right', (0.025, 0.06, 0.025), 'red', xyz=(-0.705, -0.22, 0.22)),
    ]
    base_collisions = [
        box('lower_chassis_collision', (base_len, base_width, base_h), 'silver', xyz=(-0.05, 0.0, 0.18)),
        box('upper_deck_collision', (upper_len, upper_width, upper_h), 'white', xyz=(0.00, 0.0, 0.32)),
    ]
    add_link(links, LinkDef(
        'base_link',
        visuals=base_visuals,
        collisions=base_collisions,
        mass=24.0,
        inertia=inertia_box(24.0, base_len, base_width, 0.30),
        inertial_origin=(-0.04, 0.0, 0.20),
    ))
    joints.append(JointDef('base_footprint_joint', 'fixed', 'base_footprint', 'base_link'))

    # Straight standard geometric body, no curved shell.
    body_visuals = [
        box('straight_main_body', body_size, 'silver'),
        box('front_dark_service_panel', (0.025, 0.42, 1.08), 'dark_silver', xyz=(0.175, 0.0, -0.02)),
        box('rear_camera_recess_panel', (0.025, 0.30, 0.46), 'black', xyz=(-0.175, 0.0, 0.10)),
        box('left_vertical_trim', (0.34, 0.025, 1.26), 'white', xyz=(0.0, 0.278, 0.0)),
        box('right_vertical_trim', (0.34, 0.025, 1.26), 'white', xyz=(0.0, -0.278, 0.0)),
        box('top_blue_cap', (0.34, 0.54, 0.055), 'blue', xyz=(0.0, 0.0, 0.707)),
    ]
    body_collisions = [box('body_collision', body_size, 'silver')]
    add_link(links, LinkDef(
        'body_link',
        visuals=body_visuals,
        collisions=body_collisions,
        mass=20.0,
        inertia=inertia_box(20.0, *body_size),
    ))
    joints.append(JointDef('base_to_body_joint', 'fixed', 'base_link', 'body_link', xyz=body_origin))

    # Trays are fixed to body_link with origins written relative to body_link.
    tray_mass = 2.0
    tray_inertia = inertia_box(tray_mass, tray_size[0], tray_size[1], 0.08)
    for idx, gxyz in enumerate(tray_origin_global, start=1):
        rel = (gxyz[0] - body_origin[0], gxyz[1] - body_origin[1], gxyz[2] - body_origin[2])
        visuals = [
            box(f'tray_{idx}_deck', tray_size, 'silver'),
            box(f'tray_{idx}_dark_inset', (0.72, 0.50, 0.012), 'dark_silver', xyz=(0.02, 0.0, 0.031)),
            box(f'tray_{idx}_left_rail', (0.82, 0.035, 0.065), 'silver', xyz=(0.0, 0.315, 0.053)),
            box(f'tray_{idx}_right_rail', (0.82, 0.035, 0.065), 'silver', xyz=(0.0, -0.315, 0.053)),
            box(f'tray_{idx}_front_rail', (0.035, 0.60, 0.065), 'silver', xyz=(0.412, 0.0, 0.053)),
            box(f'tray_{idx}_rear_bracket', (0.035, 0.60, 0.12), 'dark_silver', xyz=(-0.412, 0.0, 0.04)),
        ]
        collisions = [box(f'tray_{idx}_collision', (0.84, 0.64, 0.085), 'silver', xyz=(0.0, 0.0, 0.035))]
        lname = f'tray_{idx}_link'
        add_link(links, LinkDef(lname, visuals=visuals, collisions=collisions, mass=tray_mass, inertia=tray_inertia))
        joints.append(JointDef(f'tray_{idx}_joint', 'fixed', 'body_link', lname, xyz=rel))
        surface = f'tray_{idx}_surface_frame'
        add_link(links, LinkDef(surface))
        joints.append(JointDef(f'{surface}_joint', 'fixed', lname, surface, xyz=(0.0, 0.0, 0.080)))

    # Wheel links. Axis +Y gives positive wheel rotation rolling toward +X under the no-slip convention.
    wheel_positions = {
        'front_left_wheel_link': (wheel_front_x, wheel_y, wheel_z),
        'front_right_wheel_link': (wheel_front_x, -wheel_y, wheel_z),
        'rear_left_wheel_link': (wheel_rear_x, wheel_y, wheel_z),
        'rear_right_wheel_link': (wheel_rear_x, -wheel_y, wheel_z),
    }
    wheel_joints = {
        'front_left_wheel_link': 'front_left_wheel_joint',
        'front_right_wheel_link': 'front_right_wheel_joint',
        'rear_left_wheel_link': 'rear_left_wheel_joint',
        'rear_right_wheel_link': 'rear_right_wheel_joint',
    }
    wheel_inertia = inertia_cylinder_y(2.0, wheel_radius, wheel_width)
    for lname, origin in wheel_positions.items():
        visuals = [
            cyl(f'{lname}_rubber_tire', wheel_radius, wheel_width, 'rubber', rpy=(-math.pi / 2.0, 0.0, 0.0)),
            cyl(f'{lname}_silver_hub', 0.115, wheel_width + 0.008, 'silver', rpy=(-math.pi / 2.0, 0.0, 0.0)),
            cyl(f'{lname}_dark_center_cap', 0.052, wheel_width + 0.012, 'dark_silver', rpy=(-math.pi / 2.0, 0.0, 0.0)),
        ]
        collisions = [cyl('wheel_collision', wheel_radius, wheel_width, 'rubber', rpy=(-math.pi / 2.0, 0.0, 0.0))]
        add_link(links, LinkDef(lname, visuals=visuals, collisions=collisions, mass=2.0, inertia=wheel_inertia))
        joints.append(JointDef(wheel_joints[lname], 'continuous', 'base_link', lname, xyz=origin, axis=(0.0, 1.0, 0.0), dynamics=(0.02, 0.01)))

    # Rear-facing camera on the back face of the body. Its camera_link local +X points out of the camera.
    rear_cam_visuals = [
        box('rear_camera_black_panel', (0.035, 0.24, 0.32), 'black'),
        cyl('rear_camera_silver_ring', 0.070, 0.040, 'silver', xyz=(0.025, 0.0, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        cyl('rear_camera_glass_lens', 0.045, 0.044, 'glass', xyz=(0.050, 0.0, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        box('rear_small_scanner_slot', (0.032, 0.16, 0.035), 'black', xyz=(0.02, 0.0, -0.165)),
        box('rear_small_red_line', (0.034, 0.11, 0.010), 'red', xyz=(0.041, 0.0, -0.165)),
    ]
    add_link(links, LinkDef(
        'rear_camera_link',
        visuals=rear_cam_visuals,
        collisions=[box('rear_camera_collision', (0.07, 0.26, 0.36), 'black')],
        mass=0.35,
        inertia=inertia_box(0.35, 0.07, 0.26, 0.36),
    ))
    joints.append(JointDef('body_to_rear_camera_joint', 'fixed', 'body_link', 'rear_camera_link', xyz=(-0.185, 0.0, 0.12), rpy=(0.0, 0.0, math.pi)))
    add_link(links, LinkDef('rear_camera_optical_frame'))
    joints.append(JointDef('rear_camera_optical_joint', 'fixed', 'rear_camera_link', 'rear_camera_optical_frame', rpy=(-math.pi / 2.0, 0.0, -math.pi / 2.0)))

    # Side cameras. Local +X points outward from the side; optical frame follows ROS camera convention.
    for side, y, yaw in [('left', 0.285, math.pi / 2.0), ('right', -0.285, -math.pi / 2.0)]:
        lname = f'{side}_side_camera_link'
        visuals = [
            cyl(f'{side}_side_camera_housing', 0.065, 0.055, 'silver', rpy=(0.0, math.pi / 2.0, 0.0)),
            cyl(f'{side}_side_camera_lens', 0.040, 0.060, 'glass', xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            cyl(f'{side}_side_camera_black_ring', 0.047, 0.064, 'black', xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        ]
        add_link(links, LinkDef(
            lname,
            visuals=visuals,
            collisions=[cyl(f'{lname}_collision', 0.070, 0.070, 'silver', rpy=(0.0, math.pi / 2.0, 0.0))],
            mass=0.25,
            inertia=inertia_cylinder_x(0.25, 0.065, 0.055),
        ))
        joints.append(JointDef(f'body_to_{lname}_joint', 'fixed', 'body_link', lname, xyz=(0.00, y, 0.36), rpy=(0.0, 0.0, yaw)))
        optical = f'{side}_side_camera_optical_frame'
        add_link(links, LinkDef(optical))
        joints.append(JointDef(f'{optical}_joint', 'fixed', lname, optical, rpy=(-math.pi / 2.0, 0.0, -math.pi / 2.0)))

    # Top display, a simple rectangular geometric console.
    display_visuals = [
        box('top_display_black_housing', (0.38, 0.46, 0.055), 'black'),
        box('top_display_blue_bezel', (0.34, 0.42, 0.018), 'blue', xyz=(0.0, 0.0, 0.038)),
        box('top_display_screen', (0.24, 0.31, 0.021), 'screen', xyz=(0.0, 0.0, 0.058)),
        box('top_display_slot', (0.03, 0.22, 0.012), 'black', xyz=(0.16, 0.0, 0.064)),
    ]
    add_link(links, LinkDef(
        'top_display_link',
        visuals=display_visuals,
        collisions=[box('top_display_collision', (0.40, 0.48, 0.08), 'black')],
        mass=0.45,
        inertia=inertia_box(0.45, 0.40, 0.48, 0.08),
    ))
    joints.append(JointDef('body_to_top_display_joint', 'fixed', 'body_link', 'top_display_link', xyz=(-0.02, 0.0, 0.735)))

    # Bumper sensor links. These are fixed to base_link, not body_link.
    for name, x, yaw in [('front_sensor_link', 0.605, 0.0), ('rear_sensor_link', -0.705, math.pi)]:
        add_link(links, LinkDef(
            name,
            visuals=[
                cyl(f'{name}_silver_ring', 0.040, 0.025, 'silver', rpy=(0.0, math.pi / 2.0, 0.0)),
                cyl(f'{name}_glass', 0.026, 0.030, 'glass', xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            ],
            collisions=[cyl(f'{name}_collision', 0.045, 0.035, 'silver', rpy=(0.0, math.pi / 2.0, 0.0))],
            mass=0.10,
            inertia=inertia_cylinder_x(0.10, 0.04, 0.025),
        ))
        joints.append(JointDef(f'{name}_joint', 'fixed', 'base_link', name, xyz=(x, 0.0, 0.22), rpy=(0.0, 0.0, yaw)))

    return RobotDef(links=links, joints=joints)


def material_tags() -> str:
    return '\n'.join(
        f'  <material name="{name}">\n    <color rgba="{MATERIAL_RGBA[name]}"/>\n  </material>'
        for name in COLORS.keys()
    )


def inertial_tag(link: LinkDef) -> str:
    if link.mass is None or link.inertia is None:
        return ''
    ixx, iyy, izz = link.inertia
    return f'''    <inertial>
      <origin xyz="{xyz_str(link.inertial_origin)}" rpy="0 0 0"/>
      <mass value="{fmt(link.mass)}"/>
      <inertia ixx="{fmt(ixx)}" ixy="0" ixz="0" iyy="{fmt(iyy)}" iyz="0" izz="{fmt(izz)}"/>
    </inertial>'''


def primitive_geometry_tag(p: Primitive) -> str:
    if p.kind == 'box':
        return f'<box size="{xyz_str(p.size)}"/>'
    if p.kind == 'cylinder':
        return f'<cylinder radius="{fmt(p.radius)}" length="{fmt(p.length)}"/>'
    raise ValueError(p.kind)


def visual_tag(p: Primitive) -> str:
    return f'''    <visual name="{p.name}">
      <origin xyz="{xyz_str(p.xyz)}" rpy="{xyz_str(p.rpy)}"/>
      <geometry>
        {primitive_geometry_tag(p)}
      </geometry>
      <material name="{p.material}"/>
    </visual>'''


def collision_tag(p: Primitive) -> str:
    return f'''    <collision name="{p.name}">
      <origin xyz="{xyz_str(p.xyz)}" rpy="{xyz_str(p.rpy)}"/>
      <geometry>
        {primitive_geometry_tag(p)}
      </geometry>
    </collision>'''


def joint_tag(j: JointDef) -> str:
    out = [f'  <joint name="{j.name}" type="{j.jtype}">']
    out.append(f'    <parent link="{j.parent}"/>')
    out.append(f'    <child link="{j.child}"/>')
    out.append(f'    <origin xyz="{xyz_str(j.xyz)}" rpy="{xyz_str(j.rpy)}"/>')
    if j.axis is not None:
        out.append(f'    <axis xyz="{xyz_str(j.axis)}"/>')
    if j.dynamics is not None:
        out.append(f'    <dynamics damping="{fmt(j.dynamics[0])}" friction="{fmt(j.dynamics[1])}"/>')
    out.append('  </joint>')
    return '\n'.join(out)


def link_tag(link: LinkDef) -> str:
    parts = [f'  <link name="{link.name}">']
    it = inertial_tag(link)
    if it:
        parts.append(it)
    for v in link.visuals:
        parts.append(visual_tag(v))
    for c in link.collisions:
        parts.append(collision_tag(c))
    parts.append('  </link>')
    return '\n'.join(parts)


def build_urdf(robot: RobotDef, xacro: bool = False) -> str:
    root_attrs = f'name="service_robot_cart_geometric"'
    if xacro:
        root_attrs += ' xmlns:xacro="http://www.ros.org/wiki/xacro"'
    parts = [
        '<?xml version="1.0"?>',
        f'<robot {root_attrs}>',
        '  <!-- Simplified geometric URDF. Units: meters. -->',
        '  <!-- Coordinate convention: +X forward/toward shelf side, +Y left, +Z up. -->',
        material_tags(),
    ]
    # Keep a stable order in the file: root, base, body, trays, wheels, sensors.
    ordered_links = [
        'base_footprint', 'base_link', 'body_link',
        'tray_1_link', 'tray_1_surface_frame',
        'tray_2_link', 'tray_2_surface_frame',
        'tray_3_link', 'tray_3_surface_frame',
        'tray_4_link', 'tray_4_surface_frame',
        'front_left_wheel_link', 'front_right_wheel_link', 'rear_left_wheel_link', 'rear_right_wheel_link',
        'rear_camera_link', 'rear_camera_optical_frame',
        'left_side_camera_link', 'left_side_camera_optical_frame',
        'right_side_camera_link', 'right_side_camera_optical_frame',
        'top_display_link', 'front_sensor_link', 'rear_sensor_link',
    ]
    for name in ordered_links:
        parts.append(link_tag(robot.links[name]))
    for j in robot.joints:
        parts.append(joint_tag(j))
    parts.append('</robot>')
    return '\n\n'.join(parts) + '\n'


def ensure_clean_dir(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def export_package(robot: RobotDef, urdf: str, xacro: str, report: str) -> None:
    ensure_clean_dir(OUT_ROOT)
    for sub in ['urdf', 'launch', 'rviz', 'meshes/visual', 'preview', 'scripts']:
        (PKG_ROOT / sub).mkdir(parents=True, exist_ok=True)

    (PKG_ROOT / 'urdf' / 'service_robot_cart_geometric.urdf').write_text(urdf, encoding='utf-8')
    (PKG_ROOT / 'urdf' / 'service_robot_cart_geometric.urdf.xacro').write_text(xacro, encoding='utf-8')
    URDF_STANDALONE.write_text(urdf, encoding='utf-8')
    XACRO_STANDALONE.write_text(xacro, encoding='utf-8')

    (PKG_ROOT / 'joint_check_report.md').write_text(report, encoding='utf-8')
    REPORT.write_text(report, encoding='utf-8')

    package_xml = f'''<?xml version="1.0"?>
<package format="3">
  <name>{PKG_NAME}</name>
  <version>0.3.0</version>
  <description>Geometric URDF description package for a service robot cart.</description>
  <maintainer email="designer@example.com">ryne</maintainer>
  <license>Proprietary</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
'''
    (PKG_ROOT / 'package.xml').write_text(package_xml, encoding='utf-8')

    cmake = f'''cmake_minimum_required(VERSION 3.8)
project({PKG_NAME})

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes preview rviz urdf
  DESTINATION share/${{PROJECT_NAME}}
)
install(FILES joint_check_report.md
  DESTINATION share/${{PROJECT_NAME}}
)

ament_package()
'''
    (PKG_ROOT / 'CMakeLists.txt').write_text(cmake, encoding='utf-8')

    launch_py = f'''from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('{PKG_NAME}')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'service_robot_cart_geometric.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'service_robot_cart.rviz'])

    robot_description = {{'robot_description': Command(['xacro ', xacro_file])}}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
'''
    (PKG_ROOT / 'launch' / 'display.launch.py').write_text(launch_py, encoding='utf-8')

    rviz = '''Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Alpha: 0.5
      Cell Size: 0.5
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
      Plane: XY
      Reference Frame: base_footprint
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Description Source: Topic
      Description Topic:
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      Value: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_footprint
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3.2
      Focal Point:
        X: 0
        Y: 0
        Z: 0.8
      Pitch: 0.45
      Target Frame: base_footprint
      Yaw: 0.8
Window Geometry:
  Height: 900
  Width: 1200
'''
    (PKG_ROOT / 'rviz' / 'service_robot_cart.rviz').write_text(rviz, encoding='utf-8')

    readme = f'''# Service Robot Cart Geometric URDF

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
ros2 launch {PKG_NAME} display.launch.py
```

See `joint_check_report.md` for the generated URDF joint audit.
'''
    (PKG_ROOT / 'README.md').write_text(readme, encoding='utf-8')

    shutil.copy2(Path(__file__), PKG_ROOT / 'scripts' / 'create_service_robot_cart_geo_urdf.py')


def primitive_mesh(p: Primitive) -> trimesh.Trimesh:
    if p.kind == 'box':
        m = trimesh.creation.box(extents=p.size)
    elif p.kind == 'cylinder':
        m = trimesh.creation.cylinder(radius=p.radius, height=p.length, sections=48)
    else:
        raise ValueError(p.kind)
    m.visual = trimesh.visual.ColorVisuals(mesh=m, face_colors=COLORS[p.material])
    m.apply_transform(tf_from_xyz_rpy(p.xyz, p.rpy))
    return m


def compute_link_world_transforms(robot: RobotDef) -> Dict[str, np.ndarray]:
    child_to_joint: Dict[str, JointDef] = {j.child: j for j in robot.joints}
    children = defaultdict(list)
    for j in robot.joints:
        children[j.parent].append(j)
    roots = [name for name in robot.links if name not in child_to_joint]
    if len(roots) != 1:
        raise RuntimeError(f'expected one root, got {roots}')
    root = roots[0]
    transforms = {root: np.eye(4)}
    queue = deque([root])
    while queue:
        parent = queue.popleft()
        for j in children[parent]:
            transforms[j.child] = transforms[parent] @ tf_from_xyz_rpy(j.xyz, j.rpy)
            queue.append(j.child)
    return transforms


def build_visual_scene(robot: RobotDef, y_up: bool = False) -> Tuple[trimesh.Scene, List[Tuple[trimesh.Trimesh, str]]]:
    transforms = compute_link_world_transforms(robot)
    scene = trimesh.Scene()
    mesh_items: List[Tuple[trimesh.Trimesh, str]] = []
    up_tf = rotation_matrix(-math.pi / 2.0, [1, 0, 0]) if y_up else np.eye(4)
    for lname, link in robot.links.items():
        link_tf = transforms.get(lname, np.eye(4))
        for v in link.visuals:
            m = primitive_mesh(v)
            m.apply_transform(link_tf)
            if y_up:
                m.apply_transform(up_tf)
            name = f'{lname}_{v.name}'
            scene.add_geometry(m, geom_name=name, node_name=name)
            mesh_items.append((m.copy(), v.material))
    return scene, mesh_items


def export_glb_and_previews(robot: RobotDef) -> None:
    scene_ros, mesh_items_ros = build_visual_scene(robot, y_up=False)
    scene_yup, _ = build_visual_scene(robot, y_up=True)
    scene_ros.export(str(GLB_ROS))
    scene_yup.export(str(GLB_YUP))
    shutil.copy2(GLB_ROS, PKG_ROOT / 'meshes' / 'visual' / GLB_ROS.name)
    shutil.copy2(GLB_YUP, PKG_ROOT / 'meshes' / 'visual' / GLB_YUP.name)

    render_preview(mesh_items_ros, PREVIEW, elev=22, azim=-45, title='Geometric URDF visual assembly')
    render_three_views(mesh_items_ros, PREVIEW_3VIEWS)
    shutil.copy2(PREVIEW, PKG_ROOT / 'preview' / PREVIEW.name)
    shutil.copy2(PREVIEW_3VIEWS, PKG_ROOT / 'preview' / PREVIEW_3VIEWS.name)


def axis_equal_3d(ax, bounds: np.ndarray) -> None:
    mins, maxs = bounds[0], bounds[1]
    center = (mins + maxs) / 2.0
    radius = float(np.max(maxs - mins) / 2.0) * 1.08
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(max(0.0, center[2] - radius), center[2] + radius)


def render_preview(mesh_items: List[Tuple[trimesh.Trimesh, str]], path: Path, elev=22, azim=-45, title: Optional[str] = None) -> None:
    all_vertices = np.vstack([m.vertices for m, _ in mesh_items if len(m.vertices)])
    bounds = np.array([all_vertices.min(axis=0), all_vertices.max(axis=0)])
    fig = plt.figure(figsize=(9, 10), dpi=160)
    ax = fig.add_subplot(111, projection='3d')
    # Draw bigger/closer surfaces last by sorting by mean depth from current view approx.
    for m, mat in mesh_items:
        color = np.array(COLORS[mat], dtype=float) / 255.0
        faces = m.vertices[m.faces]
        poly = Poly3DCollection(faces, facecolors=[color], edgecolors=(0, 0, 0, 0.10), linewidths=0.12)
        ax.add_collection3d(poly)
    axis_equal_3d(ax, bounds)
    ax.view_init(elev=elev, azim=azim)
    try:
        ax.set_proj_type('ortho')
    except Exception:
        pass
    ax.set_axis_off()
    if title:
        ax.set_title(title, fontsize=12, pad=8)
    fig.tight_layout(pad=0)
    fig.savefig(path, bbox_inches='tight', pad_inches=0.03)
    plt.close(fig)


def render_three_views(mesh_items: List[Tuple[trimesh.Trimesh, str]], path: Path) -> None:
    all_vertices = np.vstack([m.vertices for m, _ in mesh_items if len(m.vertices)])
    bounds = np.array([all_vertices.min(axis=0), all_vertices.max(axis=0)])
    fig = plt.figure(figsize=(14, 5), dpi=160)
    views = [('Front / +X', 8, -90), ('Side / -Y', 8, 0), ('Rear-Isometric', 22, 135)]
    for i, (title, elev, azim) in enumerate(views, 1):
        ax = fig.add_subplot(1, 3, i, projection='3d')
        for m, mat in mesh_items:
            color = np.array(COLORS[mat], dtype=float) / 255.0
            faces = m.vertices[m.faces]
            poly = Poly3DCollection(faces, facecolors=[color], edgecolors=(0, 0, 0, 0.10), linewidths=0.10)
            ax.add_collection3d(poly)
        axis_equal_3d(ax, bounds)
        ax.view_init(elev=elev, azim=azim)
        try:
            ax.set_proj_type('ortho')
        except Exception:
            pass
        ax.set_axis_off()
        ax.set_title(title, fontsize=10)
    fig.tight_layout(pad=0.2)
    fig.savefig(path, bbox_inches='tight', pad_inches=0.03)
    plt.close(fig)


def parse_float_list(text: str, expected: int = 3) -> List[float]:
    vals = [float(x) for x in text.strip().split()]
    if len(vals) != expected:
        raise ValueError(f'expected {expected} values, got {text}')
    return vals


def validate_urdf(urdf_path: Path, robot: RobotDef) -> Tuple[bool, str]:
    issues: List[str] = []
    warnings: List[str] = []
    notes: List[str] = []

    try:
        xml_root = ET.parse(str(urdf_path)).getroot()
        notes.append('XML parse: PASS')
    except Exception as exc:
        return False, f'# URDF Joint Check Report\n\nXML parse failed: `{exc}`\n'

    links = [e.attrib['name'] for e in xml_root.findall('link')]
    joints = xml_root.findall('joint')
    joint_names = [j.attrib['name'] for j in joints]
    link_set = set(links)

    if len(links) != len(link_set):
        issues.append('Duplicate link names found.')
    else:
        notes.append(f'Unique links: PASS ({len(links)})')
    if len(joint_names) != len(set(joint_names)):
        issues.append('Duplicate joint names found.')
    else:
        notes.append(f'Unique joints: PASS ({len(joint_names)})')

    parents: Dict[str, str] = {}
    children_graph = defaultdict(list)
    joint_rows = []
    for j in joints:
        name = j.attrib['name']
        jtype = j.attrib.get('type', '')
        parent_elem = j.find('parent')
        child_elem = j.find('child')
        origin_elem = j.find('origin')
        axis_elem = j.find('axis')
        parent = parent_elem.attrib.get('link') if parent_elem is not None else None
        child = child_elem.attrib.get('link') if child_elem is not None else None
        if parent not in link_set:
            issues.append(f'Joint `{name}` references missing parent link `{parent}`.')
        if child not in link_set:
            issues.append(f'Joint `{name}` references missing child link `{child}`.')
        if child in parents:
            issues.append(f'Link `{child}` has more than one parent: `{parents[child]}` and `{name}`.')
        elif child:
            parents[child] = name
        if parent and child:
            children_graph[parent].append(child)
        if origin_elem is None:
            warnings.append(f'Joint `{name}` has no origin tag; URDF defaults to identity, but explicit origin is recommended.')
            origin_xyz = [0.0, 0.0, 0.0]
            origin_rpy = [0.0, 0.0, 0.0]
        else:
            try:
                origin_xyz = parse_float_list(origin_elem.attrib.get('xyz', '0 0 0'))
                origin_rpy = parse_float_list(origin_elem.attrib.get('rpy', '0 0 0'))
                if not all(math.isfinite(v) for v in origin_xyz + origin_rpy):
                    issues.append(f'Joint `{name}` origin contains non-finite values.')
            except Exception as exc:
                issues.append(f'Joint `{name}` origin parse error: {exc}')
                origin_xyz, origin_rpy = [0, 0, 0], [0, 0, 0]
        axis_text = ''
        if jtype in {'continuous', 'revolute', 'prismatic'}:
            if axis_elem is None:
                issues.append(f'Movable joint `{name}` has no axis tag.')
                axis = None
            else:
                try:
                    axis = parse_float_list(axis_elem.attrib.get('xyz', '0 0 0'))
                    norm = math.sqrt(sum(v * v for v in axis))
                    if not (0.999 <= norm <= 1.001):
                        issues.append(f'Joint `{name}` axis is not unit length: {axis} norm={norm:.6f}.')
                    axis_text = xyz_str(axis)
                except Exception as exc:
                    issues.append(f'Joint `{name}` axis parse error: {exc}')
                    axis = None
        elif axis_elem is not None:
            warnings.append(f'Fixed joint `{name}` contains an axis tag; it is ignored by URDF consumers.')
        joint_rows.append((name, jtype, parent, child, xyz_str(origin_xyz), xyz_str(origin_rpy), axis_text or '-'))

    roots = [l for l in links if l not in parents]
    if roots == ['base_footprint']:
        notes.append('Single root link: PASS (`base_footprint`)')
    else:
        issues.append(f'Expected single root `base_footprint`, got {roots}.')

    # Connectivity and cycle check.
    visited = set()
    stack = set()
    cycle_found = False

    def dfs(node: str):
        nonlocal cycle_found
        if node in stack:
            cycle_found = True
            return
        if node in visited:
            return
        stack.add(node)
        visited.add(node)
        for c in children_graph[node]:
            dfs(c)
        stack.remove(node)

    if roots:
        dfs(roots[0])
    if cycle_found:
        issues.append('Joint graph contains a cycle.')
    else:
        notes.append('Joint graph cycle check: PASS')
    if set(links) == visited:
        notes.append('Joint graph connectivity: PASS')
    else:
        missing = sorted(set(links) - visited)
        issues.append(f'Unreachable links from root: {missing}.')

    # Wheel-specific checks.
    expected_wheels = {
        'front_left_wheel_joint': ('front_left_wheel_link', (0.43, 0.41, 0.16)),
        'front_right_wheel_joint': ('front_right_wheel_link', (0.43, -0.41, 0.16)),
        'rear_left_wheel_joint': ('rear_left_wheel_link', (-0.49, 0.41, 0.16)),
        'rear_right_wheel_joint': ('rear_right_wheel_link', (-0.49, -0.41, 0.16)),
    }
    joint_lookup = {j.attrib['name']: j for j in joints}
    for name, (child, expected_xyz) in expected_wheels.items():
        j = joint_lookup.get(name)
        if j is None:
            issues.append(f'Missing wheel joint `{name}`.')
            continue
        if j.attrib.get('type') != 'continuous':
            issues.append(f'Wheel joint `{name}` should be continuous, found `{j.attrib.get("type")}`.')
        if j.find('child').attrib.get('link') != child:
            issues.append(f'Wheel joint `{name}` expected child `{child}`.')
        axis = parse_float_list(j.find('axis').attrib.get('xyz', '0 0 0')) if j.find('axis') is not None else []
        if axis != [0.0, 1.0, 0.0]:
            issues.append(f'Wheel joint `{name}` axis expected `0 1 0`, found `{xyz_str(axis)}`.')
        xyz = parse_float_list(j.find('origin').attrib.get('xyz', '0 0 0'))
        if max(abs(xyz[i] - expected_xyz[i]) for i in range(3)) > 1e-6:
            warnings.append(f'Wheel joint `{name}` origin differs from design value: `{xyz_str(xyz)}` vs `{xyz_str(expected_xyz)}`.')
    notes.append('Wheel joint type/axis/origin check: PASS' if not any('Wheel joint' in x or 'Missing wheel' in x for x in issues) else 'Wheel joint type/axis/origin check: see issues')

    # Inertial check for dynamic links (except intentionally massless frames).
    massless_ok = {
        'base_footprint',
        'tray_1_surface_frame', 'tray_2_surface_frame', 'tray_3_surface_frame', 'tray_4_surface_frame',
        'rear_camera_optical_frame', 'left_side_camera_optical_frame', 'right_side_camera_optical_frame',
    }
    for link_elem in xml_root.findall('link'):
        lname = link_elem.attrib['name']
        inertial = link_elem.find('inertial')
        if inertial is None:
            if lname not in massless_ok:
                warnings.append(f'Link `{lname}` has no inertial block. RViz is OK; Gazebo may need inertial.')
            continue
        try:
            mass = float(inertial.find('mass').attrib.get('value', 'nan'))
            inertia = inertial.find('inertia').attrib
            vals = [float(inertia[k]) for k in ('ixx', 'iyy', 'izz')]
            if mass <= 0 or any(v <= 0 for v in vals):
                issues.append(f'Link `{lname}` has non-positive mass/inertia.')
        except Exception as exc:
            issues.append(f'Link `{lname}` inertial parse error: {exc}')
    notes.append('Inertial positivity check: PASS' if not any('inertial' in x or 'mass/inertia' in x for x in issues) else 'Inertial positivity check: see issues')

    # Compose report.
    ok = not issues
    lines = ['# URDF Joint Check Report', '', f'Overall status: **{"PASS" if ok else "CHECK REQUIRED"}**', '']
    lines.append('## Checks')
    for n in notes:
        lines.append(f'- {n}')
    if warnings:
        lines.extend(['', '## Warnings'])
        for w in warnings:
            lines.append(f'- {w}')
    if issues:
        lines.extend(['', '## Issues'])
        for item in issues:
            lines.append(f'- {item}')
    lines.extend(['', '## Joint table', '', '| Joint | Type | Parent | Child | xyz | rpy | axis |', '|---|---:|---|---|---|---|---|'])
    for row in joint_rows:
        lines.append('| ' + ' | '.join(f'`{c}`' for c in row) + ' |')

    # Include concise differences/fixes relative to the previous generated curved package.
    lines.extend([
        '',
        '## 修正说明',
        '',
        '- 机体已改为直立标准几何盒体，不再使用上一版的曲面外壳网格。',
        '- 货盘、顶部屏幕、后置/侧向相机均改为 `body_link` 的子关节，避免视觉模块看似在机身上但 TF 却挂在 `base_link` 下。',
        '- 四个轮关节保持 `continuous`，父级为 `base_link`，轴向统一为 `0 1 0`，与机器人坐标系的轮轴方向一致。',
        '- `base_footprint` 是唯一根节点；所有非根 link 都只有一个父 joint。',
    ])
    return ok, '\n'.join(lines) + '\n'


def zip_package() -> None:
    if ZIP_PATH.exists():
        ZIP_PATH.unlink()
    with zipfile.ZipFile(ZIP_PATH, 'w', compression=zipfile.ZIP_DEFLATED) as zf:
        for p in PKG_ROOT.rglob('*'):
            zf.write(p, p.relative_to(OUT_ROOT))


def main() -> None:
    robot = build_robot()
    urdf = build_urdf(robot, xacro=False)
    xacro = build_urdf(robot, xacro=True)

    # Need a temporary file for validation before package export.
    tmp = Path('/mnt/data/_service_robot_cart_geo_tmp.urdf')
    tmp.write_text(urdf, encoding='utf-8')
    ok, report = validate_urdf(tmp, robot)
    if not ok:
        print(report)
        raise RuntimeError('URDF validation failed')

    export_package(robot, urdf, xacro, report)
    # XML validation for package files.
    ET.parse(str(PKG_ROOT / 'urdf' / 'service_robot_cart_geometric.urdf'))
    ET.parse(str(PKG_ROOT / 'urdf' / 'service_robot_cart_geometric.urdf.xacro'))

    export_glb_and_previews(robot)
    zip_package()
    tmp.unlink(missing_ok=True)

    print(f'URDF: {URDF_STANDALONE}')
    print(f'Xacro: {XACRO_STANDALONE}')
    print(f'GLB ROS: {GLB_ROS}')
    print(f'GLB Y-up: {GLB_YUP}')
    print(f'Preview: {PREVIEW}')
    print(f'3 views: {PREVIEW_3VIEWS}')
    print(f'Report: {REPORT}')
    print(f'Package: {ZIP_PATH}')


if __name__ == '__main__':
    main()
