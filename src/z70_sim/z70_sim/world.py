"""MuJoCo 3D physics, dual calibrated cameras, ArUco ID68 wrist marker, Roborock firmware state machine, and HTTP/SSE sidecar."""
from __future__ import annotations

import argparse
import base64
import hashlib
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import queue
import secrets
import struct
import tempfile
import threading
import time
from typing import Any
import urllib.error
import urllib.request
import uuid
import zlib

import cv2
import mujoco
import numpy as np

from ament_index_python.packages import get_package_share_directory
from z70_description import arm_model as hand
from z70_description.arm_model import (
    HAND_STEP_M,
    JOINT_STEP_DEG,
    LIMITS,
    POSES,
    vendor_check,
    vendor_forward,
    vendor_inverse,
)
from z70_driver.driver import app_to_urdf
from z70_manipulation import perception, planner
from z70_manipulation.executor import OBJECT_SIZE_M

# RGB values rendered under ambient=0.82 + directional=0.22 light so OpenCV HSV falls inside config/colors.yaml
COLOR_RGBA = {
    'yellow': (1.0, 0.72, 0.0, 1.0),
    'orange': (1.0, 0.36, 0.0, 1.0),
    'green': (0.0, 0.85, 0.15, 1.0),
    'blue': (0.0, 0.35, 1.0, 1.0),
    'purple': (0.82, 0.12, 0.88, 1.0),
}
DEFAULT_OBJECTS = (
    dict(name='yellow', color='yellow', x=0.68, y=0.0, z=OBJECT_SIZE_M / 2),
    dict(name='purple', color='purple', x=0.72, y=0.14, z=OBJECT_SIZE_M / 2),
    dict(name='blue', color='blue', x=0.66, y=-0.14, z=OBJECT_SIZE_M / 2),
)
WASHING_MACHINE_DROP_M = (-3.85, -0.30)
DEFAULT_GARMENTS = (
    dict(name='sock_living', color='orange', category='sock', shape='box', size=(0.025, 0.030, 0.008), x=-0.125, y=-0.55, z=0.008),
    dict(name='towel_corridor', color='green', category='towel', shape='box', size=(0.030, 0.110, 0.008), x=1.18, y=-0.075, z=0.008),
)
APARTMENT_ROOMS = (
    dict(id=49977281, segment_id=1, name='Corridor'),
    dict(id=49977308, segment_id=2, name='Kitchen'),
    dict(id=49977237, segment_id=4, name='Living room'),
    dict(id=49977266, segment_id=5, name='Bedroom'),
    dict(id=49977297, segment_id=6, name='Bathroom'),
)
_APARTMENT_ORIGIN = (-4.55, -2.15)
_APARTMENT_SHAPE = (132, 231)
_APARTMENT_PIXELS_B64 = (
    'eNrt3L3OgyAYBWCYvjgZJuNkOhkm4tT7v7NPbW2V+gMKAi/nLKYxGp9AUF+wjCFI0PDOIgkzn+YBE8wo0oEJJphRKjNhPsEEk7Ay'
    'D2Y/WoFJqM+CCWZiSkrMbpf5jBdaruVdBnLErNZe1zl/8MdR2q0MO4vL6S+C63RzpjDItsA8pY9YMKvjxKo0YE7OpJllYTD4dmCC'
    'CSaYNzK1mhhZ5hNMMMEE8y5mnzyYmbSmvqyCaqf9JPRT0JZzWSDQtj/7fg8vd+YH52tm7mJueItrZSJuetztTHWJyZJhLlJI0yyZ'
    '7w6cCtM4OtOyM6TMtOjrqTHnVupMZs9cJhEms2UuZxaSYhb0mQM0ZqZyxWRgUmKej2+mOh8wwQQTTDBTYoosmNz3i1gUTFHFzFSZ'
    'MJXKgel5mAWTFlMYrLIkMNKK9/62JcHkG5W9L3NcHJv+UxDbZb6sYIIJJphgXnzhFDkwq+qIycAEE0www420FJjV8X0TTDDBBNNP'
    'OXqTWZG7oYCZGXNer7X8Zj4hZv/Kyfjrqv/+VlcncydMKeW0na+Ovo85lWzbrbO4YSq5+B2CObYY88rUE4S5m8tMKQMwKyHuZnoD'
    '7s4WgQnmSaaUfHafostcuBJknnneuY8pOHfCVJEzbc/ikuklPpnfZ1Wzb/sSZyoWPD6ZzRQqzAeY1JkNOSZfq5XQYw51FL1NG4rM'
    'sU3rumnmPJrMHtqsJjyTr/4H5unTxcqcJhnGXD9P3EpnyYTZj0L1MBARZ46ps1CyPBpTd4JJiMnAJOQEM/k75yeM0W9NhiAO8g+i'
    'mXBS'
)


def _xyaxes(R: np.ndarray) -> str:
    return f'{R[0, 0]:.8f} {R[1, 0]:.8f} {R[2, 0]:.8f} {R[0, 1]:.8f} {R[1, 1]:.8f} {R[2, 1]:.8f}'


def _distortion_maps(width: int, height: int, fx: float, fy: float, cx: float, cy: float, k1: float, k2: float):
    ud, vd = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))
    xd = (ud - cx) / fx
    yd = (vd - cy) / fy
    x, y = xd.copy(), yd.copy()
    for _ in range(10):
        r2 = x * x + y * y
        s = 1.0 + k1 * r2 + k2 * r2 * r2
        x, y = xd / s, yd / s
    return (fx * x + cx).astype(np.float32), (fy * y + cy).astype(np.float32)


def _write_floor_texture(path: Path) -> None:
    if path.is_file():
        return
    rng = np.random.default_rng(42)
    tex = np.full((1024, 1024), 195, dtype=np.uint8)
    for _ in range(4000):
        cx_t, cy_t = rng.integers(5, 1019, size=2)
        rad = int(rng.integers(2, 7))
        val = int(rng.integers(90, 245))
        cv2.circle(tex, (int(cx_t), int(cy_t)), rad, val, -1)
    for i in range(0, 1024, 64):
        cv2.line(tex, (i, 0), (i, 1023), 140, 2)
        cv2.line(tex, (0, i), (1023, i), 140, 2)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), cv2.cvtColor(tex, cv2.COLOR_GRAY2BGR))


def encode_v1_map(
    robot_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
    dock_pose: tuple[float, float] = (-1.2, 0.0),
    width: int = 120,
    height: int = 120,
    origin: tuple[float, float] = (-3.0, -3.0),
    resolution: float = 0.05,
    preset: str = 'two_room',
) -> bytes:
    """Encode a valid Roborock V1 LiDAR map with segmented rooms, walls, robot pose, and dock."""
    if preset == 'apartment':
        origin = _APARTMENT_ORIGIN
        height, width = _APARTMENT_SHAPE
        pixels = np.frombuffer(zlib.decompress(base64.b64decode(_APARTMENT_PIXELS_B64)), dtype=np.uint8).reshape(
            height, width
        )
    else:
        pixels = np.zeros((height, width), dtype=np.uint8)
        # Interior floor: segment 1 (left half: Living Room) and segment 2 (right half: Study)
        # Segment pixel encoding: (segment_id << 3) | 7
        seg1 = (1 << 3) | 7  # 15
        seg2 = (2 << 3) | 7  # 23
        pixels[4 : height - 4, 4 : width // 2] = seg1
        pixels[4 : height - 4, width // 2 : width - 4] = seg2
        # Outer walls: (pixel & 7) == 1
        pixels[3, 3 : width - 3] = 1
        pixels[height - 4, 3 : width - 3] = 1
        pixels[3 : height - 3, 3] = 1
        pixels[3 : height - 3, width - 4] = 1

    def block(kind: int, data: bytes, extra: bytes = b'') -> bytes:
        return struct.pack('<HHI', kind, 8 + len(extra), len(data)) + extra + data

    left_cells = int(round(origin[0] / resolution))
    top_cells = int(round(origin[1] / resolution))
    image_blk = block(2, pixels.tobytes(), struct.pack('<iiII', top_cells, left_cells, height, width))
    rx_mm = int(round(robot_pose[0] * 1000))
    ry_mm = int(round(robot_pose[1] * 1000))
    ryaw_deg = int(round(math.degrees(robot_pose[2])))
    robot_blk = block(8, struct.pack('<iii', rx_mm, ry_mm, ryaw_deg))
    dock_blk = block(1, struct.pack('<ii', int(round(dock_pose[0] * 1000)), int(round(dock_pose[1] * 1000))))
    obstacles = b''.join(struct.pack('<HHIHH', kind, 12, 0, 0, 0) for kind in (12, 15, 16))
    body = image_blk + robot_blk + dock_blk + obstacles + struct.pack('<HHI', 1024, 8, 20)
    header = b'rr' + struct.pack('<HI', 20, len(body) + 20) + bytes(12)
    raw = header + body
    return raw + hashlib.sha1(raw).digest()


class SimWorld:
    """Thread-safe MuJoCo world & Roborock Saros Z70 firmware emulator."""

    def __init__(
        self,
        objects: list[dict[str, Any]] | tuple[dict[str, Any], ...] = DEFAULT_OBJECTS,
        map_preset: str = 'apartment',
    ):
        self.lock = threading.RLock()
        self.map_preset = map_preset
        self.objects_spec = [dict(o) for o in objects]
        desc_share = Path(get_package_share_directory('z70_description'))
        self.mesh_dir = desc_share / 'meshes/cad'
        self.cal, self.cad, self.hand_cfg = hand.CAL, hand.CAD, hand.CONFIG
        self.front_cam = perception.front_camera()
        self.gripper_info = perception.gripper_camera_info()

        scratch = Path(tempfile.gettempdir()) / 'z70_sim'
        self.floor_tex_path = scratch / 'sim_floor_vo.png'
        _write_floor_texture(self.floor_tex_path)

        self.front_maps = _distortion_maps(
            self.front_cam.width,
            self.front_cam.height,
            self.front_cam.f,
            self.front_cam.f,
            self.front_cam.cx,
            self.front_cam.cy,
            self.front_cam.k1,
            self.front_cam.k2,
        )
        g_k = self.gripper_info['camera_matrix']['data']
        g_d = self.gripper_info['distortion_coefficients']['data']
        self.gripper_maps = _distortion_maps(
            int(self.gripper_info['image_width']),
            int(self.gripper_info['image_height']),
            float(g_k[0]),
            float(g_k[4]),
            float(g_k[2]),
            float(g_k[5]),
            float(g_d[0]),
            float(g_d[1]),
        )

        self.xml = self._build_mjcf()
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.scn_option = mujoco.MjvOption()
        self.scn_option.geomgroup[2] = 0
        self.scn_option_marker = mujoco.MjvOption()
        self.scn_option_marker.geomgroup[2] = 1
        self.scn_option_marker.geomgroup[1] = 0
        self._renderer_front: mujoco.Renderer | None = None
        self._renderer_gripper: mujoco.Renderer | None = None
        self._renderer_thread_id: int | None = None
        self.dock = (0.832, 0.041) if self.map_preset == 'apartment' else (-1.2, 0.0)
        self.map_sequence = 1
        self.map_saved_at = int(time.time() * 1000)
        self.reset()

    def _build_mjcf(self) -> str:
        cam = self.front_cam
        R_front_mj = cam.R.T @ np.diag([1.0, -1.0, -1.0])
        front_xy = _xyaxes(R_front_mj)

        C_grip, R_grip_mount = planner.CAD_MOUNT
        R_grip_opt = hand.rpy(R_grip_mount) @ np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        R_grip_mj = R_grip_opt @ np.diag([1.0, -1.0, -1.0])
        grip_xy = _xyaxes(R_grip_mj)
        grip_pos = f'{C_grip[0]} {C_grip[1]} {C_grip[2]}'
        g_k = self.gripper_info['camera_matrix']['data']
        fx_g, fy_g, cx_g, cy_g = float(g_k[0]), float(g_k[4]), float(g_k[2]), float(g_k[5])

        M_pos = f'{hand.MARK_C[0]} {hand.MARK_C[1]} {hand.MARK_C[2]}'
        M_xy = _xyaxes(hand.MARK_R)

        arm_yaw_rad = math.radians(self.cal['arm_yaw_deg'])
        R_vendor = hand.rpy([0.0, 0.0, arm_yaw_rad])
        vendor_xy = _xyaxes(R_vendor)
        vendor_pos = f"{self.cal['arm_origin_x_m']} {self.cal['arm_origin_y_m']} {self.cal['arm_origin_z_m']}"

        sh = self.cad['shoulder_in_vendor_m']
        sh_pos = f'{sh[0]} {sh[1]} {sh[2]}'
        m3_off_rad = math.radians(-self.cal['measured_M3_offset_deg'])
        R_m3 = hand.rpy([0.0, m3_off_rad, 0.0])
        m3_xy = _xyaxes(R_m3)
        R_m4 = hand.rpy([0.0, -m3_off_rad, 0.0])
        m4_xy = _xyaxes(R_m4)
        l1 = float(self.cad['L1_m'])
        l2 = float(self.cad['L2_m'])

        object_bodies = []
        for obj in self.objects_spec:
            rgba = ' '.join(map(str, COLOR_RGBA.get(obj['color'], (1.0, 0.7, 0.0, 1.0))))
            gtype = obj.get('shape', 'cylinder')
            z0 = obj.get('z', self._object_half_z(obj))
            if gtype == 'box':
                sx, sy, sz = obj.get('size', (0.03, 0.03, 0.008))
                geom_xml = f'type="box" size="{sx} {sy} {sz}"'
            else:
                r = float(obj.get('radius', OBJECT_SIZE_M / 2))
                h = float(obj.get('half_height', r))
                geom_xml = f'type="cylinder" size="{r} {h}"'
            object_bodies.append(f"""
    <body name="obj_{obj['name']}" pos="{obj['x']} {obj['y']} {z0}">
      <freejoint name="joint_obj_{obj['name']}"/>
      <inertial pos="0 0 0" mass="0.035" diaginertia="2.5e-5 2.5e-5 2.5e-5"/>
      <geom name="geom_obj_{obj['name']}" {geom_xml} rgba="{rgba}" friction="1.2 0.02 0.005" condim="4"/>
    </body>""")

        return f"""<mujoco model="z70_sim">
  <compiler meshdir="{self.mesh_dir}" angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <visual>
    <global offwidth="1600" offheight="1200"/>
    <map znear="0.001" zfar="5.0"/>
    <headlight ambient="0.80 0.80 0.80" diffuse="0.22 0.22 0.22" specular="0 0 0"/>
  </visual>
  <asset>
    <texture name="floor_tex" type="2d" file="{self.floor_tex_path}"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="2 2" texuniform="true" reflectance="0" specular="0"/>
    <material name="wall_mat" rgba="0.86 0.86 0.84 1" reflectance="0" specular="0"/>
    <material name="body_mat" rgba="0.96 0.96 0.96 1" specular="0.05"/>
    <material name="dark_mat" rgba="0.14 0.14 0.15 1" specular="0.05"/>
    <mesh name="cad_body" file="cad_body.stl"/>
    <mesh name="cad_bumper_front" file="cad_bumper_front.stl"/>
    <mesh name="cad_link1" file="cad_link1.stl"/>
    <mesh name="cad_link2" file="cad_link2.stl"/>
    <mesh name="cad_hand" file="cad_hand.stl"/>
    <mesh name="cad_jaw_left" file="cad_jaw_left.stl"/>
    <mesh name="cad_jaw_right" file="cad_jaw_right.stl"/>
  </asset>
  <worldbody>
    <light pos="0 0 2.5" dir="0 0 -1" diffuse="0.22 0.22 0.22" directional="true" castshadow="false"/>
    <geom name="floor" type="plane" size="8 8 0.05" material="floor_mat" friction="1.0 0.01 0.002"/>
    <geom name="wall_n" type="box" pos="1.0 4.5 0.25" size="6.5 0.05 0.25" material="wall_mat"/>
    <geom name="wall_s" type="box" pos="1.0 -2.8 0.25" size="6.5 0.05 0.25" material="wall_mat"/>
    <geom name="wall_e" type="box" pos="7.2 0.8 0.25" size="0.05 4.0 0.25" material="wall_mat"/>
    <geom name="wall_w" type="box" pos="-4.8 0.8 0.25" size="0.05 4.0 0.25" material="wall_mat"/>

    <body name="base_footprint" pos="0 0 0">
      <inertial pos="0 0 0.045" mass="4.8" diaginertia="0.06 0.06 0.09"/>
      <joint name="base_x" type="slide" axis="1 0 0"/>
      <joint name="base_y" type="slide" axis="0 1 0"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1"/>
      <geom name="chassis_visual" type="mesh" mesh="cad_body" material="body_mat" contype="0" conaffinity="0"/>
      <geom name="bumper_visual" type="mesh" mesh="cad_bumper_front" material="dark_mat" contype="0" conaffinity="0"/>

      <camera name="front" pos="{self.cal['front_camera_x_m']} 0 {cam.h / 1000.0}" xyaxes="{front_xy}"
              resolution="1600 1200" sensorsize="1.6 1.2" focalpixel="{cam.f} {cam.f}"
              principalpixel="{800.0 - cam.cx} {cam.cy - 600.0}"/>

      <body name="arm_vendor_frame" pos="{vendor_pos}" xyaxes="{vendor_xy}">
        <body name="arm_shoulder_link" pos="{sh_pos}">
          <body name="arm_link1" pos="0 0 0" xyaxes="{m3_xy}">
            <inertial pos="{l1 / 2} 0 0" mass="0.18" diaginertia="0.0005 0.0005 0.0002"/>
            <joint name="m3_joint" type="hinge" axis="0 1 0"/>
            <geom name="link1_visual" type="mesh" mesh="cad_link1" material="dark_mat" contype="0" conaffinity="0"/>
            <body name="arm_link2" pos="{l1} 0 0" xyaxes="{m4_xy}">
              <inertial pos="{l2 / 2} 0 0" mass="0.14" diaginertia="0.0003 0.0003 0.0001"/>
              <joint name="m4_joint" type="hinge" axis="0 1 0"/>
              <geom name="link2_visual" type="mesh" mesh="cad_link2" material="dark_mat" contype="0" conaffinity="0"/>
              <body name="arm_wrist_link" pos="{l2} 0 0">
                <inertial pos="0.02 0 0" mass="0.10" diaginertia="0.0001 0.0001 0.0001"/>
                <joint name="m5_joint" type="hinge" axis="-1 0 0"/>
                <geom name="hand_visual" type="mesh" mesh="cad_hand" material="dark_mat" group="1" contype="0" conaffinity="0"/>
                <camera name="gripper" pos="{grip_pos}" xyaxes="{grip_xy}"
                        resolution="1600 896" sensorsize="1.6 0.896" focalpixel="{fx_g} {fy_g}"
                        principalpixel="{800.0 - cx_g} {cy_g - 448.0}"/>
                <body name="jaw_left" pos="0.0515 0.0106 0.0041">
                  <inertial pos="0.015 0 0" mass="0.015" diaginertia="1e-5 1e-5 1e-5"/>
                  <joint name="jaw_left_joint" type="hinge" axis="0 0 1"/>
                  <geom name="jaw_left_visual" type="mesh" mesh="cad_jaw_left" material="dark_mat" group="1" contype="0" conaffinity="0"/>
                </body>
                <body name="jaw_right" pos="0.0515 -0.0101 0.0041">
                  <inertial pos="0.015 0 0" mass="0.015" diaginertia="1e-5 1e-5 1e-5"/>
                  <joint name="jaw_right_joint" type="hinge" axis="0 0 -1"/>
                  <geom name="jaw_right_visual" type="mesh" mesh="cad_jaw_right" material="dark_mat" group="1" contype="0" conaffinity="0"/>
                </body>
                <body name="pincher_marker_68" pos="{M_pos}" xyaxes="{M_xy}">
                  <geom name="m68_border" type="box" pos="0 0 -0.00003" size="0.010 0.010 0.00001" rgba="1 1 1 1" group="2" contype="0" conaffinity="0"/>
                  <geom name="m68_black" type="box" pos="0 0 -0.000015" size="0.0075 0.0075 0.000005" rgba="0.02 0.02 0.02 1" group="2" contype="0" conaffinity="0"/>
                  <geom name="m68_b0" type="box" pos="-0.00375 0.00375 0.0" size="0.00125 0.00125 0.000005" rgba="1 1 1 1" group="2" contype="0" conaffinity="0"/>
                  <geom name="m68_b1" type="box" pos="-0.00125 0.00125 0.0" size="0.00125 0.00125 0.000005" rgba="1 1 1 1" group="2" contype="0" conaffinity="0"/>
                  <geom name="m68_b2" type="box" pos="0.00375 0.00125 0.0" size="0.00125 0.00125 0.000005" rgba="1 1 1 1" group="2" contype="0" conaffinity="0"/>
                  <geom name="m68_b3" type="box" pos="-0.00375 -0.00125 0.0" size="0.00125 0.00125 0.000005" rgba="1 1 1 1" group="2" contype="0" conaffinity="0"/>
                  <geom name="m68_b4" type="box" pos="0.00375 -0.00125 0.0" size="0.00125 0.00125 0.000005" rgba="1 1 1 1" group="2" contype="0" conaffinity="0"/>
                  <geom name="m68_b5" type="box" pos="-0.00375 -0.00375 0.0" size="0.00125 0.00125 0.000005" rgba="1 1 1 1" group="2" contype="0" conaffinity="0"/>
                  <geom name="m68_b6" type="box" pos="-0.00125 -0.00375 0.0" size="0.00125 0.00125 0.000005" rgba="1 1 1 1" group="2" contype="0" conaffinity="0"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
    {''.join(object_bodies)}
  </worldbody>
</mujoco>"""

    def reset(self) -> None:
        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            self.robot_state = 3
            self.error_code = 0
            self.battery = 100
            self.in_cleaning = 0
            self.in_returning = 0
            self.deployed = False
            self.grab_mode = 0
            self.gripped_object = None
            self.gripped_local_offset = np.zeros(3)
            self.camera = 'front'
            self.preview_client = 'none'
            self.preview_status = 0
            self.joints = dict(POSES['IN_ROBOT'])
            self.pose = [0.0, 0.0, 0.0]
            for obj in self.objects_spec:
                self._set_object_pose(obj['name'], [obj['x'], obj['y'], obj.get('z', self._object_half_z(obj))])
            self._sync_kinematics()

    def _joint_qpos_adr(self, name: str) -> int:
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self.model.jnt_qposadr[jid])

    def _set_object_pose(self, name: str, xyz: list[float] | np.ndarray, yaw: float = 0.0) -> None:
        adr = self._joint_qpos_adr(f'joint_obj_{name}')
        self.data.qpos[adr : adr + 3] = xyz
        self.data.qpos[adr + 3 : adr + 7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        vadr = int(self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f'joint_obj_{name}')])
        self.data.qvel[vadr : vadr + 6] = 0.0

    def _get_object_pos(self, name: str) -> np.ndarray:
        adr = self._joint_qpos_adr(f'joint_obj_{name}')
        return self.data.qpos[adr : adr + 3].copy()

    def _active_joints(self) -> tuple[float, float, float, float]:
        if not self.deployed or self.joints['m3'] < LIMITS['m3'][0]:
            return (90.0, 170.0, 0.0, 0.0)
        return (self.joints['m3'], self.joints['m4'], self.joints['m5'], self.joints['m6'])

    def _world_hand_pose(self) -> np.ndarray:
        T_base_hand = hand.hand_pose(self._active_joints())
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        T_world_base = np.array(
            [
                [c, -s, 0.0, x],
                [s, c, 0.0, y],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        return T_world_base @ T_base_hand

    def _sync_kinematics(self) -> None:
        self.data.qpos[self._joint_qpos_adr('base_x')] = self.pose[0]
        self.data.qpos[self._joint_qpos_adr('base_y')] = self.pose[1]
        self.data.qpos[self._joint_qpos_adr('base_yaw')] = self.pose[2]
        q = app_to_urdf(*self._active_joints())
        self.data.qpos[self._joint_qpos_adr('m3_joint')] = q['m3_joint']
        self.data.qpos[self._joint_qpos_adr('m4_joint')] = q['m4_joint']
        self.data.qpos[self._joint_qpos_adr('m5_joint')] = q['m5_joint']
        opening = float(self.hand_cfg['opening_ratio']) * 2.0
        self.data.qpos[self._joint_qpos_adr('jaw_left_joint')] = q['m6_left_joint'] * opening
        self.data.qpos[self._joint_qpos_adr('jaw_right_joint')] = q['m6_right_joint'] * opening

        if self.gripped_object is not None:
            T_wh = self._world_hand_pose()
            world_pt = T_wh[:3, :3] @ self.gripped_local_offset + T_wh[:3, 3]
            world_pt[2] = max(self._object_half_z(self.gripped_object), float(world_pt[2]))
            obj_yaw = float(self.pose[2]) + float(getattr(self, 'gripped_rel_yaw', 0.0))
            self._set_object_pose(self.gripped_object, world_pt, yaw=obj_yaw)

        mujoco.mj_forward(self.model, self.data)

    def common_status(self) -> int:
        action_bits = 3 if self.deployed else 0
        mode_bits = self.grab_mode & 0x3
        return (50178 & ~0xFC0) | (action_bits << 6) | (mode_bits << 10)

    def status_dict(self) -> dict[str, Any]:
        with self.lock:
            cs = self.common_status()
            return dict(
                state=self.robot_state,
                error_code=self.error_code,
                battery=self.battery,
                common_status=cs,
                grab_status=(cs >> 6) & 0xF,
                in_cleaning=bool(self.in_cleaning),
                in_returning=bool(self.in_returning),
            )

    def camera_status_int(self) -> int:
        with self.lock:
            return 44453 if self.camera == 'gripper' else 11685

    def render_jpeg(self, camera: str | None = None) -> tuple[bytes, str, int, int]:
        with self.lock:
            active = camera or self.camera
            self._sync_kinematics()
            tid = threading.get_ident()
            if self._renderer_thread_id != tid:
                if self._renderer_front is not None:
                    try:
                        self._renderer_front.close()
                        self._renderer_gripper.close()  # type: ignore[union-attr]
                    except Exception:
                        pass
                self._renderer_front = mujoco.Renderer(self.model, height=1200, width=1600)
                self._renderer_gripper = mujoco.Renderer(self.model, height=896, width=1600)
                self._renderer_thread_id = tid
            if active == 'front':
                renderer, maps, w, h = self._renderer_front, self.front_maps, 1600, 1200
                opt = self.scn_option_marker if self.deployed else self.scn_option
            else:
                renderer, maps, w, h = self._renderer_gripper, self.gripper_maps, 1600, 896
                opt = self.scn_option
            assert renderer is not None
            renderer.update_scene(self.data, camera=active, scene_option=opt)
            bgr = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            distorted = cv2.remap(bgr, maps[0], maps[1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
            ok, buf = cv2.imencode('.jpg', distorted, [cv2.IMWRITE_JPEG_QUALITY, 94])
            if not ok:
                raise RuntimeError('Failed to encode simulated JPEG frame')
            return buf.tobytes(), active, w, h

    def handle_rpc(self, method: str, params: Any) -> tuple[str, Any]:
        """Execute a Roborock vendor RPC against the simulated state and physics.

        Returns ('ok', result) or ('timeout', None) or ('vendor_error', code).
        """
        with self.lock:
            p = params[0] if isinstance(params, list) and len(params) == 1 else params
            if method == 'get_status':
                return 'ok', [self.status_dict()]
            if method == 'get_camera_status':
                return 'ok', [self.camera_status_int()]
            if method == 'get_room_mapping':
                if self.map_preset == 'apartment':
                    return 'ok', [[r['segment_id'], str(r['id'])] for r in APARTMENT_ROOMS]
                return 'ok', [[1, 10], [2, 11]]
            if method == 'get_homesec_connect_status':
                return 'ok', [dict(status=self.preview_status, client_id=self.preview_client)]
            if method == 'stop_camera_preview':
                self.preview_status = 0
                self.preview_client = 'none'
                return 'ok', ['ok']
            if method == 'app_pick_up_enter_exit':
                if isinstance(p, dict) and isinstance(p.get('flag'), int) and p['flag'] in (0, 1, 2):
                    self.grab_mode = p['flag']
                    self.robot_state = 39 if self.grab_mode in (1, 2) else 3
                    return 'ok', ['ok']
                return 'vendor_error', -10007
            if method == 'app_arm_out_compartment':
                self.deployed = True
                if self.grab_mode == 0:
                    self.grab_mode = 1
                self.robot_state = 39
                self.joints = dict(POSES['WORKING_POS'])
                self._sync_kinematics()
                return 'ok', ['ok']
            if method in ('app_arm_in_compartment', 'app_arm_recover'):
                self.deployed = False
                self.gripped_object = None
                self.joints = dict(POSES['IN_ROBOT'])
                if method == 'app_arm_in_compartment':
                    self.robot_state = 3
                self._sync_kinematics()
                return 'ok', ['ok']
            if method in ('app_open_gripper', 'app_put_down'):
                self.joints['m6'] = 90.0
                self._release_object()
                if method == 'app_put_down' and self.grab_mode == 2:
                    self.joints = dict(POSES['WORKING_POS'])
                self._sync_kinematics()
                return 'ok', ['ok']
            if method == 'app_grip':
                self.joints['m6'] = 0.0
                self._try_grasp_nearest()
                if self.grab_mode == 2:
                    self.joints = dict(POSES['TRANSFER_POS'])
                self._sync_kinematics()
                return 'ok', ['ok']
            if method == 'app_arm_move':
                if not isinstance(p, dict) or not isinstance(p.get('m'), list) or len(p['m']) not in (3, 5):
                    return 'vendor_error', -10007
                m = p['m'][:3]
                cand = dict(self.joints)
                for key, sel in zip(('m3', 'm4', 'm5'), m):
                    direction = 1 if sel == 1 else -1 if sel == 2 else 0
                    if direction:
                        lo, hi = LIMITS[key]
                        cand[key] = max(lo, min(hi, cand[key] + direction * JOINT_STEP_DEG))
                new_x = vendor_forward(cand['m3'], cand['m4'])[1][0]
                cur_x = vendor_forward(self.joints['m3'], self.joints['m4'])[1][0]
                if not (new_x > 0.367 and new_x > cur_x):
                    self.joints.update(cand)
                self._sync_kinematics()
                return 'ok', ['ok']
            if method == 'app_arm_direction_move':
                if not isinstance(p, dict) or p.get('direction') not in (1, 2, 3, 4):
                    return 'vendor_error', -10007
                delta = {1: (0, 1), 2: (0, -1), 3: (1, 0), 4: (-1, 0)}[p['direction']]
                x, y = vendor_forward(self.joints['m3'], self.joints['m4'])[1]
                sol = vendor_inverse(
                    x + delta[0] * HAND_STEP_M,
                    y + delta[1] * HAND_STEP_M,
                    (self.joints['m3'], self.joints['m4']),
                )
                if sol is not None and vendor_check(sol[0], sol[1], self.joints['m5']) == 'SUCCESS':
                    self.joints['m3'], self.joints['m4'] = float(sol[0]), float(sol[1])
                self._sync_kinematics()
                return 'ok', ['ok']
            if method in ('app_rc_start', 'app_rc_stop', 'app_rc_end', 'app_stop_grasp', 'app_stop', 'stop_goto_target'):
                if method == 'stop_goto_target':
                    self.robot_state = 3
                return 'ok', ['ok']
            if method == 'app_rc_move':
                if not isinstance(params, list) or len(params) != 1 or not isinstance(params[0], dict):
                    return 'vendor_error', -10007
                cmd = params[0]
                vel = float(cmd.get('velocity', 0.0))
                omega = float(cmd.get('omega', 0.0))
                dur_s = float(cmd.get('duration', 0.0)) / 1000.0
                yaw_gain = 0.70 if omega >= 0 else 0.65
                fwd_gain = 0.65
                dtheta = omega * yaw_gain * dur_s
                dist = vel * fwd_gain * dur_s
                mid_yaw = self.pose[2] + dtheta * 0.5
                self.pose[0] += dist * math.cos(mid_yaw)
                self.pose[1] += dist * math.sin(mid_yaw)
                self.pose[2] = math.atan2(math.sin(self.pose[2] + dtheta), math.cos(self.pose[2] + dtheta))
                self._sync_kinematics()
                return 'ok', ['ok']
            if method == 'app_goto_target':
                if not isinstance(params, list) or len(params) != 2:
                    return 'vendor_error', -10007
                tx_m, ty_m = float(params[0]) / 1000.0, float(params[1]) / 1000.0
                dx, dy = tx_m - self.pose[0], ty_m - self.pose[1]
                if math.hypot(dx, dy) > 1e-4:
                    self.pose[2] = math.atan2(dy, dx)
                self.pose[0], self.pose[1] = tx_m, ty_m
                self.robot_state = 3
                self.map_sequence += 1
                self.map_saved_at = int(time.time() * 1000)
                self._sync_kinematics()
                return 'ok', ['ok']
            if method == 'app_change_camera':
                flag = p.get('flag') if isinstance(p, dict) else None
                if flag not in (0, 1):
                    return 'vendor_error', -10007
                if flag == 1 and (not self.deployed or self.grab_mode not in (1, 2)):
                    return 'vendor_error', -10002
                self.camera = 'gripper' if flag == 1 else 'front'
                # Firmware switches lens without sending an RPC reply (expected 500ms timeout)
                return 'timeout', None
            return 'ok', ['ok']

    def _try_grasp_nearest(self) -> None:
        if not self.deployed:
            return
        T_wh = self._world_hand_pose()
        jaw_world = T_wh[:3, :3] @ hand.grasp_reference() + T_wh[:3, 3]
        best_name = None
        best_dist = 0.085
        for obj in self.objects_spec:
            pos = self._get_object_pos(obj['name'])
            dist = float(np.linalg.norm(pos - jaw_world))
            if dist < best_dist:
                best_dist = dist
                best_name = obj['name']
        if best_name is not None:
            self.gripped_object = best_name
            pos = self._get_object_pos(best_name)
            self.gripped_local_offset = T_wh[:3, :3].T @ (pos - T_wh[:3, 3])
            adr = self._joint_qpos_adr(f'joint_obj_{best_name}')
            qw, qx, qy, qz = self.data.qpos[adr + 3 : adr + 7]
            cur_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            self.gripped_rel_yaw = cur_yaw - float(self.pose[2])

    def _object_half_z(self, target: str | dict[str, Any]) -> float:
        spec = target if isinstance(target, dict) else next((o for o in self.objects_spec if o['name'] == target), None)
        return float(spec['size'][2]) if spec and spec.get('shape') == 'box' else OBJECT_SIZE_M / 2

    def _release_object(self) -> None:
        if self.gripped_object is None:
            return
        name = self.gripped_object
        pos = self._get_object_pos(name)
        self.gripped_object = None
        half_z = self._object_half_z(name)
        # Settle onto floor or onto a supporting object directly below
        support_z = 0.0
        for other in self.objects_spec:
            if other['name'] == name:
                continue
            opos = self._get_object_pos(other['name'])
            if math.hypot(float(pos[0] - opos[0]), float(pos[1] - opos[1])) < OBJECT_SIZE_M * 0.75:
                if opos[2] < pos[2]:
                    support_z = max(support_z, float(opos[2]) + self._object_half_z(other))
        pos[2] = support_z + half_z
        obj_yaw = float(self.pose[2]) + float(getattr(self, 'gripped_rel_yaw', 0.0))
        self._set_object_pose(name, pos, yaw=obj_yaw)
        for _ in range(25):
            mujoco.mj_step(self.model, self.data)

    def home_rooms(self) -> list[dict[str, Any]]:
        if self.map_preset == 'apartment':
            return [dict(id=r['id'], name=r['name']) for r in APARTMENT_ROOMS]
        return [dict(id=10, name='Living Room'), dict(id=11, name='Study')]

    def raw_map(self, refresh: bool = False) -> tuple[bytes, int, int]:
        with self.lock:
            if refresh:
                self.map_sequence += 1
                self.map_saved_at = int(time.time() * 1000)
            raw = encode_v1_map(
                robot_pose=(self.pose[0], self.pose[1], self.pose[2]),
                dock_pose=self.dock,
                preset=self.map_preset,
            )
            return raw, self.map_sequence, self.map_saved_at


def ensure_token_file(path: str | Path) -> str:
    p = Path(path)
    if p.is_file():
        return p.read_text().strip()
    p.parent.mkdir(parents=True, exist_ok=True)
    token = secrets.token_urlsafe(32)
    p.write_text(token)
    p.chmod(0o600)
    return token


def _pulse_params(method: str, params: Any) -> dict[str, float] | None:
    if method != 'app_rc_move' or not isinstance(params, list) or len(params) != 1:
        return None
    p = params[0]
    if not isinstance(p, dict):
        return None
    try:
        vel = float(p['velocity'])
        omega = float(p['omega'])
        dur = float(p['duration'])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(abs(x) <= 10.0 for x in (vel, omega)) or not (0.0 < dur <= 120000.0):
        return None
    return dict(velocity=vel, omega=omega, duration=dur)


class SimServer:
    """Thread-safe HTTP server implementing the z70_driver private transport and optional /api/ros/* proxy."""

    def __init__(
        self,
        world: SimWorld | None = None,
        host: str = '127.0.0.1',
        port: int = 0,
        token: str = '',
        ros_gateway_origin: str = 'http://127.0.0.1:57020',
    ):
        self.world = world or SimWorld()
        self.host = host
        self.requested_port = port
        self.token = token
        self.ros_gateway_origin = ros_gateway_origin.rstrip('/')
        self.observation_session = str(uuid.uuid4())
        self.events_session = str(uuid.uuid4())
        self.event_sequence = 0
        self.event_lock = threading.Lock()
        self.event_clients: list[queue.Queue[bytes | None]] = []
        self.session_enabled = True
        self.stop_event = threading.Event()
        self.httpd = ThreadingHTTPServer((self.host, self.requested_port), self._make_handler())
        self.httpd.daemon_threads = True
        self._thread: threading.Thread | None = None

    @property
    def port(self) -> int:
        return int(self.httpd.server_port)

    @property
    def origin(self) -> str:
        return f'http://{self.host}:{self.port}'

    def start(self) -> 'SimServer':
        if self._thread is None:
            self._thread = threading.Thread(target=self.httpd.serve_forever, name='z70_sim_http', daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self.stop_event.set()
        with self.event_lock:
            for q in self.event_clients:
                try:
                    q.put_nowait(None)
                except queue.Full:
                    pass
            self.event_clients.clear()
        self.httpd.shutdown()
        self.httpd.server_close()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=5.0)

    def emit_event(
        self,
        *,
        command_id: str,
        attempt_id: str,
        attempt_index: int,
        phase: str,
        operation: str,
        requested_at: int,
        attempted_at: int | None,
        finished_at: int | None,
        dispatch: str,
        response: str,
        error_code: str = 'NONE',
        vendor_code: int | None = None,
        params: dict[str, float] | None = None,
    ) -> None:
        with self.event_lock:
            self.event_sequence += 1
            row: dict[str, Any] = dict(
                sessionId=self.events_session,
                sequence=self.event_sequence,
                commandId=command_id,
                attemptId=attempt_id,
                attemptIndex=attempt_index,
                phase=phase,
                operation=operation,
                requestedAt=requested_at,
                attemptedAt=attempted_at,
                finishedAt=finished_at,
                writeTimeKnown=False,
                dispatch=dispatch,
                response=response,
                errorCode=error_code,
            )
            if vendor_code is not None:
                row['vendorCode'] = vendor_code
            if params is not None:
                row['params'] = params
            line = json.dumps(row).encode('utf-8') + b'\n'
            for q in list(self.event_clients):
                try:
                    q.put_nowait(line)
                except queue.Full:
                    self.event_clients.remove(q)

    def dispatch_rpc(
        self,
        method: str,
        params: Any,
        command_id: str | None = None,
        transport: str = 'local',
    ) -> dict[str, Any]:
        cid = command_id or str(uuid.uuid4())
        aid = str(uuid.uuid4())
        started_at = int(time.time() * 1000)
        pp = _pulse_params(method, params)
        attempted_at: int | None = None
        finished_at: int | None = None

        def emit(phase: str, dispatch: str, response: str, error_code: str = 'NONE',
                 vendor_code: int | None = None, attempt: str = aid) -> None:
            self.emit_event(
                command_id=cid, attempt_id=attempt, attempt_index=0, phase=phase, operation=method,
                requested_at=started_at, attempted_at=attempted_at, finished_at=finished_at,
                dispatch=dispatch, response=response, error_code=error_code,
                vendor_code=vendor_code, params=pp,
            )

        emit('accepted', 'not_sent', 'none', attempt=cid)
        attempted_at = int(time.time() * 1000)
        emit('dispatched', 'unknown', 'none')

        kind, val = self.world.handle_rpc(method, params)
        finished_at = int(time.time() * 1000)
        base = dict(requestId=cid, commandId=cid, attemptId=aid, attemptIndex=0,
                    startedAt=started_at, finishedAt=finished_at, deviceId='sim-z70-duid', transport=transport)

        def finish(dispatch: str, response: str, error_code: str = 'NONE', vendor_code: int | None = None) -> None:
            for phase in ('replied', 'finished'):
                emit(phase, dispatch, response, error_code, vendor_code)

        if kind == 'ok':
            finish('sent', 'received')
            return dict(base, dispatch='sent', response='received', result=val)

        if kind == 'vendor_error':
            vcode = int(val) if isinstance(val, int) else -10007
            finish('sent', 'vendor_error', 'VENDOR_REJECTED', vcode)
            return dict(base, dispatch='sent', response='vendor_error', vendorCode=vcode, error='Robot rejected RPC')

        finish('unknown', 'transport_error', 'TIMEOUT')
        return dict(base, dispatch='unknown', response='transport_error',
                    transportReason='timeout', error='Device transport failed')

    def _make_handler(self):
        sim = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: Any) -> None:
                pass

            def _check_auth(self) -> bool:
                if not sim.token:
                    return True
                if hmac.compare_digest(self.headers.get('Authorization', ''), f'Bearer {sim.token}'):
                    return True
                self._json(403, {'error': 'Private driver bearer authentication required.', 'dispatch': 'not_sent'})
                return False

            def _send(self, status: int, content_type: str, payload: bytes, headers: dict[str, Any] | None = None) -> None:
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(payload)))
                for k, v in (headers or {}).items():
                    self.send_header(k, str(v))
                self.end_headers()
                self.wfile.write(payload)

            def _json(self, status: int, body: Any, headers: dict[str, Any] | None = None) -> None:
                self._send(status, 'application/json; charset=utf-8', json.dumps(body).encode('utf-8'), headers)

            def _read_json(self) -> dict[str, Any]:
                length = int(self.headers.get('Content-Length', '0'))
                raw = self.rfile.read(length) if length > 0 else b'{}'
                return json.loads(raw.decode('utf-8'))

            def _proxy_ros(self, method: str) -> None:
                length = int(self.headers.get('Content-Length', '0'))
                data = self.rfile.read(length) if length > 0 else None
                headers = {'Accept': 'application/json'}
                if data is not None:
                    headers['Content-Type'] = 'application/json'
                if sim.token:
                    headers['Authorization'] = f'Bearer {sim.token}'
                req = urllib.request.Request(
                    sim.ros_gateway_origin + self.path,
                    data=data,
                    headers=headers,
                    method=method,
                )
                try:
                    with urllib.request.urlopen(req, timeout=35.0) as resp:
                        forwarded = {
                            k: v for k, v in resp.headers.items()
                            if k.lower() not in ('transfer-encoding', 'connection', 'content-length', 'content-type')
                        }
                        self._send(resp.status, resp.headers.get('Content-Type', 'application/json'), resp.read(), forwarded)
                except urllib.error.HTTPError as e:
                    self._send(e.code, e.headers.get('Content-Type', 'application/json'), e.read())
                except Exception:
                    self._json(503, {'error': 'ROS_UNAVAILABLE', 'dispatch': 'not_sent'})

            def do_GET(self) -> None:
                if self.path == '/healthz':
                    self._json(200, {'available': not sim.stop_event.is_set(), 'simulator': 'mujoco'})
                    return
                if self.path == '/api/session':
                    self._json(200, {'authenticated': True, 'setupAvailable': False, 'setupSession': False})
                    return
                if self.path.startswith('/api/ros/'):
                    self._proxy_ros('GET')
                    return
                if not self._check_auth():
                    return
                if self.path == '/api/commands/events':
                    q: queue.Queue[bytes | None] = queue.Queue(maxsize=256)
                    with sim.event_lock:
                        sim.event_clients.append(q)
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/x-ndjson')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    try:
                        while not sim.stop_event.is_set():
                            try:
                                item = q.get(timeout=0.4)
                                if item is None:
                                    break
                                self.wfile.write(item)
                                self.wfile.flush()
                            except queue.Empty:
                                self.wfile.write(b'\n')
                                self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        pass
                    finally:
                        with sim.event_lock:
                            if q in sim.event_clients:
                                sim.event_clients.remove(q)
                    return

                if self.path == '/api/state':
                    now = int(time.time() * 1000)
                    status_val = sim.world.status_dict()
                    cam_val = sim.world.camera_status_int()
                    self._json(
                        200,
                        dict(
                            observationSession=sim.observation_session,
                            device=dict(
                                id='sim-z70-duid',
                                name='Roborock Saros Z70 (MuJoCo Sim)',
                                model='roborock.vacuum.a179',
                                cloudProtocolVersion='L01',
                                localProtocol='L01',
                            ),
                            connection=dict(state='connected', protocol='L01', connectedAt=now),
                            telemetry=dict(
                                status=dict(value=status_val, observedAt=now, requestedAt=now, error=None),
                                camera=dict(value=[cam_val], observedAt=now, requestedAt=now, error=None),
                            ),
                            api=dict(
                                rpc=None,
                                commandOwner='ROS',
                                rosRawRpc='/z70/driver/raw_rpc',
                                genericVendorRpc=True,
                            ),
                            camera=dict(
                                configured=True,
                                pinCached=True,
                                feedLensVerified=True,
                                acquisition=dict(available=True, stage='connected', failure='none'),
                                streamDiagnostics=None,
                                sourceAudit=dict(available=True, matches=True),
                            ),
                        ),
                    )
                    return

                if self.path == '/api/home/rooms':
                    self._json(
                        200,
                        dict(
                            source='cached',
                            rooms=sim.world.home_rooms(),
                        ),
                    )
                    return

                if self.path == '/api/map/cache/raw':
                    raw, seq, saved_at = sim.world.raw_map(refresh=False)
                    self._send(
                        200,
                        'application/octet-stream',
                        raw,
                        {
                            'X-Roboclaw-Observation-Session': sim.observation_session,
                            'X-Map-Saved-At': saved_at,
                            'X-Map-Index': 0,
                            'X-Map-Sequence': seq,
                        },
                    )
                    return

                self._json(404, {'error': 'Not found.'})

            def do_DELETE(self) -> None:
                if self.path == '/api/session':
                    self._send(204, 'text/plain', b'')
                    return
                self._json(404, {'error': 'Not found.'})

            def do_POST(self) -> None:
                if self.path == '/api/session':
                    self._json(200, {'authenticated': True, 'setupAvailable': False, 'setupSession': False})
                    return
                if self.path.startswith('/api/ros/'):
                    self._proxy_ros('POST')
                    return
                if not self._check_auth():
                    return
                body = self._read_json()

                if self.path == '/api/driver/control/session':
                    enabled = bool(body.get('enabled', True))
                    sim.session_enabled = enabled
                    if not enabled:
                        sim.world.preview_status = 0
                        sim.world.preview_client = 'none'
                    self._json(200, {'enabled': sim.session_enabled, 'streams_released': not sim.session_enabled})
                    return

                if self.path == '/api/driver/rpc':
                    method = str(body.get('method', ''))
                    params = body.get('params', [])
                    command_id = body.get('commandId')
                    transport = str(body.get('transport', 'local'))
                    outcome = sim.dispatch_rpc(method, params, command_id=command_id, transport=transport)
                    self._json(200 if outcome['response'] == 'received' else 502, outcome)
                    return

                if self.path == '/api/camera/frame':
                    if not sim.session_enabled:
                        self._json(409, {'error': 'SESSION_STOPPED', 'dispatch': 'not_sent'})
                        return
                    req_cam = body.get('camera')
                    if req_cam is not None and req_cam not in ('front', 'gripper'):
                        self._json(400, {'error': 'INVALID_INPUT', 'dispatch': 'not_sent'})
                        return
                    jpeg, active, w, h = sim.world.render_jpeg(req_cam)
                    self._send(
                        200,
                        'image/jpeg',
                        jpeg,
                        {
                            'X-Roboclaw-Frame-Id': uuid.uuid4(),
                            'X-Roboclaw-Camera-Session': sim.observation_session,
                            'X-Roboclaw-Received-At': int(time.time() * 1000),
                            'X-Roboclaw-Camera': active,
                            'X-Roboclaw-Size-Camera': active,
                            'X-Roboclaw-Image-Width': w,
                            'X-Roboclaw-Image-Height': h,
                        },
                    )
                    return

                if self.path == '/api/driver/camera/select':
                    target = body.get('camera')
                    if target not in ('front', 'gripper'):
                        self._json(400, {'error': 'Unknown camera name.', 'dispatch': 'not_sent'})
                        return
                    outcomes = []
                    if sim.world.camera != target:
                        flag = 1 if target == 'gripper' else 0
                        out = sim.dispatch_rpc('app_change_camera', {'flag': flag})
                        outcomes.append({'method': 'app_change_camera', 'outcome': out})
                        if out['response'] == 'vendor_error':
                            self._json(
                                409,
                                {
                                    'error': 'Robot refused camera selection; gripper needs pickup mode with arm out.',
                                    'dispatch': 'sent',
                                },
                            )
                            return
                    self._json(200, {'camera': sim.world.camera, 'outcomes': outcomes})
                    return

                if self.path == '/api/map/cache/refresh':
                    raw, seq, saved_at = sim.world.raw_map(refresh=True)
                    self._json(
                        200,
                        dict(cached=True, savedAt=saved_at, mapIndex=0, mapSequence=seq, bytes=len(raw)),
                    )
                    return

                self._json(404, {'error': 'Not found.'})

        return Handler


def create_sim_server(
    world: SimWorld | None = None,
    host: str = '127.0.0.1',
    port: int = 0,
    token: str = '',
) -> SimServer:
    return SimServer(world=world, host=host, port=port, token=token).start()


def main() -> None:
    parser = argparse.ArgumentParser(description='MuJoCo Roborock Z70 simulation transport sidecar.')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=57010)
    parser.add_argument('--token-file', default='')
    parser.add_argument('--ros-gateway-origin', default='http://127.0.0.1:57020')
    args = parser.parse_args()

    token = ensure_token_file(args.token_file) if args.token_file else ''

    server = SimServer(host=args.host, port=args.port, token=token, ros_gateway_origin=args.ros_gateway_origin)
    server.start()
    print(f'z70_sim MuJoCo sidecar listening on {server.origin}')
    try:
        while not server.stop_event.wait(1.0):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == '__main__':
    main()
