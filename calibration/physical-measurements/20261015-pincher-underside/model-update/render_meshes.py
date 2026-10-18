#!/usr/bin/env python3
"""Offline orthographic views of actual URDF triangles, no robot/GUI access."""
import itertools
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import xacro

ROOT = Path(__file__).resolve().parents[4]
TEST = ROOT / 'ros2/src/z70_description/test'
sys.path.insert(0, str(TEST))
from test_physical_visuals import SOURCE, origin, relative_frames
from test_pincher_mesh import triangles

OUT = Path(__file__).resolve().parent
ROBOT = ET.fromstring(xacro.process_file(str(SOURCE)).toxml())


def box_triangles(size):
    p = np.array(list(itertools.product([-1, 1], repeat=3)))*size/2
    return p[np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5],
                      [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
                      [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])]


def assembly(opening):
    frames = relative_frames(ROBOT, 'arm_link2', {'m6_left_joint': opening, 'm6_right_joint': opening})
    surfaces = []
    for name in ['arm_link2', 'arm_wrist_link', 'gripper_camera_link', 'finger_left', 'finger_right', 'pincher_marker_68']:
        for visual in ROBOT.find(f"link[@name='{name}']").findall('visual'):
            mesh, box = visual.find('geometry/mesh'), visual.find('geometry/box')
            if mesh is not None:
                local = triangles(SOURCE.parents[1] / mesh.get('filename').split('package://z70_description/')[1])
            else:
                local = box_triangles(np.fromstring(box.get('size'), sep=' '))
            transform = frames[name] @ origin(visual.find('origin'))
            p = local @ transform[:3, :3].T + transform[:3, 3]
            color = {'pincher_marker_68': (220, 220, 216) if box.get('size').startswith('0.02 ') else (30, 32, 35),
                     'gripper_camera_link': (20, 25, 30)}.get(name, (62, 67, 73)) if box is not None else ((20, 25, 30) if name == 'gripper_camera_link' else (62, 67, 73))
            surfaces.extend((face, color) for face in p)
    return surfaces, frames


def panel(surfaces, direction, title, center, scale=6500):
    toward = np.array(direction, float)
    toward /= np.linalg.norm(toward)
    right = np.cross([0, 0, 1], toward)
    if np.linalg.norm(right) < .01:
        right = np.array([0, 1, 0.])
    right /= np.linalg.norm(right)
    up = np.cross(toward, right)
    image = np.full((650, 950, 3), (237, 239, 241), np.uint8)
    depth_buffer = np.full(image.shape[:2], -np.inf)
    for face, base in surfaces:
        # Keep all triangle faces; depth-sort actual concave pocket geometry.
        p = face-center
        xy = np.column_stack([p @ right, -p @ up])*scale + [475, 335]
        normal = np.cross(face[1]-face[0], face[2]-face[0])
        normal /= np.linalg.norm(normal)
        light = np.array([.6, -.4, .9]); light /= np.linalg.norm(light)
        intensity = .48 + .52*abs(normal @ light)
        color = tuple(int(v*intensity) for v in base)
        low = np.maximum(np.floor(xy.min(axis=0)).astype(int), [0, 0])
        high = np.minimum(np.ceil(xy.max(axis=0)).astype(int), [949, 649])
        if np.any(high < low):
            continue
        xx, yy = np.meshgrid(np.arange(low[0], high[0]+1), np.arange(low[1], high[1]+1))
        a, b, c = xy
        denominator = (b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if abs(denominator) < 1e-8:
            continue
        u = ((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/denominator
        v = ((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/denominator
        w = 1-u-v
        depth = (p @ toward) @ np.array([u, v, w]).reshape(3, -1)
        depth = depth.reshape(xx.shape)
        old = depth_buffer[low[1]:high[1]+1, low[0]:high[0]+1]
        mask = (u >= -1e-7) & (v >= -1e-7) & (w >= -1e-7) & (depth > old)
        old[mask] = depth[mask]
        image[low[1]:high[1]+1, low[0]:high[0]+1][mask] = color
    cv2.putText(image, title, (24, 35), cv2.FONT_HERSHEY_SIMPLEX, .7, (30, 35, 40), 2, cv2.LINE_AA)
    cv2.putText(image, 'Actual STL triangles | illustrative depth/hidden anatomy | metres, not photo scale', (24, 620), cv2.FONT_HERSHEY_SIMPLEX, .43, (65, 70, 75), 1, cv2.LINE_AA)
    cv2.line(image, (760, 575), (825, 575), (30, 35, 40), 2)
    cv2.putText(image, '10 mm', (760, 600), cv2.FONT_HERSHEY_SIMPLEX, .5, (30, 35, 40), 1, cv2.LINE_AA)
    return image


if __name__ == '__main__':
    surfaces, frames = assembly(np.pi/4)
    center = np.array([.140, 0, .010])
    views = [panel(surfaces, [.65, -.85, 1.2], 'Upper face / open fingers', center),
             panel(surfaces, [.65, -.85, -1.2], 'Underside / adjoining marker plate', center),
             panel(surfaces, [1, -.10, -.20], 'Distal view / real recessed small lens', center),
             panel(assembly(0)[0], [.65, -.85, 1.2], 'Closed articulation (existing M6 frames)', center)]
    cv2.imwrite(str(OUT / 'pincher-multiview.png'), np.vstack([np.hstack(views[:2]), np.hstack(views[2:])]))
    OUT.joinpath('after.urdf').write_text(ET.tostring(ROBOT, encoding='unicode'))
    print('Saved', OUT / 'pincher-multiview.png')
