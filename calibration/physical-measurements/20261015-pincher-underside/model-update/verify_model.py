#!/usr/bin/env python3
"""Reproduce offline before/after kinematics and actual mesh diagnostics."""
import hashlib
import itertools
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import xacro

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path.insert(0, str(ROOT / 'ros2/src/z70_description/test'))
from test_physical_visuals import SOURCE, origin, relative_frames
from test_pincher_mesh import triangles, visual_triangles, ray_hits

before = ET.fromstring((OUT / 'before.urdf').read_text())
after = ET.fromstring(xacro.process_file(str(SOURCE)).toxml())


def structure(robot):
    def node(element):
        return {'tag': element.tag, 'attributes': element.attrib, 'children': [node(c) for c in element]}
    return {'links': sorted(link.get('name') for link in robot.findall('link')),
            'joints': {j.get('name'): node(j) for j in robot.findall('joint')}}


assert structure(before) == structure(after), 'link/joint graph/origins/axes/limits changed'
max_difference = 0.
poses = []
for m3, m4, m5, m6 in itertools.product([160, 200, 240], [45, 95, 125], [0, 35, 155], [0, 90]):
    angles = {'m3_joint': np.radians(m3-270), 'm4_joint': np.radians(180-m4),
              'm5_joint': -np.radians(m5), 'm6_left_joint': np.radians(m6/2), 'm6_right_joint': np.radians(m6/2)}
    b, a = [relative_frames(r, 'base_footprint', angles) for r in [before, after]]
    assert a.keys() == b.keys()
    error = max(float(np.max(np.abs(a[name]-b[name]))) for name in a)
    assert error < 1e-14
    max_difference = max(max_difference, error)
    poses.append({'vendor_degrees': [m3, m4, m5, m6], 'all_link_transform_max_abs_difference': error})

shell = visual_triangles(after, 'arm_wrist_link')
local = relative_frames(after, 'arm_wrist_link', {})
camera = local['gripper_camera_link']
metrics = {'pose_count': len(poses), 'link_count': len(a), 'joint_count': len(after.findall('joint')),
           'all_joint_structure_exact_equal': True, 'max_all_link_transform_difference': max_difference,
           'optical_forward_shell_intersections_m': ray_hits(shell, camera[:3, 3], camera[:3, 0]).tolist(),
           'optical_back_wall_distance_m': float(ray_hits(shell, camera[:3, 3], -camera[:3, 0])[0]),
           'pose_checks': poses, 'meshes': {}, 'open_envelopes_m': []}
for path in sorted((SOURCE.parents[1] / 'meshes').glob('*.stl')):
    points = triangles(path)
    volume = float(np.einsum('ij,ij->', points[:, 0], np.cross(points[:, 1], points[:, 2]))/6)
    metrics['meshes'][path.name] = {'triangles': len(points), 'volume_m3': volume, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
for m5 in [0, 35, 155]:
    frames = relative_frames(after, 'arm_link2', {'m5_joint': -np.radians(m5), 'm6_left_joint': np.pi/4, 'm6_right_joint': np.pi/4})
    all_points = []
    box = after.find("link[@name='arm_link2']/visual")
    half = np.fromstring(box.find('geometry/box').get('size'), sep=' ')/2
    all_points.extend(np.array(list(itertools.product([-1, 1], repeat=3)))*half+origin(box.find('origin'))[:3, 3])
    for link in ['arm_wrist_link', 'finger_left', 'finger_right']:
        mesh = visual_triangles(after, link)
        all_points.extend((mesh @ frames[link][:3, :3].T+frames[link][:3, 3]).reshape(-1, 3))
    all_points = np.array(all_points)
    metrics['open_envelopes_m'].append({'M5': m5, 'x_min': float(all_points[:, 0].min()), 'x_max': float(all_points[:, 0].max())})
metrics['photo_sha256'] = hashlib.sha256((OUT.parent / 'IMG_0647_4ec758.png').read_bytes()).hexdigest()
(OUT / 'verification.json').write_text(json.dumps(metrics, indent=2)+'\n')
print(json.dumps({k: v for k, v in metrics.items() if k != 'pose_checks'}, indent=2))
