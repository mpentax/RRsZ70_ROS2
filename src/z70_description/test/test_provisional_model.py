"""Observable geometry and explicit provisional anatomy, expanded without hardware."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest
import xacro
import yaml

from test_physical_visuals import SOURCE, origin, relative_frames
from z70_manipulation import planner

ROOT = SOURCE.parents[3]
CONFIG = SOURCE.parents[1] / 'config'
RUN = ROOT / 'calibration/arm-calibration/20261015T080103Z-pincher-intrinsics-arm'
MODEL = json.loads((ROOT / 'calibration/arm-calibration/20261015-provisional-model/model.json').read_text())


def test_copied_body_carries_the_cad_shoulder():
    robot = ET.fromstring(xacro.process_file(str(SOURCE)).toxml())
    frames = relative_frames(robot, 'base_footprint', {})
    mesh = robot.find("link[@name='base_link']/visual/geometry/mesh")
    assert mesh is not None and mesh.get('filename').endswith('meshes/cad/cad_body.stl')
    shoulder = frames['arm_shoulder_link'][:3, 3]
    cad = yaml.safe_load((CONFIG / 'cad_arm.yaml').read_text())
    assert shoulder == pytest.approx(cad['shoulder_in_base_m'], abs=1e-3)
    # Nose of the copied shell meets the measured front camera. The optical
    # shoulder remains in mount_calibration.yaml as the old gauge.
    cal = yaml.safe_load((CONFIG / 'mount_calibration.yaml').read_text())
    assert frames['camera_link'][0, 3] == pytest.approx(cal['front_camera_x_m'])
    assert cal['measured_shoulder_base_m'] != pytest.approx(cad['shoulder_in_base_m'], abs=1e-3)


def test_new_marker_registration_is_a_published_full_pose_not_a_wrist_proxy():
    robot = ET.fromstring(xacro.process_file(str(SOURCE)).toxml())
    frames = relative_frames(robot, 'arm_wrist_link', {'m5_joint': -math.radians(35)})
    assert frames['gripper_camera_link'][:3, 3] == pytest.approx([0.0355, 0.0075, -0.0109], abs=1e-4)
    # Transform the measured elbow-to-lens vector using its actual observed G
    # pose; a lever radius substituted for the x component would fail here.
    derivation = MODEL['camera_vector_derivation']
    observed_G = np.array(derivation['T_W_G'])
    retained_R = np.array(MODEL['prior_mount']['effective_camera_optical_rotation'])
    expected_vector = retained_R @ observed_G[:3, :3].T @ derivation['v_W']
    cal = yaml.safe_load((CONFIG / 'mount_calibration.yaml').read_text())
    assert cal['effective_camera_from_elbow_m'] == pytest.approx(expected_vector, abs=1e-12)
    assert np.linalg.norm(expected_vector) == pytest.approx(MODEL['source_measurements']['pincher_perpendicular_radius_m'], abs=1e-12)
    marker = robot.find("link[@name='pincher_marker_68']")
    sizes = [np.fromstring(v.find('geometry/box').get('size'), sep=' ')[:2] for v in marker.findall('visual')]
    assert any(np.allclose(s, [.015, .015]) for s in sizes)
    assert any(np.allclose(s, [.02, .02]) for s in sizes)
    local = relative_frames(robot, 'arm_link2', {})
    finger_z = local['finger_left'][2, 3]
    # Side roots sit in the cover, above the underside camera recess and marker.
    assert finger_z == pytest.approx(0.)
    assert local['gripper_camera_link'][:3, 3] == pytest.approx([0.1157+0.0355, 0.0075, -0.0109], abs=1e-3)
    assert local['pincher_marker_68'][2, 3] < -0.01


def test_planner_projection_uses_the_validated_asymmetric_lens():
    lens = json.loads((RUN / 'lens/proposed-camera-info.json').read_text())
    K, D = np.array(lens['K']), np.array(lens['D'])
    info = yaml.safe_load((CONFIG / 'z70_gripper_camera_info.yaml').read_text())
    assert np.array(info['camera_matrix']['data']).reshape(3, 3) == pytest.approx(K)
    assert info['distortion_coefficients']['data'] == pytest.approx(D)
    assert np.array(info['projection_matrix']['data']).reshape(3, 4)[:, :3] == pytest.approx(K)
    state = (200, 95, 35)
    C, R = planner.camera_pose(state)
    optical = np.array([.04, -.025, .18])
    point = C + R @ optical
    expected = cv2.projectPoints(optical[None], np.zeros(3), np.zeros(3), K, D)[0].reshape(2)
    assert planner.gripper_pixel(state, point)[:2] == pytest.approx(expected, abs=1e-9)
