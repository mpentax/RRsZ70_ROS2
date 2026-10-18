"""Source-pinned CAD chain math; no publisher, hardware or optical accuracy claim."""
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

from test_physical_visuals import SOURCE, relative_frames, rotation
from z70_description import pincher_geometry as hand
from z70_description.arm_model import vendor_forward
from z70_manipulation import planner


@pytest.mark.parametrize('state', [(150, 45, 0, 90), (200, 90, 35, 45), (240, 130, 155, 0)])
def test_cad_wrist_and_camera_follow_expanded_urdf_independently(state):
    m3, m4, m5, m6 = state
    yaw = math.radians(hand.CAL['arm_yaw_deg'])
    a1 = math.radians(270-m3+hand.CAL['measured_M3_offset_deg'])
    a2 = math.radians(m4-m3+90)
    Rz = rotation([0, 0, 1], yaw)
    B = Rz @ rotation([0, 1, 0], -a2)
    R = B @ rotation([1, 0, 0], math.radians(m5))
    elbow = np.array([.1429, -.0117, .1266]) + Rz @ np.array([.1772*math.cos(a1), 0, .1772*math.sin(a1)])
    wrist = elbow + B @ np.array([.1157, 0, 0])
    robot = ET.fromstring(xacro.process_file(str(SOURCE)).toxml())
    frames = relative_frames(robot, 'base_footprint', {
        'm3_joint': math.radians(m3-270), 'm4_joint': math.radians(180-m4),
        'm5_joint': -math.radians(m5), 'm6_left_joint': math.radians(m6)/2,
        'm6_right_joint': math.radians(m6)/2,
    })
    assert frames['arm_wrist_link'][:3, 3] == pytest.approx(wrist)
    assert frames['arm_wrist_link'][:3, :3] == pytest.approx(R)
    assert hand.hand_pose(state) == pytest.approx(frames['arm_wrist_link'])
    C, optical_R = planner.camera_pose(state)
    assert C == pytest.approx(frames['gripper_camera_optical_frame'][:3, 3])
    assert optical_R == pytest.approx(frames['gripper_camera_optical_frame'][:3, :3])
    assert not np.allclose(wrist[[0, 2]], vendor_forward(m3, m4)[1])
