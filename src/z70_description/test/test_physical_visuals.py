"""URDF visual link lengths and relative frame transforms."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
from z70_description.pincher_geometry import rpy


SOURCE = Path(__file__).resolve().parents[1] / 'urdf/z70.urdf.xacro'
ROBOT = ET.fromstring(xacro.process_file(str(SOURCE)).toxml())


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross@cross)


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        transform[:3, :3] = rpy(np.fromstring(element.get('rpy', '0 0 0'), sep=' '))
    return transform


def relative_frames(robot, root, angles):
    frames = {root: np.eye(4)}
    pending = list(robot.findall('joint'))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in frames:
                continue
            motion = np.eye(4)
            if joint.get('type') != 'fixed':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                mimic=joint.find('mimic')
                angle=angles.get(joint.get('name'),0.)
                if mimic is not None:
                    angle=angles.get(mimic.get('joint'),0.)*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0'))
                motion[:3, :3] = rotation(axis, angle)
            frames[joint.find('child').get('link')] = frames[parent] @ origin(joint.find('origin')) @ motion
            pending.remove(joint)
            progressed = True
        if not progressed:
            break  # Other ancestors of root are deliberately outside this local chain.
    return frames


@pytest.mark.parametrize('m5', [0, 35, 155])
def test_open_second_assembly_uses_the_copied_wrist(m5):
    frames = relative_frames(ROBOT, 'arm_link2', {
        'm5_joint': -math.radians(m5),
        'm6_left_joint': math.pi/4,
        'm6_right_joint': math.pi/4,
    })
    assert frames['arm_wrist_link'][:3, 3] == pytest.approx([0.1157, 0, 0])


def test_first_segment_uses_the_copied_link_length():
    joint_origin = ROBOT.find("joint[@name='m4_joint']/origin").get('xyz').split()
    assert float(joint_origin[0]) == pytest.approx(0.1772, abs=1e-4)
    mesh = ROBOT.find("link[@name='arm_link1']").find("visual/geometry/mesh[@filename='package://z70_description/meshes/cad/cad_link1.stl']")
    assert mesh is not None
