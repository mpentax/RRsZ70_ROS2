"""ID68 CAD-face registration; not an independent anatomical calibration."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

from test_physical_visuals import origin, relative_frames

ROOT = Path(__file__).resolve().parents[1]


def test_marker_is_distal_and_flat_on_cad_underside():
    robot = ET.fromstring(xacro.process_file(str(ROOT / 'urdf/z70.urdf.xacro')).toxml())
    mount = origin(robot.find("joint[@name='pincher_marker_68_mount']/origin"))
    # Owner reports about 1cm placement error; underside photo establishes distal direction.
    assert mount[0, 3] == pytest.approx(.024)
    assert mount[1, 3] == pytest.approx(0.)
    # CAD underside has dz/dx=-.078125; marker +Y follows that face distally.
    distal = np.array([1., 0., -.078125]); distal /= np.linalg.norm(distal)
    assert mount[:3, 1] == pytest.approx(distal, abs=1e-8)
    assert mount[:3, 2] == pytest.approx(np.cross(mount[:3, 0], distal), abs=1e-8)
    assert mount[2, 2] < 0  # outward, not inside the pincher


@pytest.mark.parametrize('roll', [0., .6, 2.7])
def test_marker_rotation_recovers_pincher_rotation_without_translation(roll):
    robot = ET.fromstring(xacro.process_file(str(ROOT / 'urdf/z70.urdf.xacro')).toxml())
    frames = relative_frames(robot, 'arm_link2', {'m5_joint': -roll})
    mount = origin(robot.find("joint[@name='pincher_marker_68_mount']/origin"))
    # For a front-camera observation R_front_marker, use R_front_marker @ R_mount.T.
    recovered = frames['pincher_marker_68'][:3, :3] @ mount[:3, :3].T
    assert recovered == pytest.approx(frames['arm_wrist_link'][:3, :3], abs=1e-10)
