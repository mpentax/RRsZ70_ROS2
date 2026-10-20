"""Pincher ArUco ID68 marker mount pose and rotation recovery tests."""
import numpy as np
import pytest

from test_physical_visuals import ROBOT, origin, relative_frames


def test_marker_is_distal_and_flat_on_cad_underside():
    mount = origin(ROBOT.find("joint[@name='pincher_marker_68_mount']/origin"))
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
    frames = relative_frames(ROBOT, 'arm_link2', {'m5_joint': -roll})
    mount = origin(ROBOT.find("joint[@name='pincher_marker_68_mount']/origin"))
    # For a front-camera observation R_front_marker, use R_front_marker @ R_mount.T.
    recovered = frames['pincher_marker_68'][:3, :3] @ mount[:3, :3].T
    assert recovered == pytest.approx(frames['arm_wrist_link'][:3, :3], abs=1e-10)
