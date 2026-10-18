"""One provisional new-run floor for the active projector and real ROS optical TF."""
import json
import time
from pathlib import Path

import numpy as np
import pytest
import rclpy
from rclpy.time import Time
from tf2_ros import TransformException

from test_wrist_tf import tf_robot
from z70_manipulation import perception
from z70_manipulation.node import rotation

ROOT = Path(__file__).resolve().parents[3]
RUN = ROOT / 'calibration/arm-calibration/20261015T080103Z-pincher-intrinsics-arm/arm'


def test_front_projector_and_real_tf_use_the_same_measured_floor(tf_robot):
    registration = json.loads((RUN / 'front-registration.json').read_text())
    T_F_W = np.array(registration['T_F_W'])
    up = -T_F_W[:3, 2]
    forward = np.array([0., 0., 1.]) - up * up[2]
    forward /= np.linalg.norm(forward)
    floor_to_cam = np.column_stack([forward, np.cross(up, forward), up])
    cam = perception.front_camera(ROOT)
    assert cam.h / 1000 == pytest.approx(registration['camera_height_above_board_m'], abs=1e-12)
    assert cam.R == pytest.approx(floor_to_cam, abs=1e-12)
    # Principal ray has zero floor-left bearing; board yaw is NOT robot heading.
    assert cam.pixel_to_ray(cam.cx, cam.cy)[1] == pytest.approx(0., abs=1e-12)
    assert cam.pixel_to_floor(cam.cx, cam.cy) is None  # slight observed pitch UP
    assert cam.pixel_to_ray(800., 900.)[2] < 0
    # Lens selection is independent of the provisional floor-pose replacement.
    import hashlib
    archive = ROOT / 'calibration/arm-calibration/20261015-provisional-model/prior-front-camera.json'
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == 'b1d00207c78d5f56da9781ce24cebb0719c6146a94b1812576365e62061d19ae'
    old = json.loads(archive.read_text())
    active = json.loads((ROOT / 'calibration/front-camera/front_camera.json').read_text())
    for field in ('width', 'height', 'model', 'f_px', 'cx', 'cy', 'k1', 'k2'):
        assert active[field] == old[field]

    node, buffer, _, process = tf_robot
    deadline = time.monotonic() + 8
    tr = None
    while time.monotonic() < deadline:
        assert process.poll() is None
        rclpy.spin_once(node, timeout_sec=.1)
        try:
            tr = buffer.lookup_transform('base_footprint', 'camera_optical_frame', Time()).transform
            shoulder = buffer.lookup_transform('base_footprint', 'arm_vendor_frame', Time()).transform
            break
        except TransformException:
            pass
    assert tr is not None, 'front optical TF missing'
    C = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
    R = rotation(tr.rotation)
    assert C == pytest.approx([.275, 0., cam.h/1000], abs=1e-10)
    assert R == pytest.approx(cam.R.T, abs=1e-10)
    # A point above the board has positive floor z; floor rays point down.
    assert (R @ up)[2] == pytest.approx(1., abs=1e-10)
    for pixel in [(400., 950.), (800., 900.), (1200., 1050.)]:
        expected = cam.pixel_to_floor(*pixel)
        actual = perception.ray_floor(pixel, np.array([[cam.f,0,cam.cx],[0,cam.f,cam.cy],[0,0,1.]]),
                                      np.array([cam.k1,cam.k2,0.,0.,0.]), C, R)
        # FrontCamera iterates radial inversion to convergence; OpenCV's default
        # undistortPoints stops earlier. Allow only 1 micrometre of solver error.
        assert actual == pytest.approx([.275+expected['forward_mm']/1000, expected['left_mm']/1000, 0.], abs=1e-6)
    # Shoulder origin gauge is translated consistently, rather than mixing heights.
    assert shoulder.translation.z + .14 == pytest.approx(.12643029152430296, abs=1e-10)
