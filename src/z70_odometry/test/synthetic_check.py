"""Synthetic check of floor VO: render frame-3 as seen after known robot motions, then recover them.
The synthetic views are exact for a flat world, so this tests the geometry + matching + RANSAC chain,
not real-scene effects (furniture parallax, low-texture floors, motion blur)."""
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from z70_odometry.floor_vo import FloorCamera, FloorOdometry, render_floor_view  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
cam = FloorCamera.load(REPO / "calibration/front-camera/front_camera.json")
img = cv2.imread(str(REPO / "calibration/front-camera/raw/frame-3-telegram.jpg"))
CAM_X = 0.16
vo = FloorOdometry(cam, camera_x_m=CAM_X)

cases = [  # robot base motions: (dx m, dy m, dtheta deg) -- the observed RC pulses are ~15 deg and a few cm
    (0.0, 0.0, 15.0), (0.0, 0.0, -15.0), (0.0, 0.0, 5.0), (0.03, 0.0, 0.0), (0.06, 0.0, 0.0),
    (0.03, 0.0, 8.0), (-0.03, 0.0, 0.0), (0.0, 0.0, 30.0),
]
c = np.array([CAM_X, 0.0])
worst = 0.0
for dx, dy, dth in cases:
    th = math.radians(dth)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    tc = np.array([dx, dy]) - c + R @ c               # camera motion that produces this base motion
    img1 = render_floor_view(img, cam, (tc[0], tc[1], th))
    m = vo.estimate(img, img1)
    if dth == 30:
        assert m is None, "out-of-envelope rotation must be rejected"
        print("base (+0.000,+0.000,+30.0deg): correctly rejected")
        continue
    assert m is not None, f"missing estimate for {dx, dy, dth}"
    et = math.hypot(m.dx - dx, m.dy - dy) * 1000; ea = m.dtheta_deg - dth
    assert et < .4 and abs(ea) < .08, (et, ea)
    worst = max(worst, et)
    print(f"base ({dx:+.3f},{dy:+.3f},{dth:+5.1f}deg) -> ({m.dx:+.4f},{m.dy:+.4f},{m.dtheta_deg:+6.2f}deg)  "
          f"err {et:5.1f} mm {ea:+5.2f} deg  inliers {m.inliers}/{m.matches} rms {m.rms_m*1000:.1f} mm")
print("worst translation error mm:", round(worst, 1))
