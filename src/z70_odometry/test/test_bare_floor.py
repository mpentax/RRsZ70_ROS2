"""Rendered real bare kitchen floor: geometry/texture, not hardware validation."""
import math
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from z70_odometry.floor_vo import FloorCamera, FloorOdometry, render_floor_view
REPO=Path(__file__).resolve().parents[3]

class BareFloorTest(unittest.TestCase):
    def test_recovers_small_motion_on_uncovered_tile(self):
        cam=FloorCamera.load(REPO/'calibration/front-camera/front_camera.json')
        image=cv2.imread(str(REPO/'calibration/arm-validation/camera-fixed-frame-1.jpg'))
        vo=FloorOdometry(cam)
        for dx,angle in [(.03,0),(.06,0),(0,5),(0,15)]:
            th=math.radians(angle)
            c=np.array([vo.cam_x,0]); R=np.array([[math.cos(th),-math.sin(th)],[math.sin(th),math.cos(th)]])
            t=np.array([dx,0])-c+R@c
            with self.subTest(dx=dx,angle=angle):
                m=vo.estimate(image,render_floor_view(image,cam,(*t,th)))
                self.assertIsNotNone(m)
                self.assertLess(math.hypot(m.dx-dx,m.dy),.001)
                self.assertLess(abs(m.dtheta_deg-angle),.2)

if __name__=='__main__': unittest.main()
