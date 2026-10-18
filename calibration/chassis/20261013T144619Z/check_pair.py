#!/usr/bin/env python3
"""Offline settling/step check, using the prescribed unmodified VO limits."""
import dataclasses
import json
import math
from pathlib import Path
import sys
import cv2
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'ros2/src/z70_odometry'))
from z70_odometry.floor_vo import FloorCamera, FloorOdometry
cam = FloorCamera.load(REPO / 'calibration/front-camera/front_camera.json')
vo = FloorOdometry(cam, camera_x_m=0, n_features=6000)
a, b = [cv2.imread(p) for p in sys.argv[1:3]]
assert a is not None and b is not None
assert a.shape[:2] == b.shape[:2] == (1200,1600)
m = vo.estimate(a,b)
result = {'accepted': m is not None}
if m is not None:
    result.update(dataclasses.asdict(m))
    result['translation_m'] = math.hypot(m.dx,m.dy)
    result['yaw_deg'] = m.dtheta_deg
    result['settled'] = result['translation_m'] < .002 and abs(m.dtheta_deg) < .3
    result['within_step_envelope'] = result['translation_m'] <= .06 and abs(m.dtheta_deg) <= 15
print(json.dumps(result, indent=2))
