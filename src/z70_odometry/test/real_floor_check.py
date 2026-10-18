"""Offline feature/motion audit. Kitchen frames visually confirmed to have no boards.
Static same-session pairs test repeatability, NOT physical motion accuracy.
The conservative ROI removes furniture/shoes from the geometric floor mask.
"""
import json
import sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from z70_odometry.floor_vo import FloorCamera, FloorOdometry, render_floor_view
REPO = Path(__file__).resolve().parents[3]

def audit(vo, frames):
    out = {"features": {}, "pairs": []}
    for name, image in frames.items():
        out["features"][name] = len(vo.features(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))[0])
    for a, b in [("camera-fixed-frame-1", "camera-fixed-frame-2")]:
        k0,d0 = vo.features(cv2.cvtColor(frames[a],cv2.COLOR_BGR2GRAY))
        k1,d1 = vo.features(cv2.cvtColor(frames[b],cv2.COLOR_BGR2GRAY))
        good = [m for p in vo.bf.knnMatch(d0,d1,k=2) if len(p)==2 for m,n in [p] if m.distance < .8*n.distance]
        m = vo.estimate(frames[a],frames[b])
        out["pairs"].append({"a":a,"b":b,"ratio_matches":len(good),"motion":vars(m) if m else None})
    image=frames["camera-fixed-frame-1"]
    out["rendered_bare_floor"] = []
    for dx, angle in [(.03,0),(.06,0),(0,5),(0,15)]:
        th=np.deg2rad(angle); c=vo.cam_x
        tc=(dx+c*(np.cos(th)-1),c*np.sin(th),th)
        m=vo.estimate(image,render_floor_view(image,vo.cam,tc))
        out["rendered_bare_floor"].append({"dx":dx,"angle_deg":angle,"motion":vars(m) if m else None})
    return out

if __name__ == "__main__":
    cam=FloorCamera.load(REPO/"calibration/front-camera/front_camera.json")
    frames={n:cv2.imread(str(REPO/"calibration/arm-validation"/(n+".jpg"))) for n in
            ["camera-fixed-frame-1","camera-fixed-frame-2","deploy-frame-2"]}
    vo=FloorOdometry(cam)
    full=audit(vo,frames)
    # Near open kitchen floor: x pixels 80..1150, y 900..1190.
    roi=np.zeros_like(vo.mask); roi[900:1190,80:1150]=255
    vo.mask=cv2.bitwise_and(vo.mask,roi)
    result={"full_geometric_mask":full,"bare_floor_roi":audit(vo,frames)}
    print(json.dumps(result,indent=2))
    if len(sys.argv)>1: Path(sys.argv[1]).write_text(json.dumps(result,indent=2)+"\n")
