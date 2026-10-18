"""Offline 15mm marker pose; output marker not wrist translation."""
import cv2
import json
import numpy as np
import pathlib
import sys
root=pathlib.Path(__file__).resolve().parents[2]
c=json.loads((root/'front-camera/front_camera.json').read_text())
im=cv2.imread(sys.argv[1])
K=np.array([[c['f_px'],0,c['cx']],[0,c['f_px'],c['cy']],[0,0,1.]])
dist=np.array([c['k1'],c['k2'],0.,0.,0.])
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000))
corners,ids,_=det.detectMarkers(im)
obj=np.array([[-.0075,.0075,0],[.0075,.0075,0],[.0075,-.0075,0],[-.0075,-.0075,0]])
out=[]
for pts,i in zip(corners, [] if ids is None else ids.flatten()):
    ok,r,t=cv2.solvePnP(obj,pts.reshape(4,2),K,dist,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    pc=t.flatten()
    pf=np.array(c['R_floor_to_cam']).T@pc+np.array([0,0,c['camera_height_mm']/1000])
    out.append({'id':int(i),'corners':pts.tolist(),'camera_m':pc.tolist(),'floor_frame_m':pf.tolist(),'note':'marker centre, not wrist; upper image extrapolation'})
print(json.dumps(out))
