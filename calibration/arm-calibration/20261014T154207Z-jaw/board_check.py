"""Independent ChArUco floor-plane check, after the retained marker/ray fit."""
import json
import cv2,numpy as np
from measure import D,ROOT,K,dist,Rf,C
fit=json.loads((D/'jaw-fit.json').read_text());dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
b=cv2.aruco.CharucoBoard((10,14),.020,.015,dic);b.setLegacyPattern(True)
p=cv2.aruco.DetectorParameters();p.adaptiveThreshWinSizeMax=53
cp=cv2.aruco.CharucoParameters();cp.minMarkers=1;cp.tryRefineMarkers=True;det=cv2.aruco.CharucoDetector(b,cp,p)
rows=[]
for file,data in fit['poses'].items():
 im=cv2.imread(str(D/file));g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);g[:800]=255
 cc,ids,_,_=det.detectBoard(g)
 if ids is None or len(ids)<8:continue
 px=cc.reshape(-1,2);obj=b.getChessboardCorners()[ids.flatten()]
 ok,r,t=cv2.solvePnP(obj,px,K,dist)
 for _ in range(3):
  uv=cv2.projectPoints(obj,r,t,K,dist)[0].reshape(-1,2);good=np.linalg.norm(uv-px,axis=1)<4
  if good.sum()<8:break
  r,t=cv2.solvePnPRefineLM(obj[good],px[good],K,dist,r,t)
 R=cv2.Rodrigues(r)[0];normal=R[:,2]
 if normal@(Rf@np.array([0,0,1.]))<0:normal=-normal
 J=Rf@(np.array(data['jaw_CF'])-C)
 height=float(normal@(J-t.ravel()));cam_height=float(-normal@t.ravel())
 rows.append(dict(file=file,board_corners=int(good.sum()),rms_px=float(np.sqrt(np.mean(np.sum((uv[good]-px[good])**2,axis=1)))),jaw_height_from_board_m=height,front_camera_height_from_board_m=cam_height,normal_CF=(Rf.T@normal).tolist(),jaw_height_frozen_floor_m=data['jaw_CF'][2]))
(D/'board-check.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
