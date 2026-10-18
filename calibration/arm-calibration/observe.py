import sys,json
from pathlib import Path
import cv2,numpy as np
D=Path(sys.argv[1]); dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
b=cv2.aruco.CharucoBoard((10,14),.020,.015,dic);b.setLegacyPattern(True)
p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;p.adaptiveThreshWinSizeMax=53
cp=cv2.aruco.CharucoParameters();cp.minMarkers=1;cp.tryRefineMarkers=True
cd=cv2.aruco.CharucoDetector(b,cp,p)
gcal=json.loads((D.parent/'20261014T090217Z/arm_calibration.json').read_text())['gripper_intrinsics']
fcal=json.loads((D.parent.parent/'front-camera/front_camera.json').read_text())
Kf=np.array([[fcal['f_px'],0,fcal['cx']],[0,fcal['f_px'],fcal['cy']],[0,0,1.]])
df=np.array([fcal['k1'],fcal['k2'],0,0,0])
old=None
for file in sys.argv[2:]:
 im=cv2.imread(str(D/file));g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);cc,ci,mc,mi=cd.detectBoard(g)
 cam='gripper' if im.shape[0]==896 else 'front';K=np.array(gcal['K']) if cam=='gripper' else Kf;dist=np.array(gcal['dist']) if cam=='gripper' else df
 rec={'file':file,'camera':cam,'corners':len(ci) if ci is not None else 0,'markers':mi.flatten().tolist() if mi is not None else []}
 if ci is not None and len(ci)>=8:
  ids=ci.flatten();px=cc.reshape(-1,2);obj=b.getChessboardCorners()[ids]
  ok,r,t=cv2.solvePnP(obj,px,K,dist)
  uv=cv2.projectPoints(obj,r,t,K,dist)[0].reshape(-1,2);rec.update(ids=ids.tolist(),px=px.tolist(),rvec=r.ravel().tolist(),tvec=t.ravel().tolist(),pnp_rms_px=float(np.sqrt(np.mean(np.sum((uv-px)**2,axis=1)))))
 if old is not None and old.shape==g.shape:
  pts=cv2.goodFeaturesToTrack(old,400,.01,10)
  if pts is not None:
   nxt,st,err=cv2.calcOpticalFlowPyrLK(old,g,pts,None);rec['flow_median_px']=float(np.median(np.linalg.norm((nxt-pts)[st.flatten()==1],axis=-1)))
 old=g
 c,ids,_=cv2.aruco.ArucoDetector(dic,p).detectMarkers(g)
 rec['id0']=[q.reshape(4,2).tolist() for q,i in zip(c,ids.flatten()) if i==0] if ids is not None else []
 if ci is not None:
  for q in cc.reshape(-1,2):cv2.circle(im,tuple(q.astype(int)),3,(0,255,0),-1)
 if ids is not None:cv2.aruco.drawDetectedMarkers(im,c,ids)
 cv2.imwrite(str(D/(file.replace('.jpg','-overlay.jpg'))),im)
 (D/file.replace('.jpg','-obs.json')).write_text(json.dumps(rec,indent=1))
 print(json.dumps({k:v for k,v in rec.items() if k not in ['ids','px','markers'] and k!='id0'}), 'id0',rec['id0'])
