import sys,json,hashlib
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation as Rot
ROOT=Path(__file__).resolve().parents[3]
B=cv2.aruco.CharucoBoard((10,14),.020,.015,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000));B.setLegacyPattern(True)
DICT=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
X=np.array([[-.0075,.0075,0],[.0075,.0075,0],[.0075,-.0075,0],[-.0075,-.0075,0]])
def lens(cam):
 if cam=='front':
  f=json.loads((ROOT/'calibration/front-camera/front_camera.json').read_text());return np.array([[f['f_px'],0,f['cx']],[0,f['f_px'],f['cy']],[0,0,1.]]),np.array([f['k1'],f['k2'],0,0,0.])
 f=json.loads((ROOT/'ros2/src/z70_description/config/z70_gripper_camera_info.yaml').read_text());return np.array(f['camera_matrix']['data']).reshape(3,3),np.array(f['distortion_coefficients']['data'])
def delta(a,b):return Rot.from_matrix(b@a.T).as_rotvec()
def branches(px,K,d):
 _,rs,ts,_=cv2.solvePnPGeneric(X,np.array(px,dtype=float),K,d,flags=cv2.SOLVEPNP_IPPE_SQUARE);out=[]
 for r,t in zip(rs,ts):
  R=cv2.Rodrigues(r)[0];err=cv2.projectPoints(X,r,t,K,d)[0].reshape(4,2)-px
  out.append(dict(R=R.tolist(),quaternion_xyzw=Rot.from_matrix(R).as_quat().tolist(),t=t.ravel().tolist(),positive_depth=bool(np.all((X@R.T+t.ravel())[:,2]>0)),rms_px=float(np.sqrt(np.mean(np.sum(err**2,axis=1))))))
 return out
def detect(path,cam):
 im=cv2.imread(str(path));gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);K,d=lens(cam)
 p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=1;p.adaptiveThreshWinSizeMax=53
 cs,ids,rejected=cv2.aruco.ArucoDetector(DICT,p).detectMarkers(gray);raw=[];marker=[]
 if ids is not None:
  for c,i in zip(cs,ids.ravel()):
   px=c.reshape(4,2).astype(float);raw.append(dict(id=int(i),px=px.tolist()))
   if cam=='front' and i==68 and px[:,1].mean()<680:marker.append(dict(px=px.tolist(),branches=branches(px,K,d)))
 g=gray.copy()
 if cam=='front':g[:680]=255
 cp=cv2.aruco.CharucoParameters();cp.minMarkers=1;cp.tryRefineMarkers=True
 cc,ci,_,_=cv2.aruco.CharucoDetector(B,cp,p).detectBoard(g);method='refined_charuco'
 if ci is None or len(ci)<8:
  p.cornerRefinementMethod=0;cc,ci,_,_=cv2.aruco.CharucoDetector(B,cp,p).detectBoard(g);method='unrefined_charuco_fallback'
 board=None
 if ci is not None and len(ci)>=8:obj=B.getChessboardCorners()[ci.ravel()].astype(float);px=cc.reshape(-1,2).astype(float)
 else:
  method='refined_sparse_marker_fallback';p.cornerRefinementMethod=1;csb,ib,_=cv2.aruco.ArucoDetector(DICT,p).detectMarkers(g);lut=dict(zip(B.getIds().ravel(),B.getObjPoints()));pairs=[] if ib is None else [(lut[int(i)],c.reshape(4,2)) for c,i in zip(csb,ib.ravel()) if int(i) in lut]
  obj=np.concatenate([a for a,b in pairs]).astype(float) if len(pairs)>=3 else None;px=np.concatenate([b for a,b in pairs]).astype(float) if len(pairs)>=3 else None
 if obj is not None:
  ok,r,t=cv2.solvePnP(obj,px,K,d,flags=cv2.SOLVEPNP_ITERATIVE)
  if ok:
   R=cv2.Rodrigues(r)[0];err=np.linalg.norm(cv2.projectPoints(obj,r,t,K,d)[0].reshape(-1,2)-px,axis=1)
   board=dict(method=method,R_cam_board=R.tolist(),t=t.ravel().tolist(),obj=obj.tolist(),px=px.tolist(),full_rms_px=float(np.sqrt(np.mean(err**2))),accepted=bool(np.mean(err<4)>=.65 and np.all((obj@R.T+t.ravel())[:,2]>0)),normal_cam=R[:,2].tolist(),optical_angle_to_floor_deg=float(np.degrees(np.arcsin(np.clip(abs(R[2,2]),0,1)))))
 o=dict(file=str(path),camera=cam,dimensions=[im.shape[1],im.shape[0]],sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),raw_markers=raw,rejected_quads=len(rejected),marker=marker[0] if len(marker)==1 else None,marker_ambiguous=len(marker)>1,board=board)
 Path(path).with_suffix('.json').write_text(json.dumps(o,indent=2));print(json.dumps(dict(file=Path(path).name,marker=None if not o['marker'] else dict(px=o['marker']['px'],branches=[dict(rms_px=b['rms_px'],positive_depth=b['positive_depth']) for b in o['marker']['branches']]),board=None if not board else {k:board[k] for k in ['method','full_rms_px','accepted','optical_angle_to_floor_deg']})))
 return o
if __name__=='__main__':detect(Path(sys.argv[1]),sys.argv[2])
