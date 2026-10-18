import json,math
from pathlib import Path
import cv2,numpy as np
D=Path(__file__).resolve().parent;f=json.loads((D/'fit.json').read_text());obs=json.loads((D/'observations.json').read_text());P=f['params'];G=json.loads((D.parent/'20261014T090217Z/arm_calibration.json').read_text())['gripper_intrinsics'];F=json.loads((D.parent.parent/'front-camera/front_camera.json').read_text())
Kf=np.array([[F['f_px'],0,F['cx']],[0,F['f_px'],F['cy']],[0,0,1.]]);df=np.array([F['k1'],F['k2'],0,0,0]);yaw=P['arm_yaw_rad'];cy,sy=math.cos(yaw),math.sin(yaw)
def basis(a):return np.array([[cy*math.cos(a),-sy,-cy*math.sin(a)],[sy*math.cos(a),cy,-sy*math.sin(a)],[math.sin(a),0,math.cos(a)]])
for o in obs['board']:
 if o['file'] not in ['g170-55-0.jpg','g200-65-0.jpg','g240-95-0.jpg']:continue
 a,b,_=o['j'];R=basis(math.radians(b-a+90));C=np.array([P['shoulder_CF_'+x] for x in 'xyz'])+P['L1_m']*basis(math.radians(270-a)+P['M3_offset_rad'])[:,0]+R@np.array([P['camera_effective_'+x] for x in 'xyz']);Rc=R@cv2.Rodrigues(np.array([P['camera_mount_r'+str(i)] for i in range(3)]))[0]
 X=np.array([[(i%9+1)*.02,(i//9+1)*.02,0] for i in o['ids']]);X=X@np.array(f['board_in_CF']['r']).T+np.array(f['board_in_CF']['t']);uv=cv2.projectPoints((X-C)@Rc,np.zeros(3),np.zeros(3),np.array(G['K']),np.array(G['dist']))[0].reshape(-1,2);im=cv2.imread(str(D/o['file']))
 for seen,pred in zip(o['px'],uv):cv2.circle(im,tuple(np.round(seen).astype(int)),4,(0,255,0),1);cv2.drawMarker(im,tuple(np.round(pred).astype(int)),(0,0,255),cv2.MARKER_CROSS,7,1)
 cv2.putText(im,'Observed green / effective model red; physical L2 unresolved',(20,35),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2);cv2.imwrite(str(D/o['file'].replace('.jpg','-fit-overlay.jpg')),im)
for o in obs['wrist_marker']:
 if o['file'] not in f['jaw_annotations']:continue
 T=np.array(o['Tcam']);X=T[:3,:3]@np.array(f['jaw_marker_frame'])+T[:3,3];uv=cv2.projectPoints(X[None,:],np.zeros(3),np.zeros(3),Kf,df)[0].reshape(2);im=cv2.imread(str(D/o['file']));seen=f['jaw_annotations'][o['file']][:2];cv2.circle(im,tuple(np.round(seen).astype(int)),7,(0,255,255),1);cv2.drawMarker(im,tuple(np.round(uv).astype(int)),(0,0,255),cv2.MARKER_CROSS,12,2);cv2.putText(im,'Provisional contact seam: annotation yellow / metric fit red',(20,35),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2);cv2.imwrite(str(D/o['file'].replace('.jpg','-jaw-fit-overlay.jpg')),im)
print('Rendered 3 gripper model overlays and 5 contact-seam overlays')
