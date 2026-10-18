"""Front wrist marker observations and bounded incremental acquisition."""
import sys,json,subprocess
from pathlib import Path
import cv2,numpy as np
D=Path(__file__).resolve().parent;ROOT=D.parents[2]
F=json.loads((ROOT/'calibration/front-camera/front_camera.json').read_text());K=np.array([[F['f_px'],0,F['cx']],[0,F['f_px'],F['cy']],[0,0,1.]])
dist=np.array([F['k1'],F['k2'],0.,0.,0.]);Rf=np.array(F['R_floor_to_cam']);C=np.array([0,0,F['camera_height_mm']/1000])
p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;p.adaptiveThreshWinSizeMax=53
ad=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),p)
def observe(file):
 im=cv2.imread(str(D/file));g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);cc,ii,_=ad.detectMarkers(g);marks=[]
 for q,i in zip(cc,ii.flatten() if ii is not None else []):
  if i!=68:continue
  px=q.reshape(4,2);s=.0075;obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]])
  ok,r,t=cv2.solvePnP(obj,px,K,dist,flags=cv2.SOLVEPNP_IPPE_SQUARE);r,t=cv2.solvePnPRefineLM(obj,px,K,dist,r,t)
  R=cv2.Rodrigues(r)[0];CF=C+Rf.T@t.ravel();res=np.sqrt(np.mean(np.sum((cv2.projectPoints(obj,r,t,K,dist)[0].reshape(4,2)-px)**2,axis=1)))
  rec=dict(pixel=px.tolist(),position_CF=CF.tolist(),R_CF=(Rf.T@R).tolist(),tcam=t.ravel().tolist(),Rcam=R.tolist(),rms=float(res))
  if CF[2]>.025:marks.append(rec)
  cv2.polylines(im,[px.astype(int)],True,(0,0,255),2)
 cv2.imwrite(str(D/file.replace('.jpg','-overlay.jpg')),im)
 row=dict(file=file,markers=marks)
 with (D/'marker.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
 print(json.dumps(row),flush=True);return row
if __name__=='__main__':
 if sys.argv[1]=='observe':
  for f in sys.argv[2:]:observe(f)
 elif sys.argv[1]=='acquire':
  prev=sys.argv[2];start=list(map(int,sys.argv[3].split(',')))
  for target in json.loads(sys.argv[4]):
   delta=np.array(target)-start
   if np.count_nonzero(delta)!=1 or max(abs(delta))!=10:raise RuntimeError('single jog only')
   m=[0 if v==0 else (1 if v>0 else 2) for v in delta];file='f'+'-'.join(map(str,target))+'-open.jpg'
   if (D/file).exists():file=file.replace('.jpg','-return.jpg')
   a=[dict(type='jog',m=m,file=file,camera='front',joints=target)]
   r=subprocess.run(['node',str(ROOT/'calibration/arm-calibration/live.mjs'),str(D),json.dumps(a)],cwd=ROOT,text=True,capture_output=True,timeout=80);print(r.stdout,flush=True)
   if r.returncode:raise RuntimeError(r.stderr)
   old=cv2.imread(str(D/prev));new=cv2.imread(str(D/file));diff=np.max(cv2.absdiff(old,new),axis=2);changed=int(np.sum(diff[:900]>35));rec=observe(file)
   print('CHANGE_PIXELS',changed,flush=True)
   with (D/'motion.jsonl').open('a') as f:f.write(json.dumps(dict(before=prev,after=file,from_joints=start,to_joints=target,changed_pixels=changed))+'\n')
   if changed<500:raise RuntimeError('No significant visual jog: inspect before continuing')
   prev=file;start=target
  print('END',prev,start)
