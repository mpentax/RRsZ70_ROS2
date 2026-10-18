"""Incremental MCP acquisition: observe each jog before another is sent."""
import sys,json,subprocess,math
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[3]; D=Path(__file__).resolve().parent
py=str(ROOT/'calibration/arm-validation/.venv/bin/python')
last=sys.argv[1]; start=list(map(int,sys.argv[2].split(',')))
plan=json.loads(sys.argv[3])
gcal=json.loads((D.parent/'20261014T090217Z/arm_calibration.json').read_text())['gripper_intrinsics']
K=np.array(gcal['K']); dist=np.array(gcal['dist'])
b=cv2.aruco.CharucoBoard((10,14),.020,.015,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)); b.setLegacyPattern(True)
p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;p.adaptiveThreshWinSizeMax=53
cp=cv2.aruco.CharucoParameters();cp.minMarkers=1;cp.tryRefineMarkers=True;det=cv2.aruco.CharucoDetector(b,cp,p)
def pose(file):
 im=cv2.imread(str(D/file))
 if im is None or im.shape[0]!=896:raise RuntimeError('Missing/wrong camera image; no more motion')
 cc,ci,_,_=det.detectBoard(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))
 if ci is None or len(ci)<8:raise RuntimeError('Board PnP unavailable; inspect before next motion')
 ok,r,t=cv2.solvePnP(b.getChessboardCorners()[ci.flatten()],cc.reshape(-1,2),K,dist)
 R=cv2.Rodrigues(r)[0];return R.T,-R.T@t
old=pose(last)
def call(a):
 r=subprocess.run(['node',str(ROOT/'calibration/arm-calibration/live.mjs'),str(D),json.dumps(a)],cwd=ROOT,text=True,capture_output=True,timeout=100)
 print(r.stdout,flush=True)
 if r.returncode:raise RuntimeError(r.stderr)
for target in plan:
 q=target['j'];delta=np.array(q)-np.array(start)
 if np.count_nonzero(delta)!=1 or np.max(abs(delta))!=10:raise RuntimeError('Must be single joint 10 degree move')
 a1=math.radians(270-q[0]);a2=math.radians(q[1]-q[0]+90)
 wx=.0475+.1775*math.cos(a1)+.205*math.cos(a2);wz=.14+.1775*math.sin(a1)+.205*math.sin(a2)
 if wz<target.get('min_vendor_wrist_z',.10) or wx>.364:raise RuntimeError(f'Conservative model excludes {q}: {wx,wz}')
 stem=f'g{q[0]}-{q[1]}-{q[2]}';file=stem+'.jpg';n=1
 while (D/file).exists():file=f'{stem}-r{n}.jpg';n+=1
 m=[0 if x==0 else (1 if x>0 else 2) for x in delta]
 call([dict(type='jog',m=m,file=file,camera='gripper',joints=q)])
 subprocess.run([py,str(D/'observe69.py'),str(D),last,file],check=True)
 new=pose(file);angle=math.degrees(math.acos(np.clip((np.trace(new[0]@old[0].T)-1)/2,-1,1)));move=float(np.linalg.norm(new[1]-old[1]))
 with (D/'log.jsonl').open('a') as f:f.write(json.dumps(dict(event='jog_observation',file=file,from_joints=start,to_joints=q,rotation_deg=angle,camera_translation_m=move,accepted=angle>2 or move>.008))+'\n')
 print('OBSERVED',q,'rotation',angle,'translation',move,flush=True)
 if angle<=2 and move<=.008:raise RuntimeError('Jog refused/ambiguous: not counted; inspect before next motion')
 start=q;last=file;old=new
 if target.get('front'):
  ffile=file.replace('g','f',1)
  call([dict(type='select',camera='front'),dict(type='wait',ms=3000),dict(type='capture',file=ffile,camera='front',joints=q),dict(type='select',camera='gripper'),dict(type='wait',ms=3000)])
  subprocess.run([py,str(D/'observe69.py'),str(D),ffile],check=True)
print('END',start,last)
