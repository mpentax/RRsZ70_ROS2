import sys,json,subprocess
from pathlib import Path
import cv2,numpy as np
D=Path(__file__).resolve().parent;R=D.parents[2];py=R/'calibration/arm-validation/.venv/bin/python'
p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;p.adaptiveThreshWinSizeMax=53
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),p)
def marker(f):
 im=cv2.imread(str(D/f));c,i,_=det.detectMarkers(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))
 found=[q.reshape(4,2) for q,id in zip(c,i.flatten()) if id==0 and q.reshape(4,2).mean(0)[1]<600] if i is not None else []
 if len(found)!=1:raise RuntimeError('Independent wrist marker unavailable; inspect')
 return found[0]
last=sys.argv[1];j=list(map(int,sys.argv[2].split(',')));old=marker(last)
for q in json.loads(sys.argv[3]):
 delta=np.array(q)-j
 if np.count_nonzero(delta)!=1 or np.max(abs(delta))!=10:raise RuntimeError('Single step required')
 if q[:2]!=j[:2]:raise RuntimeError('This helper permits M5 only')
 m=[0 if v==0 else (1 if v>0 else 2) for v in delta];suffix=('-'+sys.argv[4]) if len(sys.argv)>4 else '';f=f'f{q[0]}-{q[1]}-{q[2]}{suffix}.jpg'
 r=subprocess.run(['node',str(R/'calibration/arm-calibration/live.mjs'),str(D),json.dumps([dict(type='jog',m=m,file=f,camera='front',joints=q)])],capture_output=True,text=True,timeout=80,cwd=R);print(r.stdout,flush=True)
 if r.returncode:raise RuntimeError(r.stderr)
 new=marker(f);shift=float(np.sqrt(np.mean(np.sum((new-old)**2,axis=1))))
 with (D/'log.jsonl').open('a') as out:out.write(json.dumps(dict(event='front_jog_observation',file=f,joints=q,marker_corner_shift_px=shift,accepted=shift>2))+'\n')
 subprocess.run([str(py),str(D.parent/'observe.py'),str(D),f],check=True)
 if shift<2:raise RuntimeError('M5 jog unverified; inspect before next move')
 print('OBSERVED M5',q,'marker shift',shift,flush=True);j=q;old=new;last=f
