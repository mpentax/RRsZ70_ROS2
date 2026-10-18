import json,subprocess,sys
from pathlib import Path
import cv2,numpy as np
D=Path(__file__).resolve().parent;ROOT=D.parents[2]
A=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000))
def marker(f):
 c,i,_=A.detectMarkers(cv2.imread(str(D/f))[:620])
 return next((q.reshape(4,2) for q,n in zip(c,i.flatten()) if n==68),None) if i is not None else None
last=sys.argv[1];old=marker(last);previous=int(last.split('-')[2])
for m5 in json.loads(sys.argv[2]):
 name=f'f240-105-{m5}-closed.jpg';g=f'g240-105-{m5}-closed.jpg'
 if (D/name).exists():name=name.replace('.jpg','-return.jpg');g=g.replace('.jpg','-return.jpg')
 actions=[dict(type='select',camera='front'),dict(type='jog',m=[0,0,1 if m5>previous else 2],file=name,camera='front',joints=[240,105,m5]),dict(type='select',camera='gripper'),dict(type='capture',file=g,camera='gripper',joints=[240,105,m5])]
 r=subprocess.run(['node',str(D.parent/'live.mjs'),str(D),json.dumps(actions)],cwd=ROOT,text=True,capture_output=True,timeout=100);print(r.stdout,flush=True)
 if r.returncode:raise RuntimeError(r.stderr)
 new=marker(name)
 if old is None or new is None:raise RuntimeError('marker unavailable: inspect before more motion')
 delta=float(np.sqrt(np.mean((new-old)**2)));print('M5 marker point displacement',delta,flush=True)
 with (D/'log.jsonl').open('a') as f:f.write(json.dumps(dict(event='m5_marker_check',file=name,observed_id=68,corner_coordinate_rms_displacement_px=delta))+'\n')
 if delta<2:raise RuntimeError('M5 did not visibly rotate marker')
 old=new;previous=m5
