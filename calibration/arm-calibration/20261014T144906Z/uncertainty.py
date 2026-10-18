"""Frame resampling for the measured M5 lens lever arm; no robot access."""
import json
from pathlib import Path
import cv2,numpy as np
D=Path(__file__).resolve().parent;f=json.loads((D/'fit.json').read_text());obs=json.loads((D/'observations.json').read_text())['board'];m=[o for o in obs if not o['front'] and o['j'][:2]==[240,105]];z=next(o for o in m if o['j'][2]==0);Rb=np.array(f['board_CF']['R']);tb=np.array(f['board_CF']['t']);R0=Rb@cv2.Rodrigues(np.array(z['rvec']))[0].T;C0=Rb@np.array(z['camera_in_board'])+tb
A=[];b=[];axes=[]
for o in m:
 if o['j'][2]<20:continue
 R=Rb@cv2.Rodrigues(np.array(o['rvec']))[0].T;C=Rb@np.array(o['camera_in_board'])+tb;Q=R@R0.T;rv=cv2.Rodrigues(Q)[0].ravel();axes.append(rv/np.linalg.norm(rv));A.append(np.eye(3)-Q);b.append(C-C0)
rng=np.random.default_rng(41);samples=[]
for _ in range(800):
 ids=rng.integers(0,len(A),len(A));axis=np.mean([axes[i] for i in ids],axis=0);axis/=np.linalg.norm(axis);aa=np.vstack([A[i] for i in ids]+[axis[None,:]]);bb=np.r_[np.concatenate([b[i] for i in ids]),0.];v=np.linalg.lstsq(aa,bb,rcond=1e-8)[0];samples.append(np.linalg.norm(v))
f['M5_camera_axis']['frame_bootstrap_lens_offset_sigma_m']=float(np.std(samples,ddof=1));f['M5_camera_axis']['uncertainty_note']='800 resamples of nonzero M5 frames with zero reference held fixed; excludes board/lens/zero-reference systematics'
(D/'fit.json').write_text(json.dumps(f,indent=1));print(f['M5_camera_axis'])
