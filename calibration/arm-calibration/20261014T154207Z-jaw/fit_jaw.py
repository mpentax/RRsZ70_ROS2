"""Fit a rigid terminal-pad region centre from observed marker poses and rays.
The rubber contact seam is unresolved: region centre is a measured surrogate.
"""
import json,math
from pathlib import Path
import numpy as np,cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from measure import D,ROOT,K,dist,Rf,C
ann={'f240-85-closed.jpg':[630,749,10,20], 'f240-95-0-open.jpg':[680,657,6,7], 'f240-105-0-open.jpg':[673,520,7,6], 'f240-115-0-open.jpg':[679,442,8,7], 'f250-115-0-open.jpg':[676,664,7,7]}
obs={r['file']:r['markers'][0] for r in map(json.loads,(D/'marker.jsonl').read_text().splitlines()) if r['markers']}
def residual(q,annotations=ann):
 out=[]
 for file,a in annotations.items():
  o=obs[file];X=np.array(o['Rcam'])@q+o['tcam'];uv=cv2.projectPoints(X[None,:],np.zeros(3),np.zeros(3),K,dist)[0].reshape(2);out.extend((uv-a[:2])/a[2:])
 return np.array(out)
s=least_squares(residual,[0,.06,-.02]);q=s.x;sig=np.sqrt(np.diag(np.linalg.pinv(s.jac.T@s.jac)))
poses={}
mount=json.loads((ROOT/'ros2/src/z70_description/config/mount_calibration.yaml').read_text().split('\n',1)[1]);yaw=math.radians(mount['arm_yaw_deg'])
def basis(j):
 a=math.radians(j[1]-j[0]+90);cy,sy=math.cos(yaw),math.sin(yaw);ca,sa=math.cos(a),math.sin(a);return np.array([[cy*ca,-sy,-cy*sa],[sy*ca,cy,-sy*sa],[sa,0,ca]])
joints={'f240-85-closed.jpg':[240,85], 'f240-95-0-open.jpg':[240,95], 'f240-105-0-open.jpg':[240,105], 'f240-115-0-open.jpg':[240,115], 'f250-115-0-open.jpg':[250,115]}
vectors=[];markers=[];rotations=[]
for f,j in joints.items():
 o=obs[f];J=np.array(o['position_CF'])+np.array(o['R_CF'])@q
 a1=math.radians(270-j[0]+mount['measured_M3_offset_deg']);E=np.array(mount['measured_shoulder_base_m'])-[.275,0,0]+mount['measured_L1_m']*np.array([math.cos(a1)*math.cos(yaw),math.cos(a1)*math.sin(yaw),math.sin(a1)])
 v=basis(j).T@(J-E);vectors.append(v);markers.append(basis(j).T@(np.array(o['position_CF'])-E));rotations.append(basis(j).T@np.array(o['R_CF']))
 poses[f]=dict(j=j,jaw_CF=J.tolist(),jaw_base=(J+[.275,0,0]).tolist(),effective_from_elbow=v.tolist())
out=dict(status='terminal-pad-region surrogate; contact seam unresolved',marker_id=68,marker_black_side_m=.015,jaw_region_in_marker_m=q.tolist(),annotation_conditional_sigma_m=sig.tolist(),normalized_coordinate_rms=float(np.sqrt(np.mean(s.fun**2))),annotations=ann,poses=poses,effective_jaw_from_elbow_m=np.mean(vectors,axis=0).tolist(),effective_vector_scatter_m=np.std(vectors,axis=0,ddof=1).tolist(),marker_effective_from_elbow_m=np.mean(markers,axis=0).tolist(),marker_rotation_link2=Rotation.from_matrix(rotations).mean().as_matrix().tolist(),physical_M4_zero_adopted=False,uncertainty_bound_m=.015)
(D/'jaw-fit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
