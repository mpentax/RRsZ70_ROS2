"""Measured effective chain; physical L2/M4-zero remain gauge ambiguous."""
import json,math
from pathlib import Path
import cv2,numpy as np
from scipy.optimize import least_squares
D=Path(__file__).resolve().parent;ROOT=D.parents[2]
F=json.loads((ROOT/'calibration/front-camera/front_camera.json').read_text());G=json.loads((D.parent/'20261014T090217Z/arm_calibration.json').read_text())['gripper_intrinsics']
Kf=np.array([[F['f_px'],0,F['cx']],[0,F['f_px'],F['cy']],[0,0,1.]]);df=np.array([F['k1'],F['k2'],0,0,0]);Kg=np.array(G['K']);dg=np.array(G['dist'])
Rf=np.array(F['R_floor_to_cam']);Cf=np.array([0,0,F['camera_height_mm']/1000]);Tcf=np.eye(4);Tcf[:3,:3]=Rf.T;Tcf[:3,3]=Cf
p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;p.adaptiveThreshWinSizeMax=53
DI=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000);B=cv2.aruco.CharucoBoard((10,14),.020,.015,DI);B.setLegacyPattern(True);CP=cv2.aruco.CharucoParameters();CP.minMarkers=1;CP.tryRefineMarkers=True
CD=cv2.aruco.CharucoDetector(B,CP,p);AD=cv2.aruco.ArucoDetector(DI,p);OBJ=B.getChessboardCorners().astype(float)
caps={x['file']:x for x in map(json.loads,(D/'log.jsonl').read_text().splitlines()) if x.get('event')=='capture'}
obs=[];mk=[]
for file,c in caps.items():
 im=cv2.imread(str(D/file));gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);front=im.shape[0]==1200;K,dist=(Kf,df) if front else (Kg,dg)
 ccs,ids,_=AD.detectMarkers(gray)
 if front and ids is not None:
  for q,i in zip(ccs,ids.flatten()):
   px=q.reshape(4,2)
   if i==0 and px.mean(0)[1]<600:
    s=.0075;obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]])
    ok,r,t=cv2.solvePnP(obj,px,K,dist,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    # refine pose; planar 4 points still subject to front-intrinsics systematics
    r,t=cv2.solvePnPRefineLM(obj,px,K,dist,r,t)
    uv=cv2.projectPoints(obj,r,t,K,dist)[0].reshape(-1,2);T=np.eye(4);T[:3,:3]=cv2.Rodrigues(r)[0];T[:3,3]=t.ravel()
    mk.append(dict(file=file,j=c['tracked'],px=px.tolist(),Tcam=T.tolist(),Tcf=(Tcf@T).tolist(),rms=float(np.sqrt(np.mean(np.sum((uv-px)**2,1))))))
 if front:gray[:620]=255 # wrist ID0 must not contaminate floor-board corner identities
 cc,ci,_,_=CD.detectBoard(gray)
 if ci is None or len(ci)<8:continue
 px=cc.reshape(-1,2).astype(float);obj=OBJ[ci.flatten()]
 ok,r,t,ix=cv2.solvePnPRansac(obj,px,K,dist,iterationsCount=300,reprojectionError=4,confidence=.999,flags=cv2.SOLVEPNP_ITERATIVE)
 if not ok or ix is None or len(ix)<8:continue
 ix=ix.flatten();r,t=cv2.solvePnPRefineLM(obj[ix],px[ix],K,dist,r,t)
 uv=cv2.projectPoints(obj,r,t,K,dist)[0].reshape(-1,2);err=np.linalg.norm(uv-px,axis=1);ix=np.where(err<4)[0]
 if len(ix)<8:continue
 r,t=cv2.solvePnPRefineLM(obj[ix],px[ix],K,dist,r,t)
 R=cv2.Rodrigues(r)[0];uv=cv2.projectPoints(obj[ix],r,t,K,dist)[0].reshape(-1,2)
 obs.append(dict(file=file,front=front,j=c['tracked'],ids=ci.flatten()[ix].tolist(),px=px[ix].tolist(),rvec=r.ravel().tolist(),tvec=t.ravel().tolist(),camera_in_board=(-R.T@t).ravel().tolist(),rms=float(np.sqrt(np.mean(np.sum((uv-px[ix])**2,1)))),all_corners=len(ci),accepted_corners=len(ix)))
(D/'observations.json').write_text(json.dumps(dict(board=obs,wrist_marker=mk),indent=1))
# Board fixed to calibrated floor: initialize planar transform via front pixel rays.
fr=[o for o in obs if o['front']];gr=[o for o in obs if not o['front']]
XY=[];PQ=[]
for o in fr:
 rays=cv2.undistortPoints(np.array(o['px']).reshape(-1,1,2),Kf,df).reshape(-1,2);rays=np.c_[rays,np.ones(len(rays))]@Rf
 for ray,id in zip(rays,o['ids']):
  if ray[2]<0:XY.append((Cf-ray*Cf[2]/ray[2])[:2]);PQ.append(OBJ[id,:2])
XY=np.array(XY);PQ=np.array(PQ)
def bres(v):
 a,x,y=v;R=np.array([[math.cos(a),math.sin(a)],[math.sin(a),-math.cos(a)]])
 return (PQ@R.T+[x,y]-XY).ravel()
bsol=least_squares(bres,[-math.pi/2,.365,.127],loss='soft_l1',f_scale=.003)
# refine planar board mount in front pixel space
fobj=np.concatenate([OBJ[o['ids']] for o in fr]);fpx=np.concatenate([o['px'] for o in fr])
def board(v):
 a,x,y=v;R=np.array([[math.cos(a),math.sin(a),0],[math.sin(a),-math.cos(a),0],[0,0,-1.]])
 return R,np.array([x,y,0.])
def board_res(v):
 R,t=board(v);X=(fobj@R.T+t-Cf)@Rf.T;uv=cv2.projectPoints(X,np.zeros(3),np.zeros(3),Kf,df)[0].reshape(-1,2);return (uv-fpx).ravel()
bsol=least_squares(board_res,bsol.x,loss='soft_l1',f_scale=3);Rb,tb=board(bsol.x)
# Effective camera chain: S + L1*e(a1+o3) + R_link2 * effective_camera_vector.
# a2 is fixed to nominal m4-m3+90; M4 offset absorbed in effective vector and mount orientation.
def basis(a,yaw):
 ca,sa,cy,sy=math.cos(a),math.sin(a),math.cos(yaw),math.sin(yaw)
 return np.array([[cy*ca,-sy,-cy*sa],[sy*ca,cy,-sy*sa],[sa,0,ca]])
def chain(v,j,endpoint=None):
 sx,sy,sz,yaw,L1,o3=v[:6];a1=math.radians(270-j[0])+o3;a2=math.radians(j[1]-j[0]+90)
 E=np.array([sx,sy,sz])+L1*basis(a1,yaw)[:,0];R=basis(a2,yaw)
 return E+R@(v[6:9] if endpoint is None else endpoint),R@cv2.Rodrigues(v[9:12])[0]
def res(v,chosen=gr):
 out=[]
 for o in chosen:
  C,R=chain(v,o['j']);X=OBJ[o['ids']]@Rb.T+tb;Xc=(X-C)@R;uv=cv2.projectPoints(Xc,np.zeros(3),np.zeros(3),Kg,dg)[0].reshape(-1,2);out.extend((uv-np.array(o['px'])).ravel())
 return np.array(out)
R0=np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
v0=np.r_[-.155,.026,.21,0,.1775,0,.13,0,0,cv2.Rodrigues(R0)[0].ravel()]
lo=np.r_[-.35,-.12,.10,-.20,.10,-.5,-.10,-1e-9,-.15,[-6]*3];hi=np.r_[.0,.12,.40,.20,.26,.5,.35,1e-9,.15,[6]*3]
sol=least_squares(res,v0,bounds=(lo,hi),loss='soft_l1',f_scale=3,max_nfev=1000,xtol=1e-11,ftol=1e-11,gtol=1e-11)
rms=float(np.sqrt(np.mean(sol.fun**2)));sigma=np.sqrt(np.diag(np.linalg.pinv(sol.jac.T@sol.jac))*(np.sum(sol.fun**2)/(len(sol.fun)-len(sol.x))))
params=sol.x
print('BOARD',bsol.x,'front scalar RMS',np.sqrt(np.mean(board_res(bsol.x)**2)))
print('COUNTS',len(gr),len(fr),len(mk),'CAMERA FIT',params,'SIGMA',sigma,'scalar RMS',rms)
# Marker effective location relative to elbow, independently fitted to front PnP.
mk0=[o for o in mk if o['j'][2]==0 and o['rms']<2]
def marker_res(q):return np.concatenate([(chain(params,o['j'],q)[0]-np.array(o['Tcf'])[:3,3]) for o in mk0])
ms=least_squares(marker_res,[.20,0,.02],loss='soft_l1',f_scale=.002)
print('MARKER effective vector',ms.x,'position RMS mm',1000*np.sqrt(np.mean(ms.fun**2)))
# 3D closed seam midpoint, triangulated via rigid marker PnP: measured pixels with 6-9px annotation uncertainty.
annotations={'f250-115-closed-0.jpg':[680,630,8],'f250-115-10-closed.jpg':[681,632,6],'f250-115-20-closed.jpg':[687,629,6],'f250-115-30-closed.jpg':[695,625,9],'f250-115-40-closed.jpg':[700,626,9]}
selected=[o for o in mk if o['file'] in annotations]
def jaw_res(q):
 out=[]
 for o in selected:
  T=np.array(o['Tcam']);X=T[:3,:3]@q+T[:3,3];uv=cv2.projectPoints(X[None,:],np.zeros(3),np.zeros(3),Kf,df)[0].reshape(2)
  out.extend((uv-np.array(annotations[o['file']][:2]))/annotations[o['file']][2])
 return np.array(out)
js=least_squares(jaw_res,[0,.04,.01]);jcov=np.linalg.pinv(js.jac.T@js.jac);jsig=np.sqrt(np.diag(jcov))
points=[]
for o in selected:
 T=np.array(o['Tcf']);points.append(T[:3,:3]@js.x+T[:3,3])
closed_base=np.array(points);j0=selected[0]
# Rotate jaw marker offset to nominal link2 gauge using 0-M5 closed marker frame.
base0=next(o for o in selected if o['j'][2]==0);Tm=np.array(base0['Tcf']);Rlink=basis(math.radians(base0['j'][1]-base0['j'][0]+90),params[3]);offset=Rlink.T@Tm[:3,:3]@js.x
jawvec=ms.x+offset
jawrms_px=float(np.sqrt(np.mean(np.array([((jaw_res(js.x)[2*i:2*i+2])*annotations[o['file']][2])**2 for i,o in enumerate(selected)]))))
print('JAW marker',js.x,'sigma',jsig,'link2 effective from elbow',jawvec,'rms_px',jawrms_px,'closed CF positions',closed_base)
# With nominal L2 convention, express camera/jaw offsets relative to vendor wrist endpoint, NOT physical wrist measurement.
per=[]
for o in gr:per.append(dict(file=o['file'],j=o['j'],rms_px=float(np.sqrt(np.mean(res(params,[o])**2)*2))))
# frame-block bootstrap handles repeated corners from same pose (intrinsics/board systematic excluded).
rng=np.random.default_rng(17);boot=[]
for _ in range(100):
 sample=[gr[i] for i in rng.integers(0,len(gr),len(gr))]
 ss=least_squares(lambda v:res(v,sample),params,bounds=(lo,hi),loss='soft_l1',f_scale=3,max_nfev=150)
 if ss.success:boot.append(ss.x)
bs=np.std(boot,axis=0,ddof=1)
name=['shoulder_CF_x','shoulder_CF_y','shoulder_CF_z','arm_yaw_rad','L1_m','M3_offset_rad','camera_effective_x','camera_effective_y','camera_effective_z','camera_mount_r0','camera_mount_r1','camera_mount_r2']
poses={}
for j in [[240,105,0],[230,95,0],[250,115,0]]:
 xyz=chain(params,j,jawvec)[0];poses[str(j[:2])]=dict(CF=xyz.tolist(),rotation_center_frame=[xyz[0]+.275,xyz[1],xyz[2]])
choices=[]
for m3 in range(90,271,10):
 for m4 in range(35,171,10):
  a1=math.radians(270-m3);a2=math.radians(m4-m3+90);wx=.0475+.1775*math.cos(a1)+.205*math.cos(a2)
  xyz=chain(params,[m3,m4,0],jawvec)[0]
  if wx<.364 and .45<=xyz[0]+.275<=.50 and .015<xyz[2]<.06:choices.append(dict(j=[m3,m4],jaw_CF=xyz.tolist(),jaw_rotation_center_x=xyz[0]+.275,height_error=abs(xyz[2]-.033)))
choices.sort(key=lambda x:x['height_error'])
out=dict(status='effective_geometry_identified_physical_wrist_gauge_unresolved',counts=dict(gripper_board=len(gr),front_board=len(fr),front_wrist=len(mk),distinct_gripper_M3M4=len(set(tuple(o['j'][:2]) for o in gr)),closed_jaw_views=len(selected)),board_in_CF=dict(r=Rb.tolist(),t=tb.tolist(),front_scalar_rms_px=float(np.sqrt(np.mean(board_res(bsol.x)**2)))),params=dict(zip(name,params.tolist())),linearized_sigma=dict(zip(name,sigma.tolist())),frame_bootstrap_sigma=dict(zip(name,bs.tolist())),bootstrap_count=len(boot),gripper_scalar_rms_px=rms,gripper_point_rms_px=rms*math.sqrt(2),per_frame=per,marker_effective_from_elbow=ms.x.tolist(),marker_position_scalar_rms_m=float(np.sqrt(np.mean(ms.fun**2))),jaw_marker_frame=js.x.tolist(),jaw_marker_sigma_m=jsig.tolist(),jaw_effective_from_elbow=jawvec.tolist(),jaw_nominal_vendor_wrist_offset=(jawvec-[.205,0,0]).tolist(),camera_nominal_vendor_wrist_offset=(params[6:9]-[.205,0,0]).tolist(),jaw_scalar_rms_px=jawrms_px,jaw_annotations=annotations,jaw_poses=poses,candidate_poses=choices,unidentifiable=dict(L2_m=None,M4_offset_deg=None,physical_wrist_marker_mount=None),vendor=dict(shoulder_vendor_x_m=.0475,shoulder_vendor_z_m=.14,L1_m=.1775,L2_m=.205,M3_offset_deg=0,M4_offset_deg=0),limitations=['M4 zero and physical L2 trade exactly against free rigid marker/camera mount. ID0 attachment transform is not independently measured.','Front intrinsics were floor-only calibrated; elevated-marker systematic error excluded from sigmas.','Closed jaw contact region is blurred; seam midpoint is a provisional annotation, not a proven pad-centre.','Pose recommendations are offline predictions, not exercised grasps.'])
# Compare vendor link1/zero with best-fitting origin and free camera mount; same data, same gauges.
free=[i for i in range(12) if i not in (4,5,7)]
vp=params.copy();vp[4]=.1775;vp[5]=0.;vp[7]=0.
def vr(v):
 q=vp.copy();q[free]=v;return res(q)
vs=least_squares(vr,vp[free],bounds=(lo[free],hi[free]),loss='soft_l1',f_scale=3,max_nfev=400)
vendor_rms=float(np.sqrt(np.mean(vs.fun**2))*math.sqrt(2))
# Five folds hold out entire poses, not correlated corners from one image.
cv=[]
keys=sorted(set(tuple(o['j'][:2]) for o in gr))
for fold in range(5):
 held=set(keys[fold::5]);train=[o for o in gr if tuple(o['j'][:2]) not in held];test=[o for o in gr if tuple(o['j'][:2]) in held]
 ss=least_squares(lambda v:res(v,train),params,bounds=(lo,hi),loss='soft_l1',f_scale=3,max_nfev=300)
 cv.append(dict(poses=[list(x) for x in sorted(held)],rms_px=float(np.sqrt(np.mean(res(ss.x,test)**2))*math.sqrt(2))))
out['vendor_fixed_L1_zero_M3_free_mount_point_rms_px']=vendor_rms
out['held_out_pose_folds']=cv
out['frame']='CF: floor under front camera; x forward, y left, z up. base_footprint/rotation centre adds +0.275m x.'
out['gauge_conventions']=['Nominal link2 orientation uses M4-M3+90 degrees: M4-zero is absorbed in mount vectors/orientation.','L2=0.205m only defines nominal wrist endpoint; all reported nominal-wrist offsets change oppositely with L2.','Camera lateral effective vector fixed to zero to define arm-plane y; shoulder_y is convention-dependent.']
out['linearized_sigma']['camera_effective_y']=None
out['frame_bootstrap_sigma']['camera_effective_y']=None
out['unidentifiable']['physical_shoulder_y']=None
out['shoulder_base_m']=[float(params[0]+.275),float(params[1]),float(params[2])]
out['M3_offset_deg']=math.degrees(params[5]);out['M3_offset_sigma_deg']=math.degrees(bs[5])
# Contact-point covariance includes pixel annotation uncertainty, not only residual scatter.
out['jaw_contact_uncertainty_bound_m']=.010
out['jaw_contact_status']='provisional closed contact-region seam midpoint, multi-view metric estimate; actual pad centre unresolved'
# Continuous equator candidates; firmware only exposes 10deg jogging, so these are NOT executable joint targets.
continuous=[]
for x in [.45,.475,.50]:
 def target_res(j):
  q=chain(params,[j[0],j[1],0],jawvec)[0];return [q[0]+.275-x,q[2]-.033]
 ss=least_squares(target_res,[250,110],bounds=([230,85],[270,150]))
 a1=math.radians(270-ss.x[0]);a2=math.radians(ss.x[1]-ss.x[0]+90)
 vendor_wrist_x=.0475+.1775*math.cos(a1)+.205*math.cos(a2)
 continuous.append(dict(rotation_center_x_m=x,M3_M4_deg=ss.x.tolist(),jaw_CF=chain(params,[*ss.x,0],jawvec)[0].tolist(),executable_by_10deg_jogs=False,vendor_wrist_x_m=vendor_wrist_x,within_conservative_firmware_reach=bool(vendor_wrist_x<=.364)))
out['continuous_equator_solutions']=continuous
(D/'fit.json').write_text(json.dumps(out,indent=1));print('BOOTSTRAP',bs,'POSES',poses,'CANDIDATES',choices[:8],'VENDOR_RMS',vendor_rms,'HOLDOUT',cv,'CONTINUOUS',continuous)
