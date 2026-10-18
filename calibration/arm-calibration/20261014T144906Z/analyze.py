"""Rotation-based effective calibration. Unobservable anatomy stays null."""
import json,math
from pathlib import Path
import cv2,numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
D=Path(__file__).resolve().parent;ROOT=D.parents[2]
F=json.loads((ROOT/'calibration/front-camera/front_camera.json').read_text());G=json.loads((D.parent/'20261014T090217Z/arm_calibration.json').read_text())['gripper_intrinsics']
Kf=np.array([[F['f_px'],0,F['cx']],[0,F['f_px'],F['cy']],[0,0,1.]]);df=np.array([F['k1'],F['k2'],0,0,0]);Kg=np.array(G['K']);dg=np.array(G['dist'])
Rf=np.array(F['R_floor_to_cam']);Cf=np.array([0,0,F['camera_height_mm']/1000]);Tcf=np.eye(4);Tcf[:3,:3]=Rf.T;Tcf[:3,3]=Cf
p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;p.adaptiveThreshWinSizeMax=53
DI=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000);B=cv2.aruco.CharucoBoard((10,14),.020,.015,DI);B.setLegacyPattern(True);CP=cv2.aruco.CharucoParameters();CP.minMarkers=1;CP.tryRefineMarkers=True
CD=cv2.aruco.CharucoDetector(B,CP,p);AD=cv2.aruco.ArucoDetector(DI,p);OBJ=B.getChessboardCorners().astype(float)
caps={x['file']:x for x in map(json.loads,(D/'log.jsonl').read_text().splitlines()) if x.get('event')=='capture'}
obs=[];mk=[];rejected=[]
for file,c in caps.items():
 if file in ['g200-45-0.jpg','g200-45-settled.jpg','f-partial-200-45.jpg']:rejected.append(dict(file=file,reason='partial jog / joint identity not nominal'));continue
 j=c.get('tracked')
 if file in ['g250-115-0-closed.jpg','f250-115-0-closed.jpg']:j=[230,105,0] # acquisition correction: prior helper stopped at its height guard
 im=cv2.imread(str(D/file));gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);front=im.shape[0]==1200;K,dist=(Kf,df) if front else (Kg,dg)
 if front:
  ccs,ids,_=AD.detectMarkers(gray[:620])
  if ids is not None:
   for q,i in zip(ccs,ids.flatten()):
    if i!=68:continue
    px=q.reshape(4,2);s=.0075;obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]])
    ok,r,t=cv2.solvePnP(obj,px,K,dist,flags=cv2.SOLVEPNP_IPPE_SQUARE);r,t=cv2.solvePnPRefineLM(obj,px,K,dist,r,t)
    uv=cv2.projectPoints(obj,r,t,K,dist)[0].reshape(-1,2);T=np.eye(4);T[:3,:3]=cv2.Rodrigues(r)[0];T[:3,3]=t.ravel()
    mk.append(dict(file=file,j=j,observed_id=int(i),px=px.tolist(),Tcam=T.tolist(),Tcf=(Tcf@T).tolist(),rms=float(np.sqrt(np.mean(np.sum((uv-px)**2,1))))))
  gray[:700]=255 # wrist marker, and raised object, excluded from floor-board identities
 cc,ci,_,_=CD.detectBoard(gray)
 if ci is None or len(ci)<8:
  p0=cv2.aruco.DetectorParameters();p0.adaptiveThreshWinSizeMax=53
  cc,ci,_,_=cv2.aruco.CharucoDetector(B,CP,p0).detectBoard(gray)
 if ci is None or len(ci)<8:continue
 px=cc.reshape(-1,2).astype(float);obj=OBJ[ci.flatten()]
 ok,r,t=cv2.solvePnP(obj,px,K,dist)
 uv=cv2.projectPoints(obj,r,t,K,dist)[0].reshape(-1,2);ix=np.where(np.linalg.norm(uv-px,axis=1)<4)[0].reshape(-1,1)
 if not ok or len(ix)<8:continue
 ix=ix.flatten();r,t=cv2.solvePnPRefineLM(obj[ix],px[ix],K,dist,r,t);uv=cv2.projectPoints(obj,r,t,K,dist)[0].reshape(-1,2);ix=np.where(np.linalg.norm(uv-px,axis=1)<4)[0]
 if len(ix)<8:continue
 r,t=cv2.solvePnPRefineLM(obj[ix],px[ix],K,dist,r,t);R=cv2.Rodrigues(r)[0];uv=cv2.projectPoints(obj[ix],r,t,K,dist)[0].reshape(-1,2)
 obs.append(dict(file=file,front=front,j=j,ids=ci.flatten()[ix].tolist(),px=px[ix].tolist(),rvec=r.ravel().tolist(),tvec=t.ravel().tolist(),camera_in_board=(-R.T@t).ravel().tolist(),rms=float(np.sqrt(np.mean(np.sum((uv-px[ix])**2,1)))),accepted_corners=len(ix)))
(D/'observations.json').write_text(json.dumps(dict(board=obs,wrist_marker=mk,rejected=rejected),indent=1))
fr=[o for o in obs if o['front']];gr_all=[o for o in obs if not o['front']];gr=[o for o in gr_all if o['j'][2]==0]
fobj=np.concatenate([OBJ[o['ids']] for o in fr]);fpx=np.concatenate([o['px'] for o in fr])
def board(v):
 a,x,y=v;return np.array([[math.cos(a),math.sin(a),0],[math.sin(a),-math.cos(a),0],[0,0,-1.]]),np.array([x,y,0.])
def board_res(v):
 R,t=board(v);X=np.einsum('ij,kj->ik',fobj,R)+t-Cf;X=np.einsum('ij,kj->ik',X,Rf)
 assert np.isfinite(X).all()
 return (cv2.projectPoints(X,np.zeros(3),np.zeros(3),Kf,df)[0].reshape(-1,2)-fpx).ravel()
# Initialization directly from PnP transformed into calibrated front floor frame.
o=fr[0];r=cv2.Rodrigues(np.array(o['rvec']))[0];Rb0=Rf.T@r;tb0=Cf+Rf.T@np.array(o['tvec'])
bsol=least_squares(board_res,[math.atan2(Rb0[1,0],Rb0[0,0]),*tb0[:2]],loss='soft_l1',f_scale=3);Rb,tb=board(bsol.x)
def basis(a,yaw=0):
 ca,sa,cy,sy=math.cos(a),math.sin(a),math.cos(yaw),math.sin(yaw);return np.array([[cy*ca,-sy,-cy*sa],[sy*ca,cy,-sy*sa],[sa,0,ca]])
def angles(v,j):
 m3=150+v[12]*(j[0]-150);m4=45+v[13]*(j[1]-45)
 return math.radians(270-m3)+v[5],math.radians(m4-m3+90)
def chain(v,j,endpoint=None):
 a1,a2=angles(v,j);E=v[:3]+v[4]*basis(a1,v[3])[:,0];R=basis(a2,v[3]);return E+R@(v[6:9] if endpoint is None else endpoint),R@cv2.Rodrigues(v[9:12])[0]
def res(v,chosen=gr):
 out=[]
 for o in chosen:
  C,R=chain(v,o['j']);X=OBJ[o['ids']]@Rb.T+tb;Xc=(X-C)@R;uv=cv2.projectPoints(Xc,np.zeros(3),np.zeros(3),Kg,dg)[0].reshape(-1,2);out.extend((uv-np.array(o['px'])).ravel())
 return np.array(out)
old=json.loads((D.parent/'20261014T135228Z/fit.json').read_text());v0=np.r_[list(old['params'].values()),1,1];v0[7]=0.
lo=np.r_[-.35,-.12,.07,-.20,.10,-.6,-.10,-1e-10,-.15,[-6]*3,.85,.85];hi=np.r_[.05,.12,.40,.20,.26,.6,.35,1e-10,.15,[6]*3,1.15,1.15]
sol=least_squares(res,v0,bounds=(lo,hi),loss='soft_l1',f_scale=3,max_nfev=800,xtol=1e-10,ftol=1e-10);v=sol.x
print('CAMERA FIT',v,'RMS',np.sqrt(np.mean(res(v)**2)*2),flush=True)
names=list(old['params'])+['M3_gain','M4_gain'];rng=np.random.default_rng(29);boots=[]
for _ in range(60):
 sample=[gr[i] for i in rng.integers(0,len(gr),len(gr))];ss=least_squares(lambda q:res(q,sample),v,bounds=(lo,hi),loss='soft_l1',f_scale=3,max_nfev=180)
 if ss.success:boots.append(ss.x)
bs=np.std(boots,axis=0,ddof=1)
# Board PnP finite single-joint screw: (I-Q)*axis_point = C2-Q*C1.
byfile={o['file']:o for o in gr};pairs=[]
logs=list(map(json.loads,(D/'log.jsonl').read_text().splitlines()))
# Use capture ordering; excludes camera switches and nominal duplicates automatically.
previous=None
for o in gr:
 if previous is not None and o['j'] is not None and previous['j'] is not None:
  delta=np.array(o['j'])-np.array(previous['j']);where=np.where(delta!=0)[0]
  if len(where)==1 and abs(delta[where[0]])==10:
   R1=cv2.Rodrigues(np.array(previous['rvec']))[0].T;R2=cv2.Rodrigues(np.array(o['rvec']))[0].T;C1=np.array(previous['camera_in_board']);C2=np.array(o['camera_in_board']);Q=R2@R1.T;rv=cv2.Rodrigues(Q)[0].ravel();ang=np.linalg.norm(rv)
   if where[0]<2 and ang>.06:
    A=np.eye(3)-Q;axispoint=np.linalg.pinv(A,rcond=1e-8)@(C2-Q@C1);local=R1.T@(axispoint-C1);axis=R1.T@(rv/ang);local-=axis*np.dot(local,axis)
    pairs.append(dict(before=previous['file'],after=o['file'],joint=int(where[0]+3),command_delta_deg=float(delta[where[0]]),observed_angle_deg=math.degrees(ang),camera_axis_perpendicular_vector_m=local.tolist(),camera_axis_perpendicular_distance_m=float(np.linalg.norm(local))))
 previous=o
# Camera M5 null effect from all same-M3M4 retained views, against last zero.
m5=[o for o in gr_all if o['j'] and o['j'][:2]==[240,105]];zero=next(o for o in m5 if o['j'][2]==0);Rz=cv2.Rodrigues(np.array(zero['rvec']))[0];Cz=np.array(zero['camera_in_board']);m5changes=[]
for o in m5:
 rr=cv2.Rodrigues(np.array(o['rvec']))[0];m5changes.append(dict(file=o['file'],M5=o['j'][2],rotation_deg=math.degrees(np.linalg.norm(cv2.Rodrigues(rr@Rz.T)[0])),translation_m=float(np.linalg.norm(np.array(o['camera_in_board'])-Cz))))
# Re-register observed wrist marker68 at M5=0 using its own newly measured mount.
mk0=[o for o in mk if o['j'] and o['j'][2]==0 and o['rms']<2]
ms=least_squares(lambda q:np.concatenate([chain(v,o['j'],q)[0]-np.array(o['Tcf'])[:3,3] for o in mk0]),[.2,0,.03],loss='soft_l1',f_scale=.002)
rotmounts=[]
for o in mk0:
 a1,a2=angles(v,o['j']);rotmounts.append(basis(a2,v[3]).T@np.array(o['Tcf'])[:3,:3])
Rm=Rotation.from_matrix(np.array(rotmounts)).mean().as_matrix()
# M5 moving marker screw reveals an axis LINE, not a unique wrist origin along that line.
rolled=[o for o in mk if o['j'][:2]==[240,105]];axisvals=[]
for o in rolled:
 z=next(t for t in rolled if t['j'][2]==0);T1=np.array(z['Tcf']);T2=np.array(o['Tcf']);Q=T2[:3,:3]@T1[:3,:3].T;rv=cv2.Rodrigues(Q)[0].ravel();ang=np.linalg.norm(rv)
 if o['j'][2] and ang>.03:
  n=rv/ang;P=np.linalg.pinv(np.eye(3)-Q,rcond=1e-8)@(T2[:3,3]-Q@T1[:3,3]);axisvals.append(dict(M5=o['j'][2],angle_deg=math.degrees(ang),axis_CF=n.tolist(),nearest_point_to_CF_origin_m=P.tolist(),axial_station_unobservable=True))
# Retain prior jaw as prior ONLY; evaluate new rays rather than declaring dark pixels resolved.
jaw=np.array(old['jaw_effective_from_elbow']);priorjawpred={}
for j in [[240,105,0],[250,115,0],[250,105,0]]:
 C,R=chain(v,j);J=chain(v,j,jaw)[0];Xp=R.T@(J-C);uv=cv2.projectPoints(Xp[None,:],np.zeros(3),np.zeros(3),Kg,dg)[0].reshape(2);priorjawpred[str(j)]=dict(CF=J.tolist(),base=(J+[.275,0,0]).tolist(),pincher_pixel=uv.tolist())
# Conditional front seam hypotheses, NOT adopted as pad-centre truth.
ann={f'f240-105-{k}-closed.jpg':[560+x/3,360+y/3,s/3] for k,x,y,s in [(0,345,445,25),(10,350,445,25),(20,367,438,30),(30,388,428,35),(40,406,423,40)]}
selected=[o for o in rolled if o['file'] in ann]
def jr(q):
 out=[]
 for o in selected:
  T=np.array(o['Tcam']);X=T[:3,:3]@q+T[:3,3];uv=cv2.projectPoints(X[None,:],np.zeros(3),np.zeros(3),Kf,df)[0].reshape(2);out.extend((uv-ann[o['file']][:2])/ann[o['file']][2])
 return np.array(out)
js=least_squares(jr,[0,.06,-.02]);jsig=np.sqrt(np.diag(np.linalg.pinv(js.jac.T@js.jac)));z=next(o for o in selected if o['j'][2]==0);T=np.array(z['Tcf']);a1,a2=angles(v,z['j']);conditionaljaw=ms.x+basis(a2,v[3]).T@T[:3,:3]@js.x
# Measured target range from final front HSV contour, mapped via frozen calibrated floor rays.
im=cv2.imread(str(D/'99-stowed-front.jpg'));hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);mask=cv2.inRange(hsv,np.array([12,80,70]),np.array([45,255,255]));mask[:550]=0;mask[800:]=0;mask[:,:600]=0;mask[:,950:]=0
cts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE);ct=max(cts,key=cv2.contourArea).reshape(-1,2);low=ct[ct[:,1]>=np.percentile(ct[:,1],97)];contact=np.median(low,axis=0)
def floorxy(px):
 ray=Rf.T@np.r_[cv2.undistortPoints(np.array(px,dtype=float).reshape(1,1,2),Kf,df).ravel(),1.];return Cf-ray*Cf[2]/ray[2]
# HSV merges this yellow target with the wood and its shadow; use reviewed target-only contact.
contact=np.array([784.,744.])
obj=floorxy(contact);objsamples=np.array([floorxy(contact+rng.normal(0,[8,5])) for _ in range(500)])
out=dict(status='rotation_measurements_complete_anatomy_and_true_pad_centre_not_resolved',counts=dict(retained_raw_frames=len(caps),gripper_board=len(gr),front_board=len(fr),wrist_marker=len(mk),frame_bootstraps=len(boots)),expected_wrist_id=69,observed_wrist_id=68,wrist_dictionary='DICT_4X4_1000',params=dict(zip(names,v.tolist())),frame_bootstrap_sigma=dict(zip(names,bs.tolist())),camera_effective_mount_rotation=cv2.Rodrigues(v[9:12])[0].tolist(),gripper_point_rms_px=float(np.sqrt(np.mean(res(v)**2)*2)),front_board_point_rms_px=float(np.sqrt(np.mean(board_res(bsol.x)**2)*2)),previous_chain_same_data_point_rms_px=float(np.sqrt(np.mean(res(v0)**2)*2)),board_CF=dict(R=Rb.tolist(),t=tb.tolist()),rotation_pairs=pairs,M5_camera_changes=m5changes,M5_marker_axes=axisvals,marker_observed68_mount=dict(effective_vector_from_elbow_m=ms.x.tolist(),rotation_link2_to_marker=Rm.tolist(),position_point_rms_m=float(np.sqrt(np.mean(ms.fun**2)*3)),frames=[o['file'] for o in mk0]),unidentifiable=dict(physical_L2_m=None,M4_zero_deg=None,physical_marker_to_wrist_mount=None,jaw_true_pad_center_m=None),prior_jaw_predictions_under_new_camera_fit=priorjawpred,conditional_front_seam=dict(status='inferred seam not identifiable material point; NOT ADOPTED',marker_frame_m=js.x.tolist(),annotation_sigma_m=jsig.tolist(),effective_vector_from_elbow_m=conditionaljaw.tolist(),annotations=ann,normalized_coordinate_rms=float(np.sqrt(np.mean(js.fun**2)))),closed_pincher_region=dict(owner_region_px=[700,65,890,110],actual_fingertip_pixel=None,status='cropped dark assembly; specific finger contact not discernible even in contrast-enhanced crops'),object_final=dict(contact_pixel=contact.tolist(),CF_m=obj.tolist(),rotation_center_m=(obj+[.275,0,0]).tolist(),pixel_bootstrap_sigma_m=np.std(objsamples,axis=0,ddof=1).tolist(),size_m=.066,equator_height_m=.033),limitations=['Rotations determine an axis line, not a marked physical station along the axis. L2 and M4 zero remain gauge-confounded with unknown rigid mounts.','Owner reported ID69; observed elevated marker consistently decodes68. Never reuse ID0 offsets or silently relabel68 as69.','The exact closed fingertip material point cannot be seen in gripper frames; topmiddle region is not a measured ray.','Sigmas conditional on intrinsics, rigid printed marker, planar board and frame assignments; not absolute physical uncertainties.','Acquisition filenames containing250-115 without actual suffix were at230-105; corrected here. Partial200-45 jog excluded.','One repeated gripper zero filename was overwritten by return frame; unique retained files deduplicated.'])
# Independent rotation summaries and a multi-view M5 axis line.
rotstats={}
for joint in [3,4]:
 ps=[q for q in pairs if q['joint']==joint];aa=np.array([q['observed_angle_deg'] for q in ps]);dd=np.array([q['camera_axis_perpendicular_distance_m'] for q in ps])
 rotstats[str(joint)]=dict(n=len(ps),step_mean_deg=float(aa.mean()),step_std_deg=float(aa.std(ddof=1)),step_mean_sigma_deg=float(aa.std(ddof=1)/np.sqrt(len(aa))),gain=float(aa.mean()/10),axis_perpendicular_distance_mean_m=float(dd.mean()),axis_perpendicular_distance_std_m=float(dd.std(ddof=1)))
zeroR=Rb@Rz.T;zeroC=Rb@Cz+tb;axes=[];lineA=[];lineb=[]
for o in m5:
 if o['j'][2]<20:continue
 Ri=Rb@cv2.Rodrigues(np.array(o['rvec']))[0].T;Ci=Rb@np.array(o['camera_in_board'])+tb;Q=Ri@zeroR.T;rv=cv2.Rodrigues(Q)[0].ravel();n=rv/np.linalg.norm(rv);axes.append(n);lineA.append(np.eye(3)-Q);lineb.append(Ci-zeroC)
axis=np.mean(axes,axis=0);axis/=np.linalg.norm(axis);A=np.vstack(lineA);line_target=np.concatenate(lineb);axoff=np.linalg.lstsq(np.vstack([A,axis[None,:]]),np.r_[line_target,0.],rcond=1e-8)[0]
axis_pitch=math.atan2(axis[2],np.hypot(axis[0],axis[1]));a1,a2=angles(v,[240,105,0]);gauge_rotation=axis_pitch-a2
conditional_zero_deg=math.degrees(gauge_rotation-v[5]);zero_samples=[]
for vb in boots:
 _,a2b=angles(vb,[240,105,0]);zero_samples.append(math.degrees(axis_pitch-a2b-vb[5]))
mountsamples=[]
for vb in boots:
 chosen=[mk0[i] for i in rng.integers(0,len(mk0),len(mk0))];coords=[]
 for o in chosen:
  a,b=angles(vb,o['j']);E=chain(vb,o['j'],np.zeros(3))[0];coords.append(basis(b,vb[3]).T@(np.array(o['Tcf'])[:3,3]-E))
 mountsamples.append(np.mean(coords,axis=0))
out['rotation_statistics']=rotstats
out['M5_camera_axis']=dict(direction_CF=axis.tolist(),direction_component_std=np.std(axes,axis=0,ddof=1).tolist(),nearest_point_to_zero_lens_CF_m=(zeroC+axoff).tolist(),perpendicular_lens_offset_m=float(np.linalg.norm(axoff)),axis_line_fit_point_rms_m=float(np.sqrt(np.mean((A@axoff-line_target)**2)*3)),axial_station_unobservable=True)
out['conditional_M4_zero']=dict(offset_deg=conditional_zero_deg,frame_bootstrap_sigma_deg=float(np.std(zero_samples,ddof=1)),link2_gauge_rotation_deg=math.degrees(gauge_rotation),definition='physical elbow bend M4cal + offset, with link2 direction DEFINED by measured M5 roll axis; excludes axis/intrinsics systematics',adopted=False)
out['marker_observed68_mount']['frame_bootstrap_sigma_m']=np.std(mountsamples,axis=0,ddof=1).tolist()
out['counts']['all_gripper_board']=len(gr_all)
out['adoption']=dict(adopted='M5 camera positive roll and lens lever-arm only; keep prior M5=0 chain and provisional jaw',not_adopted='new effective gain/length fit, conditional M4-zero, inferred front seam: floor-constrained front board RMS 15.72px and unknown exact pad centre')
out['object_final']['method']='reviewed target-only visible bottom edge, not merged wood/shadow HSV contour; floor projection is visible-contact estimate, not exact object centre'
out['unidentifiable_sigma']=dict(physical_L2_m=None,physical_M4_zero_deg=None,exact_pad_center_m=None)
(D/'fit.json').write_text(json.dumps(out,indent=1));print(json.dumps({k:out[k] for k in ['counts','params','frame_bootstrap_sigma','gripper_point_rms_px','front_board_point_rms_px','previous_chain_same_data_point_rms_px','marker_observed68_mount','rotation_statistics','M5_camera_axis','conditional_M4_zero','object_final']},indent=1))
