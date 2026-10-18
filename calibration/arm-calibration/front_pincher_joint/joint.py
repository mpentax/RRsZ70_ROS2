"""Retained-image optical bundle; metres; T_A_B maps B into A."""
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
MARKER=np.array([[-.0075,.0075,0],[.0075,.0075,0],[.0075,-.0075,0],[-.0075,-.0075,0]])
def output_guard(out,sources):
    out=Path(out).resolve()
    for source in sources:
        source=Path(source).resolve()
        if out==source or source in out.parents or out in source.parents:
            raise ValueError('Output overlaps preserved source tree')
    return out

def project(X,T,K,d):
    return cv2.projectPoints(np.asarray(X,float),Rotation.from_matrix(T[:3,:3]).as_rotvec(),T[:3,3],K,d)[0].reshape(-1,2)

def gripper_pose(data,p):
    init=camera_pose(p['gripper'][0],data['Kg'],data['dg'])
    def res(v):
        T=np.linalg.inv(mat(v))
        return np.concatenate([((project(o['obj'],T,data['Kg'],data['dg'])-np.array(o['px']))/np.sqrt(len(o['px']))).ravel() for o in p['gripper']])
    sol=least_squares(res,vec(init),loss='soft_l1',f_scale=.5,max_nfev=150)
    if not sol.success:raise ValueError(f'G-only pose {p["id"]}: nonconverged ({sol.message})')
    WG=mat(sol.x);T=np.linalg.inv(WG)
    if not all(np.all((np.array(o['obj'])@T[:3,:3].T+T[:3,3])[:,2]>0) for o in p['gripper']):
        raise ValueError(f'G-only pose {p["id"]}: board corners lack positive depth')
    return WG

def evaluate(data,pairs,fit):
    K=np.array(fit['K']);d=np.array(fit['dist']);TF=np.array(fit['T_F_W']);M=np.array(fit['T_G_M']);WGs=[gripper_pose(data,p) for p in pairs]
    ee,depth=errors(data,pairs,TF,M,K,d,WGs);poses=[]
    for p,WG in zip(pairs,WGs):
        pred=TF@WG@M;diag=[]
        for o in p['front']:
            ok,rs,ts,_=cv2.solvePnPGeneric(MARKER,np.array(o['marker']['px'],float),K,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            branches=[]
            for r,t in zip(rs,ts):
                T=mat(np.r_[r.ravel(),t.ravel()/.1]);branches.append(dict(T_F_M=T.tolist(),orientation_disagreement_deg=float(np.degrees(Rotation.from_matrix(pred[:3,:3]@T[:3,:3].T).magnitude())),translation_disagreement_m=float(np.linalg.norm(pred[:3,3]-T[:3,3])),rms_px=stats([project(MARKER,T,K,d)-np.array(o['marker']['px'])])['rms_px'],positive_depth=bool(t[2]>0)))
            branches.sort(key=lambda b:b['rms_px']);diag.append(dict(file=o['file'],branches=branches,marker_errors_px=stats([project(MARKER,pred,K,d)-np.array(o['marker']['px'])])))
        poses.append(dict(id=p['id'],nominal=p['nominal'],T_W_G=WG.tolist(),T_F_M_pred=pred.tolist(),independent_front_pixels=True,front=diag))
    return dict(metrics={k:stats(v) for k,v in ee.items()},poses=poses,positive_depth=depth,scope='G pose estimated from G floor pixels ONLY; all F marker and floor pixels withheld; K/TF/GM frozen; prior-seen holdouts exploratory, not prospective independent validation')

def design_view(data,fit,center_px,depth_m,tilt_x_deg,tilt_y_deg,observed_WG=None):
    """Local Fisher-information design, not a simulated hardware observation."""
    K=np.array(fit['K']);d=np.array(fit['dist']);TF=np.array(fit['T_F_W']);M=np.array(fit['T_G_M']);WG0=np.array(fit['T_W_G'][0]);P0=TF@WG0@M
    xy=cv2.undistortPoints(np.array(center_px,float).reshape(1,1,2),K,d).ravel()
    P=np.eye(4);P[:3,:3]=Rotation.from_euler('xy',[tilt_x_deg,tilt_y_deg],degrees=True).as_matrix()@P0[:3,:3];P[:3,3]=np.r_[xy,1]*depth_m
    WG=np.linalg.inv(TF)@P@np.linalg.inv(M)
    if observed_WG is not None:
        WG=np.array(observed_WG);P=TF@WG@M;center_px=project(np.zeros((1,3)),P,K,d)[0].tolist();depth_m=float(P[2,3])
    X=np.unique(np.concatenate([np.array(o['obj']) for p in data['pairs'] for o in p['gripper']]),axis=0)
    T=np.linalg.inv(WG);uv=project(X,T,data['Kg'],data['dg']);dep=(X@T[:3,:3].T+T[:3,3])[:,2];keep=(uv[:,0]>15)&(uv[:,0]<1585)&(uv[:,1]>15)&(uv[:,1]<881)&(dep>0)
    X=X[keep];marker=project(MARKER,P,K,d);visible=bool(np.all((marker[:,0]>15)&(marker[:,0]<1585)&(marker[:,1]>15)&(marker[:,1]<1185)))
    x=np.r_[vec(TF),vec(M),np.zeros(6),vec(WG)]
    def res(v):
        F=mat(v[:6]);GM=mat(v[6:12]);Ki,di=optics(v[12:18],MODELS['full'],K,d);W=mat(v[18:]);r=[(project(MARKER,F@W@GM,Ki,di)/2).ravel()]
        if len(X):r.append((project(X,np.linalg.inv(W),data['Kg'],data['dg'])/np.sqrt(len(X))).ravel())
        return np.concatenate(r)*np.sqrt(2) # two exposures per bracket, covariance caveat disclosed
    J=np.column_stack([(res(x+np.eye(24)[i]*1e-5)-res(x-np.eye(24)[i]*1e-5))/2e-5 for i in range(24)]);A=J[:,18:];B=J[:,:18];Q=B-np.einsum('ij,jk->ik',A,np.linalg.lstsq(A,B,rcond=1e-10)[0]);H=np.einsum('ki,kj->ij',Q,Q)
    return dict(center_px=center_px,depth_m=depth_m,tilt_x_deg=tilt_x_deg,tilt_y_deg=tilt_y_deg,G_floor_corners=int(len(X)),front_marker_fully_visible=visible,information=H.tolist(),T_W_G=WG.tolist(),pose_source='retained observed G board pose; F counterpart design prediction only' if observed_WG is not None else 'unconstrained optical-space design; mechanical reach not established',scope='design-only geometric prediction; reach/occlusion/rigidity unverified')

import json,sys
from scipy.optimize import least_squares
BASE=Path(__file__).resolve().parent.parent
SESSION=BASE/'20261015T080103Z-pincher-intrinsics-arm'
sys.path.insert(0,str(BASE/'physical_base'))
from observations import intrinsics
MODELS={'fixed':[],'f':['f'],'fcy':['f','cy'],'fcyk1':['f','cy','k1'],'full':['fx','fy','cx','cy','k1','k2']}
SCALES={'f':100.,'fx':100.,'fy':100.,'cx':100.,'cy':100.,'k1':.1,'k2':.1}

def vec(T):return np.r_[Rotation.from_matrix(T[:3,:3]).as_rotvec(),T[:3,3]/.1]
def mat(v):
    T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(v[:3]).as_matrix();T[:3,3]=v[3:6]*.1;return T

def camera_pose(o,K,d):
    ok,r,t=cv2.solvePnP(np.array(o['obj'],float),np.array(o['px'],float),K,d)
    if not ok:raise ValueError('PnP initialization failed')
    return np.linalg.inv(mat(np.r_[r.ravel(),t.ravel()/.1]))

def load_data():
    read=lambda p:json.loads(p.read_text())
    policy=read(SESSION/'arm/pairing.json');fronts={int(p.name[:3]):read(p) for p in (SESSION/'arm').glob('*-front.json')}
    gs={int(Path(o['file']).name[:3]):o for o in read(SESSION/'intrinsics-observations.json')}
    models=read(SESSION/'lens/models.json');lens=models[models['frozen_selected_model']]['fit'];Kg=np.array(lens['K']);dg=np.array(lens['dist']);Kf,df=intrinsics('front')
    pairs=[]
    for p in policy['pairs']:
        if not p['accepted']:continue
        f0,g1,g2,f1=p['seqs'];pairs.append(dict(p,id=f0,front=[fronts[f0],fronts[f1]],gripper=[gs[g1],gs[g2]]))
    return dict(pairs=pairs,train=[p for p in pairs if p['split']=='train'],test=[p for p in pairs if p['split']=='holdout'],policy=policy,Kg=Kg,dg=dg,Kf=Kf,df=df)

def optics(v,names,K0,d0):
    K=K0.copy();d=d0.copy()
    for val,key in zip(v,names):
        delta=val*SCALES[key]
        if key=='f':K[0,0]+=delta;K[1,1]+=delta
        elif key=='fx':K[0,0]+=delta
        elif key=='fy':K[1,1]+=delta
        elif key=='cx':K[0,2]+=delta
        elif key=='cy':K[1,2]+=delta
        else:d[0 if key=='k1' else 1]+=delta
    return K,d

def errors(data,pairs,TF,M,K,d,WGs):
    out={'floor':[],'marker':[],'gripper':[]};depth=True
    for p,WG in zip(pairs,WGs):
        for o in p['front']:
            b=o['board'];out['floor'].append(project(b['obj'],TF,K,d)-np.array(b['px']))
            depth=depth and bool(np.all((np.array(b['obj'])@TF[:3,:3].T+TF[:3,3])[:,2]>0))
            out['marker'].append(project(MARKER,TF@WG@M,K,d)-np.array(o['marker']['px']))
        for o in p['gripper']:
            T=np.linalg.inv(WG);out['gripper'].append(project(o['obj'],T,data['Kg'],data['dg'])-np.array(o['px']))
            depth=depth and bool(np.all((np.array(o['obj'])@T[:3,:3].T+T[:3,3])[:,2]>0))
        depth=depth and bool(np.all((MARKER@(TF@WG@M)[:3,:3].T+(TF@WG@M)[:3,3])[:,2]>0))
    return out,depth

def stats(e):
    e=np.concatenate(e);r=np.linalg.norm(e,axis=1)
    return dict(n=len(r),rms_px=float(np.sqrt(np.mean(r*r))),p95_px=float(np.percentile(r,95)),max_px=float(r.max()))

def fit_bundle(data,pairs,model='fixed',initial=None,fixed_values=None,max_nfev=300,fixed_gripper=False):
    names=[n for n in MODELS[model] if n not in (fixed_values or {})];K0=data['Kf'];d0=data['df']
    if fixed_values:K0,d0=optics([fixed_values[n]/SCALES[n] for n in fixed_values],list(fixed_values),K0,d0)
    if initial is None:
        TF=np.array(pairs[0]['front'][0]['board']['T_cam_board']);WGs=[camera_pose(p['gripper'][0],data['Kg'],data['dg']) for p in pairs]
        M=np.linalg.inv(TF@WGs[0])@np.array(pairs[0]['front'][0]['marker']['T_cam_marker']);x=np.r_[vec(TF),vec(M),np.zeros(len(names)),np.concatenate([vec(T) for T in WGs])]
    else:
        TF=np.array(initial['T_F_W']);M=np.array(initial['T_G_M']);Kinit=np.array(initial['K']);dinit=np.array(initial['dist']);vals={'f':Kinit[0,0]-K0[0,0],'fx':Kinit[0,0]-K0[0,0],'fy':Kinit[1,1]-K0[1,1],'cx':Kinit[0,2]-K0[0,2],'cy':Kinit[1,2]-K0[1,2],'k1':dinit[0]-d0[0],'k2':dinit[1]-d0[1]}
        x=np.r_[vec(TF),vec(M),[vals[n]/SCALES[n] for n in names],np.concatenate([vec(camera_pose(p['gripper'][0],data['Kg'],data['dg'])) for p in pairs])]
    ng=12+len(names)
    frozen_WGs=[gripper_pose(data,p) for p in pairs] if fixed_gripper else None
    if fixed_gripper:x=x[:ng]
    def unpack(x):return mat(x[:6]),mat(x[6:12]),*optics(x[12:ng],names,K0,d0),(frozen_WGs if fixed_gripper else [mat(v) for v in x[ng:].reshape(-1,6)])
    def res(x):
        TF,M,K,d,WGs=unpack(x);ee,_=errors(data,pairs,TF,M,K,d,WGs)
        return np.concatenate([(r/np.sqrt(len(r))).ravel() for k in ['floor','marker','gripper'] for r in ee[k]])
    lo=np.full(len(x),-np.inf);hi=-lo
    for i,n in enumerate(names):
        bounds={'f':(-220,280),'fx':(-220,280),'fy':(-220,280),'cx':(-150,150),'cy':(-150,150),'k1':(-.2,.2),'k2':(-.2,.2)}[n];lo[12+i]=bounds[0]/SCALES[n];hi[12+i]=bounds[1]/SCALES[n]
    x=np.clip(x,lo+1e-8,hi-1e-8)
    sol=least_squares(res,x,bounds=(lo,hi),loss='soft_l1',f_scale=.5,max_nfev=max_nfev,ftol=1e-9,xtol=1e-9,gtol=1e-8)
    TF,M,K,d,WGs=unpack(sol.x);ee,depth=errors(data,pairs,TF,M,K,d,WGs)
    # Unrobust numerical Jacobian: do not mistake robust downweights for geometry.
    J=np.column_stack([(res(sol.x+np.eye(len(x))[i]*1e-5)-res(sol.x-np.eye(len(x))[i]*1e-5))/2e-5 for i in range(len(x))]);A=J[:,ng:];B=J[:,:ng];Q=B-np.einsum('ij,jk->ik',A,np.linalg.lstsq(A,B,rcond=1e-10)[0]);_,s,V=np.linalg.svd(Q,full_matrices=False)
    global_names=['TF_rx','TF_ry','TF_rz','TF_tx/.1m','TF_ty/.1m','TF_tz/.1m','GM_rx','GM_ry','GM_rz','GM_tx/.1m','GM_ty/.1m','GM_tz/.1m']+names
    gripper_scope='fixed G-only estimate; conditional' if fixed_gripper else 'latent G poses in G-floor/F-marker joint pixel objective'
    return dict(model=model,gripper_scope=gripper_scope,K=K.tolist(),dist=d.tolist(),T_F_W=TF.tolist(),T_G_M=M.tolist(),T_W_G=[T.tolist() for T in WGs],fit_ids=[p['id'] for p in pairs],success=bool(sol.success),positive_depth=depth,nfev=sol.nfev,cost=float(sol.cost),metrics={k:stats(v) for k,v in ee.items()},conditioning=dict(information=np.einsum('ki,kj->ij',Q,Q).tolist(),parameter_order=global_names,singular_values=s.tolist(),rank=int(np.sum(s>s[0]*1e-6)),condition=float(s[0]/s[-1]),weakest_directions=V[-3:].tolist(),scaling='angles rad; translations/0.1m; focal/principal deltas/100px; radial deltas/0.1; residuals/√corners per image; G pose nuisance eliminated',conditional_covariance=(np.einsum('ki,k,kj->ij',V,1/np.maximum(s*s,1e-20),V)*np.sum(res(sol.x)**2)/max(1,len(res(sol.x))-len(sol.x))).tolist()),bound_hits=[n for i,n in enumerate(names) if min(sol.x[12+i]-lo[12+i],hi[12+i]-sol.x[12+i])<1e-4],fixed_values=fixed_values)
