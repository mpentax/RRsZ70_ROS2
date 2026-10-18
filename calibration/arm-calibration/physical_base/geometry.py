"""Offline SE(3) screw lines. Optical x right, y down, z forward; metres."""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import cv2

def predict_pose(fit,angle):
    n=np.asarray(fit['direction']);c=np.asarray(fit['closest_point']);Q=Rotation.from_rotvec(n*angle).as_matrix()
    T=np.eye(4);T[:3,:3]=Q@np.asarray(fit['R0']);T[:3,3]=c+Q@(np.asarray(fit['C0'])-c);return T

def fit_pixel_axis(records,K,d,max_nfev=300,initial_shift=None):
    """One pure-joint block, fixed board gauge; actual angles are latent.

    Each pose contributes equally after division by sqrt(corner count).
    No exact nominal angle, link length, elbow zero or mount is imposed.
    """
    Ts=[np.asarray(r['T']) for r in records];init=fit_axis(Ts);n0=np.asarray(init['direction']);c0=np.asarray(init['closest_point'])
    phi=np.arctan2(n0[1],n0[0]);theta=np.arcsin(n0[2])
    def basis(n):
        ref=np.array([0.,0,1.]) if abs(n[2])<.9 else np.array([1.,0,0.])
        a=np.cross(n,ref);a/=np.linalg.norm(a);return a,np.cross(n,a)
    a,b=basis(n0);R0=Ts[0][:3,:3];C0=Ts[0][:3,3]
    angles=[float(n0@Rotation.from_matrix(T[:3,:3]@R0.T).as_rotvec()) for T in Ts]
    x=np.r_[phi,theta,c0@a/.1,c0@b/.1,Rotation.from_matrix(R0).as_rotvec(),C0/.1,angles[1:]]
    if initial_shift is not None:x[2:4]+=np.asarray(initial_shift)/.1
    def unpack(v):
        ph,th=v[:2];n=np.array([np.cos(th)*np.cos(ph),np.cos(th)*np.sin(ph),np.sin(th)]);a,b=basis(n);c=.1*(v[2]*a+v[3]*b)
        return n,c,Rotation.from_rotvec(v[4:7]).as_matrix(),.1*v[7:10],np.r_[0,v[10:]]
    def residual(v):
        n,c,R0,C0,aa=unpack(v);out=[]
        for r,angle in zip(records,aa):
            Q=Rotation.from_rotvec(n*angle).as_matrix();R=Q@R0;C=c+Q@(C0-c)
            X=np.asarray(r['obj']);px=np.asarray(r['px']);uv=cv2.projectPoints((X-C)@R,np.zeros(3),np.zeros(3),K,d)[0].reshape(-1,2)
            out.extend(((uv-px)/np.sqrt(len(X))).ravel())
        return np.asarray(out)
    sol=least_squares(residual,x,loss='soft_l1',f_scale=.35,max_nfev=max_nfev,xtol=1e-10,ftol=1e-10,gtol=1e-10)
    n,c,R0,C0,aa=unpack(sol.x);raw=[]
    for r,ang in zip(records,aa):
        Q=Rotation.from_rotvec(n*ang).as_matrix();R=Q@R0;C=c+Q@(C0-c)
        raw.extend(np.linalg.norm(cv2.projectPoints((np.asarray(r['obj'])-C)@R,np.zeros(3),np.zeros(3),K,d)[0].reshape(-1,2)-r['px'],axis=1))
    sv=np.linalg.svd(sol.jac,compute_uv=False)
    return dict(direction=n.tolist(),closest_point=c.tolist(),R0=R0.tolist(),C0=C0.tolist(),latent_angles_rad=aa.tolist(),point_rms_px=float(np.sqrt(np.mean(np.square(raw)))),point_p95_px=float(np.percentile(raw,95)),point_max_px=float(np.max(raw)),success=bool(sol.success),nfev=sol.nfev,jacobian_singular_values=sv.tolist(),jacobian_rank=int(np.sum(sv>sv[0]*1e-6)),parameter_count=len(x),parameter_scaling='translations/0.1m; rotations/rad; residuals/√N corners per pose',axial_station_unobservable=True,cost=float(sol.cost),initial_model_free=init)

def fit_circle_axis(points):
    pts=np.asarray(points);origin=pts.mean(axis=0);_,sv,V=np.linalg.svd(pts-origin,full_matrices=False)
    if len(pts)<3 or sv[1]<1e-7:raise ValueError('Circle excitation insufficient')
    e1,e2,n=V;uv=(pts-origin)@V[:2].T
    A=np.c_[2*uv,np.ones(len(pts))];b=np.sum(uv**2,axis=1);q=np.linalg.lstsq(A,b,rcond=1e-9)[0];center=origin+q[0]*e1+q[1]*e2
    rr=np.linalg.norm(uv-q[:2],axis=1);rad=float(rr.mean());c=center-n*(center@n)
    return {'direction':n.tolist(),'closest_point':c.tolist(),'circle_center_at_marker_station':center.tolist(),'radius_m':rad,'circle_rms_m':float(np.sqrt(np.mean((rr-rad)**2))),'plane_rms_m':float(np.sqrt(np.mean(((pts-origin)@n)**2))),'singular_values':sv.tolist(),'axial_station_unobservable':True}

def fit_axis(poses):
    if len(poses)<3: raise ValueError('At least three observed poses required')
    T0=np.asarray(poses[0]);A=[];b=[];directions=[];angles=[];pitch=[]
    for Ti in poses[1:]:
        Ti=np.asarray(Ti);Q=Ti[:3,:3]@T0[:3,:3].T
        rv=Rotation.from_matrix(Q).as_rotvec();angle=np.linalg.norm(rv)
        if angle<.025: continue
        n=rv/angle
        if directions and n@directions[0]<0:n=-n
        t=Ti[:3,3]-Q@T0[:3,3]
        directions.append(n);angles.append(float(np.degrees(angle)));pitch.append(float(n@t))
        A.append(np.eye(3)-Q);b.append(t)
    if len(A)<2:raise ValueError('Insufficient observed angular excitation')
    n=np.mean(directions,axis=0);n/=np.linalg.norm(n)
    AA=np.vstack(A);bb=np.concatenate(b)
    c=np.linalg.lstsq(np.vstack([AA,n]),np.r_[bb,0.],rcond=1e-9)[0]
    residual=(AA@c-bb).reshape(-1,3)
    return dict(direction=n.tolist(),closest_point=c.tolist(),rms_m=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))),observed_angles_deg=angles,screw_translation_m=pitch,singular_values=np.linalg.svd(AA,compute_uv=False).tolist(),axial_station_unobservable=True)
