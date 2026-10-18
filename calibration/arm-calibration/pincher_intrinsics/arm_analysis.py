"""Offline axis conventions; no physical wrist station is inferred."""
import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
MARKER=np.array([[-.0075,.0075,0],[.0075,.0075,0],[.0075,-.0075,0],[-.0075,-.0075,0]])

def as_transform(v):
    T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(v[:3]).as_matrix();T[:3,3]=v[3:];return T

def project_marker(T,K,d):
    return cv2.projectPoints(MARKER,Rotation.from_matrix(T[:3,:3]).as_rotvec(),T[:3,3],K,d)[0].reshape(-1,2)

def fit_mount(T_F_G,pixels,K,d,initial):
    v=np.r_[Rotation.from_matrix(initial[:3,:3]).as_rotvec(),initial[:3,3]/.1]
    def unpack(v):return as_transform(np.r_[v[:3],v[3:]*.1])
    def residual(v):
        M=unpack(v)
        return np.concatenate([(project_marker(T@M,K,d)-p).ravel() for T,p in zip(T_F_G,pixels)])
    sol=least_squares(residual,v,loss='soft_l1',f_scale=1,max_nfev=500)
    s=np.linalg.svd(sol.jac,compute_uv=False)
    return {'T_G_M':unpack(sol.x).tolist(),'rms_px':float(np.sqrt(np.mean(residual(sol.x)**2)*2)),'success':bool(sol.success),'singular_values':s.tolist(),'rank':int(np.sum(s>s[0]*1e-6)),'scaling':'rotation radians, translation/0.1m, residual pixels','nfev':sol.nfev}

def axis_distance_profile(shoulder_poses,elbow_poses,ns,ne,lengths):
    ns=np.asarray(ns,float);ne=np.asarray(ne,float);ns/=np.linalg.norm(ns);ne/=np.linalg.norm(ne)
    def basis(n):
        ref=np.array([0.,0,1.]) if abs(n[2])<.9 else np.array([0.,1,0])
        a=np.cross(n,ref);a/=np.linalg.norm(a);return np.column_stack([a,np.cross(n,a)])
    Bs,Be=basis(ns),basis(ne)
    def equations(poses):
        T0=np.asarray(poses[0]);A=[];b=[]
        for T in poses[1:]:
            Q=T[:3,:3]@T0[:3,:3].T;A.append(np.eye(3)-Q);b.append(T[:3,3]-Q@T0[:3,3])
        return np.vstack(A),np.concatenate(b)
    As,bs=equations(shoulder_poses);Ae,be=equations(elbow_poses)
    x0=np.r_[np.linalg.lstsq(As@Bs,bs,rcond=1e-9)[0],np.linalg.lstsq(Ae@Be,be,rcond=1e-9)[0]]
    output=[]
    for length in lengths:
        def residual(x):
            cs=Bs@x[:2];ce=Be@x[2:];distance=station_separation({'direction':ns,'closest_point':cs},{'direction':ne,'closest_point':ce},[.1,.14,0])['separation_m']
            return np.r_[As@cs-bs,Ae@ce-be,(distance-length)*1e4]
        sol=least_squares(residual,x0,max_nfev=300);raw=residual(sol.x)[:-1]
        output.append({'fixed_L1_m':float(length),'translation_rms_m':float(np.sqrt(np.mean(raw**2))),'constraint_error_m':float(residual(sol.x)[-1]/1e4),'success':bool(sol.success),'scope':'conditional SE3 translation profile with observed rotations and axis directions frozen; not full pixel joint-bundle profile or systematic confidence'})
    return output

def station_separation(shoulder,elbow,station):
    n=np.asarray(shoulder['direction'],float);n/=np.linalg.norm(n)
    ne=np.asarray(elbow['direction'],float);ne/=np.linalg.norm(ne)
    if n@ne<0:ne=-ne
    axis=n+ne;axis/=np.linalg.norm(axis);origin=np.asarray(station,float)
    def point(line,direction):
        c=np.asarray(line['closest_point'],float)
        return c+direction*((origin-c)@axis)/(direction@axis)
    s=point(shoulder,n);e=point(elbow,ne)
    return {'separation_m':float(np.linalg.norm(e-s)),'shoulder_point_m':s.tolist(),'elbow_point_m':e.tolist(),'station_origin_m':origin.tolist(),'station_plane_normal':axis.tolist(),'axis_nonparallel_deg':float(np.degrees(np.arccos(np.clip(n@ne,-1,1)))),'convention':'both axis lines intersect same station plane normal to their mean direction; not infinite skew-line minimum, not surveyed anatomical centres'}
