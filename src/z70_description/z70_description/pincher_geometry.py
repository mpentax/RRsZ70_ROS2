"""Shared photo-estimated hand anatomy, in metres (elbow/link2-zero gauge).

No STL parsing on planning paths. Small cached OBBs conservatively bound each
loft segment; the housing proxy intentionally FILLS the optical recess.
M6 coupling is nominal, not feedback, force/contact physics or surveyed CAD.
"""
from dataclasses import dataclass
from functools import lru_cache
from .resources import config_path
import math
import numpy as np
import yaml


def _config(name):
    return yaml.safe_load(config_path(name).read_text())

CONFIG=_config('pincher_geometry.yaml')
CAL=_config('mount_calibration.yaml')
CAD=_config('cad_arm.yaml')
L1=float(CAD['L1_m'])
L2=float(CAD['L2_m'])
N=96
THETA=np.arange(N)*2*np.pi/N


def rpy(angles):
    r,p,y=angles
    cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])

CAM_R=rpy(np.radians([CAL['gripper_roll_deg'],CAL['gripper_pitch_down_deg'],CAL['gripper_yaw_deg']]))
CAM_C=np.array(CAD['camera_from_wrist_m'], float)
MARK_R=np.array(CAL['marker_rotation_link2'])
MARK_C=np.array(CAL['marker_effective_from_elbow_m'])-np.array([L2,0,0])


def rounded_rectangle(half_y,half_z,radius):
    directions=np.column_stack([np.cos(THETA),np.sin(THETA)])
    lo,hi=np.zeros(N),np.full(N,np.hypot(half_y,half_z))
    for _ in range(45):
        mid=(lo+hi)/2
        q=np.abs(directions*mid[:,None])-(np.array([half_y,half_z])-radius)
        sdf=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.maximum(q[:,0],q[:,1]),0)-radius
        lo=np.where(sdf<=0,mid,lo);hi=np.where(sdf>0,mid,hi)
    return directions*((lo+hi)/2)[:,None]


def _underside_surround(ring, x):
    """Scoop the nose recess into the body. Side edges stay down, so the side
    silhouette does not grow a hanging pod. The marker span is left flat."""
    if not 0.148 <= x <= 0.164:
        return ring
    axial = math.exp(-((x-0.156)/0.006)**2)
    midline = float(np.median(ring[:, 2]))
    low = ring[:, 2] < midline
    bowl = np.exp(-(ring[:, 1]/0.006)**2)
    ring = ring.copy()
    ring[low, 2] += axial*(0.005*bowl[low])
    return ring


@lru_cache(maxsize=1)
def shell_rings():
    rings=[]
    for x,width,bottom,top,radius in CONFIG['housing_sections']:
        yz=rounded_rectangle(width,(top-bottom)/2,radius)
        ring=np.column_stack([np.full(N,x-L2),yz[:,0],yz[:,1]+(top+bottom)/2])
        ring=_underside_surround(ring, x)
        rings.append(ring)
    for x,radius in CONFIG['pocket_sections']:
        local=np.column_stack([np.full(N,x),radius*np.cos(THETA),radius*np.sin(THETA)])
        rings.append(local@CAM_R.T+CAM_C)
    return tuple(rings)


@lru_cache(maxsize=2)
def finger_rings(sign):
    rings=[]
    for t,bend,w,h in CONFIG['finger_sections']:
        radius = min(w, h)* (0.92 if t > 0.9 else 0.42)
        yz=rounded_rectangle(w,h,radius)
        rings.append(np.column_stack([np.full(N,t),yz[:,0]-sign*bend,yz[:,1]]))
    # Anatomy is independent of assumed drive coupling. Solving an exact open
    # envelope by1/cos(angle) made the draft fingers unbounded near90deg.
    # The rough owner envelope is only an uncertain external cross-check.
    length=CONFIG['finger_forward_reach_m']
    for ring in rings: ring[:,0]*=length
    return tuple(rings)


def root(sign):
    return np.array([CONFIG['root_elbow_x_m']-L2,sign*CONFIG['root_half_gap_m'],CONFIG['root_z_m']])


@dataclass(frozen=True)
class Body:
    name: str
    center: np.ndarray
    half: np.ndarray
    rotation: np.ndarray
    pad: bool=False

    def contains(self,point,tolerance=0.):
        return bool(np.all(np.abs(self.rotation.T@(np.asarray(point)-self.center))<=self.half+tolerance))

    def corners(self):
        signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
        return (signs*self.half)@self.rotation.T+self.center

    def sphere_distance(self,point):
        return float(np.linalg.norm(np.maximum(np.abs(self.rotation.T@(np.asarray(point)-self.center))-self.half,0)))


def _box(name,points,pad=False):
    p=np.concatenate(points);lo=p.min(axis=0);hi=p.max(axis=0)
    return Body(name,(lo+hi)/2,(hi-lo)/2,np.eye(3),pad)


@lru_cache(maxsize=1)
def rest_bodies():
    bodies=[_box('arm_wrist_link',shell_rings())]
    for side,sign in (('left',1),('right',-1)):
        rings=finger_rings(sign)
        bodies.extend(_box('finger_'+side,(a,b),CONFIG['finger_sections'][index][0]>=CONFIG['pad_start_fraction']) for index,(a,b) in enumerate(zip(rings,rings[1:])))
    return tuple(bodies)


def validate_state(state):
    try: st=tuple(float(v) for v in state)
    except (TypeError,ValueError): raise ValueError('joint state must be finite numeric angles') from None
    if len(st) not in (2,3,4) or not all(math.isfinite(v) for v in st):
        raise ValueError('expected 2, 3 or 4 finite app joint angles')
    st=st+(0.,90.)[len(st)-2:] if len(st)<4 else st
    if not (90<=st[0]<=270 and 35<=st[1]<=170 and 0<=st[2]<=155 and 0<=st[3]<=90):
        raise ValueError('app joint angle outside model limits')
    return st


def finger_frame(sign,m6):
    if not math.isfinite(m6) or not 0<=m6<=90: raise ValueError('M6 outside 0..90')
    T=np.eye(4);T[:3,:3]=rpy([0,0,sign*math.radians(m6)*CONFIG['opening_ratio']]);T[:3,3]=root(sign)
    return T


@lru_cache(maxsize=4096)
def local_bodies(m6=90.):
    if not math.isfinite(m6) or not 0<=m6<=90: raise ValueError('M6 outside 0..90')
    result=[]
    for body in rest_bodies():
        if body.name=='arm_wrist_link':result.append(body);continue
        T=finger_frame(1 if body.name=='finger_left' else -1,m6)
        result.append(Body(body.name,T[:3,:3]@body.center+T[:3,3],body.half,T[:3,:3],body.pad))
    return tuple(result)


def hand_pose(state):
    m3,m4,m5,_=validate_state(state)
    a1=math.radians(270-m3+CAL['measured_M3_offset_deg'])
    a2=math.radians(m4-m3+90);yaw=math.radians(CAL['arm_yaw_deg'])
    B=rpy([0,-a2,yaw]);Q=rpy([math.radians(m5),0,0])
    elbow=np.array(CAD['shoulder_in_base_m'])+rpy([0,0,yaw])@np.array([L1*math.cos(a1),0,L1*math.sin(a1)])
    T=np.eye(4);T[:3,:3]=B@Q;T[:3,3]=elbow+B@np.array([L2,0,0])
    return T


def transform_bodies(T,m6=90.):
    T=np.asarray(T,float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5) or np.linalg.det(T[:3,:3])<.999:
        raise ValueError('expected finite rigid hand transform')
    return tuple(Body(b.name,T[:3,:3]@b.center+T[:3,3],b.half,T[:3,:3]@b.rotation,b.pad) for b in local_bodies(m6))


def world_bodies(state):
    st=validate_state(state)
    return transform_bodies(hand_pose(st),st[3])


def lowest(bodies):
    return min(float(b.center[2]-np.abs(b.rotation[2])@b.half) for b in bodies)


def grasp_reference():
    # PHOTO-ESTIMATED grasp-centre aim just ahead of the opposed closed pads,
    # not a replacement calibrated TCP. Legacy gripper_tcp remains unchanged.
    tip=np.mean(finger_rings(1)[-1],axis=0)+root(1)
    return np.array([tip[0]+CONFIG['grasp_aim_ahead_of_tip_m'],0,CONFIG['root_z_m']])


def articulated_geometry(state):
    """Nominal per-finger poses, terminal points, contact regions and gap.

    Gap is terminal-surface separation in the hand frame, not a measured
    parallel-jaw opening or a claim that an object of that diameter fits.
    """
    st=validate_state(state);T=hand_pose(st)
    frames={};tips={};inside=[]
    for side,sign in (('left',1),('right',-1)):
        F=finger_frame(sign,st[3]);frames['finger_'+side]=T@F
        ring=finger_rings(sign)[-1]@F[:3,:3].T+F[:3,3]
        local=ring.mean(axis=0);tips['finger_'+side]=T[:3,:3]@local+T[:3,3]
        inside.append(sign*ring[:,1].min() if sign==1 else -ring[:,1].max())
    bodies=world_bodies(st)
    return dict(finger_frames=frames,terminal_points=tips,pad_regions=tuple(b for b in bodies if b.pad),
                bodies=bodies,terminal_gap_m=sum(inside),status=CONFIG['status'])


def marker_hand_pose(marker_pose):
    M=np.eye(4);M[:3,:3]=MARK_R;M[:3,3]=MARK_C
    return np.asarray(marker_pose,float)@np.linalg.inv(M)
