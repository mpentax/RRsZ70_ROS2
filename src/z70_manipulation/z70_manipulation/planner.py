"""Provisional articulated grid planner; retained firmware and floor limits."""
from collections import deque
import math
import cv2
import numpy as np
from functools import lru_cache
from z70_description import pincher_geometry as hand
from z70_description.arm_model import vendor_forward, vendor_check
CAD_L1, CAD_L2, _CAD = hand.L1, hand.L2, hand.CAD
CAD_MOUNT=(np.array(_CAD['camera_from_wrist_m'],float), np.array(_CAD['camera_rpy'],float))
from .perception import calibration, gripper_camera_info

GRASP=(250,95)  # retained historical successful nominal preset, NOT retuned/validated on this provisional geometry
MIN_TIP=.010  # retained bound; ~43mm at250/95 was a terminal-region surrogate, NOT exact pad anatomy
# Jaw ray inferred from metric multi-view seam; pads remain provisional/partly above image.
JAW_V_BAND=(-130.,140.)
REACH_LIMIT=.375  # 240/115 vendor-x=.3691 executed and front marker confirmed; refusals remain image-checked
_CALIBRATION=calibration()
MOUNT=tuple(_CALIBRATION['arm_origin_'+axis+'_m'] for axis in ('x','y','z'))
YAW=math.radians(_CALIBRATION['arm_yaw_deg'])

def vendor_endpoint(state):
    """App planar end_pos for firmware checks, not the CAD wrist in base TF."""
    return vendor_forward(*state[:2])[1]


wrist = vendor_endpoint  # Historical public name; never use for physical hand TF.

def base_point(x,z):
    return [MOUNT[0]+x*math.cos(YAW),MOUNT[1]+x*math.sin(YAW),MOUNT[2]+z]

def measured_point(state,endpoint):
    """Link2-zero point in the CAD chain with retained yaw/M3 registration.

    Historical name: endpoint may be a legacy optical/contact surrogate, not
    necessarily a CAD landmark or an independently measured physical point.
    """
    a1=math.radians(270-state[0]+_CALIBRATION['measured_M3_offset_deg'])
    a2=math.radians(state[1]-state[0]+90)
    shoulder=tuple(_CAD['shoulder_in_base_m']); length=CAD_L1
    x=length*math.cos(a1)+endpoint[0]*math.cos(a2)-endpoint[2]*math.sin(a2)
    z=length*math.sin(a1)+endpoint[0]*math.sin(a2)+endpoint[2]*math.cos(a2)
    return [shoulder[0]+x*math.cos(YAW)-endpoint[1]*math.sin(YAW),
            shoulder[1]+x*math.sin(YAW)+endpoint[1]*math.cos(YAW),shoulder[2]+z]

def operational_point(state):
    """Photo-estimated grasp aim; historical measured surrogate stays distinct."""
    T=hand.hand_pose(state)
    return (T[:3,:3]@hand.grasp_reference()+T[:3,3]).tolist()


def grasp_point(state=(240,95)):
    # Initial approach aim from a floor-safe model equator pose, NOT a newly
    # hardware-validated preset. Actual descent selects a reachable grid goal.
    return operational_point(state)[:2]+[0.]


@lru_cache(maxsize=16384)
def _body_floor(state):
    T=hand.hand_pose(state)
    corners=_local_corners(state[3])
    return float(np.min(corners@T[2,:3]+T[2,3]))-hand.CONFIG['uncertainty_m']


@lru_cache(maxsize=4096)
def _local_corners(m6):
    return np.concatenate([b.corners() for b in hand.local_bodies(m6)])


def body_floor_bound(state):
    return _body_floor(hand.validate_state(state))


def safe(state,obstacles=()):
    """Firmware checks PLUS full housing/finger floor bounds.

    Two angles imply M5=0/M6=90 (open approach). Optional world AABBs are
    conservative per-piece broad-phase refusals, not a mapped scene model.
    """
    try: st=hand.validate_state(state)
    except ValueError:return False
    x,_=vendor_endpoint(st)
    if not (vendor_check(*st[:3])=='SUCCESS' and x<=REACH_LIMIT-.003
            and tip_floor_bound(st)>=MIN_TIP and body_floor_bound(st)>=MIN_TIP):return False
    for body in hand.world_bodies(st) if obstacles else ():
        corners=body.corners();lo=corners.min(axis=0);hi=corners.max(axis=0)
        for lower,upper in obstacles:
            if np.all(hi>=np.asarray(lower)) and np.all(lo<=np.asarray(upper)):return False
    return True


def safe_edge(a,b,obstacles=()):
    if len(a)!=len(b):raise ValueError('edge states must have matching dimensions')
    try: hand.validate_state(a);hand.validate_state(b)
    except ValueError:return False
    # <=0.5deg nominal samples, including endpoints and M5/M6 interpolation.
    samples=max(20,math.ceil(max(abs(x-y) for x,y in zip(a,b))*2))
    return all(safe(tuple(x+(y-x)*i/samples for x,y in zip(a,b)),obstacles) for i in range(samples+1))


def joint_plan(start,goal,allowed=None,m5=0,m6=90):
    start=tuple(start); goal=tuple(goal)
    if len(start)!=2 or len(goal)!=2:raise ValueError('joint_plan emits M3/M4 commands only; use safe_edge for four-angle geometry')
    geometry=lambda st: (st[0],st[1],m5,m6) if m5!=0 or m6!=90 else st
    if not safe(geometry(start)) or not safe(geometry(goal)): raise ValueError('unsafe joint endpoint')
    near_goal=lambda st: sum(abs(a-b) for a,b in zip(st,goal))<=10   # last step onto the goal: object is right below
    ok=lambda st: st==goal or allowed is None or allowed(st) or (near_goal(st) and tip_height(st)>=.015)
    queue=deque([start]); parent={start:None}
    while queue:
        state=queue.popleft()
        if state==goal:
            steps=[]
            while parent[state] is not None:
                before,codes=parent[state]; steps.append(dict(m=codes,state=list(state))); state=before
            return steps[::-1]
        for j in range(2):
            for delta,code in [(10,1),(-10,2)]:
                new=list(state); new[j]+=delta; new=tuple(new)
                if (new in parent or not safe(geometry(new))
                        or not safe_edge(geometry(state),geometry(new)) or not ok(new)): continue
                codes=[0,0,0]; codes[j]=code; parent[new]=(state,codes); queue.append(new)
    raise ValueError('no safe 10-degree joint path')

_LENS=gripper_camera_info()
GRIPPER_MATRIX=np.array(_LENS['camera_matrix']['data']).reshape(3,3)
GRIPPER_D=np.array(_LENS['distortion_coefficients']['data'])
# Compact sizing tuple retained for callers; projections use independent fx/fy.
GRIPPER_K=(GRIPPER_MATRIX[0,0],GRIPPER_MATRIX[0,2],GRIPPER_MATRIX[1,2],
           _LENS['image_width'],_LENS['image_height'])

def camera_pose(state):
    """Base to optical frame. The mount is the copied camera, not the old gauge."""
    a=math.radians(state[1]-state[0]+90);cy,sy=math.cos(YAW),math.sin(YAW)
    ca,sa=math.cos(a),math.sin(a)
    R=np.array([[cy*ca,-sy,-cy*sa],[sy*ca,cy,-sy*sa],[sa,0,ca]])
    roll=math.radians(state[2] if len(state)>2 else 0.)
    cr,sr=math.cos(roll),math.sin(roll)
    Q=np.array([[1.,0.,0.],[0.,cr,-sr],[0.,sr,cr]])
    mount=hand.rpy(CAD_MOUNT[1])
    optical=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    elbow=np.array(measured_point(state,[0.,0.,0.]))
    point=np.array([CAD_L2,0.,0.])+Q@CAD_MOUNT[0]
    return (elbow+R@point, R@Q@mount@optical)

def pincher_camera_height_and_direction(state):
    """Pincher camera height above floor (m) and optical view direction (+Z unit vector in base_footprint)."""
    C,R=camera_pose(state)
    return dict(position_m=C.tolist(),height_m=float(C[2]),direction=R[:,2].tolist())

def gripper_pixel(state,point):
    """Project using the CAD camera mount and retained calibrated lens distortion."""
    C,R=camera_pose(state); optical=R.T@(np.array(point)-C)
    if optical[2]<.01: return None
    uv=cv2.projectPoints(optical[None,:],np.zeros(3),np.zeros(3),GRIPPER_MATRIX,GRIPPER_D)[0].reshape(2)
    return float(uv[0]),float(uv[1]),float(optical[2])

def hover_plan(start=(150,45),target=None):
    """Hover where the calibrated camera model puts the target in the lower-middle of the pincher image."""
    target=target or grasp_point(); choices=[]; f,cx,cy,w,h=GRIPPER_K
    for a in range(90,271,10):
        for b in range(35,171,10):
            state=(a,b)
            if not safe(state): continue
            px=gripper_pixel(state,target)
            if px is None: continue
            u,v,depth=px
            if not (.3*w<=u<=.7*w and .35*h<=v<=.85*h): continue
            try: steps=joint_plan(start,state)
            except ValueError: continue
            choices.append((abs(v/h-.6)+abs(u/w-.5)+len(steps)*.002,state,steps,u,v,depth))
    if not choices: raise ValueError('no safe hover pose that sees the target')
    _,state,steps,u,v,depth=min(choices)
    return dict(state=list(state),steps=steps,predicted_pixel=[u,v],camera_distance_m=depth)

def tip_height(state):
    # Optional residual is zero for the owner tape-informed closing-point registration.
    return measured_point(state,_CALIBRATION['effective_jaw_from_elbow_m'])[2]+_CALIBRATION.get('jaw_z_correction_m',0.)

def tip_floor_bound(state):
    """Conservative provisional contact-region bound; not a pad-centre guarantee."""
    return tip_height(state)-_CALIBRATION['jaw_uncertainty_bound_m']

def descent_plan(start,target=None,goal=GRASP,min_tip=.015,equator_height=.033):
    """Keep both floor contact and object equator observable during descent."""
    target=target or grasp_point(); f,cx,cy,w,h=GRIPPER_K
    def visible(st):
        px=gripper_pixel(st,target)
        eq=gripper_pixel(st,list(target[:2])+[equator_height])
        return (px is not None and eq is not None and .15*w<=px[0]<=.85*w
                and .05*h<=px[1]<=1.2*h and 0<=eq[1]<=1.1*h
                and tip_height(st)>=min_tip)
    return joint_plan(start,goal,visible)

def grasp_goal(object_xy,start,size_m=.066):
    """Contact at the equator, not at a hard-coded low wrist pose."""
    best=None
    for a in range(90,271,10):
        for b in range(35,171,10):
            st=(a,b)
            height=operational_point(st)[2]
            if not safe(st) or abs(height-size_m/2)>.012: continue
            err=math.hypot(math.dist(grasp_point(st)[:2],object_xy[:2]),height-size_m/2)
            if best and err>=best[0]: continue
            try: path=descent_plan(start,object_xy[:2]+[0.],goal=st,equator_height=size_m/2)
            except ValueError: continue
            best=(err,st,path)
    if best is None: raise ValueError('no reachable grasp pose for this object position')
    return dict(state=list(best[1]),error_m=best[0],steps=best[2])


def stack_tier_plan(tier, size_m=.066, start=(190, 65)):
    """Plan arm pose and lift requirements for placing or holding at tier 1..3 of a 3-dodecahedron stack."""
    if tier not in (1, 2, 3):
        raise ValueError('stack tier must be 1, 2, or 3')
    target_equator_z = (tier - 0.5) * size_m
    tower_top_below_z = (tier - 1) * size_m
    clearance_z = tower_top_below_z + size_m * 0.5 + 0.012
    best = None
    for a in range(90, 271, 10):
        for b in range(35, 171, 10):
            st = (a, b)
            if not safe(st):
                continue
            op = operational_point(st)
            z_err = abs(op[2] - target_equator_z)
            if z_err > .020 or op[0] < 0.36:
                continue
            try:
                steps = joint_plan(start, st) if start != st else []
            except ValueError:
                continue
            score = (round(z_err, 3), abs(op[0] - 0.41), len(steps))
            if best is None or score < best[0]:
                best = (score, st, op, steps)
    if best is None:
        raise ValueError(f'no safe grid placement pose for tier {tier}')
    _, st, op, steps = best
    lift_steps = 2 if tier <= 2 else 3
    return dict(
        tier=tier,
        target_equator_z_m=round(target_equator_z, 4),
        tower_top_below_z_m=round(tower_top_below_z, 4),
        required_clearance_z_m=round(clearance_z, 4),
        placement_state=list(st),
        operational_point_m=[round(x, 4) for x in op],
        tip_height_m=round(tip_height(st), 4),
        lift_steps=lift_steps,
        steps=steps,
    )

def approach(position,target=None):
    target=target or grasp_point(); x,y=position[:2]; gx,gy=target[:2]
    radius=math.hypot(x,y)
    if radius<=abs(gy): raise ValueError('target too close for straight approach')
    yaw=math.atan2(y,x)-math.asin(gy/radius)
    forward_distance=math.sqrt(radius*radius-gy*gy)-gx
    return dict(yaw=yaw,forward=forward_distance,bearing_error_deg=math.degrees(yaw),range_error_m=forward_distance,grasp_point=target)

def expected_size_px(size_m,distance_m,focal=GRIPPER_K[0]):
    return focal*size_m/max(distance_m,.025)

def grasp_criterion(blob,size_m,camera_distance_m,state=None):
    """Only a reconstructed equator at the contact ray can authorize closing.

    A cropped lower contour cannot localize the floor contact. Top cropping is
    allowed: at v~110 a 6.6 cm object may extend above the frame, so its HSV
    centroid is NOT its equator. Project contact-derived position at size/2.
    """
    w,h=blob['image_size']
    if state is None: return False  # no pose / floor geometry => no demonstrated straddle
    if 'position' not in blob or 'contact_px' not in blob: return False
    if not 4<=blob['contact_px'][1]<=h-5: return False
    px=gripper_pixel(state,list(blob['position'][:2])+[size_m/2])
    if px is None: return False
    u,v,depth=px
    width=blob.get('bbox',[0,0,blob['size_px'],0])[2]
    ratio=width/expected_size_px(size_m,depth,GRIPPER_K[0]*w/1600)
    return (abs(u/1600-.5)<=.08 and JAW_V_BAND[0]<=v<=JAW_V_BAND[1]
            and .6<=ratio<=1.5 and safe(state)
            and abs(tip_height(state)-size_m/2)<=.012
            and math.dist(grasp_point(state)[:2],blob['position'][:2])<=.02)
