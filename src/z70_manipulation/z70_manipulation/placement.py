"""Supported-placement geometry and descent step selection from front-camera observations."""
import math
import cv2
import numpy as np
from . import perception, planner

CONTACT_GAP_M = .003
LATERAL_TOL_M = .010


def observed_joints(hand_pose):
    """Fit [M3, M4, M5] joint angles to an observed 4x4 hand pose."""
    T = np.asarray(hand_pose, float)
    if T.shape != (4,4) or not np.isfinite(T).all():
        return None
    yaw = math.radians(planner.hand.CAL['arm_yaw_deg'])
    Z = planner.hand.rpy([0.,0.,yaw])
    R = Z.T @ T[:3,:3]
    a2 = math.atan2(R[2,0], R[0,0])
    m5 = math.degrees(math.atan2(-R[1,2], R[1,1]))
    if abs(m5) < 1e-8:
        m5 = 0.
    elbow = T[:3,3] - T[:3,0]*planner.hand.L2
    v = Z.T @ (elbow-np.asarray(planner.hand.CAD['shoulder_in_base_m']))
    m3 = (270+planner.hand.CAL['measured_M3_offset_deg']-math.degrees(math.atan2(v[2],v[0]))) % 360
    joints = [m3, math.degrees(a2)+m3-90, m5]
    try:
        fitted = planner.hand.hand_pose(joints)
    except ValueError:
        return None
    residual = float(np.linalg.norm(fitted[:3,3]-T[:3,3]))
    angle = math.degrees(math.acos(float(np.clip((np.trace(fitted[:3,:3].T@T[:3,:3])-1)/2,-1,1))))
    if residual > .010 or angle > 3.:
        return None
    return dict(joints=joints, residual_m=residual, residual_deg=angle, uncertainty_m=max(.015,residual))


def descent_step(sample, joints):
    """Select a single safe descent step within the measured vertical gap and lateral tolerance."""
    gap = sample['gap_m']
    if gap <= CONTACT_GAP_M or np.linalg.norm(sample['lateral_error_m']) > LATERAL_TOL_M:
        return None
    choices = []
    if joints is not None:
        current = tuple(joints)
        T = planner.hand.hand_pose(current)
        if 'held_bottom_m' in sample and 'jaw' in sample:
            observed = np.asarray(sample['jaw']['hand_pose_base'])
            local = observed[:3,:3].T @ (np.asarray(sample['held_bottom_m'])-observed[:3,3])
        else:
            local = planner.hand.grasp_reference()
        start = T[:3,:3]@local+T[:3,3]
        for (d3, c3) in ((0, 0), (10, 1), (-10, 2)):
            for (d4, c4) in ((0, 0), (10, 1), (-10, 2)):
                if c3 == 0 and c4 == 0:
                    continue
                goal = list(current); goal[0] += d3; goal[1] += d4
                if not planner.safe(goal) or not planner.safe_edge(current, goal):
                    continue
                path = [planner.hand.hand_pose([a+(b-a)*i/20 for a,b in zip(current,goal)]) for i in range(21)]
                offsets = np.array([p[:3,:3]@local+p[:3,3]-start for p in path])
                descent = float(-offsets[-1,2])
                if descent <= .002 or 1.2*float(-offsets[:,2].min())+.002 > gap:
                    continue
                if float(np.linalg.norm(offsets[:,:2],axis=1).max()) > .030:
                    continue
                m=[c3,c4,0]
                choices.append((gap-descent, dict(method='app_arm_move', params={'m':m}, descent_m=descent)))
    if gap >= .04 + max(.005,sample.get('uncertainty_m',.015)):
        choices.append((gap-.04, dict(method='app_arm_direction_move', params={'direction':2}, descent_m=.04)))
    return min(choices, key=lambda item:item[0])[1] if choices else None


def measure(image, held_color, support_color, base_color, tower_spot,
            object_in_hand=None, released=False):
    cam = perception.front_camera()
    if image.shape[:2] != (cam.height, cam.width):
        raise LookupError('placement requires full front frame')
    jaw = perception.front_jaw(image)
    T = np.asarray(jaw['hand_pose_base'], float)
    floor = [b for b in perception.front_objects(image) if b['color'] == base_color
             and math.dist(b['position'][:2], tower_spot[:2]) < .12]
    if not floor:
        raise LookupError('bottom base floor contact not visible')
    base = min(floor, key=lambda b: math.dist(b['position'][:2], tower_spot[:2]))
    support_depth = float(base['position'][0])
    if object_in_hand is not None and not released:
        held_depth = float((T[:3,:3] @ np.asarray(object_in_hand) + T[:3,3])[0])
    else:
        held_depth = support_depth if released else float(jaw['position'][0])
    C = np.array([perception.calibration()['front_camera_x_m'], 0., cam.h/1000])

    def at_depth(pixel, depth):
        ray = np.asarray(cam.pixel_to_ray(*pixel), float)
        if ray[0] <= 0 or depth <= C[0]:
            raise LookupError('placement ray/depth invalid')
        return C + ray * ((depth-C[0])/ray[0])

    thresholds = perception.colors()
    blobs = perception.detect(image, thresholds, keep_masks=True)

    def component(color, depth, expected):
        choices = []
        for b in blobs:
            if b['color'] != color:
                continue
            x,y,w,h = b['bbox']
            if x <= 1 or y <= 1 or x+w >= cam.width-1 or y+h >= cam.height-1:
                continue
            center = at_depth(b['pixel'], depth)
            width_m = float(np.linalg.norm(at_depth([x,y+h/2],depth)-at_depth([x+w,y+h/2],depth)))
            height_m = float(np.linalg.norm(at_depth([x+w/2,y],depth)-at_depth([x+w/2,y+h],depth)))
            if not (.018 <= width_m <= .110 and .018 <= height_m <= .110 and max(w/h,h/w) <= 2.4):
                continue
            if np.linalg.norm(center-np.asarray(expected)) > .085:
                continue
            ys, xs = np.nonzero(b['_mask'])
            top, bottom = int(ys.min()), int(ys.max())
            top_px = [x+float(np.median(xs[ys <= top+2])), y+top]
            bottom_px = [x+float(np.median(xs[ys >= bottom-2])), y+bottom]
            choices.append((np.linalg.norm(center-np.asarray(expected)), center,
                            at_depth(top_px, depth), at_depth(bottom_px, depth), b['bbox']))
        if not choices:
            raise LookupError('placement contour missing/ambiguous: '+color)
        choices.sort(key=lambda item: item[0])
        if len(choices)>1 and choices[1][0]-choices[0][0] < .015:
            raise LookupError('placement contour association ambiguous: '+color)
        return choices[0][1:]

    support_expected = [support_depth, base['position'][1], .033 if support_color==base_color else .099]
    support_center, support_top, _, support_box = component(support_color, support_depth, support_expected)
    held_expected = [held_depth, support_center[1], support_top[2]+.033] if released else jaw['position']
    center, _, bottom, held_box = component(held_color, held_depth, held_expected)
    return dict(jaw=jaw, held_center_m=center.tolist(), held_bottom_m=bottom.tolist(),
                support_top_m=support_top.tolist(), tower_spot=base['position'],
                gap_m=float(bottom[2]-support_top[2]),
                lateral_error_m=(bottom[:2]-support_top[:2]).tolist(),
                jaw_separation_m=float(np.linalg.norm(center-np.asarray(jaw['position']))),
                held_bbox=held_box, support_bbox=support_box,
                uncertainty_m=max(.015, jaw['uncertainty_bound_m']))
