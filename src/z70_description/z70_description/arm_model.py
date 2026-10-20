"""Z70 sagittal 2-link arm kinematic model, pincher housing, finger rings, and OBB collision bodies.

Source: Roborock 4.74.08 plugin `index.android.hasm` Function #32735
(`ForwardKinematics`, `InverseKinematics`, `checkArmInvalidAction`) and CAD/optical calibration.

Frame (app 2-D, sagittal plane): x forward, y up, metres.
y ~ 0 is the floor (ground-collision check y < 0.01); the robot body occupies |x| < 0.24, y < 0.14.
"""
from dataclasses import dataclass
from functools import lru_cache
import math
from pathlib import Path
import numpy as np
import yaml


def package_root(package_name: str, fallback: Path) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        return fallback
    try:
        return Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        return fallback


def config_path(name: str) -> Path:
    """Resolve a configuration basename under z70_description/config."""
    if Path(name).name != name:
        raise ValueError('configuration name must be a basename')
    path = package_root('z70_description', Path(__file__).resolve().parent.parent) / 'config' / name
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def _config(name: str):
    return yaml.safe_load(config_path(name).read_text())


SHOULDER = (0.0475, 0.14)       # m2_pos: shoulder pivot (M3)           #32735 / W #32741
VENDOR_L1, VENDOR_L2 = 0.1775, 0.205
CAD = _config("cad_arm.yaml")
L1 = float(CAD["L1_m"])         # shoulder (M3) -> elbow (M4), CAD link length
L2 = float(CAD["L2_m"])         # elbow (M4) -> wrist roll (M5), CAD link length
LIMITS = {"m3": (90, 270), "m4": (35, 170), "m5": (0, 155)}            # #32735 arrays
JOINT_STEP_DEG = 10             # arm_joints: +-10 deg per repetition    q #32745
HAND_STEP_M = 0.04              # arm_move: end point +-0.04 m in x or y, then IK   q #32745
A4_MAX_DEG = 15                 # link-2 absolute angle (m4 - m3 + 90) must be <= 15
HEIGHT_MAX = 0.37               # end_pos.y > 0.37 -> FAULT_ARM_HEIGHT
BODY_HALF_X, BODY_TOP = 0.24, 0.14  # |x| < 0.24 and y < 0.14 -> FAULT_ROBOT_COLLISION
GROUND_Y = 0.01                 # y < 0.01 (outside body) -> FAULT_GROUND_COLLISION
PROJECTION_X = 0.37             # |x| > 0.37 -> FAULT_PROJECTION

# Named poses (state machine w #32743, b #32742, W #32741)
POSES = {
    "IN_ROBOT":     dict(m3=0,   m4=0,  m5=0, m6=0),   # stowed; app collapses positions to the shoulder point
    "WORKING_POS":  dict(m3=150, m4=45, m5=0, m6=90),  # after ARM_OUT (gripper open)
    "TRANSFER_POS": dict(m3=150, m4=75, m5=0, m6=0),   # after ARM_GRASP (gripper closed)
}
# Transitions: ARM_OUT from IN_ROBOT -> WORKING_POS; ARM_GRASP (m6==90) -> TRANSFER_POS;
# ARM_PUTDOWN (state != IN_ROBOT, m6==0) -> WORKING_POS; ARM_IN -> IN_ROBOT;
# ARM_JOINTS / ARM_MOVE -> IN_PROCESS_POS.


def vendor_forward(m3, m4):
    """Sagittal 2-link forward kinematics returning (elbow_xy, end_pos_xy)."""
    a1 = math.radians(270 - m3)
    elbow = (SHOULDER[0] + VENDOR_L1 * math.cos(a1), SHOULDER[1] + VENDOR_L1 * math.sin(a1))
    a2 = math.radians(m4 - m3 + 90)
    end = (elbow[0] + VENDOR_L2 * math.cos(a2), elbow[1] + VENDOR_L2 * math.sin(a2))
    return elbow, end


def vendor_inverse(x, y, prefer_current=None):
    """Sagittal 2-link inverse kinematics for (x, y); returns (m3, m4) or None if unreachable."""
    L1, L2 = VENDOR_L1, VENDOR_L2
    dx, dy = x - SHOULDER[0], y - SHOULDER[1]
    d2 = dx * dx + dy * dy
    d = math.sqrt(d2)
    if d > L1 + L2 or d < abs(L1 - L2) or d < 1e-6:
        return None
    sols = []
    for s in (1, -1):
        c = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        th2 = s * math.acos(max(-1.0, min(1.0, c)))                  # link2 relative to link1
        th1 = math.atan2(dy, dx) - math.atan2(L2 * math.sin(th2), L1 + L2 * math.cos(th2))
        m3 = 270 - math.degrees(th1)
        m4 = math.degrees(th1 + th2) + m3 - 90
        m3 %= 360
        sols.append((m3, m4))
    if prefer_current:
        sols.sort(key=lambda s: abs(s[0] - prefer_current[0]) + abs(s[1] - prefer_current[1]))
    return sols[0]


def vendor_check(m3, m4, m5):
    """Evaluate `checkArmInvalidAction` workspace and joint limits; returns fault code or 'SUCCESS'."""
    for k, v in (("m3", m3), ("m4", m4), ("m5", m5)):
        lo, hi = LIMITS[k]
        if v < lo or v > hi:
            return "FAULT_JOINT_LIMIT"
    _, (x, y) = vendor_forward(m3, m4)
    if y > HEIGHT_MAX:
        return "FAULT_ARM_HEIGHT"
    if abs(x) < BODY_HALF_X:
        if y < BODY_TOP:
            return "FAULT_ROBOT_COLLISION"
    else:
        if y < GROUND_Y:
            return "FAULT_GROUND_COLLISION"
        if abs(x) > PROJECTION_X:
            return "FAULT_PROJECTION"
    if m4 - m3 + 90 > A4_MAX_DEG:
        return "FAULT_A4_ANGLE"
    return "SUCCESS"


CONFIG = _config('pincher_geometry.yaml')
CAL = _config('mount_calibration.yaml')
N = 96
THETA = np.arange(N) * 2 * np.pi / N


def rpy(angles):
    r, p, y = angles
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


CAM_R = rpy(np.radians([CAL['gripper_roll_deg'], CAL['gripper_pitch_down_deg'], CAL['gripper_yaw_deg']]))
CAM_C = np.array(CAD['camera_from_wrist_m'], float)
MARK_R = np.array(CAL['marker_rotation_link2'])
MARK_C = np.array(CAL['marker_effective_from_elbow_m']) - np.array([L2, 0, 0])


def rounded_rectangle(half_y, half_z, radius):
    directions = np.column_stack([np.cos(THETA), np.sin(THETA)])
    lo, hi = np.zeros(N), np.full(N, np.hypot(half_y, half_z))
    for _ in range(45):
        mid = (lo + hi) / 2
        q = np.abs(directions * mid[:, None]) - (np.array([half_y, half_z]) - radius)
        sdf = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(np.maximum(q[:, 0], q[:, 1]), 0) - radius
        lo = np.where(sdf <= 0, mid, lo); hi = np.where(sdf > 0, mid, hi)
    return directions * ((lo + hi) / 2)[:, None]


def _underside_surround(ring, x):
    """Apply the underside nose recess contour around the optical pocket."""
    if not 0.148 <= x <= 0.164:
        return ring
    axial = math.exp(-((x - 0.156) / 0.006) ** 2)
    midline = float(np.median(ring[:, 2]))
    low = ring[:, 2] < midline
    bowl = np.exp(-(ring[:, 1] / 0.006) ** 2)
    ring = ring.copy()
    ring[low, 2] += axial * (0.005 * bowl[low])
    return ring


@lru_cache(maxsize=1)
def shell_rings():
    rings = []
    for x, width, bottom, top, radius in CONFIG['housing_sections']:
        yz = rounded_rectangle(width, (top - bottom) / 2, radius)
        ring = np.column_stack([np.full(N, x - L2), yz[:, 0], yz[:, 1] + (top + bottom) / 2])
        ring = _underside_surround(ring, x)
        rings.append(ring)
    for x, radius in CONFIG['pocket_sections']:
        local = np.column_stack([np.full(N, x), radius * np.cos(THETA), radius * np.sin(THETA)])
        rings.append(local @ CAM_R.T + CAM_C)
    return tuple(rings)


@lru_cache(maxsize=2)
def finger_rings(sign):
    rings = []
    for t, bend, w, h in CONFIG['finger_sections']:
        radius = min(w, h) * (0.92 if t > 0.9 else 0.42)
        yz = rounded_rectangle(w, h, radius)
        rings.append(np.column_stack([np.full(N, t), yz[:, 0] - sign * bend, yz[:, 1]]))
    length = CONFIG['finger_forward_reach_m']
    for ring in rings:
        ring[:, 0] *= length
    return tuple(rings)


def root(sign):
    return np.array([CONFIG['root_elbow_x_m'] - L2, sign * CONFIG['root_half_gap_m'], CONFIG['root_z_m']])


@dataclass(frozen=True)
class Body:
    name: str
    center: np.ndarray
    half: np.ndarray
    rotation: np.ndarray
    pad: bool = False

    def contains(self, point, tolerance=0.):
        return bool(np.all(np.abs(self.rotation.T @ (np.asarray(point) - self.center)) <= self.half + tolerance))

    def corners(self):
        signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        return (signs * self.half) @ self.rotation.T + self.center

    def sphere_distance(self, point):
        return float(np.linalg.norm(np.maximum(np.abs(self.rotation.T @ (np.asarray(point) - self.center)) - self.half, 0)))


def _box(name, points, pad=False):
    p = np.concatenate(points); lo = p.min(axis=0); hi = p.max(axis=0)
    return Body(name, (lo + hi) / 2, (hi - lo) / 2, np.eye(3), pad)


@lru_cache(maxsize=1)
def rest_bodies():
    bodies = [_box('arm_wrist_link', shell_rings())]
    for side, sign in (('left', 1), ('right', -1)):
        rings = finger_rings(sign)
        bodies.extend(_box('finger_' + side, (a, b), CONFIG['finger_sections'][index][0] >= CONFIG['pad_start_fraction'])
                      for index, (a, b) in enumerate(zip(rings, rings[1:])))
    return tuple(bodies)


def validate_state(state):
    try:
        st = tuple(float(v) for v in state)
    except (TypeError, ValueError):
        raise ValueError('joint state must be finite numeric angles') from None
    if len(st) not in (2, 3, 4) or not all(math.isfinite(v) for v in st):
        raise ValueError('expected 2, 3 or 4 finite app joint angles')
    st = st + (0., 90.)[len(st) - 2:] if len(st) < 4 else st
    if not (90 <= st[0] <= 270 and 35 <= st[1] <= 170 and 0 <= st[2] <= 155 and 0 <= st[3] <= 90):
        raise ValueError('app joint angle outside model limits')
    return st


def _check_m6(m6):
    if not math.isfinite(m6) or not 0 <= m6 <= 90:
        raise ValueError('M6 outside 0..90')


def finger_frame(sign, m6):
    _check_m6(m6)
    T = np.eye(4); T[:3, :3] = rpy([0, 0, sign * math.radians(m6) * CONFIG['opening_ratio']]); T[:3, 3] = root(sign)
    return T


@lru_cache(maxsize=4096)
def local_bodies(m6=90.):
    _check_m6(m6)
    result = []
    for body in rest_bodies():
        if body.name == 'arm_wrist_link':
            result.append(body); continue
        T = finger_frame(1 if body.name == 'finger_left' else -1, m6)
        result.append(Body(body.name, T[:3, :3] @ body.center + T[:3, 3], body.half, T[:3, :3], body.pad))
    return tuple(result)


def hand_pose(state):
    """CAD M5 wrist in base_footprint, with retained optical yaw/M3 registration."""
    m3, m4, m5, _ = validate_state(state)
    a1 = math.radians(270 - m3 + CAL['measured_M3_offset_deg'])
    a2 = math.radians(m4 - m3 + 90); yaw = math.radians(CAL['arm_yaw_deg'])
    B = rpy([0, -a2, yaw]); Q = rpy([math.radians(m5), 0, 0])
    elbow = np.array(CAD['shoulder_in_base_m']) + rpy([0, 0, yaw]) @ np.array([L1 * math.cos(a1), 0, L1 * math.sin(a1)])
    T = np.eye(4); T[:3, :3] = B @ Q; T[:3, 3] = elbow + B @ np.array([L2, 0, 0])
    return T


def transform_bodies(T, m6=90.):
    T = np.asarray(T, float)
    if (T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1])
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5) or np.linalg.det(T[:3, :3]) < .999):
        raise ValueError('expected finite rigid hand transform')
    return tuple(Body(b.name, T[:3, :3] @ b.center + T[:3, 3], b.half, T[:3, :3] @ b.rotation, b.pad) for b in local_bodies(m6))


def world_bodies(state):
    st = validate_state(state)
    return transform_bodies(hand_pose(st), st[3])


def lowest(bodies):
    return min(float(b.center[2] - np.abs(b.rotation[2]) @ b.half) for b in bodies)


def grasp_reference():
    """Grasp reference point ahead of the closed finger pads in the hand frame."""
    tip = np.mean(finger_rings(1)[-1], axis=0) + root(1)
    return np.array([tip[0] + CONFIG['grasp_aim_ahead_of_tip_m'], 0, CONFIG['root_z_m']])


def articulated_geometry(state):
    """Per-finger poses, terminal points, pad regions, OBBs, and terminal gap."""
    st = validate_state(state); T = hand_pose(st)
    frames = {}; tips = {}; inside = []
    for side, sign in (('left', 1), ('right', -1)):
        F = finger_frame(sign, st[3]); frames['finger_' + side] = T @ F
        ring = finger_rings(sign)[-1] @ F[:3, :3].T + F[:3, 3]
        local = ring.mean(axis=0); tips['finger_' + side] = T[:3, :3] @ local + T[:3, 3]
        inside.append(sign * ring[:, 1].min() if sign == 1 else -ring[:, 1].max())
    bodies = world_bodies(st)
    return dict(finger_frames=frames, terminal_points=tips, pad_regions=tuple(b for b in bodies if b.pad),
                bodies=bodies, terminal_gap_m=sum(inside))


def marker_hand_pose(marker_pose):
    M = np.eye(4); M[:3, :3] = MARK_R; M[:3, 3] = MARK_C
    return np.asarray(marker_pose, float) @ np.linalg.inv(M)

